/**
 * @file afsk_loopback.c
 *
 * @author Emiliano Augusto Gonzalez ( lu3vea @ gmail . com)
 * @date 2026
 * @copyright GNU General Public License v3
 * @see https://github.com/hiperiondev/esp32idf_radioamateur_modem
 *
 * @note
 * This is based on other projects:
 *     VP-Digi: https://github.com/sq8vps/vp-digi
 *     ESP32APRS: https://github.com/nakhonthai/ESP32APRS_Audio
 *     LibAPRS: https://github.com/markqvist/LibAPRS
 *
 *     please contact their authors for more information.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "afsk.h"
#include "afsk_loopback_test.h"
#include "esp32idf_radioamateur_modem.h"
#include "esp32idf_radioamateur_modem_config.h"
#include "modem.h"

static const char *TAG = "loopback";

// Real APRS traffic: a position report, a status, a message with an ack
// sequence, and one frame with a digipeated path.
static const char *const kTestPackets[] = {
    "NOCALL-1>APE32I,WIDE1-1:!4903.50N/07201.75W-Test 1 de LibAPRS",
    "NOCALL-1>APE32I,WIDE1-1,WIDE2-2:>Full duplex loopback status",
    "NOCALL-9>APE32I:=4903.50N/07201.75W>PHG5132/A=001234 mobile",
    "NOCALL-1>APE32I,WIDE1-1::NOCALL-9 :ping via GPIO25 to GPIO35{001",
    AFSK_LOOPBACK_TELEMETRY_PACKET,
};
#define TEST_PACKET_COUNT (sizeof(kTestPackets) / sizeof(kTestPackets[0]))

struct profile {
    const char *name;
    modem_mode_t modem;
    int baud;
};

static const struct profile kProfiles[] = {
    { "Bell202 1200 Bd (1200/2200 Hz)", MODEM_MODEM_BELL202, 1200 },
    { "V.23 1200 Bd (1300/2100 Hz)", MODEM_MODEM_V23, 1200 },
    { "AFSK300 300 Bd (1600/1800 Hz)", MODEM_MODEM_AFSK300, 300 },
    // G3RUH is not AFSK at all: baseband NRZ, scrambled with x^17+x^12+1, and
    // the only profile the demodulator runs at the raw ADC rate rather than the
    // decimated 9600 Hz. It is included here precisely because it shares the
    // least code with the three above - the DPLL, HDLC, AX.25 and FCS stages
    // are common, everything below them is not. See run_profile() for why it
    // needs a longer preamble than its baud rate suggests.
    { "G3RUH 9600 Bd (baseband FSK)", MODEM_MODEM_G3RUH, 9600 },
};
#define PROFILE_COUNT (sizeof(kProfiles) / sizeof(kProfiles[0]))

typedef struct {
    uint8_t expect[AX25_FRAME_MAX_SIZE];
    uint16_t expect_len;
    volatile bool matched;
    volatile uint8_t last_corrected; // ::AX25_NOT_FX25 unless the matched frame arrived as FX.25
    volatile uint32_t rx_total;
    SemaphoreHandle_t done;
} loop_ctx_t;

static loop_ctx_t s_ctx;

static void hexdiff(const uint8_t *a, const uint8_t *b, uint16_t len) {
    char line[128];
    int pos = 0;
    for (uint16_t i = 0; i < len && pos < (int)sizeof(line) - 8; i++) {
        if (a[i] != b[i])
            pos += snprintf(line + pos, sizeof(line) - pos, "[%u:%02X!=%02X]", i, a[i], b[i]);
    }
    if (pos)
        ESP_LOGE(TAG, "  differences: %s", line);
}

/* Runs in the libaprs service task. */
static void on_rx_frame(const modem_rx_frame_t *f, void *arg) {
    loop_ctx_t *c = (loop_ctx_t *)arg;
    ax25_msg_t msg;
    char tnc2[AX25_FRAME_MAX_SIZE];

    c->rx_total++;

    memset(&msg, 0, sizeof(msg));
    ax25_decode((uint8_t *)f->frame, f->len, f->mVrms, &msg);
    modem_format_tnc2(&msg, tnc2, sizeof(tnc2));

    ESP_LOGI(TAG, "  RX %3u B  level=%u%% peak=%d valley=%d  %umVrms", f->len, f->level, f->peak, f->valley, f->mVrms);
    ESP_LOGI(TAG, "     %s", tnc2);

    if (c->matched)
        return; // already got this one, ignore the second demodulator

    if (f->len == c->expect_len && memcmp(f->frame, c->expect, f->len) == 0) {
        c->matched = true;
        c->last_corrected = f->corrected;
        xSemaphoreGive(c->done);
    } else {
        ESP_LOGW(TAG, "  frame mismatch: got %u bytes, expected %u", f->len, c->expect_len);
        if (f->len == c->expect_len)
            hexdiff(c->expect, f->frame, f->len);
    }
}

/**
 * @brief How long one frame can possibly take: TXDelay + everything actually
 *        clocked onto the air + tail, plus a generous margin.
 *
 * The on-air size comes from Ax25GetOnAirSize() rather than being derived from
 * @p len here. It used to be `(len + 16) * 8` bits, which quietly assumes the
 * frame goes out as itself plus flags and an FCS - true for plain AX.25, badly
 * wrong for FX.25, where a 60-byte frame is padded into a K=128/T=32
 * Reed-Solomon block and sent behind an 8-byte correlation tag: 168 bytes on
 * the air, 2.8x the payload.
 *
 * At 1200 and 9600 Bd the old estimate still fit inside the 1500 ms margin, so
 * this only ever surfaced at 300 Bd - where those 168 bytes take 4480 ms
 * against a computed timeout of 4126 ms, and the test declared "FAIL - nothing
 * matched" roughly a second before the transmitter had finished keying. Every
 * frame then arrived byte-exact, just after nobody was listening any more.
 */
static uint32_t frame_timeout_ms(uint16_t len, int baud, uint16_t preamble_ms) {
    uint32_t bits = (uint32_t)Ax25GetOnAirSize(len) * 8U;
    return preamble_ms + (bits * 1000U) / (uint32_t)baud + 1500U;
}

/**
 * @brief Encode, transmit and wait for exactly one TNC2 packet to loop back.
 *
 * Pulled out of run_profile() so afsk_loopback_stress_test_run() can fire the
 * same packet many times over without duplicating the wait/measure/log
 * logic. Behaviour is unchanged from what run_profile() always did per
 * packet; this is a pure extraction.
 *
 * @param fx25_expected Whether this frame is being sent with FX.25 FEC
 *                      encoding active (cfg.fx25_mode == 2 on the caller's
 *                      side). Purely diagnostic: it does not change what is
 *                      sent, only how the round trip is logged and whether
 *                      s_ctx.last_corrected is checked against
 *                      ::AX25_NOT_FX25 to confirm the frame actually
 *                      travelled as an FX.25 block rather than silently
 *                      falling back to plain AX.25.
 * @return  1 round-trip matched byte-exact,
 *          0 sent but nothing matched within the timeout,
 *         -1 could not even be sent (encode or TX-queue failure).
 */
static int send_and_check_one(const char *tnc2, int baud, uint16_t preamble_ms, bool fx25_expected) {
    int len = modem_build_frame_tnc2(tnc2, s_ctx.expect, sizeof(s_ctx.expect));
    if (len <= 0) {
        ESP_LOGE(TAG, "cannot encode packet");
        return -1;
    }
    s_ctx.expect_len = (uint16_t)len;
    s_ctx.matched = false;
    s_ctx.last_corrected = AX25_NOT_FX25;
    xSemaphoreTake(s_ctx.done, 0); /* drain */

    ESP_LOGI(TAG, "TX %3d B  %s%s", len, tnc2, fx25_expected ? "  [FX.25]" : "");

    if (modem_send_raw(s_ctx.expect, (uint16_t)len) != ESP_OK) {
        ESP_LOGE(TAG, "  TX queue rejected the frame");
        return -1;
    }

    /* Poll while we wait so we can report the signal level *during* the
     * transmission. Sampling it after the timeout is useless: the DAC is
     * back at mid-scale by then and always reads ~0 mV. A healthy loop
     * shows several hundred mV here. */
    uint32_t timeout = frame_timeout_ms((uint16_t)len, baud, preamble_ms);
    uint16_t rmsPeak = 0;
    bool dcdSeen = false;
    bool got = false;
    for (uint32_t waited = 0; waited < timeout; waited += 5) {
        if (xSemaphoreTake(s_ctx.done, MODEM_DELAY_TICKS(5)) == pdTRUE) {
            got = true;
            break;
        }
        uint16_t r = afskGetRms();
        if (r > rmsPeak)
            rmsPeak = r;
        if (ModemDcdState())
            dcdSeen = true;
    }

    if (got) {
        if (fx25_expected && s_ctx.last_corrected == AX25_NOT_FX25) {
            ESP_LOGW(TAG, "  OK but NOT carried as FX.25 - decoded as plain AX.25 instead (peak %u mVrms during TX)", rmsPeak);
        } else if (fx25_expected) {
            ESP_LOGI(TAG, "  OK - round trip byte exact, FX.25 decoded (%u byte%s corrected, peak %u mVrms during TX)", s_ctx.last_corrected,
                     s_ctx.last_corrected == 1 ? "" : "s", rmsPeak);
        } else {
            ESP_LOGI(TAG, "  OK - round trip byte exact (peak %u mVrms during TX)", rmsPeak);
        }
    } else {
        ESP_LOGE(TAG, "  FAIL - nothing matched within %" PRIu32 " ms", timeout);
        ESP_LOGE(TAG, "         peak %u mVrms during TX, DCD %s, DC %d mV", rmsPeak, dcdSeen ? "asserted" : "never asserted", afskGetDcOffset());
        if (rmsPeak < 50)
            ESP_LOGE(TAG, "         -> almost no audio reached the ADC: TX or wiring problem");
        else
            ESP_LOGE(TAG, "         -> audio is arriving, so this is a demodulator/rate problem");
    }

    /* Wait for the modulator to finish and the channel to go quiet. */
    while (modem_tx_busy())
        vTaskDelay(MODEM_DELAY_TICKS(10));
    vTaskDelay(MODEM_DELAY_TICKS(200));

    /* Did it turn up byte-exact, just too late? That is a completely different
     * fault from "the demodulator could not decode it", and the message above
     * cannot tell them apart on its own - it will happily blame the modem for
     * a frame that was perfect but still in flight. Worth calling out
     * explicitly: with the timeout now derived from the real on-air size, a
     * frame landing here means something took longer than the physics say it
     * should. */
    if (!got && xSemaphoreTake(s_ctx.done, 0) == pdTRUE) {
        ESP_LOGE(TAG, "         -> but the frame DID arrive byte-exact, after the timeout.");
        ESP_LOGE(TAG, "            Not a demodulator fault: the wait was too short, or TX/RX");
        ESP_LOGE(TAG, "            latency exceeds the %" PRIu32 " ms this frame should need.", timeout);
    }

    return got ? 1 : 0;
}

/**
 * @brief Run every test packet through one profile at one FX.25 mode.
 *
 * @param p         Modem profile to test.
 * @param fx25_mode ::libaprs_config_t.fx25_mode to apply: 0 for plain AX.25,
 *                  2 for FX.25 RX+TX. Callers only pass 2 when the build was
 *                  compiled with -DENABLE_FX25; with fx25_mode=2 on a plain
 *                  build, Ax25Config.fx25/fx25Tx get set but there is no
 *                  Reed-Solomon codec to act on them, so frames would just
 *                  go out as plain AX.25 and the FX.25 verification below
 *                  would (correctly) flag every one of them.
 * @param acc       Aggregate result counters to update.
 */
static bool run_profile(const struct profile *p, uint8_t fx25_mode, afsk_loopback_result_t *acc) {
    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = p->modem;
    cfg.full_duplex = true;    /* mandatory: we must hear ourselves */
    cfg.allow_non_aprs = true; /* accept whatever comes back, we compare bytes ourselves */
    cfg.fx25_mode = fx25_mode;
    const bool wantFx25 = (fx25_mode != 0);
    const char *modeName = wantFx25 ? "FX.25" : "AX.25";
    /*
     * TXDelay is a duration, but what the receiver needs from it is a COUNT of
     * symbols, and the two only track each other at a fixed baud rate.
     *
     * 300 ms buys 45 symbols at 1200 Bd and 3600 at 9600 Bd, so scaling by baud
     * rate alone would give G3RUH a preamble far shorter than the others in
     * wall-clock terms while still being ample in symbols. That is the wrong
     * way round: the analogue front end settles in TIME (the AGC and the DC
     * tracker both work on 20 ms blocks, and the DC tracker only updates once
     * per block), so a 40 ms preamble would key up and be over before the
     * receiver had adapted, no matter how many symbols it contained. G3RUH also
     * has to get its descrambler in step - 17 symbols, negligible - and its
     * DPLL locked from a scrambled, transition-rich signal.
     *
     * So: keep the same wall-clock floor everyone else gets, which G3RUH turns
     * into a very generous symbol count. Being generous here costs 300 ms per
     * frame in a self test and removes a whole class of ambiguous failure.
     */
    cfg.preamble_ms = (p->baud >= 1200) ? 300 : 600;
    cfg.slot_time_ms = 0;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- %s [%s] ---", p->name, modeName);
    modem_set_modem(&cfg);

    /* Let the AGC and the DC tracker settle on the idle DAC level. */
    vTaskDelay(MODEM_DELAY_TICKS(300));

    uint32_t ok = 0;
    bool fx25AllVerified = true;

    for (size_t i = 0; i < TEST_PACKET_COUNT; i++) {
        int rc = send_and_check_one(kTestPackets[i], p->baud, cfg.preamble_ms, wantFx25);
        if (rc < 0)
            continue; /* encode/queue failure: not counted as sent, matches old behaviour */

        acc->sent++;
        if (wantFx25)
            acc->fx25_sent++;

        if (rc == 1) {
            ok++;
            acc->matched++;
            if (wantFx25) {
                acc->fx25_matched++;
                if (s_ctx.last_corrected == AX25_NOT_FX25)
                    fx25AllVerified = false; /* matched, but send_and_check_one already logged the warning */
            }
        }
    }

    ESP_LOGI(TAG, "--- %s [%s]: %" PRIu32 "/%u frames recovered ---", p->name, modeName, ok, (unsigned)TEST_PACKET_COUNT);
    return (ok == TEST_PACKET_COUNT) && fx25AllVerified;
}

bool afsk_loopback_test_run(afsk_loopback_result_t *result) {
    afsk_loopback_result_t acc = { 0 };
    bool allPass = true;

    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, " MODEM full duplex loopback self test");
    ESP_LOGI(TAG, " Wire GPIO%d (DAC out) to GPIO%d (ADC in) and reboot.", MODEM_DAC_GPIO, MODEM_ADC_GPIO);
    ESP_LOGI(TAG, " DAC amplitude %d%% of full scale, ADC attenuation 12 dB.", MODEM_DAC_AMPLITUDE_PCT);
#ifdef ENABLE_FX25
    ESP_LOGI(TAG, " Each profile is tested twice: plain AX.25, then FX.25 (RX+TX).");
#else
    ESP_LOGI(TAG, " FX.25 not compiled in (-DENABLE_FX25 not set) - testing plain AX.25 only.");
#endif
    ESP_LOGI(TAG, "=======================================================");

    s_ctx.done = xSemaphoreCreateBinary();
    if (s_ctx.done == NULL) {
        ESP_LOGE(TAG, "no memory for the semaphore");
        return false;
    }
    s_ctx.rx_total = 0;

    modem_set_rx_callback(on_rx_frame, &s_ctx);

    /* Sanity check 1: is the ADC actually running, and at what rate?
     * A wrong rate is the single most likely reason for a total RX failure. The
     * AFSK correlators and the DPLL assume exactly 9600 Hz after the /4
     * decimation; G3RUH assumes the full ADC rate, undecimated. Both are
     * derived from this one clock, so an error here scales both identically.
     * Retune MODEM_ADC_RATE_NUM/DEN if this is far off. */
    acc.adc_rate_hz = modem_measure_adc_rate(1000);
    int32_t errPct = ((int32_t)acc.adc_rate_hz - MODEM_ADC_SAMPLERATE) * 100 / MODEM_ADC_SAMPLERATE;
    ESP_LOGI(TAG, "measured ADC rate: %" PRIu32 " Hz (nominal %d Hz, %+d%%)", acc.adc_rate_hz, MODEM_ADC_SAMPLERATE, (int)errPct);

    if (acc.adc_rate_hz == 0) {
        ESP_LOGE(TAG, "the ADC is not delivering samples at all - aborting");
        vSemaphoreDelete(s_ctx.done);
        return false;
    }

    /* Measured tolerance is about +/-2 %; beyond that frames stop decoding
     * entirely, so say so plainly instead of letting every profile fail with no
     * explanation. */
    if (errPct > 2 || errPct < -2) {
        ESP_LOGE(TAG, "ADC rate is off by %+d%% - the demodulator tolerates about +/-2%%.", (int)errPct);
        ESP_LOGE(TAG, "Nothing will decode. Fix MODEM_ADC_RATE_NUM/DEN (currently %d/%d)", MODEM_ADC_RATE_NUM, MODEM_ADC_RATE_DEN);
        ESP_LOGE(TAG, "so that the measured rate lands on %d Hz.", MODEM_ADC_SAMPLERATE);
    }

    /* Sanity check 2: is anything reaching the ADC pin?
     * With the DAC parked at mid-scale the input should sit near 1.65 V. */
    ESP_LOGI(TAG, "idle input: DC %d mV, RMS %u mV", afskGetDcOffset(), afskGetRms());

    for (size_t i = 0; i < PROFILE_COUNT; i++) {
        if (!run_profile(&kProfiles[i], 0, &acc))
            allPass = false;
#ifdef ENABLE_FX25
        /* Same packets, same profile, this time with FX.25 FEC framing
         * (RX+TX). fx25_mode=2 asks Ax25Init() to wrap every outgoing frame
         * in a correlation tag + Reed-Solomon block and to look for one on
         * receive; run_profile()'s fx25AllVerified check confirms the frame
         * that came back was actually carried that way rather than falling
         * through as plain AX.25. */
        if (!run_profile(&kProfiles[i], 2, &acc))
            allPass = false;
#endif
    }

    acc.received = s_ctx.rx_total;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, " RESULT: %s", allPass ? "PASS" : "FAIL");
    ESP_LOGI(TAG, " sent=%" PRIu32 " decoded=%" PRIu32 " byte-exact=%" PRIu32, acc.sent, acc.received, acc.matched);
    ESP_LOGI(TAG, "=======================================================");

    if (!allPass) {
        ESP_LOGW(TAG, "Troubleshooting:");
        ESP_LOGW(TAG, " * nothing decoded at all -> check the GPIO%d-GPIO%d wire", MODEM_DAC_GPIO, MODEM_ADC_GPIO);
        ESP_LOGW(TAG, " * idle DC far from ~1650 mV -> wrong pin, or ADC attenuation too low");
        ESP_LOGW(TAG, " * measured ADC rate far from nominal -> retune MODEM_ADC_RATE_NUM/DEN");
        ESP_LOGW(TAG, " * decoded but not byte exact -> clipping; lower MODEM_DAC_AMPLITUDE_PCT");
        ESP_LOGW(TAG, " * byte exact but arrived after the timeout -> not a modem fault at all;");
        ESP_LOGW(TAG, "   see frame_timeout_ms() and Ax25GetOnAirSize()");
    }

    modem_set_rx_callback(NULL, NULL);
    vSemaphoreDelete(s_ctx.done);
    s_ctx.done = NULL;

    if (result)
        *result = acc;
    return allPass;
}

bool afsk_loopback_stress_test_run(modem_mode_t modem, const char *short_packet, const char *long_packet, uint32_t iterations, uint8_t fx25_mode,
                                   afsk_stress_result_t *result) {
    afsk_stress_result_t acc = { 0 };
    const bool wantFx25 = (fx25_mode != 0);

    const struct profile *p = NULL;
    for (size_t i = 0; i < PROFILE_COUNT; i++) {
        if (kProfiles[i].modem == modem) {
            p = &kProfiles[i];
            break;
        }
    }
    if (!p) {
        ESP_LOGE(TAG, "afsk_loopback_stress_test_run: unrecognised modem %d", (int)modem);
        if (result)
            *result = acc;
        return false;
    }

    /* Own our RX callback/semaphore only if the caller hasn't already set one
     * up (e.g. by having just run afsk_loopback_test_run() in this boot). */
    bool ownCtx = (s_ctx.done == NULL);
    if (ownCtx) {
        s_ctx.done = xSemaphoreCreateBinary();
        if (s_ctx.done == NULL) {
            ESP_LOGE(TAG, "no memory for the semaphore");
            if (result)
                *result = acc;
            return false;
        }
        s_ctx.rx_total = 0;
        modem_set_rx_callback(on_rx_frame, &s_ctx);
    }

    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = p->modem;
    cfg.full_duplex = true;
    cfg.allow_non_aprs = true;
    /* Same wall-clock preamble floor run_profile() uses for this baud rate;
     * see the long comment there for why it is NOT simply scaled by baud. */
    cfg.preamble_ms = (p->baud >= 1200) ? 300 : 600;
    cfg.slot_time_ms = 0;
    cfg.fx25_mode = fx25_mode;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, " Stress test: %s [%s]", p->name, wantFx25 ? "FX.25" : "AX.25");
    ESP_LOGI(TAG, " %" PRIu32 " back-to-back rounds, short + long packet each round:", iterations);
    ESP_LOGI(TAG, "   short: %s", short_packet);
    ESP_LOGI(TAG, "   long:  %s", long_packet);
    ESP_LOGI(TAG, " Goal: tell a real, jitter-driven loss rate apart from a");
    ESP_LOGI(TAG, " one-off fluke, which a single 5-packet pass cannot do.");
    ESP_LOGI(TAG, "=======================================================");

    modem_set_modem(&cfg);

    /* Let the AGC and the DC tracker settle on the idle DAC level. */
    vTaskDelay(MODEM_DELAY_TICKS(300));

    acc.adc_rate_hz = modem_measure_adc_rate(1000);
    ESP_LOGI(TAG, "measured ADC rate: %" PRIu32 " Hz (nominal %d Hz)", acc.adc_rate_hz, MODEM_ADC_SAMPLERATE);

    uint32_t fx25Unverified = 0;

    for (uint32_t n = 0; n < iterations; n++) {
        ESP_LOGI(TAG, "--- round %" PRIu32 "/%" PRIu32 " ---", n + 1, iterations);

        int rcShort = send_and_check_one(short_packet, p->baud, cfg.preamble_ms, wantFx25);
        if (rcShort >= 0) {
            acc.attempts++;
            acc.short_attempts++;
            if (rcShort == 1) {
                acc.matched++;
                acc.short_matched++;
                if (wantFx25 && s_ctx.last_corrected == AX25_NOT_FX25)
                    fx25Unverified++; /* send_and_check_one already logged the warning */
            }
        } /* rc < 0: never made it to the wire, not counted as an attempt */

        int rcLong = send_and_check_one(long_packet, p->baud, cfg.preamble_ms, wantFx25);
        if (rcLong >= 0) {
            acc.attempts++;
            acc.long_attempts++;
            if (rcLong == 1) {
                acc.matched++;
                acc.long_matched++;
                if (wantFx25 && s_ctx.last_corrected == AX25_NOT_FX25)
                    fx25Unverified++;
            }
        }
    }

    uint32_t lostPct = (acc.attempts > 0) ? (100 * (acc.attempts - acc.matched) / acc.attempts) : 0;
    uint32_t shortLostPct = (acc.short_attempts > 0) ? (100 * (acc.short_attempts - acc.short_matched) / acc.short_attempts) : 0;
    uint32_t longLostPct = (acc.long_attempts > 0) ? (100 * (acc.long_attempts - acc.long_matched) / acc.long_attempts) : 0;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, " STRESS RESULT (%s): %" PRIu32 "/%" PRIu32 " recovered, %" PRIu32 "%% lost", p->name, acc.matched, acc.attempts, lostPct);
    ESP_LOGI(TAG, "   short: %" PRIu32 "/%" PRIu32 " recovered, %" PRIu32 "%% lost", acc.short_matched, acc.short_attempts, shortLostPct);
    ESP_LOGI(TAG, "   long:  %" PRIu32 "/%" PRIu32 " recovered, %" PRIu32 "%% lost", acc.long_matched, acc.long_attempts, longLostPct);
    if (wantFx25)
        ESP_LOGI(TAG, "   FX.25 verified on %" PRIu32 "/%" PRIu32 " matched frames", acc.matched - fx25Unverified, acc.matched);
    if (acc.attempts >= 20 && lostPct > 0)
        ESP_LOGI(TAG, " Nonzero, roughly stable loss on the same two fixed packets over many tries");
    ESP_LOGI(TAG, " points at residual DAC/ADC clock drift walking the DPLL through a bad");
    ESP_LOGI(TAG, " phase now and then (see the PLL9600 tuning notes in modem.c), rather than");
    ESP_LOGI(TAG, " a one-off bug: it should recur at roughly the same rate every run.");
    ESP_LOGI(TAG, "=======================================================");

    if (ownCtx) {
        modem_set_rx_callback(NULL, NULL);
        vSemaphoreDelete(s_ctx.done);
        s_ctx.done = NULL;
    }

    if (result)
        *result = acc;
    return acc.attempts > 0 && acc.matched == acc.attempts && fx25Unverified == 0;
}
