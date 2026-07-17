/**
 * @file app_main.c
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

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "afsk_diag.h"
#include "afsk_loopback_test.h"
#include "esp32idf_radioamateur_modem.h"

static const char *TAG = "app";

// Set to 0 to skip the self tests and go straight to normal operation.
#define RUN_DIAG          1 /* characterise the hardware/DSP first */
#define RUN_LOOPBACK_TEST 1 /* then push real AX.25 through the loop */

// Repeat-fire a short and a long packet many times over on each modem mode,
// to get an actual loss-rate estimate instead of a single small sample.
// This started life as a G3RUH-only check for the occasional 4/5-not-5/5
// loss seen there; RUN_G3RUH_STRESS_TEST is that same enable flag, now
// generalised to gate a stress run on every profile (Bell202, V23, AFSK300
// and G3RUH), not just G3RUH. Off by default: it adds
// ~4 * STRESS_ITERATIONS * (preamble + frame + gap) to boot time, on the
// order of a few minutes at 100 iterations per mode. Turn on when chasing a
// loss-rate result on any profile.
#define RUN_G3RUH_STRESS_TEST 1
#define STRESS_ITERATIONS     100

static void on_rx_frame(const modem_rx_frame_t *f, void *ctx) {
    (void)ctx;
    ax25_msg_t msg;
    char tnc2[AX25_FRAME_MAX_SIZE];

    memset(&msg, 0, sizeof(msg));
    ax25_decode((uint8_t *)f->frame, f->len, f->mVrms, &msg);
    modem_format_tnc2(&msg, tnc2, sizeof(tnc2));

    ESP_LOGI(TAG, "RX [%u%% %umV] %s", f->level, f->mVrms, tnc2);
}

void app_main(void) {
    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = MODEM_MODEM_BELL202; /* standard APRS */
    cfg.full_duplex = true;
    cfg.preamble_ms = 300;

    ESP_ERROR_CHECK(modem_init(&cfg));

#if RUN_DIAG
    // Measure the analogue path, both sample clocks, the tones and the
    // demodulator before asking AX.25 to work. If something here fails, the
    // loopback test below cannot possibly pass, and this says which stage.
    afsk_diag_result_t diag;
    bool diagOk = afsk_diag_run(&diag);
    if (!diagOk)
        ESP_LOGW(TAG, "characterisation failed - the AX.25 loopback below is"
                      " expected to fail too. Fix the first FAIL stage.");
    modem_set_modem(&cfg);
#endif

#if RUN_LOOPBACK_TEST
    afsk_loopback_result_t result;
    bool pass = afsk_loopback_test_run(&result);
    ESP_LOGI(TAG, "loopback self test: %s", pass ? "PASS" : "FAIL");
#endif

#if RUN_G3RUH_STRESS_TEST
    // Was a profile's occasional loss (first seen on G3RUH, 4/5 not 5/5) a
    // one-off fluke, or a real, repeatable rate? Fire a short and a long
    // packet at every modem mode STRESS_ITERATIONS times back to back and
    // see what fraction actually comes back.
    static const modem_mode_t kStressModems[] = {
        MODEM_MODEM_BELL202,
        MODEM_MODEM_V23,
        MODEM_MODEM_AFSK300,
        MODEM_MODEM_G3RUH,
    };
    for (size_t i = 0; i < sizeof(kStressModems) / sizeof(kStressModems[0]); i++) {
        afsk_stress_result_t stress;
        afsk_loopback_stress_test_run(kStressModems[i], AFSK_LOOPBACK_SHORT_PACKET, AFSK_LOOPBACK_TELEMETRY_PACKET, STRESS_ITERATIONS, 0, &stress);
#ifdef ENABLE_FX25
        // Same profile, same packets, this time with FX.25 FEC framing
        // (RX+TX) - see afsk_loopback_test_run() for why fx25_mode=2 needs
        // -DENABLE_FX25 to mean anything.
        afsk_stress_result_t stressFx25;
        afsk_loopback_stress_test_run(kStressModems[i], AFSK_LOOPBACK_SHORT_PACKET, AFSK_LOOPBACK_TELEMETRY_PACKET, STRESS_ITERATIONS, 2, &stressFx25);
#endif
    }
#endif

#if RUN_LOOPBACK_TEST
    // Put the modem back into the profile normal operation uses.
    modem_set_modem(&cfg);
#endif

    modem_set_rx_callback(on_rx_frame, NULL);

    APRS_setCallsign("NOCALL", 1);
    APRS_setDestination("APE32I", 0);
    APRS_setPath1("WIDE1", 1);
    APRS_setPath2("WIDE2", 2);
    APRS_setLat("4903.50N");
    APRS_setLon("07201.75W");
    APRS_setSymbol('n');
    APRS_printSettings();

    ESP_LOGI(TAG, "-- END TEST --");

    for (;;) {
        // APRS_sendLoc("LibAPRS on ESP-IDF, full duplex");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
