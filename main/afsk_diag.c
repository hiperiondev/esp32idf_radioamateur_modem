/**
 * @file afsk_diag.c
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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "modem.h"
#include "esp32idf_radioamateur_modem_config.h"
#include "afsk.h"
#include "afsk_diag.h"
#include "esp32idf_radioamateur_modem.h"

static const char *TAG = "diag";

#define CAP_LEN 2048
static int16_t s_cap[CAP_LEN];

/* Scratch copy of s_cap for the un-swap experiment in stage 3. File scope on
 * purpose: 4 kB will not fit on the diagnostic task's stack. */

/* Tolerances. The demodulator's correlator and DPLL assume exact rates; the
 * measured window before frames start dropping is about +/-2 %. */
#define RATE_TOL_PCT 2.0f
#define TONE_TOL_PCT 3.0f

/* Missed DAC alarms are held to a far tighter standard than clock accuracy: an
 * ISR that keeps up misses nothing, so anything above measurement noise means
 * it is being blocked and the modulator is freezing. */
#define DAC_MISS_TOL_PCT 0.30f

/* The tolerance that actually decides whether AX.25 works.
 *
 * A miss PERCENTAGE is the wrong thing to judge. Measured end to end against
 * the real modulator and demodulator: 1.72 % of alarms missed uniformly decodes
 * 5/5 frames on Bell202, V.23 and AFSK300 alike - the DPLL simply tracks it.
 * The same 1.72 % delivered as one contiguous blackout per 20 ms decodes 0/5 on
 * all three. The modulator freezes mid-symbol and leaves a phase step no
 * tracking loop can follow.
 *
 * What the old fixed 100 us got wrong is the claim that followed: that the
 * failure "does not care about baud rate". It cares completely. A freeze is a
 * displacement of a symbol EDGE, so what matters is its size relative to a
 * SYMBOL, and a symbol is 833 us at 1200 Bd and 104 us at 9600 Bd - eight times
 * less room. 100 us was calibrated against the three AFSK profiles, where it is
 * 12 % of a symbol and leaves real margin. For G3RUH it is 96 % of a symbol:
 * the tolerance passed a freeze that could displace an edge by nearly a whole
 * symbol, which is how a 38 us gap came back PASS from this stage and 0/5 from
 * the loopback.
 *
 * So budget it as a fraction of the shortest symbol the library can be asked to
 * emit. Host simulation of the real modem.c puts G3RUH's frame loss at roughly
 * 1 % per microsecond of edge jitter (0 us -> 3.8 %, 12 us -> 12.5 %) while
 * every AFSK profile stays at 0 % throughout, so there is no threshold to sit
 * on - only a budget to keep small. 5 % of a symbol is what a DAC ISR that does
 * not touch flash should comfortably achieve. */
#define DAC_GAP_SYMBOL_FRAC 0.05f
#define DAC_FASTEST_BAUD    9600.0f /* G3RUH: the tightest profile on offer */

/* ------------------------------------------------------------------ */
/* helpers                                                            */
/* ------------------------------------------------------------------ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Band the tone search covers. Every profile's mark/space pair (1200..2200 Hz)
 * falls well inside it, with guard room on both sides. */
#define GOERTZEL_SCAN_LO_HZ   600.0f
#define GOERTZEL_SCAN_HI_HZ   3200.0f
#define GOERTZEL_SCAN_STEP_HZ 20.0f

/**
 * @brief Goertzel power of buf at a single frequency bin.
 *
 * Equivalent to |one DFT bin|^2 but computed with a two-tap IIR recurrence,
 * so no table of sines/cosines is needed and the whole capture is a single
 * pass per frequency tried.
 */
static double goertzel_power(const int16_t *buf, int n, double mean, float fs, float freq) {
    double w = 2.0 * M_PI * (double)freq / (double)fs;
    double coeff = 2.0 * cos(w);
    double q1 = 0.0, q2 = 0.0;

    for (int i = 0; i < n; i++) {
        double q0 = coeff * q1 - q2 + ((double)buf[i] - mean);
        q2 = q1;
        q1 = q0;
    }

    double real = q1 - q2 * cos(w);
    double imag = q2 * sin(w);
    return real * real + imag * imag;
}

/**
 * @brief Estimate the dominant frequency in buf with a Goertzel power scan.
 *
 * Sweeps GOERTZEL_SCAN_LO_HZ..GOERTZEL_SCAN_HI_HZ in GOERTZEL_SCAN_STEP_HZ
 * steps, keeps the strongest bin, then refines it with a parabolic
 * interpolation across the three bins straddling the peak (the standard
 * trick for sub-bin resolution on a DFT/Goertzel magnitude peak).
 *
 * This replaces an interpolated zero-crossing counter, which is exact only
 * for a noiseless sine completing a whole number of cycles in the window.
 * Real captures do not: ADC quantisation noise and a non-integer cycle
 * count near either end of the buffer can add or drop a crossing, and for
 * a low tone each cycle spans more samples, so one bad crossing is a bigger
 * fraction of the total (period = fs/freq -> the error compounds worse the
 * lower the tone). That is exactly the shape of the fault it was causing:
 * every AFSK profile's captures were internally consistent, well-formed
 * sine data (see the raw dump this stage prints), and Stage 4's correlator
 * and the full AX.25 loopback both lock onto the same signal cleanly -
 * only the zero-crossing count in this one function was disagreeing with
 * reality, worse on the lower tone of each mark/space pair. Goertzel
 * measures power at a frequency directly instead of counting transitions,
 * which is the same principle Stage 4's correlator already relies on
 * successfully, so it is not vulnerable to that failure mode.
 */
static float measure_freq(const int16_t *buf, int n, float fs) {
    if (n < 64 || fs <= 0.0f)
        return 0.0f;

    double mean = 0;
    for (int i = 0; i < n; i++)
        mean += buf[i];
    mean /= n;

    float bestFreq = 0.0f;
    double bestPower = -1.0;

    for (float f = GOERTZEL_SCAN_LO_HZ; f <= GOERTZEL_SCAN_HI_HZ; f += GOERTZEL_SCAN_STEP_HZ) {
        double p = goertzel_power(buf, n, mean, fs, f);
        if (p > bestPower) {
            bestPower = p;
            bestFreq = f;
        }
    }

    /* Refine with the neighbouring bins. Skip it at either end of the scan
     * range, where one neighbour does not exist. */
    if (bestFreq > GOERTZEL_SCAN_LO_HZ && bestFreq < GOERTZEL_SCAN_HI_HZ) {
        double pLo = goertzel_power(buf, n, mean, fs, bestFreq - GOERTZEL_SCAN_STEP_HZ);
        double pHi = goertzel_power(buf, n, mean, fs, bestFreq + GOERTZEL_SCAN_STEP_HZ);
        double denom = pLo - 2.0 * bestPower + pHi;
        if (denom != 0.0) {
            double offset = 0.5 * (pLo - pHi) / denom;
            if (offset < -1.0)
                offset = -1.0;
            if (offset > 1.0)
                offset = 1.0;
            bestFreq += (float)(offset * GOERTZEL_SCAN_STEP_HZ);
        }
    }

    return bestFreq;
}

static float measure_rms(const int16_t *buf, int n) {
    if (n < 1)
        return 0.0f;
    double mean = 0;
    for (int i = 0; i < n; i++)
        mean += buf[i];
    mean /= n;

    double sq = 0;
    for (int i = 0; i < n; i++) {
        double d = buf[i] - mean;
        sq += d * d;
    }
    return (float)sqrt(sq / n);
}

static void measure_minmax(const int16_t *buf, int n, int16_t *lo, int16_t *hi) {
    *lo = 32767;
    *hi = -32768;
    for (int i = 0; i < n; i++) {
        if (buf[i] < *lo)
            *lo = buf[i];
        if (buf[i] > *hi)
            *hi = buf[i];
    }
}

/*
 * Swap detection used to work by un-swapping the capture at both possible
 * alignments and seeing which of the three resulting Goertzel frequency
 * estimates landed closest to the requested tone. That doesn't work at this
 * sample rate: fs/tone is 35x to 64x here, so a genuine adjacent-sample swap
 * moves a Goertzel estimate by hundredths of a Hz - far under measurement
 * noise - and "closest of three near-identical candidates" just picks up
 * noise. Verified by simulation against both a clean and a genuinely swapped
 * capture: the three candidates were statistically indistinguishable in both
 * cases, which is exactly the alternating, no-pattern false "pair-swapped"
 * warning this used to produce on a healthy build.
 *
 * What a genuine swap reliably DOES do is double the rising zero-crossing
 * rate (each reordered pair straddles the mean from both sides before the
 * waveform resumes its true slope) - see count_rising_crossings() below,
 * which measures that directly instead of inferring it from a frequency
 * comparison.
 */

/**
 * @brief Log the first 32 raw samples - one full cycle at 1200 Hz / 38.4 kHz.
 *
 * A healthy capture sweeps smoothly (~870 -> ~3270 -> ~870, centred near 2070,
 * up to ~235 counts per step). If the deltas change sign where the sine should
 * be steepest and monotonic, the DMA is handing us swapped pairs.
 */
static void dump_raw(const int16_t *buf, int n) {
    char line[256];
    int pos = 0;
    int count = (n < 32) ? n : 32;

    for (int k = 0; k < count && pos < (int)sizeof(line) - 8; k++)
        pos += snprintf(line + pos, sizeof(line) - pos, "%d ", buf[k]);

    ESP_LOGI(TAG, "  raw[0..%d]: %s", count - 1, line);
}

/* ------------------------------------------------------------------ */
/* Stage 1 - DAC -> ADC DC transfer                                   */
/* ------------------------------------------------------------------ */

/* The band the modulator actually swings over: dac_scale() centres on 128 and
 * scales by MODEM_DAC_AMPLITUDE_PCT, so at 60 % it only ever emits codes
 * 52..204. Everything outside that is territory the modem never visits. */
#define DAC_SWING   ((127 * MODEM_DAC_AMPLITUDE_PCT) / 100)
#define DAC_BAND_LO (128 - DAC_SWING)
#define DAC_BAND_HI (128 + DAC_SWING)

static bool stage_dc_sweep(afsk_diag_result_t *r) {
    static const uint8_t codes[] = { 0, 32, 64, 96, 128, 160, 192, 224, 255 };
    const int n = sizeof(codes) / sizeof(codes[0]);
    int mv[sizeof(codes) / sizeof(codes[0])];

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Stage 1: DAC -> ADC DC transfer -----------------------");
    ESP_LOGI(TAG, "  writing GPIO%d, reading GPIO%d", MODEM_DAC_GPIO, MODEM_ADC_GPIO);
    ESP_LOGI(TAG, "  the modulator only uses codes %d..%d (%d %% amplitude); the sweep goes", DAC_BAND_LO, DAC_BAND_HI, MODEM_DAC_AMPLITUDE_PCT);
    ESP_LOGI(TAG, "  wider to find the rails, but only the band is fitted");

    for (int i = 0; i < n; i++) {
        afskDiagDacWrite(codes[i]);
        /* the DC tracker averages 125 samples (~3.3 ms) and only updates once
         * per 20 ms block, so give it several blocks to settle */
        vTaskDelay(MODEM_DELAY_TICKS(120));
        mv[i] = afskGetDcOffset();
        ESP_LOGI(TAG, "  DAC code %3u (%4u mV ideal) -> ADC %4d mV%s", codes[i], (unsigned)(codes[i] * 3300U / 255U), mv[i],
                 (codes[i] >= DAC_BAND_LO && codes[i] <= DAC_BAND_HI) ? "  [band]" : "");
    }
    afskDiagDacWrite(128);

    r->dc_min_mv = mv[0];
    r->dc_max_mv = mv[n - 1];

    /*
     * Least squares fit of mv = gain*code + offset, over the modem's band ONLY.
     *
     * Fitting the whole 0..255 sweep is what made a healthy board report
     * 11.86 mV/code and 60 mV of nonlinearity: at 12 dB the ADC bottoms out
     * around 140 mV and saturates near 3.1 V, so codes 0 and 255 are clamped
     * (the 224->255 segment reads 8.97 mV/code, not 12.94). Those two points
     * drag the slope down and dominate the residual - while describing a part
     * of the transfer curve the modulator never reaches. Fitted over 52..204
     * the same board gives 12.18 mV/code and 7 mV worst error, which is the
     * truth about the path the modem uses.
     */
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int fitted = 0;
    for (int i = 0; i < n; i++) {
        if (codes[i] < DAC_BAND_LO || codes[i] > DAC_BAND_HI)
            continue;
        sx += codes[i];
        sy += mv[i];
        sxx += (double)codes[i] * codes[i];
        sxy += (double)codes[i] * mv[i];
        fitted++;
    }

    double denom = (double)fitted * sxx - sx * sx;
    double gain = (denom != 0) ? (((double)fitted * sxy - sx * sy) / denom) : 0;
    double offset = (fitted > 0) ? ((sy - gain * sx) / fitted) : 0;

    double worst = 0;
    for (int i = 0; i < n; i++) {
        if (codes[i] < DAC_BAND_LO || codes[i] > DAC_BAND_HI)
            continue;
        double err = fabs((gain * codes[i] + offset) - mv[i]);
        if (err > worst)
            worst = err;
    }

    r->dc_gain_mv_per_code = (float)gain;
    r->dc_worst_lin_err_mv = (float)worst;

    ESP_LOGI(TAG,
             "  band fit (%d points): %.2f mV/code (ideal ~12.94, %+.1f %%), offset %.0f mV,"
             " worst nonlinearity %.0f mV",
             fitted, gain, (gain - 12.94) * 100.0 / 12.94, offset, worst);
    ESP_LOGI(TAG,
             "  rails: code 0 -> %d mV, code 255 -> %d mV (the ESP32 DAC does not reach"
             " either supply and the ADC clamps near 3.1 V at 12 dB - expected)",
             mv[0], mv[n - 1]);

    bool ok = true;
    int span = r->dc_max_mv - r->dc_min_mv;

    if (fitted < 3) {
        ESP_LOGE(TAG,
                 "  FAIL: only %d sweep points fall inside the modulator's band"
                 " (%d..%d). Nothing to fit - is MODEM_DAC_AMPLITUDE_PCT sane?",
                 fitted, DAC_BAND_LO, DAC_BAND_HI);
        ok = false;
    } else if (span < 500) {
        ESP_LOGE(TAG,
                 "  FAIL: the ADC barely moved (%d mV span). The DAC is not"
                 " reaching the ADC pin - check the GPIO%d-GPIO%d wire.",
                 span, MODEM_DAC_GPIO, MODEM_ADC_GPIO);
        ok = false;
    } else if (gain < 8.0 || gain > 16.0) {
        ESP_LOGE(TAG,
                 "  FAIL: slope %.2f mV/code is not a direct connection."
                 " Expect ~12.9; a divider or the wrong attenuation?",
                 gain);
        ok = false;
    } else if (worst > 40) {
        ESP_LOGW(TAG,
                 "  WARN: %.0f mV of nonlinearity inside the modulator's own band."
                 " That is real distortion, not the rails.",
                 worst);
    }

    if (ok)
        ESP_LOGI(TAG, "  PASS: analogue path is linear and connected");

    r->dc_sweep_ok = ok;
    return ok;
}

/* ------------------------------------------------------------------ */
/* Stage 2 - the two sample clocks                                    */
/* ------------------------------------------------------------------ */

static bool stage_clocks(afsk_diag_result_t *r) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Stage 2: sample clocks --------------------------------");

    /*
     * Every reference here comes from the component, not from this file's own
     * macros. The hardware is configured by afsk.c; if main/ was compiled with a
     * different MODEM_DAC_SAMPLERATE then this file's macro describes nothing
     * that exists, and measuring against it condemns a perfectly good clock.
     * That is a build fault, reported as one below - not a clock fault.
     */
    uint32_t dacNominal = afskGetDacSampleRate();
    uint32_t adcNominal = afskGetAdcSampleRate();

    if ((dacNominal != (uint32_t)MODEM_DAC_SAMPLERATE) || (adcNominal != (uint32_t)MODEM_ADC_SAMPLERATE)) {
        ESP_LOGW(TAG, "  BUILD MISMATCH: the component runs DAC %" PRIu32 " / ADC %" PRIu32 " Hz, this file was", dacNominal, adcNominal);
        ESP_LOGW(TAG, "  compiled with %d / %d. Measuring against the component's values, which", MODEM_DAC_SAMPLERATE, MODEM_ADC_SAMPLERATE);
        ESP_LOGW(TAG, "  are the ones the hardware actually got. Fix the build: define these in");
        ESP_LOGW(TAG, "  libaprs_config.h, not per-target in a CMakeLists, so every translation");
        ESP_LOGW(TAG, "  unit agrees.");
    }

    r->adc_rate_hz = modem_measure_adc_rate(500);

    /* The DAC clock only runs while the timer does, so key up a tone to count it. */
    afskDiagToneStart(1000);
    vTaskDelay(MODEM_DELAY_TICKS(50));
    uint32_t c0 = afskGetDacIsrCount();
    int64_t t0 = esp_timer_get_time();
    afskDiagDacGapStart();
    vTaskDelay(MODEM_DELAY_TICKS(500));
    uint32_t maxGapUs = afskDiagDacGapStop();
    uint32_t c1 = afskGetDacIsrCount();
    int64_t t1 = esp_timer_get_time();
    afskDiagToneStop();
    r->dac_max_gap_us = maxGapUs;

    r->dac_rate_hz = (t1 > t0) ? (uint32_t)(((uint64_t)(c1 - c0) * 1000000ULL) / (uint64_t)(t1 - t0)) : 0;

    float adcErr = ((float)r->adc_rate_hz - (float)adcNominal) * 100.0f / (float)adcNominal;

    /*
     * The DAC needs two verdicts, not one, and conflating them is what let a
     * 2.1 % missed-alarm rate hide behind a 2 % tolerance for so long:
     *
     *  1. QUANTISATION - the alarm is an integer number of timer ticks, so the
     *     achievable rate is not exactly the nominal one. Systematic, known at
     *     compile time, and harmless while it stays small.
     *  2. MISSED ALARMS - the ISR failing to keep up with the alarm it was
     *     actually given. Every miss freezes the modulator (phase accumulator
     *     AND symbol counter) for one sample. This is the fault that matters,
     *     it must be judged against the alarm rate, and its tolerance is tight:
     *     a healthy ISR misses nothing at all.
     */
    float alarmRate = afskGetDacAlarmRate();
    if (alarmRate <= 0.0f)
        alarmRate = (float)dacNominal; /* timer not configured yet */

    float quantErr = (alarmRate - (float)dacNominal) * 100.0f / (float)dacNominal;
    float missPct = (alarmRate - (float)r->dac_rate_hz) * 100.0f / alarmRate;

    ESP_LOGI(TAG, "  ADC: %6" PRIu32 " Hz  (nominal %" PRIu32 ", %+.2f %%)", r->adc_rate_hz, adcNominal, adcErr);
    ESP_LOGI(TAG, "  DAC: %6" PRIu32 " Hz  (alarm %.1f Hz, nominal %" PRIu32 ")", r->dac_rate_hz, alarmRate, dacNominal);
    ESP_LOGI(TAG, "         missed alarms   %+.2f %%  (tolerance %.2f %%)", missPct, (double)DAC_MISS_TOL_PCT);
    ESP_LOGI(TAG, "         quantisation    %+.2f %%  (tolerance %.2f %%)", quantErr, (double)RATE_TOL_PCT);
    /* The gap includes the nominal period; the jitter is the excess over it,
     * and the jitter is what displaces the edge. */
    float periodUs = 1000000.0f / alarmRate;
    float symbolUs = 1000000.0f / DAC_FASTEST_BAUD;
    float budgetUs = symbolUs * DAC_GAP_SYMBOL_FRAC;
    float gapTolUs = periodUs + budgetUs;
    float jitterUs = ((float)maxGapUs > periodUs) ? ((float)maxGapUs - periodUs) : 0.0f;

    ESP_LOGI(TAG, "         longest freeze  %" PRIu32 " us   (one period is %.1f us, so %.1f us of jitter)", maxGapUs, periodUs, jitterUs);
    ESP_LOGI(TAG, "         edge jitter     %.1f us   (budget %.1f us = %.0f %% of a %.0f Bd symbol)", jitterUs, budgetUs,
             (double)(DAC_GAP_SYMBOL_FRAC * 100.0f), (double)DAC_FASTEST_BAUD);
    ESP_LOGI(TAG, "                         = %.1f %% of a 9600 Bd symbol, %.1f %% of a 1200 Bd one", jitterUs * 100.0f / symbolUs,
             jitterUs * 100.0f / (1000000.0f / 1200.0f));

    r->adc_rate_ok = (fabsf(adcErr) <= RATE_TOL_PCT);
    r->dac_rate_ok = (fabsf(missPct) <= DAC_MISS_TOL_PCT) && (fabsf(quantErr) <= RATE_TOL_PCT) && ((float)maxGapUs <= gapTolUs);

    if (!r->adc_rate_ok)
        ESP_LOGE(TAG,
                 "  FAIL: ADC rate off by %+.2f %% - fix MODEM_ADC_RATE_NUM/DEN"
                 " (currently %d/%d)",
                 adcErr, MODEM_ADC_RATE_NUM, MODEM_ADC_RATE_DEN);

    if ((float)maxGapUs > gapTolUs) {
        ESP_LOGE(TAG, "  FAIL: the modulator froze for up to %" PRIu32 " us in one stretch (%.1f samples),", maxGapUs, maxGapUs * alarmRate / 1000000.0f);
        ESP_LOGE(TAG, "        which displaces a transmitted symbol edge by %.1f us.", jitterUs);
        ESP_LOGE(TAG, "        This is NOT the %.2f %% of missed alarms above - that much spread", missPct);
        ESP_LOGE(TAG, "        evenly decodes every frame. The alarm is scheduled on an absolute");
        ESP_LOGE(TAG, "        count, so nothing is lost and no rate error appears: the sample");
        ESP_LOGE(TAG, "        just comes out late. It is pure edge jitter.");
        ESP_LOGE(TAG, "        At 1200 Bd that is %.1f %% of a symbol and you will not see it.", jitterUs * 100.0f / (1000000.0f / 1200.0f));
        ESP_LOGE(TAG, "        At 9600 Bd (G3RUH) it is %.1f %% of a symbol and it costs frames.", jitterUs * 100.0f / symbolUs);
        ESP_LOGE(TAG, "        Something is blocking the DAC ISR. The usual cause is a FLASH-resident");
        ESP_LOGE(TAG, "        call from inside it (an XIP fetch costs tens of us) - check that");
        ESP_LOGE(TAG, "        everything on the ISR path is IRAM_ATTR and every table it reads is");
        ESP_LOGE(TAG, "        DRAM_ATTR. Otherwise look for work at interrupt level, or a");
        ESP_LOGE(TAG, "        portENTER_CRITICAL held across a long loop.");
    }

    if (missPct > DAC_MISS_TOL_PCT) {
        ESP_LOGW(TAG, "  WARN: the DAC ISR misses %.2f %% of its alarms (~%.0f/s), against a %.1f us", missPct, (double)((missPct / 100.0f) * alarmRate),
                 1000000.0f / alarmRate);
        ESP_LOGW(TAG, "        budget. Harmless on its own at this level, but see the freeze above.");
    } else if (missPct < -DAC_MISS_TOL_PCT) {
        ESP_LOGE(TAG,
                 "  FAIL: more DAC ISRs than alarms (%+.2f %%). The measurement or the"
                 " timer configuration is wrong.",
                 missPct);
    }

    if (fabsf(quantErr) > RATE_TOL_PCT)
        ESP_LOGE(TAG,
                 "  FAIL: the timer cannot express %" PRIu32 " Hz (nearest is %.1f Hz,"
                 " %+.2f %%). Pick a sample rate the resolution divides evenly.",
                 dacNominal, alarmRate, quantErr);

    if (r->adc_rate_ok && r->dac_rate_ok)
        ESP_LOGI(TAG, "  PASS: ADC within %.0f %%, DAC ISR keeping up with its alarm, no freeze", (double)RATE_TOL_PCT);

    return r->adc_rate_ok && r->dac_rate_ok;
}

/* ------------------------------------------------------------------ */
/* Stage 3 - tone loopback                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Count rising zero crossings around @p mean.
 *
 * This is the actual swap-sensitive statistic. A genuine adjacent-sample swap
 * turns each true crossing into two (the reordered pair straddles the mean
 * from both sides before the waveform resumes its real slope), so a swapped
 * capture crosses roughly TWICE as often as a correctly-ordered one. Verified
 * numerically at these tone frequencies and this sample rate: correct capture
 * tracks the expected count (want * got / fs) to within a sample or two;
 * swapped tracks almost exactly double it, every time, regardless of phase.
 */
static int count_rising_crossings(const int16_t *buf, int n, float mean) {
    int cnt = 0;
    for (int i = 1; i < n; i++) {
        if ((float)buf[i - 1] <= mean && (float)buf[i] > mean)
            cnt++;
    }
    return cnt;
}

static bool stage_tones(afsk_diag_result_t *r) {
    static const uint32_t tones[] = { 1200, 1300, 1600, 1800, 2100, 2200 };
    const int n = sizeof(tones) / sizeof(tones[0]);
    bool ok = true;
    float worst = 0;
    int swapped = 0; /* tones where the raw capture is genuinely pair-swapped */

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Stage 3: tone loopback --------------------------------");
    ESP_LOGI(TAG, "  DAC emits a sine, ADC measures what came back round the wire");
    ESP_LOGI(TAG, "  %-9s %-11s %-9s %-9s %s", "want", "at ADC", "err", "level", "verdict");

    float fs = (r->adc_rate_hz > 0) ? (float)r->adc_rate_hz : (float)MODEM_ADC_SAMPLERATE;

    for (int i = 0; i < n; i++) {
        afskDiagToneStart(tones[i]);
        vTaskDelay(MODEM_DELAY_TICKS(60)); /* let it settle */

        int got = afskDiagCaptureRaw(s_cap, CAP_LEN, 500);
        afskDiagToneStop();

        if (got < CAP_LEN / 2) {
            ESP_LOGE(TAG, "  %-9" PRIu32 " capture failed (%d samples)", tones[i], got);
            ok = false;
            continue;
        }

        float want = (float)tones[i];

        /*
         * Frequency is always measured from the capture exactly as it came
         * out of adc_ingest(). At this oversampling ratio (fs/want is 35x to
         * 64x) a genuine adjacent-sample swap perturbs a Goertzel frequency
         * estimate by hundredths of a Hz - far below measurement noise - so
         * "which of {as-is, un-swap even, un-swap odd} lands closest to the
         * requested tone" cannot tell a swapped capture from a clean one; it
         * was measured to pick the wrong answer on essentially a coin flip,
         * which is exactly the alternating false "pair-swapped" pattern this
         * used to produce. Do not resurrect that comparison.
         */
        float f = measure_freq(s_cap, got, fs);
        float rms = measure_rms(s_cap, got);
        int16_t lo, hi;
        measure_minmax(s_cap, got, &lo, &hi);

        /*
         * Swap detection is a SEPARATE statistic: rising zero crossings, which
         * a genuine swap actually doubles (see count_rising_crossings()).
         * Compare the as-captured count against the crossing count the tone
         * itself predicts, not against an arbitrary swapped alternative -
         * that is what makes this a real measurement instead of a beauty
         * contest between three noisy candidates.
         */
        float mean = 0.0f;
        for (int k = 0; k < got; k++)
            mean += (float)s_cap[k];
        mean /= (float)got;

        int zcRaw = count_rising_crossings(s_cap, got, mean);
        float expectedZc = want * (float)got / fs;
        float zcRatio = (expectedZc > 0.0f) ? ((float)zcRaw / expectedZc) : 0.0f;

        /* Genuine swap: crossing count lands near 2x expected, not near 1x.
         * Require it to be unambiguously in the "doubled" band rather than
         * just "closer to 2x than 1x", so real jitter/noise near the 1.5x
         * midpoint is reported as inconclusive rather than as a false alarm. */
        bool isSwapped = (zcRatio > 1.7f && zcRatio < 2.3f);
        bool isClean = (zcRatio > 0.7f && zcRatio < 1.3f);

        if (isSwapped)
            swapped++;

        const char *verdictSuffix = isSwapped ? " pair-swapped" : "";

        float err = (f > 0) ? ((f - want) * 100.0f / want) : -100.0f;
        if (fabsf(err) > worst)
            worst = fabsf(err);

        bool tOk = (fabsf(err) <= TONE_TOL_PCT);
        bool clipped = (lo <= 2 || hi >= 4093);
        if (!tOk || clipped)
            ok = false;

        ESP_LOGI(TAG, "  %-9" PRIu32 " %-11.1f %-+9.2f %-9.0f %s%s%s", tones[i], f, err, rms, tOk ? "ok" : "BAD FREQ", clipped ? " CLIPPED" : "",
                 verdictSuffix);

        if (!tOk || isSwapped)
            ESP_LOGI(TAG, "      zero crossings: %d measured, %.1f expected clean (ratio %.2f)%s", zcRaw, expectedZc, zcRatio,
                     (!isSwapped && !isClean) ? " - inconclusive, not counted either way" : "");

        if (i == 0)
            dump_raw(s_cap, got);

        vTaskDelay(MODEM_DELAY_TICKS(20));
    }

    r->worst_tone_err_pct = worst;
    r->tones_ok = ok;

    if (swapped > 0) {
        ESP_LOGW(TAG, "  WARNING: %d of %d tones show a doubled zero-crossing rate, so the FIFO", swapped, n);
        ESP_LOGW(TAG, "           is still handing out the ESP32's swapped DMA pairs and the");
        ESP_LOGW(TAG, "           un-swap in adc_ingest() is not doing its job.");
        ESP_LOGW(TAG, "           The AFSK profiles will not notice (the 38400 -> 9600 decimation");
        ESP_LOGW(TAG, "           filter averages each pair back together) but G3RUH reads this");
        ESP_LOGW(TAG, "           stream raw and will lose every frame. Expect 0/5 at 9600 Bd.");
    }

    if (!ok) {
        ESP_LOGE(TAG, "  FAIL: worst tone error %.2f %%.", worst);
        ESP_LOGE(TAG, "        A consistent error on every tone means a clock rate is");
        ESP_LOGE(TAG, "        wrong (see stage 2). Errors only on high tones mean the");
        ESP_LOGE(TAG, "        DAC or ADC is not keeping up.");
    } else {
        ESP_LOGI(TAG, "  PASS: every tone within %.0f %% (worst %.2f %%)", (double)TONE_TOL_PCT, worst);
    }
    return ok;
}

/* ------------------------------------------------------------------ */
/* Stage 4 - demodulator discrimination                               */
/* ------------------------------------------------------------------ */

static bool check_profile(const char *name, modem_mode_t modem, afsk_diag_result_t *r) {
    (void)r;
    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = modem;
    cfg.full_duplex = true;
    cfg.allow_non_aprs = true;
    modem_set_modem(&cfg);
    vTaskDelay(MODEM_DELAY_TICKS(50));

    /* Same reasoning as check_g3ruh(): from here on this function drives
     * demodState[0] directly via ModemDiagDemodulate(), and the RX task is
     * already running again (afskSetModem() only suspends it across
     * ModemInit()). Without pausing it here, AFSK_Poll() on the RX task and
     * this loop both call filterRun() on the same struct Filter.samples[]
     * concurrently - a torn-state race, not a filter/config bug, and the one
     * actually responsible for the AFSK300 crash inside filterRun(). */
    afskDiagRxTaskPause();

    /*
     * Two different pairs of tones, and the difference is the point.
     *
     * ModemGetTones() is what the correlator coefficients are built from - the
     * profile's nominal 1200/2200. ModemGetStepTones() is what the modulator can
     * actually put on the wire: an integer step through a 512 entry table at
     * 38400 Hz can only make multiples of 75 Hz, so Bell 202's space comes out
     * at 2175, V.23's mark at 1275, AFSK300's mark at 1575.
     *
     * The demodulator has to separate what is TRANSMITTED, so that is what gets
     * fed to it here. Driving the nominal tones instead - which is what this
     * stage used to do, via the diagnostic's own exact 32-bit accumulator -
     * tests a signal this modem never sends and hides the step error completely.
     */
    float mark = 0, space = 0;
    ModemGetTones(&mark, &space);

    float txMark = 0, txSpace = 0;
    ModemGetStepTones(&txMark, &txSpace);

    int32_t score[2] = { 0, 0 };
    float measured[2] = { 0, 0 };
    const float want[2] = { txMark, txSpace };
    const float nominal[2] = { mark, space };

    for (int t = 0; t < 2; t++) {
        afskDiagToneStart((uint32_t)want[t]);
        vTaskDelay(MODEM_DELAY_TICKS(80));

        /* what the demodulator itself is fed: post DC removal, AGC, clamp and
         * decimation to 9600 Hz */
        int got = afskDiagCaptureDemodInput(s_cap, CAP_LEN, 800);
        afskDiagToneStop();

        if (got < 256) {
            ESP_LOGE(TAG,
                     "  %s: demodulator saw nothing (%d samples) - the RX gate"
                     " never opened",
                     name, got);
            afskDiagRxTaskResume();
            return false;
        }

        measured[t] = measure_freq(s_cap, got, (float)afskGetDemodSampleRate());

        /* run the real correlator over the tail (skip the filter warm-up) */
        int64_t sum = 0;
        int cnt = 0;
        for (int i = 64; i < got; i++) {
            int32_t v = ModemDiagDemodulate(0, s_cap[i]);
            sum += v;
            cnt++;
        }
        score[t] = (int32_t)(cnt ? (sum / cnt) : 0);
        vTaskDelay(MODEM_DELAY_TICKS(20));
    }

    /* Mark drives the "Lo" correlator, space the "Hi" one, so the two means must
     * come out with opposite signs. Absolute polarity is irrelevant - NRZI is
     * transition coded - but they must separate. */
    bool separated = ((score[0] > 0) != (score[1] > 0)) && (labs((long)score[0]) > 50) && (labs((long)score[1]) > 50);

    for (int t = 0; t < 2; t++) {
        ESP_LOGI(TAG,
                 "  %-9s %-5s want %4.0f Hz, TX emits %6.1f (%+.2f %%), saw %6.1f,"
                 " corr %+7" PRId32,
                 (t == 0) ? name : "", (t == 0) ? "mark" : "space", nominal[t], want[t], (want[t] - nominal[t]) * 100.0f / nominal[t], measured[t], score[t]);
    }
    ESP_LOGI(TAG, "  %-9s %s", "", separated ? "-> tones separate: ok" : "-> NOT SEPARATED");

    if (!separated)
        ESP_LOGE(TAG,
                 "  FAIL: %s correlator cannot tell mark from space. If the tones"
                 " above read correctly, this is a DSP/scaling fault, not wiring.",
                 name);

    afskDiagRxTaskResume();
    return separated;
}

/* ------------------------------------------------------------------ */
/* Stage 4b - G3RUH baseband slicer                                   */
/* ------------------------------------------------------------------ */

/*
 * G3RUH cannot be checked the way the three AFSK profiles above are checked,
 * and running it through check_profile() would be worse than not testing it at
 * all - it would report a pass or a fail about a property the profile does not
 * have.
 *
 * There is no mark tone and no space tone. G3RUH sends baseband NRZ: the DAC
 * sits at one of two levels for the whole symbol, and the demodulator is a low
 * pass filter (lpf9600) followed by a slicer at zero. "Do the two tones drive
 * the correlator to opposite signs" is not a question about this modem.
 * ModemGetTones() now honestly reports 0/0 for it, and driving 0 Hz measures
 * nothing.
 *
 * What matters instead is the shape of the receive path, and that is what is
 * measured here, by driving sines and watching lpf9600's gain:
 *
 *   1200 Hz  a slow run of like symbols        -> must pass ~unattenuated
 *   4800 Hz  alternating symbols at 9600 Bd,
 *            the worst case pattern the modem
 *            can be asked to carry             -> must survive with the eye open
 *   9600 Hz  above the symbol rate: noise,
 *            never signal                      -> must be rejected
 *
 * That triple is also what catches the mistake this profile is most likely to
 * be built with. The demodulator is fed the UNDECIMATED ADC stream (see
 * AFSK_Poll()); if it is ever wired to the decimated 9600 Hz path instead, the
 * 4800 Hz probe lands exactly on Nyquist and is destroyed by the anti-alias
 * filter, and this stage fails loudly and specifically rather than leaving
 * G3RUH to fail silently in the AX.25 test.
 */
struct g3ruh_probe {
    uint32_t hz;
    const char *what;
    float lo; /* acceptable gain window, output RMS / input RMS */
    float hi;
};

static bool check_g3ruh(afsk_diag_result_t *r) {
    (void)r;
    /* Windows are wide on purpose: lpf9600 is nominally 0.96 / 0.50 / 0.06 at
     * these three frequencies (Gaussian, fc=4800, fs=38400). We are looking for
     * a receive path with the right shape, not measuring the filter. */
    static const struct g3ruh_probe kProbes[] = {
        /* Recalibrated for the fs=76800 lpf9600 (see modem.c). These windows
         * describe the shape of a specific filter, so they move when it does;
         * the old 0.00..0.18 at 9600 Hz described the fs=38400 9-tap version and
         * would now fail a demodulator that is working correctly. Computed
         * response of the current coefficients: 0.981 / 0.735 / 0.291. */
        { 1200, "run of like symbols", 0.70f, 1.30f },
        { 4800, "alternating symbols", 0.50f, 0.95f },
        { 9600, "above the symbol rate", 0.15f, 0.45f },
    };
    const int n = sizeof(kProbes) / sizeof(kProbes[0]);

    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = MODEM_MODEM_G3RUH;
    cfg.full_duplex = true;
    cfg.allow_non_aprs = true;
    modem_set_modem(&cfg);
    vTaskDelay(MODEM_DELAY_TICKS(50));

    /* From here on this function calls ModemDiagDemodulate(0, ...) directly,
     * driving demodState[0] by hand. afskSetModem() (inside libaprs_set_modem()
     * above) only suspends the RX task across ModemInit() and has already
     * resumed it - the live RX task is running again and would otherwise call
     * demodulate() -> filterRun() on this exact same demodState[0] from
     * AFSK_Poll() concurrently with every ModemDiagDemodulate() call below.
     * Two cores mutating the same non-reentrant struct Filter.samples[] ring
     * buffer with no lock between them is a torn-state crash waiting to
     * happen. Hold the RX task off for the whole probe loop, not just the
     * reinit. */
    afskDiagRxTaskPause();

    float fs = (float)afskGetDemodSampleRate();
    bool ok = true;

    /* The rate the demodulator is fed at is itself a claim worth checking: this
     * profile is the only one that should NOT see the decimated stream. */
    if ((uint32_t)fs != (uint32_t)MODEM_ADC_SAMPLERATE) {
        ESP_LOGE(TAG, "  G3RUH: the demodulator is being fed at %.0f Hz, not the ADC's %d Hz.", fs, MODEM_ADC_SAMPLERATE);
        ESP_LOGE(TAG, "         At 9600 Bd that is at most one sample per symbol - the DPLL has");
        ESP_LOGE(TAG, "         nothing to recover a clock from. The decimation in AFSK_Poll()");
        ESP_LOGE(TAG, "         must be bypassed for MODEM_9600.");
        ok = false;
    }

    for (int i = 0; i < n; i++) {
        afskDiagToneStart(kProbes[i].hz);
        vTaskDelay(MODEM_DELAY_TICKS(80));

        int got = afskDiagCaptureDemodInput(s_cap, CAP_LEN, 800);
        afskDiagToneStop();

        if (got < 256) {
            ESP_LOGE(TAG, "  G3RUH: demodulator saw nothing (%d samples) - the RX gate never opened", got);
            afskDiagRxTaskResume();
            return false;
        }

        float measured = measure_freq(s_cap, got, fs);

        /* Skip the filter warm-up, then run the real thing. For MODEM_9600
         * ModemDiagDemodulate() is lpf9600 and nothing else - the correlator is
         * bypassed - so its output RMS over its input RMS is the gain. */
        double insq = 0, outsq = 0;
        int cnt = 0;
        int32_t omin = 0, omax = 0;
        for (int k = 64; k < got; k++) {
            int32_t v = ModemDiagDemodulate(0, s_cap[k]);
            insq += (double)s_cap[k] * s_cap[k];
            outsq += (double)v * v;
            if (v < omin)
                omin = v;
            if (v > omax)
                omax = v;
            cnt++;
        }

        float inRms = cnt ? (float)sqrt(insq / cnt) : 0.0f;
        float outRms = cnt ? (float)sqrt(outsq / cnt) : 0.0f;
        float gain = (inRms > 1.0f) ? (outRms / inRms) : 0.0f;

        bool gOk = (gain >= kProbes[i].lo) && (gain <= kProbes[i].hi);
        /* A rejected probe has nothing to slice, so only the passing ones are
         * asked to swing both ways. */
        bool sliced = (kProbes[i].hi <= 0.2f) || ((omin < 0) && (omax > 0));

        if (!gOk || !sliced)
            ok = false;

        ESP_LOGI(TAG, "  %-9s %-22s %4" PRIu32 " Hz, saw %6.1f, gain %.3f (want %.2f..%.2f), slicer %+" PRId32 "..%+" PRId32 "  %s", (i == 0) ? "G3RUH" : "",
                 kProbes[i].what, kProbes[i].hz, measured, gain, kProbes[i].lo, kProbes[i].hi, omin, omax,
                 (gOk && sliced) ? "ok" : (!gOk ? "BAD GAIN" : "NOT SLICED"));

        vTaskDelay(MODEM_DELAY_TICKS(20));
    }

    ESP_LOGI(TAG, "  %-9s %s", "", ok ? "-> baseband path has the right shape: ok" : "-> BASEBAND PATH WRONG");

    if (!ok)
        ESP_LOGE(TAG, "  FAIL: G3RUH's receive filter is not passing the symbol rate it has to"
                      " carry, or not rejecting what is above it.");

    afskDiagRxTaskResume();
    return ok;
}

static bool stage_demod(afsk_diag_result_t *r) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Stage 4: demodulator discrimination -------------------");
    ESP_LOGI(TAG, "  feeding steady tones through the real correlator (AFSK profiles),");
    ESP_LOGI(TAG, "  then probing the baseband filter (G3RUH, which has no tones)");

    bool ok = true;
    ok &= check_profile("Bell202", MODEM_MODEM_BELL202, r);
    ok &= check_profile("V.23", MODEM_MODEM_V23, r);
    ok &= check_profile("AFSK300", MODEM_MODEM_AFSK300, r);

    /* G3RUH last, and measured differently: it has no tones to separate. */
    ok &= check_g3ruh(r);

    r->demod_ok = ok;
    if (ok)
        ESP_LOGI(TAG, "  PASS: every AFSK profile separates its tones and G3RUH's baseband"
                      " path is open at the symbol rate");
    return ok;
}

/* ------------------------------------------------------------------ */

bool afsk_diag_run(afsk_diag_result_t *out) {
    afsk_diag_result_t r;
    memset(&r, 0, sizeof(r));

    ESP_LOGI(TAG, "===========================================================");
    ESP_LOGI(TAG, " LibAPRS modem characterisation");
    ESP_LOGI(TAG, " Measuring the analogue path, both clocks, the tones and the");
    ESP_LOGI(TAG, " demodulator BEFORE any AX.25 is attempted.");
    ESP_LOGI(TAG, "===========================================================");

    bool ok = true;
    ok &= stage_dc_sweep(&r);
    ok &= stage_clocks(&r);
    ok &= stage_tones(&r);
    ok &= stage_demod(&r);

    r.all_ok = ok;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- characterisation summary ------------------------------");
    ESP_LOGI(TAG, "  analogue path : %s  (%.2f mV/code, %d..%d mV)", r.dc_sweep_ok ? "PASS" : "FAIL", r.dc_gain_mv_per_code, r.dc_min_mv, r.dc_max_mv);
    ESP_LOGI(TAG, "  ADC clock     : %s  (%" PRIu32 " Hz)", r.adc_rate_ok ? "PASS" : "FAIL", r.adc_rate_hz);
    ESP_LOGI(TAG, "  DAC clock     : %s  (%" PRIu32 " Hz, longest freeze %" PRIu32 " us)", r.dac_rate_ok ? "PASS" : "FAIL", r.dac_rate_hz, r.dac_max_gap_us);
    ESP_LOGI(TAG, "  tone loopback : %s  (worst %.2f %%)", r.tones_ok ? "PASS" : "FAIL", r.worst_tone_err_pct);
    ESP_LOGI(TAG, "  demodulator   : %s", r.demod_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "  OVERALL       : %s", ok ? "PASS" : "FAIL");
    if (!ok)
        ESP_LOGE(TAG, "  The first FAIL above is the one to fix; later stages depend"
                      " on the earlier ones.");
    ESP_LOGI(TAG, "===========================================================");

    if (out)
        *out = r;
    return ok;
}
