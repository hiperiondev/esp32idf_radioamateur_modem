/**
 * @file afsk_diag.h
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
 *
 * @brief Hardware and DSP characterization test for the LibAPRS modem.
 *
 * This test runs before any AX.25 traffic is exchanged and measures, rather
 * than assumes, every parameter the modem depends on: the DAC-to-ADC analog
 * path, both sample clocks, the tones that actually come back around the
 * loop, and whether the demodulator can tell those tones apart.
 *
 * Stage 4 covers all four profiles, but not identically: the three AFSK ones
 * are graded on whether their correlator separates mark from space, while
 * G3RUH - which is baseband NRZ and has no tones to separate - is graded on
 * whether its receive filter passes the symbol rate it has to carry and
 * rejects what is above it.
 *
 * The goal is to fail with a specific number indicating WHICH stage is
 * broken, instead of a vague result such as "0/15 frames recovered".
 */

#ifndef AFSK_DIAG_H_
#define AFSK_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Results of the full hardware/DSP characterization sweep.
 */
typedef struct {
    /* stage 1 - analog path */
    bool dc_sweep_ok;          /**< true if the DC sweep stage passed. */
    int dc_min_mv;             /**< Minimum DC level measured, in millivolts. */
    int dc_max_mv;             /**< Maximum DC level measured, in millivolts. */
    float dc_gain_mv_per_code; /**< Measured DAC-to-ADC gain, in mV per DAC code; ideally ~12.9 mV/code for 3.3 V over 256 codes. */
    float dc_worst_lin_err_mv; /**< Worst deviation from the fitted straight line, in millivolts. */

    /* stage 2 - clocks */
    uint32_t adc_rate_hz;    /**< Measured real ADC sample rate, in Hz. */
    uint32_t dac_rate_hz;    /**< Measured real DAC sample rate, in Hz. */
    uint32_t dac_max_gap_us; /**< Longest single freeze of the modulator, in us. */
    bool adc_rate_ok;        /**< true if the ADC rate is within tolerance. */
    bool dac_rate_ok;        /**< true if the DAC rate is within tolerance. */

    /* stage 3 - tone loopback */
    bool tones_ok;            /**< true if the tone loopback stage passed. */
    float worst_tone_err_pct; /**< Worst tone frequency error observed, in percent. */

    /* stage 4 - demodulator discrimination */
    bool demod_ok; /**< true if every AFSK profile separated its mark and space tones AND G3RUH's baseband path had the right shape. */

    bool all_ok; /**< true if every stage of the sweep passed. */
} afsk_diag_result_t;

/**
 * @brief Run the full hardware/DSP characterization sweep.
 *
 * Requires libaprs_init() to have already run and a wire connecting
 * MODEM_DAC_GPIO to MODEM_ADC_GPIO. This test transmits nothing but
 * steady DC levels and pure tones; no AX.25 traffic is generated.
 *
 * @param out Optional pointer, filled with every measurement taken during
 *            the sweep; pass NULL if the details are not needed.
 * @return true if every stage of the sweep passed, false otherwise.
 */
bool afsk_diag_run(afsk_diag_result_t *out);

#endif /* AFSK_DIAG_H_ */
