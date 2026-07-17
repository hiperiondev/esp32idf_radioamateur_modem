/**
 * @file afsk_loopback_test.h
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
 * @brief Full-duplex, wire-loop self-test: GPIO25 (DAC output) connected to
 *        GPIO35 (ADC input).
 */

#ifndef AFSK_LOOPBACK_TEST_H_
#define AFSK_LOOPBACK_TEST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp32idf_radioamateur_modem.h"

/**
 * @brief Aggregate results of the loopback self-test across all tested
 *        modem profiles.
 */
typedef struct {
    uint32_t sent;         /**< Total number of frames transmitted, plain AX.25 + FX.25 combined. */
    uint32_t received;     /**< Total number of frames successfully demodulated and decoded, both kinds combined. */
    uint32_t matched;      /**< Number of received frames that matched their transmitted counterpart byte for byte, both kinds combined. */
    uint32_t adc_rate_hz;  /**< Measured real ADC sample rate during the test, in Hz. */
    uint32_t fx25_sent;    /**< Of `sent`, how many were transmitted with FX.25 FEC framing (0 unless built with -DENABLE_FX25). */
    uint32_t fx25_matched; /**< Of `matched`, how many were sent as FX.25 AND verified as having actually arrived as an FX.25 block. */
} afsk_loopback_result_t;

/**
 * @brief Results of a single-profile stress run. Every round sends and
 *        receives both a short and a long packet, so the totals below are
 *        each broken out by packet size as well as combined.
 */
typedef struct {
    uint32_t attempts;       /**< Total frames actually transmitted, short + long (excludes encode/queue failures). */
    uint32_t matched;        /**< Of those, how many looped back byte-exact, short + long. */
    uint32_t short_attempts; /**< Short-packet frames actually transmitted. */
    uint32_t short_matched;  /**< Of those, how many looped back byte-exact. */
    uint32_t long_attempts;  /**< Long-packet frames actually transmitted. */
    uint32_t long_matched;   /**< Of those, how many looped back byte-exact. */
    uint32_t adc_rate_hz;    /**< Measured real ADC sample rate at the start of the run. */
} afsk_stress_result_t;

/**
 * @brief The telemetry packet from the 5-packet test set that showed the
 *        4/5 G3RUH result: the longest frame, ending in a long run of
 *        identical digit characters. Exposed here so callers can hammer this
 *        exact packet without duplicating the string. Used as the "long
 *        packet" half of every stress round.
 */
#define AFSK_LOOPBACK_TELEMETRY_PACKET "LU1ABC-7>APZ001,RELAY*,WIDE2-1:T#123,045,067,089,012,034,00000000"

/**
 * @brief A deliberately tiny APRS status packet: worst case for the *other*
 *        end of the size range - one flag, a handful of bytes of payload,
 *        another flag. Short frames stress bit-stuffing/FCS-timing edge
 *        cases the long telemetry packet above never hits (e.g. the whole
 *        frame fitting inside a DPLL settling window). Used as the
 *        "short packet" half of every stress round.
 */
#define AFSK_LOOPBACK_SHORT_PACKET "NOCALL-1>APE32I:>hi"

/**
 * @brief Run the full-duplex loopback self-test.
 *
 * Requires LibAPRS to already be initialized (via libaprs_init()) and a
 * wire connecting MODEM_DAC_GPIO to MODEM_ADC_GPIO. The test transmits
 * real AX.25 UI frames through the DAC, demodulates them back through the
 * ADC, and compares the received bytes against the transmitted ones.
 *
 * Every profile (Bell202, V.23, AFSK300, G3RUH) is run twice: once with
 * plain AX.25 framing, and - only when the component was built with
 * -DENABLE_FX25 - a second time with FX.25 FEC framing (RX+TX). The FX.25
 * pass isn't just "does it decode": each matched frame's
 * ::libaprs_rx_frame_t.corrected field is checked against ::AX25_NOT_FX25
 * to confirm it actually travelled as an FX.25 block rather than silently
 * falling back to plain AX.25, and that check is part of the pass/fail
 * result. Without -DENABLE_FX25 (the default; see fx25.h for why) only the
 * plain AX.25 pass runs, matching prior behaviour.
 *
 * @note The test temporarily installs its own RX callback and does not
 *       restore any previous one; reinstall your own callback afterwards
 *       if one was set before calling this function.
 *
 * @param result Optional pointer, filled with the aggregate totals across
 *               every tested modem profile and mode; pass NULL if the
 *               details are not needed.
 * @return true if every tested profile passed in both modes, false otherwise.
 */
bool afsk_loopback_test_run(afsk_loopback_result_t *result);

/**
 * @brief Repeat-fire a short and a long TNC2 packet back-to-back on ONE
 *        profile.
 *
 * afsk_loopback_test_run() gives a single 5-frame sample per profile - enough
 * to notice a systemic break, not enough to tell a rare, jitter-driven frame
 * loss (a few percent) apart from a one-off fluke. This runs the same
 * encode -> transmit -> demodulate -> compare path @ref afsk_loopback_test_run
 * uses, but many times over, so the reported ratio is an actual loss-rate
 * estimate rather than a single 4/5 or 5/5 sample.
 *
 * Every round sends and receives BOTH packets - short_packet then
 * long_packet - because the two stress different failure modes: a short
 * frame can come and go inside the DPLL's settling window, while a long one
 * accumulates drift and is where a marginal clock relationship shows up.
 * A profile that only gets hammered with one size can look fine while still
 * being marginal at the other.
 *
 * Safe to call standalone (it creates its own RX callback/semaphore for the
 * duration) or immediately after afsk_loopback_test_run() in the same boot.
 *
 * @param modem        Which profile to test (e.g. MODEM_MODEM_G3RUH). Any
 *                      of the four modem modes is valid here, not just
 *                      G3RUH.
 * @param short_packet TNC2-format packet sent first every round, e.g.
 *                      ::AFSK_LOOPBACK_SHORT_PACKET.
 * @param long_packet  TNC2-format packet sent second every round, e.g.
 *                      ::AFSK_LOOPBACK_TELEMETRY_PACKET.
 * @param iterations   How many rounds to run. 50-100 is enough to resolve a
 *                      loss rate down to a couple of percent.
 * @param fx25_mode    ::libaprs_config_t.fx25_mode to test with: 0 for plain
 *                      AX.25, 2 for FX.25 RX+TX. Pass 2 only in builds
 *                      compiled with -DENABLE_FX25; each matched frame's
 *                      FX.25 provenance is checked the same way
 *                      afsk_loopback_test_run() checks it, and a frame that
 *                      matched but did not actually travel as FX.25 fails
 *                      the run.
 * @param result       Optional, filled with the attempt/match counts, both
 *                      combined and broken out by packet size.
 * @return true if every attempt, short and long, matched byte-exact and (when
 *         fx25_mode != 0) was verified as having travelled as FX.25.
 */
bool afsk_loopback_stress_test_run(modem_mode_t modem, const char *short_packet, const char *long_packet, uint32_t iterations, uint8_t fx25_mode,
                                   afsk_stress_result_t *result);

#endif /* AFSK_LOOPBACK_TEST_H_ */
