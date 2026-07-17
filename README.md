# esp32idf_radioamateur_modem

**A full-duplex, software-defined AFSK/FSK packet-radio modem for the ESP32, built as a native ESP-IDF component.**

It implements the complete TNC stack needed for amateur-radio packet data — AX.25 framing, HDLC bit-stuffing, CRC-CCITT FCS, optional FX.25 Reed–Solomon forward error correction, KISS serial framing and a small APRS convenience layer — on top of a DSP core that modulates and demodulates four classic packet-radio profiles entirely in software, using only the ESP32's built-in ADC and DAC.

> Based on and ported from three well-known open packet-radio projects — please see [Credits & Provenance](#credits--provenance) below.

---

## Table of contents

- [Key features](#key-features)
- [Supported modem profiles](#supported-modem-profiles)
- [How it works — architecture](#how-it-works--architecture)
  - [Signal path overview](#signal-path-overview)
  - [Full-duplex trick: why RX and TX can run simultaneously](#full-duplex-trick-why-rx-and-tx-can-run-simultaneously)
  - [Receive (RX) pipeline](#receive-rx-pipeline)
  - [Transmit (TX) pipeline](#transmit-tx-pipeline)
  - [AX.25 / HDLC layer](#ax25--hdlc-layer)
  - [FX.25 forward error correction](#fx25-forward-error-correction)
  - [KISS TNC framing](#kiss-tnc-framing)
  - [Concurrency & real-time design](#concurrency--real-time-design)
- [Repository layout](#repository-layout)
- [Hardware requirements & wiring](#hardware-requirements--wiring)
- [Getting started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Cloning and building](#cloning-and-building)
  - [Running the built-in self-tests](#running-the-built-in-self-tests)
- [Configuration reference](#configuration-reference)
- [Public API](#public-api)
  - [Core modem API](#core-modem-api)
  - [APRS convenience layer](#aprs-convenience-layer)
  - [Usage example](#usage-example)
- [Built-in diagnostics & self-test suite](#built-in-diagnostics--self-test-suite)
  - [Hardware/DSP characterization sweep](#hardwaredsp-characterization-sweep)
  - [Loopback self-test](#loopback-self-test)
  - [Stress test](#stress-test)
- [Design notes worth knowing before you tune anything](#design-notes-worth-knowing-before-you-tune-anything)
- [Known limitations](#known-limitations)
- [License](#license)
- [Credits & Provenance](#credits--provenance)

---

## Key features

- **Four selectable modem profiles** in one binary, switchable at runtime: AFSK300, Bell 202 (standard APRS), ITU V.23, and G3RUH 9600 Bd FSK.
- **True full-duplex operation** — the receiver keeps demodulating while the transmitter is keying, achieved by driving the DAC and ADC from two independent hardware peripherals instead of sharing the I2S bus.
- **Complete AX.25 stack**: HDLC bit-level framing, NRZI encoding/decoding, bit stuffing/un-stuffing, CRC-CCITT (X.25/AX.25) FCS generation and verification, TNC2 text encode/decode.
- **FX.25 forward error correction** (Reed–Solomon, ported from VP-Digi's `lwfec`), fully implemented and enabled by default in this build (`-DENABLE_FX25`), with 11 selectable coding rates chosen automatically by frame size.
- **KISS protocol framing** for interoperability with standard TNC host software (APRS clients, `direwolf`-style tooling, etc. — though this project *is* the modem/TNC, not a host-side client).
- **A minimal but usable APRS convenience layer** (`APRS_setCallsign`, `APRS_sendLoc`, `APRS_sendMsg`, `APRS_sendPkt`, …) for building simple beaconing/messaging applications without touching AX.25 directly.
- **An extensive, self-contained diagnostic and self-test framework** (`afsk_diag.c`, `afsk_loopback_test.c`) that characterizes the analog path, both sample clocks, tone accuracy and demodulator discrimination *before* attempting real AX.25 traffic, and can run a repeatable, statistically meaningful loss-rate stress test on every profile.
- **Every tunable parameter — pins, sample rates, buffer sizes, task priorities and CPU core affinities — centralized in one configuration header** with compile-time sanity checks (`#error` guards) that catch invalid combinations before they become a mystery at runtime.
- Extremely thorough in-source documentation: every non-obvious constant is explained with the measurement or failure mode that produced it, not just "what" but "why".

## Supported modem profiles

| Profile | Baud rate | Mark / Space tones | Typical use |
|---|---|---|---|
| `MODEM_MODEM_AFSK300` | 300 Bd | 1600 / 1800 Hz | Legacy HF/VHF packet |
| `MODEM_MODEM_BELL202` | 1200 Bd | 1200 / 2200 Hz | **Standard APRS** (VHF) |
| `MODEM_MODEM_V23` | 1200 Bd | 1300 / 2100 Hz | ITU V.23 (some European packet systems) |
| `MODEM_MODEM_G3RUH` | 9600 Bd | Baseband FSK (NRZ, no tones) | High-speed packet / 9600 baud APRS |

All four are demodulated and modulated entirely in software from raw ADC/DAC samples — there is no dedicated modem IC, discriminator chip or codec involved. The 9600 Bd G3RUH profile is baseband (it has no "tones" to correlate against); the three AFSK profiles use a matched-filter tone correlator instead.

## How it works — architecture

### Signal path overview

```
                         ┌─────────────────────────────────────────────┐
   Radio audio  ──────►  │  ADC1 (continuous/DMA mode, free-running)   │
   (RX, GPIO35* )        │  76 800 Hz raw sample stream                │
                         └───────────────┬─────────────────────────────┘
                                         │ ISR just counts samples
                                         ▼
                         ┌─────────────────────────────────────────────┐
                         │  afsk_rx_task  (task context, core 0)       │
                         │   • adc_ingest(): un-swap DMA sample pairs  │
                         │   • RX ring buffer (FIFO)                   │
                         │   • AFSK_Poll(): DC tracker, AGC, decimate  │
                         │     76 800 Hz → 9 600 Hz (AFSK) or raw      │
                         │     (G3RUH)                                 │
                         └───────────────┬─────────────────────────────┘
                                         ▼
                         ┌──────────────────────────────────────────────┐
                         │  MODEM_DECODE()  (modem.c)                   │
                         │   • boxcar matched-filter correlator +       │
                         │     quadrature NCO (AFSK profiles), OR       │
                         │     baseband slicer (G3RUH)                  │
                         │   • DPLL bit/symbol synchronizer             │
                         │   • DCD (data carrier detect) state machine  │
                         └───────────────┬──────────────────────────────┘
                                         ▼  one bit at a time
                         ┌─────────────────────────────────────────────┐
                         │  Ax25BitParse()  (ax25.c)                   │
                         │   • HDLC flag/bit-stuffing state machine    │
                         │   • CRC-CCITT FCS check                     │
                         │   • optional FX.25 Reed–Solomon decode      │
                         └───────────────┬─────────────────────────────┘
                                         ▼
                         ┌─────────────────────────────────────────────┐
                         │  frame_q  (FreeRTOS queue) → modem_rx_cb_t  │
                         │  application callback (raw AX.25 frame)     │
                         └─────────────────────────────────────────────┘

   Application           ┌─────────────────────────────────────────────┐
   modem_send_raw() ───► │  Ax25WriteTxFrame() / Ax25TransmitBuffer()  │
                         │  HDLC framing, bit stuffing, FCS append,    │
                         │  optional FX.25 RS encode                   │
                         └──────────────┬──────────────────────────────┘
                                        ▼  Ax25GetTxBit()
                         ┌─────────────────────────────────────────────┐
                         │  MODEM_BAUDRATE_TIMER_HANDLER() (modem.c)   │
                         │   • DDS tone synthesis (512-entry sine LUT) │
                         │     or G3RUH NRZI baseband levels           │
                         └───────────────┬─────────────────────────────┘
                                         ▼  called from GPTimer ISR
                         ┌─────────────────────────────────────────────┐
                         │  DAC one-shot write @ 38 400 Hz (core 1*)   │
                         │  (GPIO25*, PTT / LEDs as configured)        │
                         └─────────────────────────────────────────────┘

  * default pin/core assignments — all configurable, see esp32idf_radioamateur_modem_config.h
```

### Full-duplex trick: why RX and TX can run simultaneously

On the ESP32, the *continuous* (DMA) ADC driver and the *continuous* (DMA) DAC driver both need exclusive use of the I2S0 peripheral — they cannot run at once. This project sidesteps that limitation entirely: the DAC is driven in **one-shot mode**, one sample at a time, from a dedicated **GPTimer hardware timer interrupt** running at the configured DAC sample rate (38 400 Hz by default), while the ADC runs independently in continuous/DMA mode on I2S0. Since neither peripheral touches the other, both directions run simultaneously — which is what makes a literal wire from the DAC pin to the ADC pin (GPIO25 → GPIO35 by default) a valid, working full-duplex loopback test bench, and what makes true full-duplex operation on real radio hardware possible.

### Receive (RX) pipeline

1. **ADC ingestion.** ADC1 runs in continuous/DMA mode at `MODEM_ADC_SAMPLERATE` (76 800 Hz by default), completely free-running. The conversion-done ISR does the absolute minimum — it only counts samples — because ESP-IDF's own DMA ring-buffer hand-off (`xRingbufferSendFromISR`) already takes a critical section long enough to *freeze the DAC ISR for the length of a whole conversion frame* if any application code adds work on top of it (this is documented at length in `afsk.c`, see [Design notes](#design-notes-worth-knowing-before-you-tune-anything)). The actual sample copy and any pair-un-swapping happens in `afsk_rx_task`, at task priority, off the ISR.
2. **RX FIFO.** Samples move into a power-of-two ring buffer between `adc_ingest()` and `AFSK_Poll()`, sized to comfortably absorb `MODEM_BLOCK_SIZE` (20 ms) worth of samples.
3. **Conditioning.** `AFSK_Poll()` drains whole blocks from the FIFO and applies DC offset tracking, AGC (automatic gain control), amplitude clamping, and — for the three AFSK profiles — decimation from the raw ADC rate down to the fixed 9600 Hz demodulator rate (`MODEM_RESAMPLE_RATIO`). G3RUH is fed the **undecimated** stream, since its timing recovery needs the extra time resolution (see below).
4. **Demodulation (`MODEM_DECODE()` in `modem.c`).**
   - AFSK300 / Bell202 / V.23 use a **boxcar matched-filter correlator driven by a quadrature NCO (numerically-controlled oscillator)** tuned to each profile's mark/space pair, followed by a low-pass filter; the sign of the correlator output is the demodulated symbol.
   - G3RUH is baseband NRZ FSK: no tones to correlate, so it goes through a receive filter shaped to pass the 9600 Bd symbol rate and reject everything above it, followed by a slicer.
   - A **DPLL (digital phase-locked loop)** recovers bit/symbol timing from the demodulated stream (see the `DCD1200_*` / `DCD9600_*` / `DCD300_*` tuning tables in `modem.c` for the per-profile DCD pulse-counter parameters).
   - A **DCD (data carrier detect)** state machine tracks whether a valid signal is currently present.
   - For 1200 Bd profiles, **two demodulators run in parallel**, tuned slightly differently, to improve decode probability (`MODEM_MAX_DEMODULATOR_COUNT = 2`).
5. **HDLC / AX.25 (`Ax25BitParse()` in `ax25.c`).** Each demodulated bit is fed into the AX.25 HDLC receive state machine, which finds flag bytes (`0x7E`), un-stuffs bits, assembles bytes, verifies the CRC-CCITT FCS, and — if the frame arrived as FX.25 — runs the Reed–Solomon FEC decoder first.
6. **Delivery.** A complete, FCS-valid frame is handed to the application through a FreeRTOS queue and the `modem_rx_cb_t` callback installed via `modem_set_rx_callback()`. The callback receives the raw AX.25 bytes plus signal-quality metadata (peak/valley/level, RMS millivolts, FX.25 correction count).

### Transmit (TX) pipeline

1. **Frame submission.** The application calls `modem_send_raw()` (raw AX.25 bytes), or the convenience wrappers `modem_send_tnc2()` / `modem_build_frame_tnc2()` (TNC2 monitor-string format), or one of the `APRS_send*()` helpers.
2. **Framing (`Ax25WriteTxFrame()`).** HDLC flags, bit stuffing and the CRC-CCITT FCS are added automatically — callers never construct these themselves. If FX.25 TX is enabled, the frame is instead wrapped in a Reed–Solomon FX.25 block chosen by `Fx25GetModeForSize()`.
3. **Channel access.** In half-duplex mode, `Ax25TransmitCheck()` implements CSMA (carrier-sense, with a configurable quiet/slot time) before keying up; in full-duplex mode the modem keys immediately, which is required for the hardware loopback test (the node always hears its own carrier and would otherwise never see a clear channel).
4. **Modulation.** `MODEM_BAUDRATE_TIMER_HANDLER()` is called from the GPTimer ISR at the DAC sample rate and produces the next 8-bit DAC sample: for the AFSK profiles this walks a 32-bit phase accumulator through a 512-entry sine lookup table (a DDS — direct digital synthesizer) selecting mark or space frequency per bit; for G3RUH it outputs NRZI baseband levels directly.
5. **Preamble/postamble & PTT.** TXDelay (preamble) and slot-time are configurable in milliseconds; PTT and TX/RX status LEDs are driven through optional GPIOs (`MODEM_PTT_GPIO`, `MODEM_LED_TX_GPIO`, `MODEM_LED_RX_GPIO`), all of which can be disabled (`-1`).

### AX.25 / HDLC layer

`ax25.c` / `ax25.h` implement:

- The bit-level HDLC receive state machine (`hdlc_t`, `Ax25BitParse()`), including flag detection, bit un-stuffing and byte assembly.
- CRC-CCITT (X.25 polynomial) FCS computation and verification (`crc_ccit.c`), with a precomputed 256-entry lookup table.
- A structured decoded-message representation (`ax25_msg_t`) with source/destination callsigns, up to 8 digipeater path entries with "repeated" flags, control/PID fields and payload.
- TNC2 text encode/decode (`ax25_encode()`, `hdlcFrame()`, `modem_format_tnc2()`) — the human-readable `SRC>DST,PATH:payload` format used by APRS monitor tools.
- `Ax25GetOnAirSize()` — computes the *actual* number of bytes that will be clocked onto the air for a given frame under the current configuration, which is **not** the same as the frame's logical length once bit stuffing and/or FX.25's fixed-size RS blocks are taken into account; anything sizing a timeout or duty-cycle estimate needs this number, not `sizeof(frame)`.

### FX.25 forward error correction

FX.25 wraps a plain AX.25 frame in a Reed–Solomon error-correcting block, preceded by a 64-bit correlation tag that identifies both the presence of FX.25 framing and which of 11 predefined `(K, T)` payload/parity sizes was used (`Fx25ModeList`). The Reed–Solomon codec itself lives in `lwfec/` (`rs.c`, `gf.c` — Galois-field arithmetic and the RS encoder/decoder), ported from VP-Digi's `lwfec` library.

- `Fx25GetModeForSize()` automatically selects the smallest block that can carry a given frame.
- `Fx25Encode()` / `Fx25Decode()` perform the RS encode/decode, with `Fx25Decode()` reporting how many byte errors were actually corrected.
- Controlled by `fx25_mode` in `modem_config_t`: `0` = disabled, `1` = RX only (accept FX.25 frames but never send them), `2` = RX + TX.
- Gated at compile time by `-DENABLE_FX25` (the component's `CMakeLists.txt` in this repository **defines this by default**, so FX.25 is compiled in and ready to use out of the box).

### KISS TNC framing

`kiss.c` / `kiss.h` implement the standard KISS protocol (FEND/FESC byte stuffing, command bytes for TXDelay/persistence/slot-time/TXTail/full-duplex/hardware parameters) for wrapping and unwrapping raw AX.25 frames over a serial byte stream — the same framing used by essentially every TNC and packet-radio host application. `kiss_wrap()` encodes an outgoing frame, `kiss_serial()`/`kiss_parse()` decode an incoming byte stream. (Wiring this up to an actual UART/USB transport is left to the integrating application — see `main/app_main.c` for how the component is used standalone without a KISS host.)

### Concurrency & real-time design

This is the part of the project the in-source comments spend the most time on, because it is where most of the debugging effort went:

- **Two independent hardware interrupts drive the whole system**: the ADC DMA "conversion done" interrupt (I2S0) and the DAC sample-clock GPTimer interrupt. They are deliberately **pinned to different CPU cores** (`MODEM_ADC_ISR_CORE = 0`, `MODEM_DAC_TIMER_CORE = 1` by default) because `portENTER_CRITICAL_ISR()` only masks interrupts up to level 3 on the *local* core — if both interrupts lived on the same core, a long critical section inside the ADC driver's ring-buffer hand-off would mask the DAC's level-3 timer alarm and freeze the modulator mid-symbol. Putting them on separate cores means the ADC ISR's critical section simply doesn't affect the DAC core at all. This is enforced with a compile-time `#error` check, not just a comment.
- **The RX DSP task (`afsk_rx_task`) is pinned to the same core as the ADC ISR** (`MODEM_RX_TASK_CORE = MODEM_ADC_ISR_CORE = 0`), so the ring buffer's spinlock is only ever contended locally, and — more importantly — both stay off the core carrying the DAC's real-time sample clock.
- **ADC DMA conversion frame size is deliberately small** (`MODEM_ADC_CONV_FRAME = 128` samples, i.e. ~1.7 ms at 76 800 Hz) rather than a larger, more "efficient" value, specifically to bound the length of the critical section inside the ESP-IDF ADC driver's own ISR hand-off — see the extensive rationale in `esp32idf_radioamateur_modem_config.h`, which measured that an 11 µs modulator freeze (10 % of a 9600 Bd symbol) is fatal to G3RUH decode while a 2 µs freeze is not.
- **The receive-side "8 samples per symbol" requirement for G3RUH is why the ADC runs at 76 800 Hz (8×9600) rather than 38 400 Hz (4×9600)** even though the DAC stays at 38 400 Hz — this asymmetry, and the DPLL timing-resolution math behind it, is documented in detail in the configuration header.
- **All FreeRTOS task priorities, stack sizes and core affinities are configuration constants**, not hardcoded, so they can be tuned per application/board without touching the DSP code.

## Repository layout

```
esp32idf_radioamateur_modem/
├── CMakeLists.txt                      # top-level ESP-IDF project file
├── LICENSE                             # GNU GPLv3
├── README.md
├── sdkconfig                           # generated ESP-IDF config (ESP32, IDF 5.5.2)
├── main/                               # application: demo + diagnostics + self-test
│   ├── app_main.c                      # app_main(): init, run diag/loopback/stress, then beacon loop
│   ├── afsk_diag.c / afsk_diag.h       # hardware/DSP characterization sweep (4 stages)
│   └── afsk_loopback_test.c / .h       # full-duplex wire-loop AX.25 self-test + stress test
└── components/
    └── esp32idf_radioamateur_modem/    # the modem, as a reusable ESP-IDF component
        ├── CMakeLists.txt              # component build rules (defines -DENABLE_FX25)
        ├── esp32idf_radioamateur_modem.h  # PUBLIC API — start here when integrating
        ├── include/
        │   ├── esp32idf_radioamateur_modem_config.h  # ALL hardware/tuning constants
        │   ├── afsk.h                  # HAL: ADC/DAC/GPTimer, full-duplex driver
        │   ├── modem.h                 # DSP core: modulator/demodulator
        │   ├── ax25.h                  # AX.25 frame handling + HDLC state machine
        │   ├── fx25.h                  # FX.25 FEC framing
        │   ├── kiss.h                  # KISS TNC serial framing
        │   └── crc_ccit.h              # CRC-CCITT (FCS) computation
        ├── src/                        # implementation of every header above
        └── lwfec/                      # Reed–Solomon codec (gf.c/.h, rs.c/.h) for FX.25
```

## Hardware requirements & wiring

| Signal | Default GPIO | Notes |
|---|---|---|
| Audio **out** (DAC, TX) | **GPIO25** (`DAC_CHAN_0`) | ESP32 has exactly two DAC-capable pins: GPIO25 or **GPIO26** (`DAC_CHAN_1`). No other pin is possible — the ESP32 DAC is not routable through the GPIO matrix. |
| Audio **in** (ADC1, RX) | **GPIO33** (`ADC1_CH5`) | Must be one of the eight ADC1 pins: GPIO32–39. ADC2 pins are **not** usable (ADC2 is shared with the Wi-Fi radio and unavailable to the continuous/DMA driver). |
| PTT (push-to-talk) | disabled (`-1`) by default | Any output-capable GPIO; active-high or active-low is configurable. |
| TX status LED | disabled (`-1`) by default | Any output-capable GPIO. |
| RX/DCD status LED | disabled (`-1`) by default | Any output-capable GPIO. |

**For the built-in self-test / hardware loopback**, simply wire the DAC output pin directly to the ADC input pin (default: **GPIO25 → GPIO33/35**, board-dependent — check the exact default in your copy of `esp32idf_radioamateur_modem_config.h`). No radio, no external circuitry, no level-shifting is required: the DAC amplitude is intentionally kept to 60% of full-scale and the ADC is set to 12 dB attenuation specifically so this direct wire loopback works without clipping.

For real on-air use, the DAC/ADC pins connect to your transceiver's audio-in/audio-out (packet/data) jacks through the usual isolation and level-matching circuitry (transformer or capacitive coupling, attenuation/gain as needed) — this is standard TNC/soundcard-modem interfacing practice and is **not** provided by this project.

Target chip: **ESP32** (original, dual-core Xtensa). The DAC-based TX path specifically requires the ESP32's onboard 8-bit DACs; ESP32-S2 has DACs on different pins (GPIO17/18) and would need pin reconfiguration, while ESP32-S3/C3/C6/H2 have **no** DAC at all and would need an external DAC, PWM, or sigma-delta output stage before this component could target them.

## Getting started

### Prerequisites

- **ESP-IDF v5.5.x** (the checked-in `sdkconfig`/`sdkconfig.defaults` were generated against ESP-IDF 5.5.2; other 5.x releases will likely work but are untested).
- A physical or wired-loopback **ESP32** target board.
- The standard ESP-IDF toolchain (`idf.py`) installed and `$IDF_PATH` sourced.

### Cloning and building

```bash
git clone https://github.com/hiperiondev/esp32idf_radioamateur_modem.git
cd esp32idf_radioamateur_modem
idf.py set-target esp32
idf.py menuconfig      # optional: adjust any ESP-IDF-level settings
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

The component itself needs no `idf_component.yml` registry dependency — it's included directly via `components/esp32idf_radioamateur_modem` and picked up automatically by ESP-IDF's component discovery, as declared in `main/CMakeLists.txt`'s `REQUIRES esp32idf_radioamateur_modem`.

### Running the built-in self-tests

Out of the box, `main/app_main.c` is configured to:

1. Initialize the modem (`modem_init()`) in Bell 202 / full-duplex mode.
2. Run the **hardware/DSP characterization sweep** (`afsk_diag_run()`) — gated by `#define RUN_DIAG 1`.
3. Run the **full-duplex loopback AX.25 self-test** across all four profiles (`afsk_loopback_test_run()`) — gated by `#define RUN_LOOPBACK_TEST 1`.
4. Optionally run a **100-iteration stress test per profile** (`afsk_loopback_stress_test_run()`) to get a real loss-rate estimate rather than a single small sample — gated by `#define RUN_G3RUH_STRESS_TEST 1` and `#define STRESS_ITERATIONS 100` (this adds a few minutes to boot time; set to `0`/lower the iteration count to skip or shorten it).
5. Switch back to the normal operating profile, install an RX callback that logs every decoded frame, configure a sample APRS station identity, and idle in a 30-second loop (the periodic `APRS_sendLoc()` beacon call is present but commented out).

**All of this requires nothing but a wire between the DAC and ADC pins** — no radio hardware needed to verify the whole stack end to end.

## Configuration reference

Every tunable lives in `components/esp32idf_radioamateur_modem/include/esp32idf_radioamateur_modem_config.h` and can be overridden from your own build (e.g. via `target_compile_definitions` or `-D` in your top-level `CMakeLists.txt`) without editing the component:

| Define | Default | Purpose |
|---|---|---|
| `MODEM_DAC_GPIO` | `25` | TX audio output pin (25 or 26 only). |
| `MODEM_ADC_GPIO` | `33` | RX audio input pin (32–39, ADC1 only). |
| `MODEM_PTT_GPIO` | `-1` (disabled) | PTT output pin. |
| `MODEM_PTT_ACTIVE_HIGH` | `1` | PTT polarity. |
| `MODEM_LED_TX_GPIO` / `MODEM_LED_RX_GPIO` | `-1` (disabled) | Status LED pins. |
| `MODEM_DAC_SAMPLERATE` | `38400` | TX sample rate, Hz (must stay an exact multiple of every supported baud rate). |
| `MODEM_ADC_SAMPLERATE` | `76800` | RX sample rate, Hz (must be an integer multiple of 9600; 8× is required for G3RUH's DPLL timing resolution — see in-header rationale). |
| `MODEM_ADC_RATE_NUM` / `MODEM_ADC_RATE_DEN` | `1` / `1` | Correction factor for the ADC's real vs. requested sample rate. |
| `MODEM_DAC_AMPLITUDE_PCT` | `60` | DAC output swing, % of full scale (kept below 100% to avoid clipping the ADC in direct-wire loopback). |
| `MODEM_ADC_ATTEN` | `ADC_ATTEN_DB_12` | ADC input attenuation. |
| `MODEM_RX_FIFO_SIZE` | (power of two) | RX ring buffer size, in samples; must hold ≥2 DSP blocks. |
| `MODEM_ADC_CONV_FRAME` | `128` | ADC DMA conversion frame size, in samples — deliberately small to bound ISR critical-section length (see [Concurrency & real-time design](#concurrency--real-time-design)). |
| `MODEM_ADC_POOL_FRAMES` | `32` | Depth of the ADC driver's internal DMA pool. |
| `MODEM_RX_TASK_PRIO` | `10` | FreeRTOS priority of the RX DSP task. |
| `MODEM_RX_TASK_STACK` | `4096` | RX task stack size, bytes. |
| `MODEM_RX_TASK_CORE` | `0` | Core affinity of the RX DSP task (must match `MODEM_ADC_ISR_CORE`). |
| `MODEM_ADC_ISR_CORE` | `0` | Core the ADC DMA interrupt is allocated on. |
| `MODEM_DAC_TIMER_CORE` | `1` | Core the DAC sample-clock interrupt is allocated on (**must differ** from `MODEM_ADC_ISR_CORE`, enforced at compile time). |
| `MODEM_DAC_TIMER_INTR_PRIO` | `3` | DAC timer interrupt priority (1–3). |

Compile-time `#error` guards prevent building with an inconsistent combination (wrong pins, cores sharing the DAC/ADC interrupts, non-power-of-two FIFO too small for a block, non-multiple-of-9600 ADC rate, etc.), so invalid configurations fail at build time rather than misbehaving on hardware.

## Public API

All public symbols are declared in `esp32idf_radioamateur_modem.h`, at the component root — this is the only header application code needs to include.

### Core modem API

```c
esp_err_t modem_init(const modem_config_t *cfg);
void      modem_deinit(void);
void      modem_set_modem(const modem_config_t *cfg);
void      modem_set_rx_callback(modem_rx_cb_t cb, void *ctx);
esp_err_t modem_send_raw(const uint8_t *frame, uint16_t len);
int       modem_build_frame_tnc2(const char *tnc2, uint8_t *out, size_t out_len);
esp_err_t modem_send_tnc2(const char *tnc2);
void      modem_format_tnc2(const ax25_msg_t *msg, char *out, size_t out_len);
bool      modem_tx_busy(void);
uint32_t  modem_measure_adc_rate(uint32_t ms);
```

`modem_config_t` (with `MODEM_DEFAULT_CONFIG()` as a sane starting point):

```c
typedef struct {
    modem_mode_t modem;     // AFSK300 / BELL202 / V23 / G3RUH
    bool     flat_audio;    // true = flat/discriminator input, false = de-emphasized
    bool     full_duplex;   // true = key up immediately, no CSMA/DCD wait
    bool     allow_non_aprs;// true = accept frames with non-standard Control/PID
    uint16_t preamble_ms;   // TXDelay
    uint16_t slot_time_ms;  // CSMA slot time (ignored in full duplex)
    uint8_t  fx25_mode;     // 0 = off, 1 = RX only, 2 = RX+TX
} modem_config_t;
```

### APRS convenience layer

A thin layer on top of the core API for building simple beaconing/messaging stations without hand-building AX.25 frames:

```c
void APRS_setCallsign(const char *call, int ssid);
void APRS_setDestination(const char *call, int ssid);
void APRS_setPath1(const char *call, int ssid);
void APRS_setPath2(const char *call, int ssid);
void APRS_setMessageDestination(const char *call, int ssid);
void APRS_setLat(const char *lat);
void APRS_setLon(const char *lon);
void APRS_useAlternateSymbolTable(bool use);
void APRS_setSymbol(char sym);
void APRS_setPower(int s);
void APRS_setHeight(int s);
void APRS_setGain(int s);
void APRS_setDirectivity(int s);
esp_err_t APRS_sendLoc(const char *comment);
esp_err_t APRS_sendMsg(const char *text);
esp_err_t APRS_sendPkt(const char *info);
void APRS_printSettings(void);
```

### Usage example

```c
#include "esp32idf_radioamateur_modem.h"

static void on_rx_frame(const modem_rx_frame_t *f, void *ctx) {
    ax25_msg_t msg;
    char tnc2[AX25_FRAME_MAX_SIZE];

    ax25_decode((uint8_t *)f->frame, f->len, f->mVrms, &msg);
    modem_format_tnc2(&msg, tnc2, sizeof(tnc2));

    printf("RX [%u%% %umV] %s\n", f->level, f->mVrms, tnc2);
}

void app_main(void) {
    modem_config_t cfg = MODEM_DEFAULT_CONFIG();
    cfg.modem = MODEM_MODEM_BELL202;   // standard 1200 Bd APRS
    cfg.full_duplex = true;

    ESP_ERROR_CHECK(modem_init(&cfg));
    modem_set_rx_callback(on_rx_frame, NULL);

    APRS_setCallsign("NOCALL", 1);
    APRS_setDestination("APE32I", 0);
    APRS_setPath1("WIDE1", 1);
    APRS_setPath2("WIDE2", 2);
    APRS_setLat("4903.50N");
    APRS_setLon("07201.75W");
    APRS_setSymbol('n');

    for (;;) {
        APRS_sendLoc("ESP32 packet station");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
```

## Built-in diagnostics & self-test suite

This is one of the more distinctive parts of the project: instead of a single pass/fail "does AX.25 work" test, the suite isolates *which layer* is at fault when something doesn't decode.

### Hardware/DSP characterization sweep

`afsk_diag_run()` (`main/afsk_diag.c`) transmits **no AX.25 traffic at all** — only DC levels and pure tones — and measures four things independently, in order, so a failure points at a specific stage instead of a vague "0/N frames" result:

1. **Analog path (DC sweep).** Sweeps DAC codes and measures the resulting ADC readings to characterize gain (mV/code) and linearity of the DAC → ADC path.
2. **Clocks.** Measures the *real* ADC sample rate and DAC alarm rate (both are quantized by integer timer-tick rounding and are not necessarily exactly the configured nominal values), and measures the longest single freeze ("gap") of the DAC timer ISR — because, as extensively documented in the source, a *uniform* percentage of missed DAC alarms is harmless (the DPLL tracks it) while the same percentage delivered as one contiguous blackout per DSP block is fatal, and how fatal depends on how large the blackout is *relative to one symbol period* at the fastest supported baud rate (G3RUH, 9600 Bd → 104 µs/symbol).
3. **Tone loopback.** Transmits each profile's mark/space tones through the DAC → ADC loop and verifies (via a Goertzel power scan) that they come back at the expected frequency within tolerance.
4. **Demodulator discrimination.** For the three AFSK profiles, verifies the correlator actually separates mark from space; for G3RUH (which has no tones), verifies the receive filter has the right passband shape for a baseband NRZ symbol stream.

### Loopback self-test

`afsk_loopback_test_run()` (`main/afsk_loopback_test.c`) requires the same DAC→ADC wire loop and pushes **real AX.25 UI frames** through the whole stack — encode, modulate, demodulate, HDLC-decode, byte-compare — for every profile, in both plain-AX.25 and (when built with `-DENABLE_FX25`) FX.25 modes. For FX.25 frames it doesn't just check that the payload matched; it also confirms the frame's `corrected` field is not `AX25_NOT_FX25`, i.e. that it genuinely traveled as an FX.25 block rather than silently falling back to plain AX.25.

### Stress test

`afsk_loopback_stress_test_run()` repeats the same encode → transmit → demodulate → compare cycle many times (`STRESS_ITERATIONS`, 100 by default) per profile, firing both a short packet (`AFSK_LOOPBACK_SHORT_PACKET`) and a long telemetry-style packet (`AFSK_LOOPBACK_TELEMETRY_PACKET`) on every round, because the two stress different failure modes — a short frame can be swallowed inside the DPLL's settling window, while a long frame accumulates clock drift over its duration. This turns "does it work" into an actual, reproducible **loss-rate percentage** per profile, which is what originally caught a subtle, low-single-digit-percent G3RUH frame-loss issue that a single 5-frame test had missed.

## Design notes worth knowing before you tune anything

The in-source comments in this project are unusually detailed and record *why* specific constants have the values they do, including failure modes that were diagnosed and fixed along the way. A few worth knowing before changing anything:

- **Don't move work into the ADC ISR.** It exists purely to count samples; the real copy happens in `afsk_rx_task` at task priority. Adding any per-sample work back into the ISR context risks reintroducing multi-hundred-microsecond critical sections that freeze the DAC modulator mid-symbol (documented in detail in `afsk.c`).
- **`MODEM_ADC_CONV_FRAME` trades ISR entry frequency for critical-section length.** Smaller frames mean more frequent, shorter ISR calls, and shorter critical sections in ESP-IDF's own ADC driver hand-off — which is what actually protects the DAC's timing at 9600 Bd.
- **`MODEM_DAC_TIMER_CORE` must differ from `MODEM_ADC_ISR_CORE`.** This isn't a performance tweak; `portENTER_CRITICAL_ISR()` only masks up to interrupt level 3 on the *local* core, so if both interrupts shared a core, the ADC driver's critical section would mask the DAC's level-3 alarm outright.
- **G3RUH needs 8 ADC samples per symbol, not 4.** The DPLL's sampling instant is quantized to one ADC period; at 4 samples/symbol the sampling-decision window (a 3-sample majority vote spanning 75% of a symbol) always reaches into a transition somewhere as the independent ADC/DAC clocks drift relative to each other. Doubling the ADC rate to 76 800 Hz (8×9600) removes this failure mode entirely, at the cost of running the RX DSP over twice as many samples and needing a longer decimation FIR for AFSK profiles (`MODEM_RESAMPLE_RATIO = 8`).
- **`Ax25GetOnAirSize()` is not the same as frame length**, once bit stuffing and/or FX.25's fixed-block RS framing are accounted for; anything computing a timeout, duty cycle, or channel-occupancy estimate should call it rather than using `sizeof()`/frame length directly.

## Known limitations

- Targets the **original ESP32 only** (dual-core Xtensa, onboard 8-bit DAC on GPIO25/26). Porting to ESP32-S2 needs pin reconfiguration (different DAC pins); porting to S3/C3/C6/H2 needs an entirely different TX output stage, since those chips have no onboard DAC.
- ADC2 pins cannot be used for the RX input — only the 8 ADC1 pins (GPIO32–39) are supported by the continuous/DMA driver used here.
- The KISS layer implements framing (`kiss_wrap`/`kiss_serial`/`kiss_parse`) but this repository does not wire it to a concrete transport (UART/USB-CDC) — that integration is left to the consuming application.
- No persistent (NVS) configuration storage is included; `main/app_main.c` configures everything in code at boot.
- FX.25 requires the Reed–Solomon codec present under `lwfec/`, which **is** included and compiled in this repository (`-DENABLE_FX25` is set in the component's `CMakeLists.txt`), but the header documents that this was not always the case in the archive it was ported from — double-check `lwfec/rs.c`/`gf.c` are present if you copy just parts of this component elsewhere.

## License

Licensed under the **GNU General Public License v3.0** — see [`LICENSE`](LICENSE) for the full text.

## Credits & Provenance

Authored by **Emiliano Augusto Gonzalez** (`lu3vea [at] gmail [dot] com`).

This project is explicitly based on, and ports logic from, three established open-source packet-radio projects:

- **[VP-Digi](https://github.com/sq8vps/vp-digi)** — AX.25/HDLC state machine, DCD/DPLL tuning tables, and the `lwfec` Reed–Solomon codec used for FX.25.
- **[ESP32APRS_Audio](https://github.com/nakhonthai/ESP32APRS_Audio)** — ESP32-specific audio front-end concepts.
- **[LibAPRS](https://github.com/markqvist/LibAPRS)** — the APRS convenience-layer API surface (`APRS_setCallsign`, `APRS_sendLoc`, etc.).

Please refer to those projects and contact their respective authors for further information on the original implementations this component builds upon.
