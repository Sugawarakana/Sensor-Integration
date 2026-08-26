# Multi-Gas Sensor Integration PCB — Battery Off-Gas Monitor

A compact PCB and firmware stack that integrates multiple gas sensors and a battery off-gas detector onto one board, developed as part of a collaborative research project between the **University of Michigan** and **UL Research Institutes**. The board targets applications in **battery safety**, **robotics**, and **automotive** environments where real-time multi-gas monitoring in constrained spaces is critical.

## Project Overview

The board consolidates hydrogen (H₂), carbon monoxide (CO), CO₂ & VOC, methane (CH₄), and a **Li-ion Tamer** electrolyte-vapor (off-gas) detector onto a single PCB, interfacing through **two independent CAN buses**, **I²C**, and an **analog ADC** channel. The controller is an **STM32 Nucleo-G431KB**, chosen for its native FDCAN peripheral, I²C support, and 12-bit ADC.

Because the Nucleo-G431KB has only one CAN controller, the two buses run at different bit rates on different hardware:

- **FDCAN1 @ 500 kbps** — Telaire T3650 and SGX-BLD2, through an on-board CAN transceiver.
- **MCP2515 @ 250 kbps** (SPI) — Li-ion Tamer, which requires its own 250 kbps segment.

The project spans the full stack: **PCB design** (Autodesk Eagle), **embedded firmware** ([OffgasMonitor.ino](OffgasMonitor/OffgasMonitor.ino) — non-blocking dual-CAN, I²C and ADC acquisition with byte-level protocol decoding and on-chip health monitoring), and a **Python data-acquisition pipeline** ([serial_logger.py](OffgasMonitor/serial_logger.py) — serial parsing into a wide-table CSV).

## Specifications

| Parameter | Detail |
|---|---|
| **Controller** | STM32 Nucleo-G431KB (ARM Cortex-M4) |
| **Layers** | 1-layer PCB |
| **Board Thickness** | 1.6 mm (JLCPCB default) |
| **Power Supply** | 12 V battery input via terminal blocks |
| **Communication** | FDCAN1 (500 kbps), MCP2515 CAN (250 kbps) over SPI, I²C, analog ADC |
| **Log Rate** | 1 Hz (`LOG_INTERVAL = 1000 ms`) |
| **Serial** | 115200 baud |
| **Fabrication** | JLCPCB |

## Sensor Summary

| Sensor | Measurement | Interface | Address / ID | Power |
|---|---|---|---|---|
| Telaire T3650 | H₂, pressure, humidity, temperature | FDCAN1 @ 500 kbps | `0x18FF0CEB` (J1939, ext.) | 12 V direct |
| SGX-BLD2 | H₂ %, CO level, status flags | FDCAN1 @ 500 kbps | `0x256` (standard) | 12 V direct |
| Li-ion Tamer | Electrolyte-vapor off-gas, state, temperature | CAN @ 250 kbps via MCP2515 (SPI) | PGN `0xFF01` (J1939, ext.) | 12 V direct |
| MiCS-VZ-89TE | CO₂-equivalent & VOC | I²C | — | 3.3 V (Nucleo output) |
| MP7227 | CH₄ (methane) | Analog → ADC (`A0`) | — | 3.0 V (MIC5233 LDO) |
| SEN66 | PM / RH / T / VOC / NOx / CO₂ | I²C | — | 3.3 V (buck from 12 V) |

> **Note:** the SEN66 is provisioned in the power and I²C architecture but is **not yet decoded** in [OffgasMonitor.ino](OffgasMonitor/OffgasMonitor.ino); the Sensirion driver below is listed for the planned integration.

## Power Architecture

```
12V Supply
  │
  ├──[Terminal Blocks]──► Telaire T3650 (H₂)      — 12V direct
  ├──[Terminal Blocks]──► SGX-BLD2 (CO/H₂)        — 12V direct
  ├──[CAN Terminal]─────► Li-ion Tamer (off-gas)  — 12V direct
  │
  ├──► Buck regulator ──► 3.3V ──► SEN66 (tentative)
  │
  └──► Buck regulator ──► 5V ──► Nucleo-G431KB (onboard regulator)
                                  └──► 3.3V ──► MiCS-VZ-89TE (CO₂ & VOC)
                                  └──► 3.3V ──► MIC5233 LDO ──► 3.0V ──► MP7227 (CH₄)
                                  <!-- └──► 3.3V ──► SEN66  -->
```

## Signal Chain

- **Telaire T3650 & SGX-BLD2:** CAN → on-board transceiver → **FDCAN1 @ 500 kbps** → MCU.
- **Li-ion Tamer:** CAN @ **250 kbps** → **MCP2515** controller → **SPI1** → MCU.
- **MiCS-VZ-89TE:** **I²C** directly to the MCU.
- **MP7227 (CH₄):** analog output → zeroed via potentiometer → amplified (ADA4528 precision op-amp stage with HF filtering) → **ADC** (`A0`, 12-bit). Sensor rail from the **MIC5233** LDO.
- **SEN66:** **I²C** to the MCU (planned).

### Li-ion Tamer → Power / CAN bus

| Li-ion Tamer pin | Connects to | Target terminal | Notes |
|---|---|---|---|
| Vin | PCB | 12 V (CAN terminal) | Supply positive |
| GND | PCB | GND (CAN terminal) | Supply / common ground |
| CANH | MCP2515 module | CANH | CAN differential high |
| CANL | MCP2515 module | CANL | CAN differential low |

### MCP2515 → Nucleo (SPI)

| MCP2515 pin | Nucleo pin | Type | Notes |
|---|---|---|---|
| SCK (18) | `PB3` / D13 | SPI clock | SPI1 default |
| MOSI (19) | `PB5` / D11 | Master out, slave in | SPI1 default |
| MISO (16) | `PB4` / D12 | Master in, slave out | SPI1 default |
| CS (20) | `PA8` / **D9** | Chip select | D9 instead of the default D10, which is `fdcan1_rx` |
| INT (21) | `PB0` / **D3** | Interrupt input | D3 instead of the default D2, which is `fdcan1_tx` |
| VCC / GND | 3V3 / GND | Power | Shared ground with the CAN terminal ground |
| RST | NRST | Reset | Pull-up reset |
| — | CAN ground | Power | GND (CAN terminal) |

The MCP2515 module runs from a **16 MHz** crystal (`QUARTZ_FREQUENCY`); the firmware prints whether the requested 250 kbps is an exact bit rate at boot.

## Design Highlights

- **Sensor placement:** all gas inlets sit along the board edges to maximize exposure and airflow.
- **Unidirectional signal flow:** voltage and signal routing follows a single direction across the board for a clean layout and easy debugging.
- **Ground stability:** a large copper pour provides a solid ground plane and minimizes ground bounce.
- **Filtering:** decoupling capacitors and filtering suppress high-frequency noise on power and signal lines.
- **Differential pair integrity:** CANH/CANL pairs are routed with controlled impedance and matched length.
- **Mechanical:** mounting holes for integration into enclosures or test fixtures.

## Firmware — `OffgasMonitor/OffgasMonitor.ino`

A single non-blocking acquisition loop that services two CAN buses, I²C, and the ADC, then emits one human-readable report per second.

### Architecture

- **Continuous CAN polling.** `pollAllCan()` drains both receive FIFOs completely on every loop iteration, so neither bus backs up while the report is being printed. It is called again immediately after `logData()` to catch frames that arrived during printing.
- **Latest-frame slots.** Each device gets a `Latest<T>` slot holding the most recent frame of the current cycle plus a `valid` flag. The flag is cleared after every report, so **a device that stays silent for a second leaves its section out of the output entirely** — silence is preserved as a diagnostic rather than being masked by stale data.
- **CAN polling during ADC averaging.** `readAvgPolled()` averages 16 ADC samples to suppress noise; the ~200 µs settling gap between samples is spent polling CAN instead of idling. Averaging 16 samples across 2 channels takes ≈ 6.4 ms, which is too long to leave the buses unattended.
- **Rollover-safe scheduling.** The 1 Hz cadence uses a signed `millis()` comparison against `g_nextLog`, so it survives the 49-day `millis()` rollover, and the deadline advances by a fixed `LOG_INTERVAL` rather than from "now" to avoid drift.
- **Boot markers.** Setup prints tagged progress lines `A: boot` → `F: setup done`, giving both a human and the Python logger an unambiguous reboot marker.

### Decoding

- **SGX-BLD2 (`0x256`)** — temperature (offset −55 °C), H₂ % (16-bit, ×0.01), an 8-bit status byte decoded bit-by-bit into named faults, supply voltage, humidity, roll counter, and CO level.
- **Telaire T3650 (`0x18FF0CEB`)** — pressure (16-bit LE, ×0.0078125 − 250 kPa), humidity (×0.4 %), H₂ concentration (×0.0025 %), temperature (×0.03125 − 273 °C), and a raw status byte.
- **Li-ion Tamer (PGN `0xFF01`)** — J1939 source address from the low ID byte, state (`Illegal / Error / Warmup / Normal / Alarm`), signed temperature, and a signed 16-bit off-gas scalar (÷100). The report is annotated `<<< ALARM` in the Alarm state, or flagged when the scalar reaches the alarm trigger level of 1.0.
- **Unknown frames** on either bus are captured and printed with bus name, ID, and raw data bytes, so an unexpected node on the wire is visible instead of silently dropped.

### Board health monitoring

Each cycle also reports the MCU's own condition using the internal channels and STM32G4 factory calibration constants:

1. **Actual VDDA** is back-computed from the internal reference (`VREFINT_CAL` at 3.0 V) — the reference voltage is fixed, so a lower reading implies a higher supply.
2. **Junction temperature (Tj)** is derived by normalizing the temperature-sensor reading to the 3.0 V calibration reference and interpolating between the factory `TS_CAL1` (30 °C) and `TS_CAL2` (130 °C) points.
3. Running **max Tj** and **min VDDA** are tracked across the whole run, and the status line reports `OK` / `HIGH` (> 60 °C) / `OVERHEAT` (> 85 °C).

Tj is die temperature, not ambient, and factory calibration is only accurate to roughly ±5 °C — it is a "is the board cooking?" check, not an ambient-temperature log.

### Serial output format

One block per second, sections separated by `=============================`. Each round begins with `Raw ADC Value:`, which the Python logger uses as the round boundary:

```
Raw ADC Value: 2048
Voltage: 1.650 V
=============================
Board health (STM32G431KB):
Die Temp:           42.3 C
Die Temp Max:       44.1 C
VDDA:               3298 mV
VDDA Min:           3290 mV
ADC raw (ts/vref):  1035/1512
Board Status:       OK
=============================
CO2 equ(ppm): 412 ppm
VOC(isobutylene) equ: 87 ppb
=============================
Message received from SGX-BLD2:
...
=============================
Message received from Telaire T3650:
...
=============================
Message received from Li-ion Tamer:
Tamer SA=0xEB  State=Normal  Temp=24C  OffGas=0.12
=============================
```

## Host Data Acquisition — `OffgasMonitor/serial_logger.py`

A dependency-light (`pyserial` only) logger that parses the live serial stream and aggregates each one-second round into a single wide-table CSV row.

```bash
pip install pyserial

python serial_logger.py                 # use the PORT default in the file (COM4)
python serial_logger.py COM3            # specify the port
python serial_logger.py /dev/ttyACM0
python serial_logger.py --list-ports    # list available ports
python serial_logger.py --auto          # auto-pick a port that looks like an STM32
python serial_logger.py --selftest      # no hardware needed; verify parsing on built-in samples
```

| Flag | Effect |
|---|---|
| `--baud` | Baud rate (default 115200) |
| `--prefix` | Output filename prefix (default `sensor_log`; the start time is appended) |
| `--auto` | Pick the STMicroelectronics VID/description port automatically |
| `--list-ports` | List ports and exit |
| `--no-raw` | Skip the timestamped raw-text `.log` alongside the CSV |
| `--no-reconnect` | Exit on a dropout instead of retrying every 2 s |
| `--quiet` | Do not echo each row to the console |
| `--selftest` | Run the parser over built-in samples and check every field |

### Behavior

- **Section state machine.** The firmware reuses labels across sections (`Voltage:` appears under both ADC and SGX; `Temperature:` / `Humidity:` under both SGX and Telaire), so the parser disambiguates by *which section it is currently in*; a separator line clears the section and prevents bleed-over.
- **Empty means silent.** ADC, board health, and I²C columns are populated every round. If one of the CAN devices sent nothing that second, its columns are left **empty on purpose** — that emptiness is the record that the device was offline, so it must not be forward-filled in post-processing. The `can_device` column lists which devices were heard from that round.
- **Fault flags reassembled.** Individual SGX flag lines are collected into a semicolon-separated `sgx_flags` list *and* re-packed into `sgx_flags_hex` for easy comparison.
- **Crash-safe.** Every row is flushed to disk immediately, a Ctrl+C or serial dropout still writes the in-progress round, the port is reconnected automatically after a dropout, and board reboots (`A: boot`) are counted and reported at exit.
- **Self-test.** `--selftest` runs the parser against embedded sample output covering the normal case, an overheat + fault-flag + Tamer-alarm case, unknown frames on both buses, and a truncated final round — then writes `selftest_output.csv`.

### CSV columns

| Group | Columns |
|---|---|
| Timestamp | `date`, `time` |
| ADC (A0 / CH₄) | `raw_adc`, `voltage_adc` |
| Board health | `die_temp_c`, `die_temp_max_c`, `vdda_mv`, `vdda_min_mv`, `adc_raw_ts`, `adc_raw_vref`, `board_status` |
| MiCS-VZ-89TE | `co2_ppm`, `voc_ppb` |
| Devices heard | `can_device` |
| SGX-BLD2 | `sgx_temp_c`, `sgx_h2_pct`, `sgx_voltage_v`, `sgx_humidity_pct`, `sgx_roll_counter`, `sgx_level_co`, `sgx_flags`, `sgx_flags_hex` |
| Telaire T3650 | `tel_pressure_kpa`, `tel_humidity_pct`, `tel_h2_conc_pct`, `tel_temp_c`, `tel_status` |
| Li-ion Tamer | `tamer_sa`, `tamer_state`, `tamer_temp_c`, `tamer_offgas`, `tamer_alarm` |
| Unknown frames | `unk_fdcan_id`, `unk_fdcan_data`, `unk_mcp_id`, `unk_mcp_data` |

## Library Dependencies

| Library | Version | Purpose | Source |
|---|---|---|---|
| `ACANFD_STM32` — Pierre Molinaro | 1.1.2-rc1 | FDCAN1 @ 500 kbps (T3650, SGX-BLD2) | https://github.com/pierremolinaro/acanfd-stm32 |
| `ACAN2515` — Pierre Molinaro | 2.1.5 | MCP2515 CAN @ 250 kbps (Li-ion Tamer) | https://github.com/pierremolinaro/acan2515 |
| `MICS-VZ-89TE` — Herve Grabas | — | CO₂-equivalent & VOC over I²C | https://github.com/HGrabas/MICS-VZ-89TE |
| `arduino-i2c-sen66` — Sensirion (LeonieFierz) | — | SEN66 over I²C (planned) | https://github.com/Sensirion/arduino-i2c-sen66 |

Board support: **STM32duino** (STM32 MCU based boards), target *Nucleo-32 / Nucleo G431KB*.

## Testing

The board was validated on an Agilent E3641A bench power supply at **12.00 V / 0.200 A**. Sensor data from all channels was streamed over USB serial and captured by the Python logger, confirming correct FDCAN, MCP2515 CAN, I²C, and ADC acquisition — real-time temperature, humidity, raw ADC values, off-gas state, and computed gas concentrations (ppm/ppb) from each module. The logger's `--selftest` mode covers the parsing path without hardware.

## Repository Structure

| Path | Contents |
|---|---|
| [OffgasMonitor/OffgasMonitor.ino](OffgasMonitor/OffgasMonitor.ino) | Current firmware — dual CAN + I²C + ADC + board health |
| [OffgasMonitor/serial_logger.py](OffgasMonitor/serial_logger.py) | Current host logger — serial → wide-table CSV |
| [schematic.pdf](schematic.pdf) / [pcb_layout.pdf](pcb_layout.pdf) | Full schematic and PCB layout (top view) |
| [guides/](guides/) | Sensor and component datasheets (T3650, SGX-BLD2, MiCS-VZ-89TE, MP7227, MIC5233, Li-ion Tamer, Nucleo-G431KB) |
| [Deprecated/PCB Design/](Deprecated/PCB%20Design/) | Autodesk Eagle sources (`.sch` / `.brd`) and JLCPCB Gerber/CAM output, by revision (3.11, 3.22, 5.1) |
| [Deprecated/modules/](Deprecated/modules/) | Superseded per-sensor firmware & logging sketches (CAN, CH₄/ADC, CO₂/I²C) |
| [Deprecated/sensor_read/](Deprecated/sensor_read/) | Superseded integrated firmware + pandas/Excel logger |

Everything under [Deprecated/](Deprecated/) is kept for reference only; the active firmware and logger live in [OffgasMonitor/](OffgasMonitor/).

## Tools & Skills

- **EDA:** Autodesk Eagle
- **Fabrication:** JLCPCB
- **Controller Platform:** STM32 Nucleo-G431KB (ARM Cortex-M4)
- **Firmware:** Arduino / STM32duino — `ACANFD_STM32` (FDCAN), `ACAN2515` (SPI CAN), I²C, 12-bit ADC, on-chip Tj/VDDA calibration
- **Software:** Python (`pyserial`) for serial data acquisition & CSV logging
- **Skills demonstrated:** PCB design · multi-protocol embedded firmware (dual CAN / I²C / ADC) · J1939 & proprietary CAN frame decoding · non-blocking real-time acquisition · sensor data logging & diagnostics

## License

This project was developed under the University of Michigan and UL Research Institutes collaboration. Contact the author for usage permissions.
