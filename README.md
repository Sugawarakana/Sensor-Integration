# Multi-Gas Sensor Integration PCB

A compact single-layer PCB designed to integrate multiple gas sensors onto one board, developed as part of a collaborative research project between the **University of Michigan** and **UL Research Institutes**. The board targets future applications in **robotics** and **automotive** environments where real-time multi-gas monitoring in constrained spaces is critical.

## Project Overview

This board consolidates four distinct gas sensors — hydrogen (H₂), carbon monoxide (CO), CO₂ & VOC, and methane (CH₄) — into a single PCB, interfacing through a combination of CAN-FD bus, I²C, and analog ADC channels. The central controller is an **STM32 Nucleo-G431KB**, chosen for its native FDCAN peripheral, I²C support, and 12-bit ADC capability.

The project spans the full stack: **PCB design** (Autodesk Eagle), **embedded firmware** (STM32 / Arduino — concurrent CAN-FD, I²C, and ADC acquisition with byte-level protocol decoding), and a **Python data-acquisition pipeline** (serial parsing into Excel/CSV logs).

## Specifications

| Parameter | Detail |
|---|---|
| **Controller** | STM32 Nucleo-G431KB |
| **Layers** | 1-layer PCB |
| **Board Thickness** | 1.6 mm (JLCPCB default) |
| **Power Supply** | 12V battery input via terminal blocks |
| **Communication** | CAN bus, I²C, Analog (ADC) |
| **Fabrication** | JLCPCB |

## Sensor Summary

| Sensor | Target Gas | Communication | Power Supply |
|---|---|---|---|
| T3650 | H₂ (Hydrogen) | CAN (via transceiver) | 12V direct |
| SGX-BLD2 | CO (Carbon Monoxide) | CAN (via transceiver) | 12V direct |
| MiCS-VZ-89TE | CO₂ & VOC | I²C | 3.3V (Nucleo output) |
| MP7227 | CH₄ (Methane) | Analog → ADC GPIO | 3.3V (external LDO, 3.3-3) |

## Power Architecture

```
12V Battery
  │
  ├──[Terminal Blocks]──► T3650 (H₂) — 12V direct
  │
  ├──[Terminal Blocks]──► SGX-BLD2 (CO) — 12V direct
  │
  ├──► Nucleo-G431KB (onboard regulator)
  │         └──► 3.3V out ──► MiCS-VZ-89TE (CO₂ & VOC)
  │
  └──► 3.3-3 LDO ──► MP7227 (CH₄)
```

## Signal Chain

- **T3650 (H₂) & SGX-BLD2 (CO):** Digital output via CAN bus through an external CAN transceiver to the Nucleo's CAN peripheral.
- **MiCS-VZ-89TE (CO₂ & VOC):** Digital output via I²C directly to the Nucleo.
- **MP7227 (CH₄):** Analog output → zeroed via potentiometer → amplified (ADA4528 precision op-amp stage with HF filtering) → routed to Nucleo ADC GPIO. Sensor rail supplied by an external **MIC5233** LDO.

## Design Highlights

- **Sensor placement:** All gas inlets are positioned along the board edges to maximize exposure and airflow.
- **Unidirectional signal flow:** Voltage and signal routing follows a single direction across the board for clean layout and easy debugging.
- **Ground stability:** Large copper pour ensures a solid ground plane and minimizes ground bounce.
- **Filtering:** Decoupling capacitors and filtering are applied to suppress high-frequency noise on power and signal lines.
- **Differential pair integrity:** CAN differential pairs (CANH/CANL) are routed with controlled impedance and matched length.
- **Mechanical:** Mounting holes are included for easy integration into enclosures or test fixtures.

## Firmware & Software

Beyond the hardware, the project includes the full embedded and host-side software stack to acquire, decode, and log data from all four sensors.

### Embedded Firmware (STM32 / Arduino)

- **Unified acquisition loop** servicing three interfaces concurrently — CAN-FD (`ACANFD_STM32`, 500 kbps), I²C (MiCS-VZ-89TE), and 12-bit ADC (MP7227 / CH₄).
- **CAN-FD frame decoding** for two sensor protocols, distinguished by CAN ID (`0x256` SGX-BLD2, `0x18FF0CEB` Telaire T3650): byte-level multi-signal unpacking with correct endianness and per-signal scaling/offset (temperature, H₂ %, pressure, humidity, etc.).
- **Fault-flag parsing:** bit-field decode of the SGX-BLD2 status byte into 8 individual diagnostics (over/under-voltage, sensor-replacement, out-of-range, …).

### Host Data Acquisition (Python)

- `sensor_read/data_log.py` — `pyserial` + `pandas` logger that parses the live serial stream and writes per-sensor sheets (Internal sensors, SGX-BLD2, Telaire T3650) into an Excel workbook with periodic autosave.
- Per-sensor logging and real-time plotting utilities under `modules/` (e.g. CSV logging and live plots for the CO₂/VOC channel).

## Testing

The board was validated on an Agilent E3641A bench power supply at **12.00 V / 0.200 A**. Sensor data from all four channels was streamed over USB serial and captured by the Python logger, confirming correct CAN-FD, I²C, and ADC signal acquisition — real-time temperature, humidity, raw ADC values, and computed gas concentrations (ppm/ppb) from each module.

## Repository Structure

| Path | Contents |
|---|---|
| `schematic.pdf` / `pcb_layout.pdf` | Full schematic and PCB layout (top view) |
| `PCB Design/` | Autodesk Eagle source (`.sch` / `.brd`) and JLCPCB Gerber/CAM output |
| `sensor_read/` | Integrated firmware (all sensors) + Python data logger |
| `modules/` | Per-sensor firmware & logging (CAN, CH₄/ADC, CO₂/I²C) |
| `guides/` | Sensor and component datasheets |

## Tools & Skills

- **EDA:** Autodesk Eagle
- **Fabrication:** JLCPCB
- **Controller Platform:** STM32 Nucleo-G431KB (ARM Cortex-M4)
- **Firmware:** Arduino / STM32, `ACANFD_STM32` (CAN-FD), I²C, 12-bit ADC
- **Software:** Python (pyserial, pandas) for serial data acquisition & logging
- **Skills demonstrated:** PCB design · multi-protocol embedded firmware (CAN-FD / I²C / ADC) · CAN frame decoding · sensor data logging & visualization

## License

This project was developed under the University of Michigan and UL Research Institutes collaboration. Contact the author for usage permissions.
