# Multi-Gas Sensor Integration PCB

A compact single-layer PCB designed to integrate multiple gas sensors onto one board, developed as part of a collaborative research project between the **University of Michigan** and **UL Research Institutes**. The board targets future applications in **robotics** and **automotive** environments where real-time multi-gas monitoring in constrained spaces is critical.

## Project Overview

This board consolidates four distinct gas sensors — hydrogen (H₂), carbon monoxide (CO), CO₂ & VOC, and methane (CH₄) — into a single PCB, interfacing through a combination of CAN bus, I²C, and analog ADC channels. The central controller is an **STM32 Nucleo-G431KB**, chosen for its native CAN peripheral, I²C support, and 12-bit ADC capability.

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
- **MP7227 (CH₄):** Analog output → zeroed via potentiometer → amplified → routed to Nucleo ADC GPIO.

## Design Highlights

- **Sensor placement:** All gas inlets are positioned along the board edges to maximize exposure and airflow.
- **Unidirectional signal flow:** Voltage and signal routing follows a single direction across the board for clean layout and easy debugging.
- **Ground stability:** Large copper pour ensures a solid ground plane and minimizes ground bounce.
- **Filtering:** Decoupling capacitors and filtering are applied to suppress high-frequency noise on power and signal lines.
- **Differential pair integrity:** CAN differential pairs (CANH/CANL) are routed with controlled impedance and matched length.
- **Mechanical:** Mounting holes are included for easy integration into enclosures or test fixtures.

## Testing

The board was validated using a bench power supply (12V input) with sensor data read via serial port on a PC terminal, confirming correct operation of all four sensor channels.

## Hardware

- `/schematic.pdf` — Full schematic
- `/pcb_layout.pdf` — PCB layout (top view)

## Tools

- **EDA:** Autodesk Eagle
- **Fabrication:** JLCPCB
- **Controller Platform:** STM32 Nucleo-G431KB (ARM Cortex-M4)

## License

This project was developed under the University of Michigan and UL Research Institutes collaboration. Contact the author for usage permissions.
