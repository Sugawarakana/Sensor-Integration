#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_logger.py — Read the serial output of a Nucleo-G431KB and save it to CSV.

Matching firmware: ADC + on-chip temperature (Tj/VDDA) + MICS-VZ-89TE (I2C)
                   + Sensirion SEN66 (I2C, PM/RH/T/VOC/NOx/CO2)
                   + FDCAN1: SGX-BLD2 (0x256) / Telaire T3650 (0x18FF0CEB)
                   + MCP2515: Li-ion Tamer (J1939 PGN 0xFF01)
                   + unknown messages on both buses

Requires:  pip install pyserial

Usage:
    python serial_logger.py                 # use the PORT default below
    python serial_logger.py COM5            # specify the port on the command line
    python serial_logger.py /dev/ttyACM0
    python serial_logger.py --list-ports    # list the currently available ports
    python serial_logger.py --auto          # auto-pick a port that looks like an STM32
    python serial_logger.py --selftest      # no hardware needed; verify parsing with built-in samples

Ctrl+C stops it, and the last line (the round in progress) is still written to disk.

The firmware emits one round per second, and each round starts with "Raw ADC Value:".
This script aggregates by round into one wide-table row:
ADC / board health / I2C are present every round; whichever of the three CAN devices
sent no message that second leaves its columns empty — an empty column is itself the
diagnostic that the device was silent that second, so do not forward-fill it in
post-processing.

The SEN66 is I2C, so its block appears every round once the sensor is running. Empty
SEN66 value columns therefore mean something different from an empty CAN column: read
sen66_status, where NO_FRAME = not started yet, STALE = the cached frame aged out
(check I2C / power), WARMUP = the gas-index algorithm has not converged. An individual
channel printed as "n/a" (the unknown-value sentinel) also lands as an empty cell.
"""

import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

try:
    import serial                       # pip install pyserial
    from serial.tools import list_ports
except ImportError:                     # --selftest does not need pyserial
    serial = None
    list_ports = None

# ---------------------------------------------------------------- Configuration
PORT = "COM3"               # Windows: COMx ; Linux/macOS: something like /dev/ttyACM0
BAUD = 115200               # must match Serial.begin(115200) in the Arduino sketch
CSV_PREFIX = "sensor_log"   # the filename gets the start time appended; each run creates a new file
RECONNECT_DELAY = 2.0       # reconnect interval after a dropout (seconds)
# ----------------------------------------------------------------------

# Wide-table columns. When you change these, remember to change the matching
# assignments in Parser as well.
FIELDS = [
    "date", "time",
    # ADC (A0)
    "raw_adc", "voltage_adc",
    # Board health (STM32G431KB on-chip temperature + VDDA)
    "die_temp_c", "die_temp_max_c", "vdda_mv", "vdda_min_mv",
    "adc_raw_ts", "adc_raw_vref", "board_status",
    # I2C (MICS-VZ-89TE)
    "co2_ppm", "voc_ppb",
    # I2C (Sensirion SEN66)
    "sen66_pm1", "sen66_pm25", "sen66_pm4", "sen66_pm10",
    "sen66_rh_pct", "sen66_temp_c",
    "sen66_voc_index", "sen66_nox_index", "sen66_co2_ppm",
    "sen66_status", "sen66_age_ms",
    # CAN devices heard from this round (semicolon-separated)
    "can_device",
    # SGX-BLD2 (FDCAN1, 0x256)
    "sgx_temp_c", "sgx_h2_pct", "sgx_voltage_v", "sgx_humidity_pct",
    "sgx_roll_counter", "sgx_level_co", "sgx_flags", "sgx_flags_hex",
    # Telaire T3650 (FDCAN1, 0x18FF0CEB)
    "tel_pressure_kpa", "tel_humidity_pct", "tel_h2_conc_pct",
    "tel_temp_c", "tel_status",
    # Li-ion Tamer (MCP2515, PGN 0xFF01)
    "tamer_sa", "tamer_state", "tamer_temp_c", "tamer_offgas", "tamer_alarm",
    # Unknown messages (one group per bus, holding this round's last frame)
    "unk_fdcan_id", "unk_fdcan_data",
    "unk_mcp_id", "unk_mcp_data",
]

# Bit definitions for the SGX status byte; the order matches NAMES[8] in the firmware.
SGX_FLAG_BITS = {
    "Overvoltage": 0,
    "TC issue": 1,
    "RH issue": 2,
    "H2 out of range": 3,
    "Temperature issue": 4,
    "Undervoltage": 5,
    "Sensor replacement": 6,
    "Low power bit": 7,
}

SEPARATOR = "============================="

RE_NUM = re.compile(r"-?\d+\.?\d*")
RE_TS_VREF = re.compile(r"(\d+)\s*/\s*(\d+)")
RE_SEN66_AGE = re.compile(r"frame age\s+(\d+)")
RE_TAMER = re.compile(
    r"Tamer SA=0x([0-9A-Fa-f]+)\s+"
    r"State=(\S+)\s+"
    r"Temp=(-?\d+)C\s+"
    r"OffGas=(-?\d+\.?\d*)"
)
RE_UNKNOWN = re.compile(
    r"Unknown device on (\S+?)!\s*ID=0x([0-9A-Fa-f]+)\s*Data=(.*)"
)
RE_FLAG_LINE = re.compile(r"^(.+?):\s*([01])\s*$")


def new_row():
    return {k: "" for k in FIELDS}


def num(s):
    """Pull the number out of 'Label: value unit'.
    Only looks at the part after the last colon, otherwise numbers inside the
    label (such as the 2 in CO2 or in H2, or the 1.0 in PM1.0) would be mistaken
    for the reading. A channel printed as "n/a" contains no digits and therefore
    yields an empty string, which is what we want in the CSV."""
    tail = s.split(":")[-1]
    m = RE_NUM.search(tail)
    return m.group(0) if m else ""


class Parser:
    """Feed serial text in line by line; once a full round has accumulated it emits one dict.

    Section state machine: the firmware repeats the same labels in different places
    (Voltage belongs to both ADC and SGX; Temperature/Humidity belong to SGX, Telaire
    and SEN66 alike), so disambiguation has to rely on "which section are we currently
    in" rather than on the start of the line alone. A separator line clears the section,
    which prevents bleed-over between sections.
    """

    def __init__(self):
        self.row = None
        self.section = None      # adc / board / sen66 / sgx / telaire / tamer / unknown
        self.in_flags = False
        self.flags_set = []
        self.flags_byte = 0
        self.reboots = 0

    # -------------------------------------------------- Internal helpers
    def _add_device(self, name):
        parts = self.row["can_device"].split(";") if self.row["can_device"] else []
        if name not in parts:
            parts.append(name)
        self.row["can_device"] = ";".join(parts)

    def _add_sen66_status(self, name):
        # The firmware can print more than one status line in the same round
        # (for example WARMUP together with STALE), so accumulate instead of overwrite.
        parts = self.row["sen66_status"].split(";") if self.row["sen66_status"] else []
        if name not in parts:
            parts.append(name)
        self.row["sen66_status"] = ";".join(parts)

    def _finish(self):
        """Close out the current row and return it; returns None if no row is in progress."""
        r = self.row
        if r is None:
            return None
        if self.flags_set:
            r["sgx_flags"] = ";".join(self.flags_set)
        if r.get("can_device") and "SGX-BLD2" in r["can_device"]:
            r["sgx_flags_hex"] = "0x%02X" % self.flags_byte
        self.row = None
        self.section = None
        self.in_flags = False
        self.flags_set = []
        self.flags_byte = 0
        return r

    def flush(self):
        """For external cleanup (Ctrl+C / serial port dropout)."""
        return self._finish()

    # -------------------------------------------------- Main entry point
    def feed(self, line):
        """Feed one line (already stripped). Returns the dict completed by the
        previous round, or None."""
        if not line:
            return None

        # === Sampling-period boundary: every round starts with the ADC reading ===
        if line.startswith("Raw ADC Value:"):
            done = self._finish()             # write out the previous round first
            self.row = new_row()
            now = datetime.now()
            self.row["date"] = now.strftime("%Y-%m-%d")
            self.row["time"] = now.strftime("%H:%M:%S")
            self.row["raw_adc"] = num(line)
            self.section = "adc"
            return done

        # Board reboot: A: boot ... G: setup done — discard the half-finished row here
        if line.startswith("A: boot"):
            self.reboots += 1
            self._finish()
            return None

        if self.row is None:
            # The script just started and has not hit the beginning of a round yet;
            # skip the partial data.
            return None

        # A separator line = end of a section, so clear the section state
        if line.startswith("===="):
            self.section = None
            self.in_flags = False
            return None

        # === Section headers ===
        if line.startswith("Board health"):
            self.section = "board"
            return None
        if line.startswith("Message received from SEN66"):
            self.section = "sen66"
            return None
        if "SGX-BLD2" in line:
            self.section = "sgx"
            self._add_device("SGX-BLD2")
            return None
        if "Telaire T3650" in line:
            self.section = "telaire"
            self._add_device("Telaire-T3650")
            return None
        if "Li-ion Tamer" in line:
            self.section = "tamer"
            self._add_device("Li-ion-Tamer")
            return None
        if line.startswith("Unknown device"):
            self._parse_unknown(line)
            return None

        # SEN66 has not produced a valid frame yet — the firmware prints this one-liner
        # instead of the whole block. Note the ':' directly after SEN66, which keeps it
        # distinct from the "SEN66 Status:" line inside the block.
        if line.startswith("SEN66:"):
            self._add_sen66_status("NO_FRAME")
            return None

        # === Present in every round: ADC / MICS ===
        if self.section == "adc" and line.startswith("Voltage:"):
            self.row["voltage_adc"] = num(line)
            return None
        if line.startswith("CO2 equ"):
            self.row["co2_ppm"] = num(line)
            return None
        # Must be "VOC(" and not plain "VOC": the SEN66 block contains "VOC Index:",
        # which would otherwise be logged as the MICS reading.
        if line.startswith("VOC("):
            self.row["voc_ppb"] = num(line)
            return None

        if self.section == "board":
            self._parse_board(line)
        elif self.section == "sen66":
            self._parse_sen66(line)
        elif self.section == "sgx":
            self._parse_sgx(line)
        elif self.section == "telaire":
            self._parse_telaire(line)
        elif self.section == "tamer":
            self._parse_tamer(line)
        return None

    # -------------------------------------------------- Per-section parsing
    def _parse_board(self, line):
        r = self.row
        if line.startswith("VREFINT read failed"):
            r["board_status"] = "VREFINT_FAIL"
        elif line.startswith("Die Temp Max:"):
            r["die_temp_max_c"] = num(line)
        elif line.startswith("Die Temp:"):
            r["die_temp_c"] = num(line)
        elif line.startswith("VDDA Min:"):
            r["vdda_min_mv"] = num(line)
        elif line.startswith("VDDA:"):
            r["vdda_mv"] = num(line)
        elif line.startswith("ADC raw"):
            m = RE_TS_VREF.search(line.split(":")[-1])
            if m:
                r["adc_raw_ts"], r["adc_raw_vref"] = m.group(1), m.group(2)
        elif line.startswith("Board Status:"):
            # Keep only the status word: "OVERHEAT <<< INSPECTION NEEDED!"
            # is normalized to OVERHEAT
            val = line.split(":", 1)[1].strip()
            r["board_status"] = val.split()[0] if val else ""

    def _parse_sen66(self, line):
        r = self.row
        if line.startswith("PM1.0:"):
            r["sen66_pm1"] = num(line)
        elif line.startswith("PM2.5:"):
            r["sen66_pm25"] = num(line)
        elif line.startswith("PM4.0:"):
            r["sen66_pm4"] = num(line)
        elif line.startswith("PM10:"):
            r["sen66_pm10"] = num(line)
        elif line.startswith("Humidity:"):
            r["sen66_rh_pct"] = num(line)
        elif line.startswith("Temperature:"):
            r["sen66_temp_c"] = num(line)
        elif line.startswith("VOC Index:"):
            r["sen66_voc_index"] = num(line)
        elif line.startswith("NOx Index:"):
            r["sen66_nox_index"] = num(line)
        elif line.startswith("CO2:"):
            r["sen66_co2_ppm"] = num(line)
        elif line.startswith("SEN66 Status:"):
            self._parse_sen66_status(line)

    def _parse_sen66_status(self, line):
        # "gas index warming up"  ->  WARMUP
        # "STALE, frame age 3400 ms <<< CHECK I2C / POWER"  ->  STALE + age
        val = line.split(":", 1)[1].strip()
        if not val:
            return
        if val.lower().startswith("gas index"):
            self._add_sen66_status("WARMUP")
            return
        if val.upper().startswith("STALE"):
            self._add_sen66_status("STALE")
            m = RE_SEN66_AGE.search(val)
            if m:
                self.row["sen66_age_ms"] = m.group(1)
            return
        self._add_sen66_status(val.split()[0])

    def _parse_sgx(self, line):
        r = self.row
        if line.startswith("--- Flags ---"):
            self.in_flags = True
            return
        if line.startswith("---"):          # closing line -------------
            self.in_flags = False
            return
        if line.startswith("No status error"):
            self.flags_byte = 0
            return
        if line.startswith("Temperature:"):
            r["sgx_temp_c"] = num(line)
            return
        if line.startswith("Hydrogen percent"):
            r["sgx_h2_pct"] = num(line)
            return
        if line.startswith("Voltage:"):
            r["sgx_voltage_v"] = num(line)
            return
        if line.startswith("Humidity:"):
            r["sgx_humidity_pct"] = num(line)
            return
        if line.startswith("Roll Counter:"):
            r["sgx_roll_counter"] = num(line)
            return
        if line.startswith("Level CO:"):
            r["sgx_level_co"] = num(line)
            return
        # Fault flags of the form "Overvoltage:        1"; rebuild the status byte
        # so it can be compared easily
        if self.in_flags:
            m = RE_FLAG_LINE.match(line)
            if m:
                name, val = m.group(1).strip(), m.group(2)
                if val == "1":
                    self.flags_set.append(name)
                    bit = SGX_FLAG_BITS.get(name)
                    if bit is not None:
                        self.flags_byte |= (1 << bit)

    def _parse_telaire(self, line):
        r = self.row
        if line.startswith("Pressure:"):
            r["tel_pressure_kpa"] = num(line)
        elif line.startswith("Humidity:"):
            r["tel_humidity_pct"] = num(line)
        elif line.startswith("H2 Concentration:"):
            r["tel_h2_conc_pct"] = num(line)
        elif line.startswith("Temperature:"):
            r["tel_temp_c"] = num(line)
        elif line.startswith("Sensor Status:"):
            m = re.search(r"0x[0-9A-Fa-f]+", line)
            r["tel_status"] = m.group(0) if m else num(line)

    def _parse_tamer(self, line):
        # The firmware squeezes the Tamer onto one line, so a whole-line regex
        # is needed to split it
        m = RE_TAMER.search(line)
        if not m:
            return
        r = self.row
        r["tamer_sa"] = "0x" + m.group(1).upper()
        r["tamer_state"] = m.group(2)
        r["tamer_temp_c"] = m.group(3)
        r["tamer_offgas"] = m.group(4)
        if "<<< ALARM" in line:
            r["tamer_alarm"] = "ALARM"
        elif "alarm trigger level" in line:
            r["tamer_alarm"] = "HIGH"
        else:
            r["tamer_alarm"] = "OK"

    def _parse_unknown(self, line):
        m = RE_UNKNOWN.search(line)
        if not m:
            return
        bus, ident, data = m.group(1), "0x" + m.group(2).upper(), m.group(3).strip()
        if bus.upper().startswith("FDCAN"):
            self.row["unk_fdcan_id"], self.row["unk_fdcan_data"] = ident, data
            self._add_device("Unknown-FDCAN1")
        else:
            self.row["unk_mcp_id"], self.row["unk_mcp_data"] = ident, data
            self._add_device("Unknown-MCP2515")


class RowWriter:
    """CSV output + console echo. Flushes after every row, so nothing is lost
    even if power is pulled mid-run."""

    def __init__(self, csv_path, quiet=False):
        self.path = csv_path
        self.quiet = quiet
        self.count = 0
        self.f = open(csv_path, "w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.f, fieldnames=FIELDS)
        self.writer.writeheader()
        self.f.flush()

    def write(self, row):
        if row is None:
            return
        self.writer.writerow(row)
        self.f.flush()
        self.count += 1
        if not self.quiet:
            sen66 = row["sen66_co2_ppm"] or "-"
            if row["sen66_status"]:
                sen66 += "(%s)" % row["sen66_status"]
            print("  -> wrote row %d: %s %s | ADC %s | Tj %s | MICS %s | SEN66 %s | CAN %s"
                  % (self.count, row["date"], row["time"], row["raw_adc"],
                     row["die_temp_c"], row["co2_ppm"], sen66,
                     row["can_device"] or "-"))

    def close(self):
        self.f.close()


def pick_port(auto=False):
    """List/pick a serial port. With auto=True, prefer an STMicroelectronics port."""
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    if not ports:
        return None
    if not auto:
        return None
    for p in ports:
        desc = ((p.manufacturer or "") + " " + (p.description or "")).upper()
        if "STM" in desc or "NUCLEO" in desc or (p.vid == 0x0483):
            return p.device
    return ports[0].device


def print_ports():
    if list_ports is None:
        print("pyserial is not installed, cannot list serial ports. Run pip install pyserial first")
        return
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        print("  %-20s %s" % (p.device, p.description))


# ---------------------------------------------------------------- Self-test
SELFTEST_SAMPLE = """
A: boot
B: FDCAN1 beginFD = 0
C: MCP2515 OK
E: SEN66 reset issued
G: setup done
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
CO2 equ(ppm): 420.96 ppm
VOC(isobutylene) equ: 698.69 ppb
=============================
Message received from SEN66:
PM1.0:              0.0 ug/m3
PM2.5:              0.0 ug/m3
PM4.0:              0.0 ug/m3
PM10:               0.0 ug/m3
Humidity:           54.5 %
Temperature:        25.38 C
VOC Index:          0.0
NOx Index:          0.0
CO2:                431 ppm
SEN66 Status:       gas index warming up
=============================
Message received from SGX-BLD2:
Temperature: 23 C
Hydrogen percent:   0.42 %
No status error
Voltage:            12.1 V
Humidity:           45.5 %
Roll Counter:       7
Level CO:           1
=============================
Message received from Telaire T3650:
Pressure:           101.32 kPa
Humidity:  38.4 %
H2 Concentration:   0.0125 %
Temperature:        25.44 C
Sensor Status:      0x0
=============================
Message received from Li-ion Tamer:
Tamer SA=0xEB  State=Normal  Temp=24C  OffGas=0.12
=============================
Raw ADC Value: 3000
Voltage: 2.418 V
=============================
Board health (STM32G431KB):
Die Temp:           88.7 C
Die Temp Max:       88.7 C
VDDA:               3280 mV
VDDA Min:           3280 mV
ADC raw (ts/vref):  1180/1520
Board Status:       OVERHEAT <<< INSPECTION NEEDED!
=============================
CO2 equ(ppm): 998 ppm
VOC(isobutylene) equ: 1203 ppb
=============================
Message received from SEN66:
PM1.0:              1.2 ug/m3
PM2.5:              1.4 ug/m3
PM4.0:              1.5 ug/m3
PM10:               1.6 ug/m3
Humidity:           54.0 %
Temperature:        26.10 C
VOC Index:          101.0
NOx Index:          1.0
CO2:                n/a
SEN66 Status:       STALE, frame age 3400 ms <<< CHECK I2C / POWER
=============================
Message received from SGX-BLD2:
Temperature: 31 C
Hydrogen percent:   1.75 %
--- Flags ---
 Overvoltage:        0
 TC issue:           0
 RH issue:           0
 H2 out of range:    1
 Temperature issue:  1
 Undervoltage:       0
 Sensor replacement: 0
 Low power bit:      0
-------------
Voltage:            11.8 V
Humidity:           50.0 %
Roll Counter:       8
Level CO:           3
=============================
Message received from Li-ion Tamer:
Tamer SA=0xEB  State=Alarm  Temp=26C  OffGas=1.35  <<< ALARM
=============================
Unknown device on FDCAN1! ID=0x123 Data=1 2 3 4
=============================
Unknown device on MCP2515! ID=0x18FF0CFF Data=AA BB
=============================
Raw ADC Value: 100
Voltage: 0.081 V
=============================
SEN66: no valid frame yet (starting up)
=============================
"""


def selftest():
    """No hardware: run the parser over the built-in sample and check it field by field."""
    p = Parser()
    rows = []
    for line in SELFTEST_SAMPLE.splitlines():
        r = p.feed(line.strip())
        if r:
            rows.append(r)
    tail = p.flush()
    if tail:
        rows.append(tail)

    expect = [
        # Round 1: everything normal, SEN66 still warming up its gas indices,
        # no SGX faults, both Telaire and Tamer present.
        # Note MICS voc_ppb=698.69 vs SEN66 voc_index=0.0 — the two must not collide.
        {"raw_adc": "2048", "voltage_adc": "1.650",
         "die_temp_c": "42.3", "die_temp_max_c": "44.1",
         "vdda_mv": "3298", "vdda_min_mv": "3290",
         "adc_raw_ts": "1035", "adc_raw_vref": "1512", "board_status": "OK",
         "co2_ppm": "420.96", "voc_ppb": "698.69",
         "sen66_pm1": "0.0", "sen66_pm25": "0.0", "sen66_pm4": "0.0",
         "sen66_pm10": "0.0", "sen66_rh_pct": "54.5", "sen66_temp_c": "25.38",
         "sen66_voc_index": "0.0", "sen66_nox_index": "0.0",
         "sen66_co2_ppm": "431", "sen66_status": "WARMUP", "sen66_age_ms": "",
         "can_device": "SGX-BLD2;Telaire-T3650;Li-ion-Tamer",
         "sgx_temp_c": "23", "sgx_h2_pct": "0.42", "sgx_voltage_v": "12.1",
         "sgx_humidity_pct": "45.5", "sgx_roll_counter": "7",
         "sgx_level_co": "1", "sgx_flags": "", "sgx_flags_hex": "0x00",
         "tel_pressure_kpa": "101.32", "tel_humidity_pct": "38.4",
         "tel_h2_conc_pct": "0.0125", "tel_temp_c": "25.44",
         "tel_status": "0x0",
         "tamer_sa": "0xEB", "tamer_state": "Normal", "tamer_temp_c": "24",
         "tamer_offgas": "0.12", "tamer_alarm": "OK",
         "unk_fdcan_id": "", "unk_mcp_id": ""},
        # Round 2: overheat + two SGX fault bits + Tamer alarm + one unknown frame on
        #          each bus; Telaire said nothing this second, so its columns must stay
        #          empty. SEN66 frame is stale and its CO2 channel reads "n/a".
        {"raw_adc": "3000", "voltage_adc": "2.418",
         "die_temp_c": "88.7", "board_status": "OVERHEAT",
         "co2_ppm": "998", "voc_ppb": "1203",
         "sen66_pm25": "1.4", "sen66_rh_pct": "54.0", "sen66_temp_c": "26.10",
         "sen66_voc_index": "101.0", "sen66_nox_index": "1.0",
         "sen66_co2_ppm": "", "sen66_status": "STALE", "sen66_age_ms": "3400",
         "can_device": "SGX-BLD2;Li-ion-Tamer;Unknown-FDCAN1;Unknown-MCP2515",
         "sgx_temp_c": "31", "sgx_h2_pct": "1.75", "sgx_voltage_v": "11.8",
         "sgx_humidity_pct": "50.0", "sgx_roll_counter": "8",
         "sgx_level_co": "3",
         "sgx_flags": "H2 out of range;Temperature issue",
         "sgx_flags_hex": "0x18",
         "tel_pressure_kpa": "", "tel_temp_c": "", "tel_status": "",
         "tamer_state": "Alarm", "tamer_offgas": "1.35",
         "tamer_alarm": "ALARM",
         "unk_fdcan_id": "0x123", "unk_fdcan_data": "1 2 3 4",
         "unk_mcp_id": "0x18FF0CFF", "unk_mcp_data": "AA BB"},
        # Round 3: partial round (the Ctrl+C case) with the SEN66 one-liner instead of
        #          a full block — it should still be written out, flagged NO_FRAME
        {"raw_adc": "100", "voltage_adc": "0.081", "can_device": "",
         "sen66_status": "NO_FRAME", "sen66_pm25": "", "sen66_co2_ppm": ""},
    ]

    bad = 0
    if len(rows) != len(expect):
        print("!! wrong number of rows: got %d, expected %d" % (len(rows), len(expect)))
        bad += 1
    for i, (got, exp) in enumerate(zip(rows, expect), 1):
        for k, v in exp.items():
            if got.get(k) != v:
                print("!! row %d %-18s got %r, expected %r" % (i, k, got.get(k), v))
                bad += 1
    if bad == 0:
        print("Self-test passed: %d rows, all fields match expectations." % len(rows))
    else:
        print("Self-test failed: %d mismatches." % bad)

    out = "selftest_output.csv"
    w = RowWriter(out, quiet=True)
    for r in rows:
        w.write(r)
    w.close()
    print("Sample CSV written to %s" % os.path.abspath(out))
    return 0 if bad == 0 else 1


# ---------------------------------------------------------------- Main flow
def run(port, baud, prefix, raw_log, quiet, reconnect):
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = "%s_%s.csv" % (prefix, stamp)
    raw_path = "%s_%s.log" % (prefix, stamp) if raw_log else None

    parser = Parser()
    writer = RowWriter(csv_path, quiet=quiet)
    rawf = open(raw_path, "w", encoding="utf-8") if raw_path else None

    print("Writing to %s" % os.path.abspath(csv_path))
    if raw_path:
        print("Raw serial text %s" % os.path.abspath(raw_path))
    print("Press Ctrl+C to stop.")

    ser = None
    try:
        while True:
            # ---- connect / reconnect ----
            if ser is None:
                try:
                    ser = serial.Serial(port, baud, timeout=2)
                    print("Opened %s @ %d" % (port, baud))
                except (serial.SerialException, OSError) as e:
                    if not reconnect:
                        raise
                    print("Failed to open %s (%s), retrying in %.0fs..." %
                          (port, e, RECONNECT_DELAY))
                    time.sleep(RECONNECT_DELAY)
                    continue

            # ---- read one line ----
            try:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
            except (serial.SerialException, OSError) as e:
                print("Serial read interrupted (%s)." % e)
                writer.write(parser.flush())     # write out the partial row before the dropout
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                if not reconnect:
                    break
                time.sleep(RECONNECT_DELAY)
                continue

            if not raw:
                continue
            if rawf:
                rawf.write("%s\t%s\n" %
                           (datetime.now().strftime("%H:%M:%S.%f")[:-3], raw))

            writer.write(parser.feed(raw))

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        writer.write(parser.flush())    # write out the last (in-progress) row too
        writer.close()
        if rawf:
            rawf.flush()
            rawf.close()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        print("%d rows total, saved to %s" % (writer.count, os.path.abspath(csv_path)))
        if parser.reboots > 1:
            print("Note: detected %d board reboots ('A: boot') during the run."
                  % (parser.reboots - 1))


def main():
    ap = argparse.ArgumentParser(
        description="Nucleo-G431KB serial data logger",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default=None,
                    help="serial port, e.g. COM5 or /dev/ttyACM0 (default %s)" % PORT)
    ap.add_argument("--baud", type=int, default=BAUD, help="baud rate, default %d" % BAUD)
    ap.add_argument("--prefix", default=CSV_PREFIX, help="output filename prefix")
    ap.add_argument("--auto", action="store_true", help="auto-pick the STM32 serial port")
    ap.add_argument("--list-ports", action="store_true", help="list available serial ports and exit")
    ap.add_argument("--no-raw", action="store_true", help="do not also save the raw serial text")
    ap.add_argument("--no-reconnect", action="store_true", help="exit on dropout instead of reconnecting")
    ap.add_argument("--quiet", action="store_true", help="do not echo every row to the console")
    ap.add_argument("--selftest", action="store_true",
                    help="no hardware needed; verify the parsing logic with built-in samples")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.list_ports:
        print_ports()
        return 0
    if serial is None:
        print("pyserial is missing, run: pip install pyserial")
        return 1

    port = args.port or (pick_port(auto=True) if args.auto else None) or PORT
    run(port, args.baud, args.prefix,
        raw_log=not args.no_raw, quiet=args.quiet,
        reconnect=not args.no_reconnect)
    return 0


if __name__ == "__main__":
    sys.exit(main())
