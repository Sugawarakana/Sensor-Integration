"""
serial_logger.py — 读取 Nucleo-G431KB 的串口输出并存成 CSV。

依赖:  pip install pyserial
用法:  python serial_logger.py            # 用下面 PORT 默认值
       python serial_logger.py COM5       # 命令行指定串口
       python serial_logger.py /dev/ttyACM0
Ctrl+C 停止，会把最后一行也落盘。
"""

import csv
import re
import sys
from datetime import datetime

import serial  # pip install pyserial

# ---------------------------------------------------------------- 配置
PORT = "COM3"            # Windows: COMx ；Linux/macOS: /dev/ttyACM0 之类
BAUD = 115200            # 要和 Arduino 里 Serial.begin(115200) 一致
CSV_PREFIX = "sensor_log"   # 实际文件名会自动带上启动时间，每次运行新建一个文件
# ----------------------------------------------------------------------

# 宽表：ADC/I2C 每行都有；CAN 字段按设备分组，没收到报文就留空
FIELDS = [
    "date", "time",
    # ADC
    "raw_adc", "voltage_adc",
    # I2C (MICS-VZ-89TE)
    "co2_ppm", "voc_ppb",
    # 本轮 CAN 来源设备
    "can_device",
    # SGX-BLD2 (0x256)
    "sgx_temp_c", "sgx_h2_pct", "sgx_voltage_v", "sgx_humidity_pct",
    "sgx_roll_counter", "sgx_level_co", "sgx_flags",
    # Telaire T3650 (0x18FF0CEB)
    "tel_pressure_kpa", "tel_humidity_pct", "tel_h2_conc_pct",
    "tel_temp_c", "tel_status",
]


def new_row():
    return {k: "" for k in FIELDS}


def num(s):
    """从 'Label: value unit' 里取数字。
    只看最后一个冒号后面的部分，否则会把标签里的数字
    （如 CO2 的 2、H2 的 2）误当成读数。"""
    tail = s.split(":")[-1]
    m = re.search(r"-?\d+\.?\d*", tail)
    return m.group(0) if m else ""


def add_device(row, name):
    """把设备名追加进 can_device 列（去重），这样同一行可同时记录多个设备。"""
    parts = row["can_device"].split(";") if row["can_device"] else []
    if name not in parts:
        parts.append(name)
    row["can_device"] = ";".join(parts)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT

    # 每次运行新建一个带启动时间的文件，例如 sensor_log_2026-06-15_14-30-05.csv
    csv_path = f"{CSV_PREFIX}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"

    ser = serial.Serial(port, BAUD, timeout=2)
    print(f"已打开 {port} @ {BAUD}，写入 {csv_path}。按 Ctrl+C 停止。")

    f = open(csv_path, "w", newline="", encoding="utf-8-sig")   # 新文件，"w" 覆盖
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()
    f.flush()

    row = None
    section = None                    # 'adc' / 'sgx' / 'telaire' / 'unknown'
    active_flags = []

    def flush(r, flags):
        if r is None:
            return
        if flags:
            r["sgx_flags"] = ";".join(flags)
        # 注意：timestamp 已在该轮开头（收到 Raw ADC Value 时）打好，这里不再覆盖
        writer.writerow(r)
        f.flush()
        print("  -> 写入一行:", r["date"], r["time"],
              "| ADC", r["raw_adc"], "| CO2", r["co2_ppm"],
              "| CAN", r["can_device"] or "-")

    try:
        while True:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue

            # === 采样周期边界：每轮都从 ADC 读数开始 ===
            if raw.startswith("Raw ADC Value:"):
                flush(row, active_flags)      # 先把上一轮落盘
                row = new_row()
                _now = datetime.now()
                row["date"] = _now.strftime("%Y-%m-%d")
                row["time"] = _now.strftime("%H:%M:%S")
                active_flags = []
                section = "adc"
                row["raw_adc"] = num(raw)
                continue

            if row is None:
                # 脚本刚启动、还没遇到一轮的开头，跳过半截数据
                continue

            # === CAN 设备表头：切换解析段落 ===
            if "SGX-BLD2" in raw:
                section = "sgx"; add_device(row, "SGX-BLD2"); continue
            if "Telaire T3650" in raw:
                section = "telaire"; add_device(row, "Telaire-T3650"); continue
            if raw.startswith("Unknown device"):
                section = "unknown"; add_device(row, "Unknown"); continue

            # === ADC / I2C（每轮固定有）===
            if section == "adc" and raw.startswith("Voltage:"):
                row["voltage_adc"] = num(raw); continue
            if raw.startswith("CO2 equ"):
                row["co2_ppm"] = num(raw); continue
            if raw.startswith("VOC"):
                row["voc_ppb"] = num(raw); continue

            # === SGX-BLD2 字段 ===
            if section == "sgx":
                if raw.startswith("Temperature:"):
                    row["sgx_temp_c"] = num(raw); continue
                if raw.startswith("Hydrogen percent"):
                    row["sgx_h2_pct"] = num(raw); continue
                if raw.startswith("Voltage:"):
                    row["sgx_voltage_v"] = num(raw); continue
                if raw.startswith("Humidity:"):
                    row["sgx_humidity_pct"] = num(raw); continue
                if raw.startswith("Roll Counter:"):
                    row["sgx_roll_counter"] = num(raw); continue
                if raw.startswith("Level CO:"):
                    row["sgx_level_co"] = num(raw); continue
                # 剩下形如 "Xxx: 1" 的、值为 1 的就是被置位的故障标志
                m = re.match(r"(.+?):\s*1\s*$", raw)
                if m:
                    active_flags.append(m.group(1).strip())
                continue

            # === Telaire T3650 字段 ===
            if section == "telaire":
                if raw.startswith("Pressure:"):
                    row["tel_pressure_kpa"] = num(raw); continue
                if raw.startswith("Humidity:"):
                    row["tel_humidity_pct"] = num(raw); continue
                if raw.startswith("H2 Concentration:"):
                    row["tel_h2_conc_pct"] = num(raw); continue
                if raw.startswith("Temperature:"):
                    row["tel_temp_c"] = num(raw); continue
                if raw.startswith("Sensor Status:"):
                    m = re.search(r"0x[0-9A-Fa-f]+", raw)
                    row["tel_status"] = m.group(0) if m else num(raw)
                    continue

    except KeyboardInterrupt:
        print("\n停止中...")
    finally:
        flush(row, active_flags)      # 把最后一行（进行中的）也写掉
        f.close()
        ser.close()
        print(f"已保存到 {csv_path}")


if __name__ == "__main__":
    main()