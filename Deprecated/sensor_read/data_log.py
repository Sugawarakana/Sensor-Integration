import serial
import re
import pandas as pd
from datetime import datetime
import time

# --- Configuration ---
SERIAL_PORT = 'COM3'  # Change to your actual port (e.g., '/dev/ttyACM0' on Linux)
BAUD_RATE = 115200
EXCEL_FILE = "sensor_data_log.xlsx"
SAVE_INTERVAL = 5     # Synchronize to Excel every 5 data entries

# Data structure to hold records for different devices/sensors
data_storage = {
    "Internal_Sensors": [],  # For ADC and MICS-VZ-89TE
    "SGX_BLD2": [],          # For CAN ID 0x256
    "Telaire_T3650": []      # For CAN ID 0x18FF0CEB
}

def save_to_excel():
    """Writes the current buffer to separate sheets in an Excel file."""
    try:
        with pd.ExcelWriter(EXCEL_FILE, engine='openpyxl') as writer:
            for sheet_name, rows in data_storage.items():
                if rows:
                    df = pd.DataFrame(rows)
                    df.to_excel(writer, sheet_name=sheet_name, index=False)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Data synced to {EXCEL_FILE}")
    except Exception as e:
        print(f"Error saving to Excel: {e}")

def main():
    try:
        # Initialize Serial Connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        print("Press Ctrl+C to stop recording.")

        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # Print raw serial output to console for monitoring
            print(f"> {line}")
            
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # --- Logic for Internal Sensors (ADC & MICS) ---
            if "Raw ADC Value:" in line:
                # Start a new entry when the ADC header is detected
                internal_entry = {"Timestamp": timestamp}
                internal_entry["Raw_ADC"] = re.findall(r"\d+", line)[0]
                
                # Nested loop to capture the associated values in the same block
                for _ in range(5): 
                    next_line = ser.readline().decode('utf-8').strip()
                    if "Voltage:" in next_line: 
                        internal_entry["Voltage_V"] = re.findall(r"\d+\.\d+", next_line)[0]
                    if "CO2 equ" in next_line:
                        internal_entry["CO2_ppm"] = re.findall(r"\d+\.\d+", next_line)[0]
                    if "VOC" in next_line:
                        internal_entry["VOC_ppb"] = re.findall(r"\d+\.\d+", next_line)[0]
                
                data_storage["Internal_Sensors"].append(internal_entry)

            # --- Logic for SGX-BLD2 (CAN ID 0x256) ---
            elif "Message received from SGX-BLD2:" in line:
                sgx_entry = {"Timestamp": timestamp}
                while True:
                    sgx_line = ser.readline().decode('utf-8').strip()
                    if "Temperature:" in sgx_line: sgx_entry["Temp_C"] = re.findall(r"-?\d+", sgx_line)[0]
                    if "Hydrogen percent:" in sgx_line: sgx_entry["H2_pct"] = re.findall(r"\d+\.\d+", sgx_line)[0]
                    if "Voltage:" in sgx_line: sgx_entry["Voltage_V"] = re.findall(r"\d+\.\d+", sgx_line)[0]
                    if "Humidity:" in sgx_line: sgx_entry["Humidity_pct"] = re.findall(r"\d+\.\d+", sgx_line)[0]
                    if "Level CO:" in sgx_line: sgx_entry["CO_Level"] = re.findall(r"\d+", sgx_line)[0]
                    if "====" in sgx_line: 
                        data_storage["SGX_BLD2"].append(sgx_entry)
                        break

            # --- Logic for Telaire T3650 (CAN ID 0x18FF0CEB) ---
            elif "Message received from Telaire T3650:" in line:
                tel_entry = {"Timestamp": timestamp}
                while True:
                    tel_line = ser.readline().decode('utf-8').strip()
                    if "Pressure:" in tel_line: tel_entry["Pressure_kPa"] = re.findall(r"-?\d+\.\d+", tel_line)[0]
                    if "Humidity:" in tel_line: tel_entry["Humidity_pct"] = re.findall(r"\d+\.\d+", tel_line)[0]
                    if "H2 Concentration:" in tel_line: tel_entry["H2_Conc_pct"] = re.findall(r"\d+\.\d+", tel_line)[0]
                    if "Temperature:" in tel_line: tel_entry["Temp_C"] = re.findall(r"-?\d+\.\d+", tel_line)[0]
                    if "====" in tel_line:
                        data_storage["Telaire_T3650"].append(tel_entry)
                        break

            # Periodically save data to disk
            total_count = sum(len(v) for v in data_storage.values())
            if total_count > 0 and total_count % SAVE_INTERVAL == 0:
                save_to_excel()

    except KeyboardInterrupt:
        print("\nLogging stopped by user. Saving final data...")
        save_to_excel()
        ser.close()
    except Exception as e:
        print(f"Critical Error: {e}")

if __name__ == "__main__":
    main()
