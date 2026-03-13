#include <ACANFD_STM32.h>
#include <ACANFD_STM32_NUCLEO_G431KB-objects.h> 
#include <ACANFD_STM32_CANFDMessage.h>
#include <Wire.h>
#include "MICS-VZ-89TE.h"
MICS_VZ_89TE sensor;
#define MICS_ad 0x70
const int analogInPin = A0; 

void setup() {
    Serial.begin(115200);
    // CAN initialization
    ACANFD_STM32_Settings settings(500000, DataBitRateFactor::x1, 32); // 500kbps
    const uint32_t errorCode = fdcan1.beginFD(settings);
    if (errorCode == 0) {
        Serial.println("CAN Init OK");
    } else {
        Serial.print("CAN Init Error: ");
        Serial.println(errorCode);
    }

    // I2C initialization
    Wire.begin();

    // ADC Pin mode set
    pinMode(analogInPin, INPUT_ANALOG);
    analogReadResolution(12);
}

void loop() {
    delay(10000);
    // ADC part
    int sensorValue = analogRead(analogInPin);
    Serial.print("Raw ADC Value: ");
    Serial.println(sensorValue);

    float voltage = sensorValue * (3.3 / 4095.0);
    Serial.print("Voltage: ");
    Serial.print(voltage, 3); 
    Serial.println(" V");
    Serial.println("=============================");

    // I2C part
    sensor.readSensor();
    float co2 = sensor.getCO2();
    float voc = sensor.getVOC();
    uint32_t status = sensor.getStatus();
    uint32_t rev = sensor.getRev();
    Serial.print("CO2 equ(ppm): ");
    Serial.print(co2);
    Serial.println(" ppm");
    Serial.print("VOC(isobutylene) equ: ");
    Serial.print(voc);
    Serial.println(" ppb");
    Serial.println("=============================");

    // CAN part
    CANFDMessage msg;
    CANFDMessage rcvMsg;
    if (fdcan1.receiveFD0(rcvMsg)) {
        if (rcvMsg.id == 0x256){
            Serial.println("Message received from SGX-BLD2:");

            // 1 Temperature
            int temp_raw = rcvMsg.data[0];
            int temperature_phys = temp_raw - 55;
            Serial.print("Temperature: ");
            Serial.print(temperature_phys);
            Serial.println(" C");

            // 2 Hydrogen percent
            uint16_t h2_raw = (rcvMsg.data[1] << 8) | rcvMsg.data[2];
            float h2_phys = h2_raw * 0.01;
            Serial.print("Hydrogen percent:   ");
            Serial.print(h2_phys, 2);
            Serial.println(" %");

            // 3 Flags
            uint8_t flags_byte = rcvMsg.data[3];
            bool overvoltage       = (flags_byte >> 0) & 1;
            bool tc_issue          = (flags_byte >> 1) & 1;
            bool rh_issue          = (flags_byte >> 2) & 1;
            bool h2_out_of_range   = (flags_byte >> 3) & 1;
            bool temp_issue        = (flags_byte >> 4) & 1;
            bool undervoltage      = (flags_byte >> 5) & 1;
            bool sensor_replace    = (flags_byte >> 6) & 1;
            bool low_power_bit     = (flags_byte >> 7) & 1;
            if (flags_byte & 0xFF) {
                Serial.println("--- Flags ---");
                Serial.print(" Overvoltage:        "); Serial.println(overvoltage);
                Serial.print(" TC issue:           "); Serial.println(tc_issue);
                Serial.print(" RH issue:           "); Serial.println(rh_issue);
                Serial.print(" H2 out of range:    "); Serial.println(h2_out_of_range);
                Serial.print(" Temperature issue:  "); Serial.println(temp_issue);
                Serial.print(" Undervoltage:       "); Serial.println(undervoltage);
                Serial.print(" Sensor replacement: "); Serial.println(sensor_replace);
                Serial.print(" Low power bit:      "); Serial.println(low_power_bit);
                Serial.println("-------------");
            }
            else {
                Serial.println("No status error");
            }

            // 4 Voltage
            uint8_t voltage_raw = rcvMsg.data[4];
            float voltage_phys = voltage_raw * 0.1;
            Serial.print("Voltage:            ");
            Serial.print(voltage_phys, 1);
            Serial.println(" V");

            // 5 Humidity
            uint8_t humidity_raw = rcvMsg.data[5];
            float humidity_phys = humidity_raw * 0.5;
            Serial.print("Humidity:           ");
            Serial.print(humidity_phys, 1);
            Serial.println(" %");

            // 6 Roll counter
            uint8_t byte6 = rcvMsg.data[6];
            uint8_t roll_counter = byte6 & 0x0F;
            uint8_t level_co = (byte6 >> 4) & 0x0F;
            Serial.print("Roll Counter:       ");
            Serial.println(roll_counter);
            Serial.print("Level CO:           ");
            Serial.println(level_co);
            Serial.println("=============================");
        }

        else if (rcvMsg.id == 0x18FF0CEB){
            Serial.println("Message received from Telaire T3650:");

            // 1 Pressure
            uint16_t press_raw = (rcvMsg.data[1] << 8) | rcvMsg.data[0];
            float pressure = (press_raw * 0.0078125) - 250.0;   
            Serial.print("Pressure:           ");
            Serial.print(pressure, 2);
            Serial.println(" kPa");

            // 2 Humidity
            uint8_t h_raw = rcvMsg.data[2];
            float humidity = h_raw * 0.4;
            Serial.print("Humidity:  ");
            Serial.print(humidity, 1);
            Serial.println(" %");    

            // 3 H2 concentration
            uint16_t h2_raw = (rcvMsg.data[4] << 8) | rcvMsg.data[3];
            float h2_conc = h2_raw * 0.0025;
            Serial.print("H2 Concentration:   ");
            Serial.print(h2_conc, 4); 
            Serial.println(" %");

            // 4 Temperature
            uint16_t temp_raw = (rcvMsg.data[6] << 8) | rcvMsg.data[5];
            float temperature = (temp_raw * 0.03125) - 273.0;
            Serial.print("Temperature:        ");
            Serial.print(temperature, 2);
            Serial.println(" C");
            
            // 5 Sensor status
            int8_t status = rcvMsg.data[7];
            Serial.print("Sensor Status:      0x");
            Serial.println(status, HEX);
            Serial.println("=============================");
        }

        else{
            Serial.println("Unknown device!");
            Serial.print("CAN RX: ID=");
            Serial.print(rcvMsg.id, HEX);
            Serial.print("Data=");
            for (uint8_t i = 0; i < rcvMsg.len; i++) {
                Serial.print(rcvMsg.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println("=============================");
        }
    }
}