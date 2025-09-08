#include <ACANFD_STM32.h>
#include <ACANFD_STM32_NUCLEO_G431KB-objects.h> 
#include <ACANFD_STM32_CANFDMessage.h>


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
    sensor.begin();
}

void loop() {
    delay(500);
    sensor.readSensor();
    float co2 = sensor.getCO2();
    CANFDMessage msg;
    msg.id = 0x123;
    msg.len = 2;
    msg.data[0] = 0xAB;
    msg.data[1] = 0xCD;
    fdcan1.tryToSendReturnStatusFD(msg);
    Serial.println("loading");
    CANFDMessage rcvMsg;
    if (fdcan1.receiveFD0(rcvMsg)) {
        Serial.print("CAN RX: ID=");
        Serial.print(rcvMsg.id, HEX);
        Serial.print(" Data=");
        for (uint8_t i = 0; i < rcvMsg.len; i++) {
            Serial.print(rcvMsg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}