#include <Wire.h>
#include "MICS-VZ-89TE.h"

MICS_VZ_89TE sensor;
#define AD7746_ad 0x48
#define MICS_ad 0x70

void loop() {

    delay(1000);
    sensor.readSensor();
    float co2 = sensor.getCO2();
    float voc = sensor.getVOC();
  
    Serial.print("CO2 equ(ppm): ");
    Serial.println(co2);
    Serial.print("VOC(isobutylene) equ(ppb): ");
    Serial.println(voc);

    Serial.println();
}
