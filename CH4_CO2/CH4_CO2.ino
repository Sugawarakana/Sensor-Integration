#include <Wire.h>
#include "MICS-VZ-89TE.h"

MICS_VZ_89TE sensor;
#define AD7746_ad 0x48
#define MICS_ad 0x70
#define MUL_ad 0x71
const int analogInPin = A0; 
int cnt = 0;

void tca_select(uint8_t i) {
  Wire.beginTransmission(MUL_ad);
  Wire.write(1<<i);
  Wire.endTransmission();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  // sensor.begin() is also wire.begin, confliction if both works
  pinMode(analogInPin, INPUT_ANALOG);

  analogReadResolution(12);

}

void loop() {
  // delay(1000);
  tca_select(0);
  delay(100);
  sensor.readSensor();
  float co2 = sensor.getCO2();
  float voc = sensor.getVOC();
  uint32_t status = sensor.getStatus();
  uint32_t rev = sensor.getRev();

  
  tca_select(1);
  delay(100);
  sensor.readSensor();
  co2 = sensor.getCO2();
  voc = sensor.getVOC();
  status = sensor.getStatus();
  rev = sensor.getRev();


  Serial.print("Status: ");
  Serial.println(status);
  Serial.print("Status1: ");
  Serial.println(status);

  Serial.print("CO2 equ(ppm): ");
  Serial.println(co2);
  Serial.print("VOC(isobutylene) equ(ppb): ");
  Serial.println(voc);
  Serial.print("CO21 equ(ppm): ");
  Serial.println(co2);
  Serial.print("VOC1(isobutylene) equ(ppb): ");
  Serial.println(voc);
  Serial.println();

  int sensorValue = analogRead(analogInPin);

  Serial.print("Raw ADC Value: ");
  Serial.println(sensorValue);


  float voltage = sensorValue * (3.3 / 4095.0);
  Serial.print("Voltage: ");
  Serial.print(voltage, 3); 
  Serial.println(" V");

  delay(1000); 
}
