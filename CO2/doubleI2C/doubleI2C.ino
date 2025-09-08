#include <Wire.h>
// #include "AD7746.h"
#include "MICS-VZ-89TE.h"

MICS_VZ_89TE sensor;
#define AD7746_ad 0x48
#define MICS_ad 0x70
#define TCA9548A_ADDR 0x71 // Address of multiplexer
int cnt = 0;
// AD7746 capSensor0(&Wire);  // I2C0
void tca_select(uint8_t bus) {
    // Serial.println("tca_select called");
    if (bus > 7) {
        Serial.println("bus > 7, return");
        return;
    }
    // Serial.println("Wire.beginTransmission...");
    Wire.beginTransmission(TCA9548A_ADDR);
    // Serial.println("Wire.write...");
    Wire.write(1 << bus);
    Wire.endTransmission();

}
// void sensor0_init() {
//     capSensor0.initialize();
//     capSensor0.reset();
//     delay(20);

// //     // Read factory offset and apply it
//     uint32_t offset = capSensor0.getCapacitance();
//     Serial.print("Factory Offset: ");
//     Serial.println(offset);

// //     // Configure Excitation Voltage to VDD/2 for stability
//     capSensor0.writeExcSetupRegister(0x0B);
//     // capSensor0.writeExcSetupRegister(0x03);
// //     Serial.println("Excitation Voltage Set to VDD/2");//vdd/4


// //     Serial.println("Capacitance Measurement Enabled");

// //     // Set Continuous Conversion Mode, conversion time to 109.6ms
//     capSensor0.writeConfigurationRegister(AD7746_CAPF_62P0 | AD7746_MD_CONTINUOUS_CONVERSION);
// //     Serial.println("Continuous Conversion Mode Enabled");

// //     // Configure Cap DAC to shift the reference to 8 pF (so we measure 4 - 12 pF)
//     uint8_t dacValue = 90;  // (8 pF / 0.133858 pF per unit) ≈ 60 12pF
//     capSensor0.writeCapDacARegister(dacValue | AD7746_DACAEN); // Enable Cap DAC A
// //     Serial.println("Cap DAC A Configured to 8 pF Offset");
// //     Serial.println("AD7746 Configuration Complete. Reading Capacitance...");
//     // capSensor0.writeCapSetupRegister(0);
//          // Enable capacitance measurement mode
//     capSensor0.writeCapSetupRegister(AD7746_CAPEN);
//     // capSensor0.writeCapSetupRegister(0);
    
//     // uint32_t s = capSensor0.readStatusRegister();
//     // Serial.println(s);
// }

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();
    // delay(500);
    // Serial.println("I2C Scanner. Scanning...");
    // for (byte address = 1; address < 127; address++) {
    //     Wire.beginTransmission(address);
    //     if (Wire.endTransmission() == 0) {
    //     Serial.print("Found device at 0x");
    //     Serial.println(address, HEX);
    //     }
    // }
    // Serial.println("Scan complete.");

    // sensor0_init();
    tca_select(0);
    sensor.begin();
    // delay(1000);

    // tca_select(1);

}

void loop() {
  // put your main code here, to run repeatedly:
    delay(1000);
    tca_select(0);
    delay(1000);
    sensor.readSensor();
    float co2 = sensor.getCO2();
    float voc = sensor.getVOC();
    tca_select(1);
    // delay(1000);
    sensor.readSensor();
    float co21 = sensor.getCO2();
    float voc1 = sensor.getVOC();
    // float s = sensor.getRS();
    // int month = sensor.getMonth();
    // int day = sensor.getDay();
    // uint32_t temp = capSensor0.getCapacitance();
    // float r = (float) temp * (4.00 / 0x800000) + 8.00;
    // Serial.print("Date: ");
    // Serial.print(year);
    // Serial.print(month);
    // Serial.println(day);
    Serial.print("CO2 equ(ppm): ");
    Serial.println(co2);
    Serial.print("CO21 equ(ppm): ");
    Serial.println(co21);
    Serial.print("VOC(isobutylene) equ(ppb): ");
    Serial.println(voc);
    Serial.print("VOC1(isobutylene) equ(ppb): ");
    Serial.println(voc1);
    // sensor.readRegister(uint32_t 0);
    // uint8_t s = sensor.readAD(0);
    // Serial.println(s);
    Serial.println();
}
