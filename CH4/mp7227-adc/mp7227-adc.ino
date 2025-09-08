
const int analogInPin = A0; 

void setup() {

  Serial.begin(115200);

  pinMode(analogInPin, INPUT_ANALOG);

  analogReadResolution(12);


}

void loop() {

  int sensorValue = analogRead(analogInPin);

  Serial.print("Raw ADC Value: ");
  Serial.println(sensorValue);


  float voltage = sensorValue * (3.3 / 4095.0);
  Serial.print("Voltage: ");
  Serial.print(voltage, 3); 
  Serial.println(" V");

  delay(500); 
}