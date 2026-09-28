#include <arduino.h>

void setup() {
  Serial.begin(9600);
  
  // Optional: Change ADC resolution if needed (Default is 10-bit: 0-1023)
  //analogReadResolution(12); // Sets resolution to 12-bit (0-4095)
}

void loop() {
  // You can use either 23 or A9
  int adcValue = analogRead(23); 
  
  // Convert the raw ADC value to voltage (Assuming a 3.3V reference)
  float voltage = adcValue * (3.3 / 1023.0); 

  Serial.print("Raw ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" | Voltage: ");
  Serial.println(voltage);

  delay(250); // Read 4 times a second
}
