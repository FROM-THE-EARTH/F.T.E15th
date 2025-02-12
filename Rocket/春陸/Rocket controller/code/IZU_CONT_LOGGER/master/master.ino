#include <Wire.h>

void setup() {
  Wire.begin(); // join I2C bus (address optional for master)
}

byte x = 3;

void loop() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("x is ");        // sends five bytes
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  delay(500);
}
