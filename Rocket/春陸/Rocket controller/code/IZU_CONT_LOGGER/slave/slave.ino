#include <Wire.h>

void setup() {
  Wire.begin(8);                // join I2C bus with address #8
  Serial.begin(9600);           // start serial for output
}

void loop() {
  Serial.println(Wire.available());
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}

