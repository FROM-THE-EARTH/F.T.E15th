#include <SoftwareSerial.h>
SoftwareSerial IM920Serial(2,3);
void setup() {
  IM920Serial.begin(19200);
  digitalWrite(5,HIGH);
  Serial.begin(9600);
}
void loop(){
  IM920Serial.print("TXDA ");
  IM920Serial.print(123456);
  IM920Serial.print("\r\n");
  delay(1000);
}


