#include <SPI.h> 
#include<Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial IM920Serial(2,3);

unsigned long mtime;
float time;
float a = 1000.00;

void setup() {
Wire.begin(8);                // join i2c bus with address #8
Serial.begin(9600);
pinMode(5,OUTPUT);
digitalWrite(5,HIGH);
pinMode(4,OUTPUT);
digitalWrite(4,HIGH);
IM920Serial.begin(19200);
Wire.onReceive(receiveEvent); // register event

}

void loop() {
  // put your main code here, to run repeatedl
delay(100);
}

void receiveEvent(int howMany) {
  mtime = millis();
  time=mtime/a;

  int lng = Wire.read();
  int lat = Wire.read();

  Serial.print(time);
  Serial.print(":");


  Serial.print("Lng:");
  Serial.print(lng);
  Serial.print("Lat:");
  Serial.println(lat);

  IM920Serial.print("TXDA ");
  IM920Serial.print(time);
  IM920Serial.print(":");
  IM920Serial.print("(");
  IM920Serial.print(lng);
  IM920Serial.print(",");
  IM920Serial.print(lat);
  IM920Serial.print(")");
  IM920Serial.print("\r\n");
}