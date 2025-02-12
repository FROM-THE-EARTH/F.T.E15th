#include "Seeed_BME280.h"
#include <Wire.h>
#include <Servo.h>

BME280 bme280;
float alt=0;
float maxalt=0;
float dalt=0;
int count=0;

Servo myservo;
const int servo=5;
void setup()
{ 
  myservo.attach(servo,500,2400);
  Serial.begin(9600);
  delay(1000);
  Serial.print(1);
  if(!bme280.init()){
    Serial.println("Device error!");
  }
  Serial.print(2);
}

void loop()
{count++;
float pressure;
pressure = bme280.getPressure();
alt=bme280.calcAltitude(pressure);
if (count==1){
  maxalt=alt;
}else{
  if (alt>maxalt){
    maxalt=alt;
  }
}
Serial.printldn(maxalt-alt);
if ((maxalt-alt)>5){
  Serial.println("parachute open");
myservo.write(90);
delay(10000);
}
delay(1000);
}