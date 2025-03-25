#include<Adafruit_BME280.h>
#include <Wire.h>
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

float alt=0;
float pres=0;
float maxalt=0;
float dalt=0;
int count=0;

Servo myservo;
const int servo=5;
void setup()
{ 
  myservo.attach(servo,500,2400);
  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);
  Serial.begin(9600);
  bool status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 sensor");
    while (10);
  }
    int delayTime = 1000;
    int waittime=120*60*1000;
    digitalWrite(16,HIGH);
    delay(waittime);
}

void loop()
{count++;
  float temperature = bme.readTemperature();
  float barometric = bme.readPressure() / 100.0F;
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();
if (count==1){
  maxalt=alt;
}else{
  if (alt>maxalt){
    maxalt=alt;

  }
}
Serial.println(maxalt-alt);
float a = maxalt-alt;
if (a<1){
  digitalWrite(16,HIGH);
} else{
  digitalWrite(16,LOW);
}
if ((maxalt-alt)>10){
  Serial.println("open");
  myservo.write(180);
delay(1000);
myservo.write(105);
delay(3000);
myservo.write(180);
delay(1000);
}
delay(1000);
}