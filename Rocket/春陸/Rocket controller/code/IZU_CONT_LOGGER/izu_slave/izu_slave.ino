#include <SPI.h> 
#include <SD.h>
#include<Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial IM920Serial(2,3);

unsigned long mtime;
float time;
float a = 1000.00;

File myFile;
void setup() {
Wire.begin(8);                // join i2c bus with address #8
Serial.begin(9600);
pinMode(5,OUTPUT);
digitalWrite(5,HIGH);
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

  int pressign = Wire.read();
  int presamari = Wire.read();   
  int presshou = Wire.read();
  int presl=Wire.read();
  int presf=255*presshou+presamari;
  if (pressign==0){
    presf=presf*(-1);
  }

  int tempsign = Wire.read();
  int tempamari = Wire.read();   
  int tempshou = Wire.read();
  int templ=Wire.read();
  int tempf=255*tempshou+tempamari;
  if (tempsign==0){
    tempf=tempf*(-1);
  }

  int altsign = Wire.read();
  int altamari = Wire.read();   
  int altshou = Wire.read();
  int altl=Wire.read();
  int altf=255*altshou+altamari;
  if (altsign==0){
    altf=altf*(-1);
  }
 
  int lng = Wire.read();
  int lat = Wire.read();

  int mode = Wire.read();
  

  Serial.print(time);
  Serial.print(":");
  Serial.print(presf);
  Serial.print(".");
  Serial.print(presl);
  Serial.print("hPa ");
  
  Serial.print(tempf);
  Serial.print(".");
  Serial.print(templ);
  Serial.print("C°");

  Serial.print(altf);
  Serial.print(".");
  Serial.print(altl);
  Serial.print("m");

  Serial.print("Lng:");
  Serial.print(lng);
  Serial.print("Lat:");
  Serial.print(lat);
  Serial.print("Mode:");
  Serial.println(mode);

  IM920Serial.print("TXDA ");
  IM920Serial.print(time);
  IM920Serial.print(":");
  IM920Serial.print("(");
  IM920Serial.print(lng);
  IM920Serial.print(",");
  IM920Serial.print(lat);
  IM920Serial.print(")");
  IM920Serial.print(",");
  

  switch (mode){
    case 0:
      IM920Serial.println("Not launched");
      IM920Serial.print("\r\n");
      break;
    case 1:
      IM920Serial.println("Launched!");
      IM920Serial.print("\r\n");
      break;
    case 2:
      IM920Serial.println("Parachute open!");
      IM920Serial.print("\r\n");
      break;
  }
  myFile = SD.open("log.txt", FILE_WRITE);
  SD.begin(10);
  if (myFile) {
    myFile.print(time);
    myFile.print(",");
    myFile.print(presf);
    myFile.print(".");
    myFile.print(presl);
    myFile.print(",");

    myFile.print(tempf);
    myFile.print(".");
    myFile.print(templ);
    myFile.print(",");

    myFile.print(altf);
    myFile.print(".");
    myFile.print(altl);
    myFile.print(",");

    myFile.print(lng);
    myFile.print(",");
    myFile.print(lat);
    myFile.println();
    myFile.close();
    Serial.println("done.");
  }else{
    Serial.println("writng error");
  }
}