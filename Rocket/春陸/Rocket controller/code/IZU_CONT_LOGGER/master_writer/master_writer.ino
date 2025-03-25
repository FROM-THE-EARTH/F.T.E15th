// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <math.h>
byte sign;
byte pos = 1;
byte neg = 0;

float presff,presfl;
void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}


void loop() {

float pres=-31.34;
if (pres<0){
  pres=pres*(-1);
  byte sign = neg;
}else{
  byte sign = pos;
}

presfl=modff(pres,&presff);
int presif = (int)pres;

int amari = presif%255;
int Presif = presif - amari;
int shou = Presif/255;

int a = 255*shou+amari;
Serial.println(a);

presfl=presfl*100+0.11;
int presil = (int)presfl;

char amaric = (char)amari;
char shouc = (char)shou;
char prescl = (char)presil;
Wire.beginTransmission(8); // transmit to device #8    
Wire.write(sign);
Wire.write(amaric);
Wire.write(shouc);
Wire.write(prescl);  
Wire.endTransmission();    // stop transmitting
delay(500);
}
