#include <SPI.h> 
#include <SD.h>
#include<Wire.h>
File myFile;
float a = 3.14;
char b[24];

void setup() {
  Serial.begin(9600);
}

void loop() {
  dtostrf(a,-1,2,b);
  Serial.write(b);
  Serial.println();
}

char float2char(float value,char *amaric,char *shouc)
{
  valuefl=modff(value,&valueff);
  valuefl=valuefl*100+0.11;

  int valueif=(int)value;
  int amari=valueif%255;
  int Valueif =valueif-amari;
  int shou = Valueif/255;

  *amaric=(char)amari;
  *shouc=(char)shou;
  char valuecl = (char)valuefl;

  return valuecl;
} 
