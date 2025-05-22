#include <SPI.h> 
#include <SD.h>
#include<Wire.h>

char a[3]="3.14";
File myFile;
void setup() {
Wire.begin();
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly
SD.begin(10);
myFile = SD.open("test.txt", FILE_WRITE);
if (myFile) {
  //時間書き込み
Serial.write(a);
Serial.println();
myFile.write(a);
myFile.println();
myFile.close();

}else{
  Serial.print("writng error");
}
} 