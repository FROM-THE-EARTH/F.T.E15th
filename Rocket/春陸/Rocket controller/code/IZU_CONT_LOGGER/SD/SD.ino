#include <SPI.h> 
#include <SD.h>
#include<Wire.h>

File myFile;
void setup() {
Wire.begin();
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly
Serial.print(SD.begin(10));
myFile = SD.open("test.txt", FILE_WRITE);
if (myFile) {
  //時間書き込み
Serial.println(1229);
myFile.println(1229);
myFile.close();

}else{
  Serial.print("writng error");
}
}