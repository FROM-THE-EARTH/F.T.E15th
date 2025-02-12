#include <TinyGPS++.h>
#include <Wire.h>
TinyGPSPlus gps;
char gpsBuffer;
int Lat, Lng;

// sentence : GPS関連情報をまとめたNMEAセンテンス
// lat : 緯度を受け取る変数 (参照渡し)
// lng : 経度を受け取る変数 (同上)

void requestEvent() {
  Wire.write(Lat); // respond with message of 6 bytes
  Wire.write(Lng)
  // as expected by master
}

void setup() {
  Serial.begin(9600);
  Wire.begin(8);               // join I2C bus with address #8 
  Wire.onRequest(requestEvent);
}

void loop() {
     if (Serial.available()>0) {
    Serial.write(Serial.read());
    gpsBuffer = Serial.read(); 
    gps.encode(gpsBuffer);
    if (gps.location.isUpdated()){
      Lat=gps.location.lat()
      Lng=gps.location.lng()
    }
    
}
}