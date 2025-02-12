#include <TinyGPS++.h>
TinyGPSPlus gps;
char gpsBuffer;
int Lat, Lng;
// sentence : GPS関連情報をまとめたNMEAセンテンス
// lat : 緯度を受け取る変数 (参照渡し)
// lng : 経度を受け取る変数 (同上)


void setup() {
  Serial.begin(9600);
}
void loop() {
    Serial.println(Serial.available());
     if (Serial.available()>0) {
    Serial.write(Serial.read());
    gpsBuffer = Serial.read(); 
    gps.encode(gpsBuffer);
    if (gps.location.isUpdated()){
      Serial.println(gps.location.lat(),6);
      Serial.println(gps.location.lng(),6);
    }
    
}
}