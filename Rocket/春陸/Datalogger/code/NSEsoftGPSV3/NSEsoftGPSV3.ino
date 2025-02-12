#include <SoftwareSerial.h>
// SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
SoftwareSerial GpsSerial(6,16); // 受信, 送信
float lat_buff, lng_buff; // 緯度, 経度
char gpsBuffer; // gpsから送られてくる文字データ (バッファ)
String gpsSentence = ""; // gpsの文字列データ
int Lat,Lng;
// sentence : GPS関連情報をまとめたNMEAセンテンス
// lat : 緯度を受け取る変数 (参照渡し)
// lng : 経度を受け取る変数 (同上)
bool ParseNMESentence(String sentence, float& lat, float& lng) {
    lat = 0; lng = 0;
    String datatype = "$GNGGA"; // 取得するデータタイプ
    // String datatype = "$GPRMC"; // for RMC
    // 緯度・経度が何番目の要素か. データタイプが0番目
    int idxLat = 2;
    int idxLng = 4;
    // int idxLat = 3; // for RMC
    // int idxLng = 5;
    if (!sentence.startsWith(datatype)) { // not found
        return false;
    }
    int idx = sentence.indexOf(","); // カンマのある位置を探す
    int counter = 0; // 何番目の要素かを格納
    do {
        counter ++;
        // 次のカンマのある位置を探す
        int newIdx = sentence.indexOf(",", idx+1);
        if (counter == idxLat) {
            String strLat = sentence.substring(idx+1, newIdx-1); // 要素取り出し
            String strDeg = strLat.substring(0, 2); // 先頭取り出し
            String strMin = strLat.substring(2); // 末尾
            lat = (float)strDeg.toInt() + strMin.toFloat() / 60; //10進数に変換
        }
        else if (counter == idxLng) {
            String strLng = sentence.substring(idx+1, newIdx-1); // 要素取り出し
            String strDeg = strLng.substring(0, 3); // 先頭取り出し
            String strMin = strLng.substring(3); // 末尾
            lng = (float)strDeg.toInt() + strMin.toFloat() / 60; //10進数に変換
        }
        idx = newIdx;
    } while(idx >= 0);
    return true;
}
void setup() {
  Serial.begin(9600);
  GpsSerial.begin(9600);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
}
void loop() {
  if (GpsSerial.available()) {
    Serial.write(Serial.read());
    gpsBuffer = Serial.read(); // get data as character
    if (gpsBuffer == '\n') {
//Serial.println(61);
      if (ParseNMESentence(gpsSentence, lat_buff, lng_buff)){
        float lt = (lat_buff-40.2)*10000;
        float lg = (lng_buff-140)*10000;
        Lat = (int)lt;
        Lng = (int)lg;
        // Serial.println(lat);
        //Serial.println(lng);
        Serial.println(Lat);
        Serial.println(Lng);
        if(Lat>0&&Lat<1000){
        // Serial.println(Lat);
        // Serial.println(Lng);
        GpsSerial.write(Lat);
        GpsSerial.write(Lng);
        }
      }
      else
      //Serial.println("failed to read gps sentence");
      gpsSentence = "";
    } else{
      gpsSentence.concat(gpsBuffer);
    }
  }
  //GpsSerial.write(Lat);
  //GpsSerial.write(Lng);
}