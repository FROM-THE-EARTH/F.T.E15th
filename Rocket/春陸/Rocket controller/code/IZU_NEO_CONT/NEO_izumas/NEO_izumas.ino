#include <Wire.h>
#include <SoftwareSerial.h>
int lng=0;
int lat=0;

SoftwareSerial GpsSerial(0,1); // 受信, 送信
float lat_buff, lng_buff; // 緯度, 経度
char gpsBuffer; // gpsから送られてくる文字データ (バッファ)
String gpsSentence = ""; // gpsの文字列データ
int Lat,Lng;
// sentence : GPS関連情報をまとめたNMEAセンテンス
// lat : 緯度を受け取る変数 (参照渡し)
// lng : 経度を受け取る変数 (同上)


//マスターにデータを送るための変数
float valuefl=0;
float valueff=0;

void setup() {
  pinMode(17,INPUT);
  pinMode(4,OUTPUT);
  Serial.begin(9600);//シリアル通信を9600bpsで初期化
  Wire.begin();//I2Cを初期化
  GpsSerial.begin(9600);
  delay(1000);//1000msec待機(1秒待機)
 
}
 
void loop() {


  if (GpsSerial.available()) {
    gpsBuffer = GpsSerial.read(); // get data as character
    if (gpsBuffer == '\n') {
      if (ParseNMESentence(gpsSentence, lat_buff, lng_buff)){
        } else{
        Serial.println("failed to read gps sentence");
        gpsSentence = "";
       }
    } else{
      gpsSentence.concat(gpsBuffer);
    }
  }
  
  //masterにデータを送る
  //float,intのまま送れないのでcharに変換する
    Serial.print(lng);
    Serial.print(",");
    Serial.println(lat);
    char lngc = (char)lng;
    char latc =(char)lat;

    Wire.beginTransmission(8); // transmit to device #8

    Wire.write(lngc);
    Wire.write(latc);
    
    Wire.endTransmission();    // stop transmitting
    Serial.println("done");
    
    delay(1000);
    
  }

bool ParseNMESentence(String sentence, float& lat, float& lng) {
    //Serial.println("o");
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
    delay(100);
    return true;

}

char float2char(float value,char *amaric,char *shouc, char *sign)
{ 
  if (value<0){
    *sign="-";
  }else{
    *sign="";
  }
  value=abs(value);
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
