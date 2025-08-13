#include <Wire.h>
#include <SoftwareSerial.h>
#include<Adafruit_BME280.h>
#include <math.h>
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

#define SAMPLENUM 20
#define settime 3600000.

int fpstate=0;
int Fpstate;
int fpStateArray[SAMPLENUM];
int state=0;
int mode=0;
int starttime,nowtime;
int Time;
float maxalt=0;

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
char presamaric,tempamaric,altamaric;
char presshouc,tempshouc,altshouc;
char pressign,tempsign,altsign;

void setup() {
  pinMode(17,INPUT);
  Serial.begin(9600);//シリアル通信を9600bpsで初期化
  Wire.begin();//I2Cを初期化
  bool status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 sensor");
    while (10);
  }
  GpsSerial.begin(9600);
  delay(1000);//1000msec待機(1秒待機)
 
}
 
void loop() {
  float temp,pres,alt;

  pres =  bme.readPressure() / 100.0F;//気圧データを実際の値に計算
  temp = bme.readTemperature();//温度データを実際の値に計算
  alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  if (alt>maxalt){
    maxalt=alt;
  }

  //Serial.print("Pressure:");//文字列「Pressure:」をシリアルモニタに送信
  //Serial.print(pres);//「pres」をシリアルモニタに送信
  //Serial.println("hPa ");//文字列「hPa 」をシリアルモニタに送信
  //Serial.print("Temp:");//文字列「Temp:」をシリアルモニタに送信
  //Serial.print(temp);//「temp」をシリアルモニタに送信
  //Serial.print("°C ");//文字列「°C 」をシリアルモニタに送信
  //Serial.print("Altitude:");
    Serial.print(alt);
  //Serial.println("m");


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
  fpstate = fp();
  Fpstate = isLaunched(fpstate);

  switch (mode){
    case 0:
    if(Fpstate ==1){
      starttime =millis();
      //mode++;
    }
    break;
    case 1:
    nowtime =millis();
    Time=nowtime-starttime;
    Serial.println(Time);
    if(Time>settime||maxalt-alt>10){
      //firstsv.write(openangle);
      //secondsv.write(openangle);
      mode++;
    }
   break;
   case 2:
   nowtime =millis();
   Serial.println(nowtime);
   break;
  }
  //masterにデータを送る
  //float,intのまま送れないのでcharに変換する
    char prescl=float2char(pres,&presamaric,&presshouc,&pressign);
    char tempcl=float2char(temp,&tempamaric,&tempshouc,&tempsign);
    char altcl=float2char(alt,&altamaric,&altshouc,&altsign);
    char lngc = (char)lng;
    char latc =(char)lat;
    char modec =(char)mode;

    Wire.beginTransmission(8); // transmit to device #8

    Wire.write(pressign);
    Wire.write(presamaric);
    Wire.write(presshouc);
    Wire.write(prescl);   

    Wire.write(tempsign);
    Wire.write(tempamaric);
    Wire.write(tempshouc);
    Wire.write(tempcl); 

    Wire.write(altsign);
    Wire.write(altamaric);
    Wire.write(altshouc);
    Wire.write(altcl); 

    Wire.write(lngc);
    Wire.write(latc);
    
    Wire.write(modec);
    
    Wire.endTransmission();    // stop transmitting
    Serial.println("done");
    
    delay(1000);
    
  }

float calcMedian(void *array, int n, int type) {
  if (type == 0) { // If data type is int
    int *intArray = (int*) array;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (intArray[i] > intArray[j]) {
          int changer = intArray[j];
          intArray[j] = intArray[i];
          intArray[i] = changer;
        }
      }
    }
    if (n % 2 == 0) {
      return (float) (intArray[n / 2] + intArray[n / 2 - 1]) / 2;
    } else {
      return (float) intArray[n / 2];
    }
  } else if (type == 1) { // If data type is float
    float *floatArray = (float*) array;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (floatArray[i] > floatArray[j]) {
          float changer = floatArray[j];
          floatArray[j] = floatArray[i];
          floatArray[i] = changer;
        }
      }
    }
    if (n % 2 == 0) {
      return (floatArray[n / 2] + floatArray[n / 2 - 1]) / 2;
    } else {
      return floatArray[n / 2];
    }
  } else {
    // Error or unknown data type
    return 0.0;
  }
}

uint8_t fp (){
  int pinmode=0;
  int i = digitalRead(17);
  if(i==1){
    pinmode=0;
  }else if(i==0){
    pinmode=1;
  }
  return (uint8_t)pinmode;
}

int isLaunched(int FlighPinState) {
  fpStateArray[0] = FlighPinState;
	for (int i = (SAMPLENUM - 1); i > 0; i--) {
		fpStateArray[i] = fpStateArray[i - 1];
	}
  fpStateArray[0] = FlighPinState;
	if (calcMedian(fpStateArray, SAMPLENUM, 0) == 1) { //launched
		return 1;
	} else {
		return 0;
	}
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