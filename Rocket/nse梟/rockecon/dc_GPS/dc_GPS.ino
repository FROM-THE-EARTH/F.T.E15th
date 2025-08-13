#define SAMPLENUM 20
#define settime 5000
#include <SoftwareSerial.h>
#include <Wire.h>
int AIN1=5;
int AIN2=3;
int count=0;
int fpstate=0;
int Fpstate=1;
int fpStateArray[SAMPLENUM];
int state=0;
int alt_flag=0;
bool status;
int starttime,nowtime;
int Time;
int mode=0;

int lng=0;
int lat=0;

SoftwareSerial GpsSerial(0,1); // 受信, 送信
float lat_buff, lng_buff; // 緯度, 経度
char gpsBuffer; // gpsから送られてくる文字データ (バッファ)
String gpsSentence = ""; // gpsの文字列データ
int Lat,Lng;

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
		return 0;
	} else {
		return 1;
	}
}
void setup()
{ 
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(14,INPUT);
  pinMode(17,INPUT);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  Serial.begin(9600);
  GpsSerial.begin(9600);
  Wire.begin(8);
  Wire.onRequest(requestEvent);
  delay(1000);
  
}

void loop()
{

}
void requestEvent(){
fpstate = fp();
Fpstate = isLaunched(fpstate);
switch(mode){
case 0: 
if (Fpstate==0){
starttime=millis();
mode=1;
}
break;
case 1:
if(Fpstate==1){
  mode=0;
  break;
}
nowtime=millis();
Time=nowtime-starttime;
alt_flag=digitalRead(14);
if(alt_flag==1||Time>settime){
  Serial.print("open");
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  mode=2;
}
break;
}

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
  Wire.write(lng);
  Wire.write(lat);
  Wire.write(mode);
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
