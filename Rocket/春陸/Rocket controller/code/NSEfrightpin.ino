#include <Arduino.h>
#include <SoftwareSerial.h>
#include<Adafruit_BMP085.h>
#include<Wire.h>
#include <Servo.h>
#include <SD.h>
Adafruit_BMP085 bmp;
Servo firstsv;
Servo secondsv;

SoftwareSerial IM920Serial(16,7);
//SoftwareSerial arduinoserial(8,9); // RX, TX

#define threshold 0.5
#define SAMPLENUM 20
#define openangle 125
#define setangle 175
#define sikiiti 3.00  //high
#define svpin   14
#define settime  11000 //解放機構が開くまでの時間
//#define LED 

int fpstate=0;
int Fpstate;
int fpStateArray[SAMPLENUM];
int state=0;
int mode=0;
int starttime,nowtime;
int Time;
float maxal=0;
float al,Al; 
float temp,pre;
float altitudeArray[SAMPLENUM];
int coord;
//File myFile;

float getFilteredAltitude (float rawAltitude) {
 altitudeArray[0] = rawAltitude;
	for (int i = (SAMPLENUM - 1); i > 0; i--) {
		altitudeArray[i] = altitudeArray[i - 1];
	}
	float filteredAltitude = calcMedian(altitudeArray, SAMPLENUM, 1);
	return filteredAltitude;
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
  // digitalWrite(15,HIGH);
  // delay(10);
  // digitalWrite(15,LOW);
  int pinmode=0;
  int i = digitalRead(4);
  if(i==1){
    //digitalWrite(14,HIGH);
    pinmode=0;
  }else if(i==0){
    //digitalWrite(15,HIGH);
    //digitalWrite(14,LOW);
    pinmode=1;
  }
  
  // float f = i * 3.3 / 1023.0;
  // if (f> threshold) {
  //   if(pinmode != 0){
  //      pinmode=0;
  //      digitalWrite(15,HIGH);
  //   }
  // }else{
  //   if(pinmode != 1){
  //     pinmode = 1;
  //     //digitalWrite(15,HIGH);
    
  //   }
  // }
  return (uint8_t)pinmode;
}

int isLaunched(int FlighPinState) {
  //Serial.print("b:");
  //Serial.println(FlighPinState);
  //Serial.print("array");
  fpStateArray[0] = FlighPinState;
	for (int i = (SAMPLENUM - 1); i > 0; i--) {
		fpStateArray[i] = fpStateArray[i - 1];
    //Serial.print(fpStateArray[i]);
	}
  //Serial.println("a");
  fpStateArray[0] = FlighPinState;
	if (calcMedian(fpStateArray, SAMPLENUM, 0) == 1) { //launched
		return 1;
	} else {
		return 0;
	}
}



void setup() {
  Wire.begin();
  bmp.begin();
  IM920Serial.begin(19200);
  Serial.begin(9600);
  IM920Serial.print("sbrt 3\r\n");
  IM920Serial.end();
  IM920Serial.begin(9600);
  //firstsv.attach(svpin);
  secondsv.attach(15);
  //pinMode( 15, OUTPUT );
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  pinMode(4,INPUT);
  // digitalWrite(15,HIGH);
  // delay(1000);
  // digitalWrite(15,LOW);
  //pinMode(LED,OUTPUT);
  //digitalWrite(LED,HIGH);
  //firstsv.write(setangle);
  secondsv.write(setangle);
  pinMode(3,OUTPUT);
  digitalWrite(3,LOW);

}

void loop() {
  //Serial.println(mode);
  fpstate = fp();
  //Serial.print("fpstate");
  //Serial.println(fpstate);
  // if(fpstate==0){
  //   digitalWrite(15,HIGH);
  //   //pinmode=0;
  // }else if(fpstate==1){
  //   digitalWrite(14,HIGH);
  //   digitalWrite(15,LOW);
  //   //pinmode=1;
  // }
  //arduinoserial.write(fpstate);
  Fpstate = isLaunched(fpstate);
  //Serial.print("Fpstate");
  //Serial.println(mode);
  temp= bmp.readTemperature();
  pre = bmp.readPressure();
  al = bmp.readAltitude();
  Al = getFilteredAltitude(al);
  //BMPSDwrite(pre,temp,al);
 //Serial.println(Al);
  //Serial.println(maxal);
  IM920Serial.print("TXDA ");
  IM920Serial.print(Fpstate);
  //IM920Serial.print(":");
  //IM920Serial.print(lng);
  IM920Serial.print("\r\n");
  if (IM920Serial.available()>0) {   
    coord = IM920Serial.read();  
    //IM920Serial.print("ECIO \r\n");
    IM920Serial.print("TXDA ");
    IM920Serial.print("c:");
    IM920Serial.print(coord);
    Serial.print("coord");
    Serial.println(coord);
    IM920Serial.print("\r\n");
  }

  switch (mode){
    case 0:
    //firstsv.write(setangle);
    //secondsv.write(setangle);
    if(Fpstate ==1){
      //digitalWrite(14,HIGH);
      //digitalWrite(15,LOW);
      starttime =millis();
      //Serial.println("launched");
      mode++;
    }
    break;
    case 1:
    digitalWrite(3,HIGH);
    nowtime =millis();
    Time=nowtime-starttime;
    //Serial.println(Time);
    if(Time>settime){
      digitalWrite(14,HIGH);
      //firstsv.write(openangle);
      secondsv.write(openangle);
      mode++;
      IM920Serial.print("TXDA ");
      IM920Serial.print("servo");
      IM920Serial.print("\r\n");
    }
   break;
   case 2:
   //digitalWrite( 15, HIGH );
   nowtime =millis();
   Serial.println(nowtime);
   break;
  }
}


