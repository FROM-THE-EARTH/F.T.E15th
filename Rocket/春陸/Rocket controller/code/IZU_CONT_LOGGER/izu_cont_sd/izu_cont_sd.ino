#include <Wire.h>

//アドレス指定
#define BME280_ADDR 0x76
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define CTRL_HUM 0xF2

//気温補正データ
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
 
//湿度補正データ
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
 
//気圧補正データ
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

unsigned char dac[26];
unsigned int i;

int32_t t_fine;
int32_t adc_P, adc_T, adc_H;

void setup() {
  //シリアル通信初期化
  Serial.begin(9600);//シリアル通信を9600bpsで初期化

  //I2C初期化
  Wire.begin();//I2Cを初期化
  Wire.requestFrom(8, 6);
  //BME280動作設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CONFIG);//動作設定
  Wire.write(0x00);//「単発測定」、「フィルタなし」、「SPI 4線式」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  //BME280測定条件設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_MEAS);//測定条件設定
  Wire.write(0x24);//「温度・気圧オーバーサンプリングx1」、「スリープモード」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  //BME280温度測定条件設定
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_HUM);//湿度測定条件設定
  Wire.write(0x01);//「湿度オーバーサンプリングx1」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  //BME280補正データ取得
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x88);//出力データバイトを「補正データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  
  Wire.requestFrom(BME280_ADDR, 26);//I2Cデバイス「BME280」に26Byteのデータ要求
  for (i=0; i<26; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  dig_T1 = ((uint16_t)((dac[1] << 8) | dac[0]));
  dig_T2 = ((int16_t)((dac[3] << 8) | dac[2]));
  dig_T3 = ((int16_t)((dac[5] << 8) | dac[4]));

  dig_P1 = ((uint16_t)((dac[7] << 8) | dac[6]));
  dig_P2 = ((int16_t)((dac[9] << 8) | dac[8]));
  dig_P3 = ((int16_t)((dac[11] << 8) | dac[10]));
  dig_P4 = ((int16_t)((dac[13] << 8) | dac[12]));
  dig_P5 = ((int16_t)((dac[15] << 8) | dac[14]));
  dig_P6 = ((int16_t)((dac[17] << 8) | dac[16]));
  dig_P7 = ((int16_t)((dac[19] << 8) | dac[18]));
  dig_P8 = ((int16_t)((dac[21] << 8) | dac[20]));
  dig_P9 = ((int16_t)((dac[23] << 8) | dac[22]));

  dig_H1 = ((uint8_t)(dac[25]));

  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0xE1);//出力データバイトを「補正データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  
  Wire.requestFrom(BME280_ADDR, 7);//I2Cデバイス「BME280」に7Byteのデータ要求
  for (i=0; i<7; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  dig_H2 = ((int16_t)((dac[1] << 8) | dac[0]));
  dig_H3 = ((uint8_t)(dac[2]));
  dig_H4 = ((int16_t)((dac[3] << 4) + (dac[4] & 0x0F)));
  dig_H5 = ((int16_t)((dac[5] << 4) + ((dac[4] >> 4) & 0x0F)));
  dig_H6 = ((int8_t)dac[6]);
  
  delay(1000);//1000msec待機(1秒待機)
}
 
void loop() {

  int32_t  temp_cal;
  uint32_t humi_cal, pres_cal;
  float temp, humi, pres;

  //BME280測定条件設定(1回測定後、スリープモード)
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(CTRL_MEAS);//測定条件設定
  Wire.write(0x25);//「温度・気圧オーバーサンプリングx1」、「1回測定後、スリープモード」
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  delay(10);//10msec待機

  //測定データ取得
  Wire.beginTransmission(BME280_ADDR);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0xF7);//出力データバイトを「気圧データ」のアドレスに指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了
  
  Wire.requestFrom(BME280_ADDR, 8);//I2Cデバイス「BME280」に8Byteのデータ要求
  for (i=0; i<8; i++){
    while (Wire.available() == 0 ){}
    dac[i] = Wire.read();//dacにI2Cデバイス「BME280」のデータ読み込み
  }
  
  adc_P = ((uint32_t)dac[0] << 12) | ((uint32_t)dac[1] << 4) | ((dac[2] >> 4) & 0x0F);
  adc_T = ((uint32_t)dac[3] << 12) | ((uint32_t)dac[4] << 4) | ((dac[5] >> 4) & 0x0F);
  adc_H = ((uint32_t)dac[6] << 8) | ((uint32_t)dac[7]);
  
  pres_cal = BME280_compensate_P_int32(adc_P);//気圧データ補正計算
  temp_cal = BME280_compensate_T_int32(adc_T);//温度データ補正計算
  humi_cal = bme280_compensate_H_int32(adc_H);//湿度データ補正計算

  pres = (float)pres_cal / 100.0;//気圧データを実際の値に計算
  temp = (float)temp_cal / 100.0;//温度データを実際の値に計算
  humi = (float)humi_cal / 1024.0;//湿度データを実際の値に計算

  //シリアルモニタ送信
  Serial.print("Pressure:");//文字列「Pressure:」をシリアルモニタに送信
  Serial.print(pres);//「pres」をシリアルモニタに送信
  Serial.print("hPa ");//文字列「hPa 」をシリアルモニタに送信
  Serial.print("Temp:");//文字列「Temp:」をシリアルモニタに送信
  Serial.print(temp);//「temp」をシリアルモニタに送信
  Serial.print("°C ");//文字列「°C 」をシリアルモニタに送信
  Serial.print("Humidity:");//文字列「Humidity:」をシリアルモニタに送信
  Serial.print(humi);//「humi」をシリアルモニタに送信
  Serial.println("%");//文字列「%」をシリアルモニタに送信、改行
  
  delay(1000);//1000msec待機(1秒待機)
}

//温度補正 関数
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T  = (t_fine * 5 + 128) >> 8;
  return T;
}

//湿度補正 関数
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800)); 
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r>>12);
}

//気圧補正 関数
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}