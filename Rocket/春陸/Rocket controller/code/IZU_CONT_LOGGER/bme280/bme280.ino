//BME280のライブラリーを取り込みます。
#include<Adafruit_BME280.h>

//1013.25は地球の海面上の大気圧の平均値(1気圧)です。
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
unsigned long delayTime; 

float temperature;//気温
float barometric;//気圧
float altitude;//高度
float humidity;//湿度

void setup() {
  Serial.begin(9600); 
  bool status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 sensor");
    while (10);
  }
    delayTime = 1000;
}
void loop() {
  temperature = bme.readTemperature();
  barometric = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  Serial.print("温度:");
  Serial.print(temperature);
  Serial.println("°C");
  Serial.print("気圧:");
  Serial.print(barometric);
  Serial.println("hpa");
  Serial.print("高度:");
  Serial.print(altitude);
  Serial.println("m");
  Serial.print("湿度:");
  Serial.print(humidity);
  Serial.println("%");
  Serial.println();
  delay(delayTime);
}