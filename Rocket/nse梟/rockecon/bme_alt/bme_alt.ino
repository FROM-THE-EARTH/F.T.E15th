#include <BME280I2C.h>
#include <Wire.h>
#define settime 11000
float alt=0;
float pres=0;
float maxalt=0;
float dalt=0;
float prealt=0;
int count=0;

const float SEA_LEVEL_PRESSURE_PA = 101325.0F; // 標準大気圧 (Pa)
const float CONSTANT_44330 = 44330.0F;
const float EXPONENT_FACTOR = 0.19029495F; // 1/5.255

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);

void setup() {
  pinMode(13,OUTPUT);
  pinMode(14,OUTPUT);
  Wire.begin();
  Serial.begin(9600);

   while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
   settings.tempOSR = BME280::OSR_X4;

   bme.setSettings(settings);


}

void loop() {
    float temp(NAN), hum(NAN), pres(NAN);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    bme.read(pres, temp, hum, tempUnit, presUnit);
    alt = CONSTANT_44330 * (1.0F - pow(pres/ SEA_LEVEL_PRESSURE_PA, EXPONENT_FACTOR));
    if(alt>1000){
      alt=prealt;
    }
    prealt=alt;
      if (alt>maxalt){
      maxalt=alt;

      } 

dalt=maxalt-alt;
//Serial.print("alt:");
//Serial.println(alt);
//Serial.print("maxalt:");
//Serial.println(maxalt);
Serial.print("dalt:");
Serial.println(dalt);

if (dalt>5){
count++;
Serial.println(count);
}
if (count==10){
  Serial.println("open");
  digitalWrite(14,HIGH);
  delay(5000);
}
}
