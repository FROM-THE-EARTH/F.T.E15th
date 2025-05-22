#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include<Adafruit_BME280.h>
#include <SD.h>  // Include SD card library
#include <Servo.h>

Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// Create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
const int chipSelect = 10;  // MicroSD card chip select pin
unsigned long time = 0;

float maxalt=0;
float dalt=0;
int count=0;

Servo myservo;
const int servo=5;


void setup(void) {
  myservo.attach(servo,500,2400);
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for serial port to open
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  /* Initialize the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
 
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("SD card is ready to use.");
  delay(1000);
}
void loop(void) {
  count++;
  time = millis();
  // Create event objects for each sensor type
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  // Get data from the sensor
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
 
  float pres =  bme.readPressure() / 100.0F;//気圧データを実際の値に計算
  float temp = bme.readTemperature();//温度データを実際の値に計算
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  File dataFile = SD.open("TEST.txt", FILE_WRITE);
  // Check if the file is open
  if (dataFile) {
    Serial.println(1);
    // Write sensor data to the file
    dataFile.print(time);
    dataFile.print(",");
    //dataFile.print("Orientation: ");
    //dataFile.print(orientationData.orientation.x);
    //dataFile.print(",");
    //dataFile.print(orientationData.orientation.y);
    //dataFile.print(",");
    //dataFile.println(orientationData.orientation.z);
    /*dataFile.print("Gyroscope: ");
    dataFile.print(angVelocityData.gyro.x);
    dataFile.print("\t");
    dataFile.print(angVelocityData.gyro.y);
    dataFile.print("\t");
    dataFile.println(angVelocityData.gyro.z);
    dataFile.print("Linear Acceleration: ");
    dataFile.print(linearAccelData.acceleration.x);
    dataFile.print("\t");
    dataFile.print(linearAccelData.acceleration.y);
    dataFile.print("\t");
    dataFile.println(linearAccelData.acceleration.z);
    dataFile.print("Magnetometer: ");
    dataFile.print(magnetometerData.magnetic.x);
    dataFile.print("\t");
    dataFile.print(magnetometerData.magnetic.y);
    dataFile.print("\t");
    dataFile.println(magnetometerData.magnetic.z);*/
    //dataFile.print("Accelerometer: ");
    dataFile.print(accelerometerData.acceleration.x);
    dataFile.print(",");
    dataFile.print(accelerometerData.acceleration.y);
    dataFile.print(",");
    dataFile.print(accelerometerData.acceleration.z);
    dataFile.print(",");
    dataFile.print(temp);
    dataFile.print(",");
    dataFile.println(pres);
  
    /*dataFile.print("Gravity: ");
    dataFile.print(gravityData.acceleration.x);
    dataFile.print("\t");
    dataFile.print(gravityData.acceleration.y);
    dataFile.print("\t");
    dataFile.println(gravityData.acceleration.z);*/
    // Add the temperature data to the file
    /*int8_t boardTemp = bno.getTemp();
    dataFile.print("Temperature: ");
    dataFile.println(boardTemp);*/
    // Add calibration data to the file
    uint8_t system, gyro, accel, mag = 0;
    /*bno.getCalibration(&system, &gyro, &accel, &mag);
    dataFile.print("Calibration: Sys=");
    dataFile.print(system);
    dataFile.print(" Gyro=");
    dataFile.print(gyro);*/
    //bno.getCalibration(&accel);
    //dataFile.print(" Accel=");
    /*dataFile.print(accel);
    dataFile.print(" Mag=");
    dataFile.println(mag);*/
    // Close the file
    //dataFile.println();
    dataFile.close();
  }
    if (count==1){
      maxalt=alt;
    }else{
    if (alt>maxalt){
    maxalt=alt;
  }
    }
Serial.println(maxalt-alt);
if ((maxalt-alt)>10){
  Serial.println("parachute open");
myservo.write(75);
delay(10000);
}


  
  // Delay for a short period before collecting the next set of data
  delay(BNO055_SAMPLERATE_DELAY_MS);
  analogWrite(A0,500);
  analogWrite(A1,500);
  analogWrite(A2,500);
}
