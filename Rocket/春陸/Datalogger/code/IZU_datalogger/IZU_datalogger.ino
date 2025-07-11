#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BME280I2C.h>
#include <SD.h>  // Include SD card library

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

const int chipSelect = 10;  // MicroSD card chip select pin
unsigned long time = 0;

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


void setup(void) {

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
  while(!bme.begin())
  {
    Serial.println("Could not find BME280I2C sensor!");
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


  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("SD card is ready to use.");

  delay(1000);
}

void loop(void) {

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

  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  // Open a file on the SD card to log data
  File dataFile = SD.open("TEST.txt", FILE_WRITE);

  // Check if the file is open
  if (dataFile) {
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

   
    uint8_t system, gyro, accel, mag = 0;
    /*bno.getCalibration(&system, &gyro, &accel, &mag);
    dataFile.priant("Calibration: Sys=");
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
  } else {
    // If the file didn't open, print an error
    Serial.println("Error opening TEST.txt");
  }

  // Delay for a short period before collecting the next set of data
  delay(BNO055_SAMPLERATE_DELAY_MS);

  analogWrite(A0,500);
  analogWrite(A1,500);
  analogWrite(A2,500);
}










