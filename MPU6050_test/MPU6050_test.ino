#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables used to compensate the values
float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 25);
  if (!mpu.begin()) {
    Serial.println("Error finding the MPU6050 sensor");
    while (1);
  }
  
  // Calibrate the sensor
  calibrateMPU6050();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // compensate data taken from accelerometer
  float ax = a.acceleration.x - accOffsetX;
  float ay = a.acceleration.y - accOffsetY;
  float az = a.acceleration.z - accOffsetZ;

  // compensate data taken from gyroscope
  float gx = g.gyro.x - gyroOffsetX;
  float gy = g.gyro.y - gyroOffsetY;
  float gz = g.gyro.z - gyroOffsetZ;

  Serial.printf("Accelerometer: ax=%.2f, ay=%.2f, az=%.2f\n", ax, ay, az);
  Serial.printf("Gyroscope: gx=%.2f, gy=%.2f, gz=%.2f\n", gx, gy, gz);

  delay(1000);
}

void calibrateMPU6050() {
  const int sampleSize = 1250;
  
  float accSumX = 0, accSumY = 0, accSumZ = 0;
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;

  Serial.println("Calibraring the sensor MPU6050. Please don't move the sensor...");
  
  // Samples
  for (int i = 0; i < sampleSize; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accSumX += a.acceleration.x;
    accSumY += a.acceleration.y;
    accSumZ += a.acceleration.z - 9.81; 

    gyroSumX += g.gyro.x;
    gyroSumY += g.gyro.y;
    gyroSumZ += g.gyro.z;

    delay(1);  
  }

  // Calcaulate the mid value 
  accOffsetX = accSumX / sampleSize;
  accOffsetY = accSumY / sampleSize;
  accOffsetZ = accSumZ / sampleSize;
  
  gyroOffsetX = gyroSumX / sampleSize;
  gyroOffsetY = gyroSumY / sampleSize;
  gyroOffsetZ = gyroSumZ / sampleSize;

  Serial.println("Completed calibration.");
  Serial.printf("Accelerometer compensation: X=%.2f, Y=%.2f, Z=%.2f\n", accOffsetX, accOffsetY, accOffsetZ);
  Serial.printf("Gyroscope compensation: X=%.2f, Y=%.2f, Z=%.2f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}