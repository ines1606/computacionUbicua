// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h> //library provides functions to interact with MPU
#include <Adafruit_Sensor.h>  //library provides base classes for sensors
#include <Wire.h>             //library allows Arduino to communicate with I2C devices

Adafruit_MPU6050 mpu; //MPU object, used to interact with the sensor
int SDA = 33; //pin number for SDA
int SCL = 25; //pin number for SCL

void setup(void) {
  Serial.begin(115200); //initialize serial communication, see output in serial monitor
  while (!Serial)
    delay(10); //wati until serial monitor is opened (boards like Aarduino Zero or Leonardo, ignorable on other boards)

  Wire.begin(SDA, SCL); //Initialize I2C communication with custom SDA and SCL pins 

  Serial.println("Adafruit MPU6050 test!"); //indicate that program is starting

  if (!mpu.begin()) { //attempt to initialize MPU sensor
    Serial.println("Failed to find MPU6050 chip"); //fail -> error message
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!"); 

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //call to function to set Range 

  Serial.print("Accelerometer range set to: ");

  //print current accelometer range
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //set gyroscope range to maximum (the higher the range, the lower precision)

  Serial.print("Gyro range set to: ");
  //reads and prints current range
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); //filter to smooth data, remove high frequency noise
  Serial.print("Filter bandwidth set to: ");
  //read and print current bandwith 
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  //structure to store sensor data
  sensors_event_t a, g, temp;   //accelometer, gyroscope, temperature data
  mpu.getEvent(&a, &g, &temp);  //fills variables with latest data from mpu

  // Print out the acceleration values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

// Print out the rotation values 
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

// Print out the temperature values 
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}
}