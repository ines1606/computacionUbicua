#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// Variables for SpO2 and pulse
uint32_t irBuffer[100];  // Save IR samples
uint32_t redBuffer[100]; // Save the red samples
int32_t bufferLength = 100; // Buffer length
int32_t spo2; // Blood oxygen saturation level
int8_t validSPO2; // Validity of the SpO2 measurement (0 invalid, 1 valid)
int32_t heartRate; // Heart rate
int8_t validHeartRate; // Validity of the heart rate measurement (0 invalid, 1 valid)

void setup() {
  Serial.begin(115200);
  // Initialise the sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { 
    Serial.println("Sensor MAX30102 was not found. Connect the sensor and reboot.");
    while (1);
  }
  
  Serial.println("Sensor MAX30102 initialised.");
  
  // Configure the sensor
  byte ledBrightness = 31; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Settings to optimise SpO2 and pulse
}

void loop() {
  // Fills the buffer with the initial data
  for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) { // Wait for data to become available
      particleSensor.check(); // Reads sensor data
    }
    
    // Stores the data in the corresponding buffers
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    
    particleSensor.nextSample(); // Go to the next data
  }

  // Calls the SpO2 and pulse algorithm
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Print the results if they are valid
  if (validHeartRate) {
    Serial.print("Pulse: ");
    Serial.print(heartRate);
    Serial.print(" BPM, ");
  } else {
    Serial.print("Pulse not valid, ");
  }
  
  if (validSPO2) {
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.println(" %");
  } else {
    Serial.println("SpO2 not valid");
  }

  delay(1000); // Time interval for the next measurement
}
