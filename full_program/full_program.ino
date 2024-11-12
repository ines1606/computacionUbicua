#include <WiFi.h>          // For WiFi connection
#include <PubSubClient.h>   // For MQTT communication
#include <Adafruit_MPU6050.h> // For accelerometer and gyroscope
#include <MAX30105.h>  // For Oxygen/HeartRate sensors
#include <TinyGPS++.h>  // For GPS
#include <LiquidCrystal_I2C.h>
#include "spo2_algorithm.h"
#include "heartRate.h"
<<<<<<< HEAD
#include <WiFiUdp.h>
=======
#include "time.h"
>>>>>>> fe531a06b2246b67222222b5b54de0dfed1ac67e

// WiFi configuration
const char* ssid = ""; //name of the wifi
const char* password = ""; // password of the wifi

//MQTT configuration
const char* mqttServer = "192.168.1.23";  // MQTT server address
const int mqttPort = 1883;                    // MQTT server port
const char* mqttUser = "ubicua";            // MQTT username
const char* mqttPassword = "ubicua";    // MQTT password 

// npt server for quering the hour
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;


// Sensor initialization
TinyGPSPlus gps;
MAX30105 oxiSensor;
Adafruit_MPU6050 mpu;

// Wires initialization
TwoWire oxiWire = TwoWire(0);
TwoWire accWire = TwoWire(1);

// Define button pin
const int buttonPin = 14; // Pin for the button
int buttonState;

// Display configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);

// MQTT Client initialization
WiFiClient espClient;   // WiFi client
PubSubClient client(espClient);  // MQTT client


// GPS CONF
const int RXPin = 16;          // Pin RX of ESP32 for the GPS
const int TXPin = 17;          // Pin TX of ESP32 for el GPS 
const uint32_t GPSBaud = 9600; 

// Variables for oxygen
uint32_t irBuffer[100];  // Save IR samples
uint32_t redBuffer[100]; // Save the red samples
int32_t bufferLength = 100; // Buffer length
int32_t spo2; // Blood oxygen saturation level
int8_t validSPO2; // Validity of the SpO2 measurement (0 invalid, 1 valid)
int32_t heartRate; // Heart rate
int8_t validHeartRate; // Validity of the heart rate measurement (0 invalid, 1 valid)

// Variables for heart rate
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//variables for accelerometer
float ax;
float ay; 
float az;
float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

//variables for gps
float latitude = 0.0;
float longitude = 0.0;

// 
String uniqueUserID = "";

// Setup and initialization
void setup() {

  // Unique ID from MAC address
  uniqueUserID = getMACAddressAsID();

  Serial.begin(115200);
  // Connect to WiFi
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(1000);
     Serial.println("Connecting to WiFi...");
   }
  Serial.println("Connected to WiFi!");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Setup MQTT client
  client.setServer(mqttServer, mqttPort);  // Set MQTT server and port
  client.setCallback(mqttCallback);  // Set callback for receiving messages


  // Initialize sensors
  oxiWire.begin(21, 22);
  if (!oxiSensor.begin(oxiWire, I2C_SPEED_FAST)) { 
    Serial.println("Sensor MAX30102 was not found. Connect the sensor and reboot.");
    while (1);
  }
  Serial.println("Sensor MAX30102 initialised.");

  // accWire.setPins(33, 25);
  accWire.begin(33, 25);
  if (!mpu.begin(104, &accWire, 0)) {
    Serial.println("Error al encontrar el sensor MPU6050");
    while (1);
  }
  Serial.println("Sensor MPU6050 initialised.");
  
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); // Serial2 for the GPS
  Serial.println("Initializing the gps with the esp32...");

  // Initialize Display
  Wire.begin(26, 27);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Power on");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  
  // Configure the MAX30102 sensor
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 118; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  oxiSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Settings to optimise SpO2 and pulse

  // Calibrate the MPU6050 sensor
  calibrateMPU6050();

  // Set button pin
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Clear the display
  lcd.clear();
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print("Prueba LCD");
  delay(1000);

  // Maintain MQTT connection
   if (!client.connected()) {
     reconnect();
   }
   client.loop();  // Process incoming messages

  // Read button state to know whether the kid is sending a notification or not
  readButton();

  // Display actual hour on the display
  obtainHourDateActual();
  
  // Read sensor data
  readSensors();

  sendDataMQTT();

  // delay(1000); // Small delay (1sec)
}

void sendNotification() {
  if (client.connected()) {
    String message = "{\"message\":\"Your child is asking for help!\"}";
    String messageTopic = "data/" + uniqueUserID + "/SOS_messages"; 
    client.publish(messageTopic.c_str(), message.c_str());  // Publish message to the MQTT topic
    Serial.println("Notification sent via MQTT: " + message);
  } else {
    Serial.println("MQTT not connected");
  }
}

void readSensors() {
  // Read accelerometer/gyroscope data
  readAcc();
  //get data from oximeter
  readO2_pulse();
  //get info from gps
  readGPS();
}

// Function to get MAC address as a unique user ID
String getMACAddressAsID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);

  // Convert MAC address to a string and use it as the user ID
  String macString = "";
  for (int i = 0; i < 6; i++) {
    macString += String(mac[i], HEX);
  }
  macString.toUpperCase();  // Convert to MAYUSC for consistency
  return macString;
}

void reconnect() {
  // Loop until connected to MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Subscribe to all the topics
      String topic = "data/" + uniqueUserID + "/#";
      client.subscribe(topic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages from MQTT topics
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Received message: " + message);
}

void readGPS() {
  while (Serial2.available() > 0) {
      gps.encode(Serial2.read()); // Decode data from GPS
      if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }
}

void readO2_pulse(){

  for (int i = 0; i < bufferLength; i++) {
    while (oxiSensor.available() == false) { // Wait for data to become available
      oxiSensor.check(); // Reads sensor data
    }
    
    // Stores the data in the corresponding buffers
    redBuffer[i] = oxiSensor.getRed();
    irBuffer[i] = oxiSensor.getIR();
    
    oxiSensor.nextSample(); // Go to the next data

    if (checkForBeat(irBuffer[i]) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }

  // Calls the SpO2 and pulse algorithm
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (oxiSensor.available() == false) //do we have new data?
      oxiSensor.check(); //Check the sensor for new data

    redBuffer[i] = oxiSensor.getRed();
    irBuffer[i] = oxiSensor.getIR();
    oxiSensor.nextSample(); //We're finished with this sample so move to next sample

    if (checkForBeat(irBuffer[i]) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }
  
  //After gathering 25 new samples recalculate HR and SP02
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
}

void readAcc(){
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

  Serial.println("Calibration completed.");
  Serial.printf("Accelerometer compensation: X=%.2f, Y=%.2f, Z=%.2f\n", accOffsetX, accOffsetY, accOffsetZ);
  Serial.printf("Gyroscope compensation: X=%.2f, Y=%.2f, Z=%.2f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

void sendDataMQTT(){
  // Create JSON message with sensor's data
  String accData = "{";
  accData += "\"ax\": " + String(ax, 2) + ",";
  accData += "\"ay\": " + String(ay, 2) + ",";
  accData += "\"az\": " + String(az, 2) + "}";

  String gpsData = "{";
  gpsData += "\"longitude\": " + String(longitude, 2) + ",";
  gpsData += "\"latitude\": " + String(latitude, 2) + "}";
 
  String pulseData = "{\"heartRate\": " + String(heartRate, 2) + "}";

  String o2Data = "{\"oxygenLevel\": " + String(spo2, 2) + "}";
  
  // Send the data via MQTT
  if (client.connected()) {
    String o2Topic = "data/" + uniqueUserID + "/o2" ;
    String pulseTopic = "data/" + uniqueUserID + "/pulse";
    String gpsTopic = "data/" + uniqueUserID + "/gps";
    String accTopic = "data/" + uniqueUserID + "/acc";

    client.publish(o2Topic.c_str(), o2Data.c_str());
    client.publish(pulseTopic.c_str(), pulseData.c_str());
    client.publish(gpsTopic.c_str(), gpsData.c_str());
    client.publish(accTopic.c_str(), accData.c_str());
    Serial.println("Sensor data sent via MQTT");
  } else {
    Serial.println("MQTT not connected");
  }

}

void readButton(){
  buttonState = digitalRead(buttonPin);
  Serial.print("currentButtonState: ");
  Serial.println(buttonState);
  // Check if button has been pressed
  if (buttonState == LOW) {
    sendNotification();  // Send notification when button is pressed
    delay(2000);  // Wait 2 sec to avoid bouncing
  }
  else {
    Serial.println("Boton no presionado");
  }

}

void obtainHourDateActual(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  // Display the date
  lcd.setCursor(0, 0); // column 0, row 0
  lcd.print(&timeinfo, "%A");
  lcd.setCursor(0, 1); // column 0, row 1
  lcd.print(&timeinfo, "%d %B %Y");
  
  // Display the hour
  lcd.setCursor(0, 2); // column 0, row 2
  lcd.print(&timeinfo, "%H:%M:%S");

  Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
}
