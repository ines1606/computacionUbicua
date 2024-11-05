// Libraries
#include <Wire.h>
#include <SPI.h>
#include <HTTPClient.h> // For making HTTP posts
#include <Adafruit_MPU6050.h> // For accelerometer and gyroscope
#include <TinyGPS++.h>  // For GPS
#include <MAX30105.h>  // For Oxygen/HeartRate sensors
#include <TFT_eSPI.h> // For the TFT display
#include <BMI270.h>   // Include library for BMI270
#include <WiFi.h> // Include WiFi library
#include <PubSubClient.h> // Include PubSubClient for MQTT

// WiFi configuration
const char* ssid = "username";
const char* password = "password";
const char* mqttServer = "mqtt.server.com"; // Your MQTT broker URL
const int mqttPort = 1883; // Your MQTT broker port
const char* mqttUser = "mqtt_user"; // Your MQTT username (if required)
const char* mqttPassword = "mqtt_password"; // Your MQTT password (if required)

// Sensor initialization
TinyGPSPlus gps;
MAX30105 oxiSensor;
BMI270 bmi;

// Define button pin
const int buttonPin = 25; // Pin for the button
bool lastButtonState = HIGH; // Previous button state

// TFT display configuration
TFT_eSPI tft = TFT_eSPI(); // Create display object

// GPS Serial configuration
#define GPS_SERIAL Serial2 // Change this to the appropriate Serial port for your GPS module

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Initialize MQTT
  mqttClient.setServer(mqttServer, mqttPort);

  // Initialize sensors
  if (!bmi.begin()) {
    Serial.println("Could not find a valid BMI270 sensor, check wiring!");
    while (1);
  }
  if (!oxiSensor.begin()) {
    Serial.println("Could not find a valid MAX30102 sensor, check wiring!");
    while (1);
  }

  // Initialize TFT display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Set button pin
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor

  // Start GPS Serial
  GPS_SERIAL.begin(9600); // Change baud rate if needed
}

void loop() {
  // Read button state
  bool currentButtonState = digitalRead(buttonPin);
  
  // Check if button has been pressed
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    sendNotification(); // Send notification when button is pressed
    delay(2000); // Wait 2 sec to avoid bouncing
  }
  lastButtonState = currentButtonState;

  // Read sensor data
  readSensors();

  // Reconnect to MQTT broker if disconnected
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop(); // Keep the MQTT client connected

  delay(1000); // Small delay (1 sec)
}

void sendNotification() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    // Create JSON payload for the notification
    String payload = "{\"message\":\"Your child is asking for help!\"}";

    // Send POST request
    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Notification sent: " + payload);
      Serial.println("Server response: " + response);
    } else {
      Serial.println("Error in POST request for notification");
    }
    http.end();
  }
}

void readSensors() {
  // Read accelerometer/gyroscope data
  bmi.getSensorData();

  // Get and print accelerometer data
  float ax = bmi.getAccX();
  float ay = bmi.getAccY();
  float az = bmi.getAccZ();
  Serial.printf("Accelerometer: ax=%.2f, ay=%.2f, az=%.2f\n", ax, ay, az);

  // Read oxygen and heart rate
  float heartRate = oxiSensor.getHeartRate();
  float oxygenLevel = oxiSensor.getSpO2();
  Serial.printf("Heart Rate: %.2f, SpO2: %.2f\n", heartRate, oxygenLevel);

  // Read GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Only display GPS data if a valid location is available
  if (gps.location.isUpdated()) {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    Serial.printf("Latitude: %.6f, Longitude: %.6f\n", latitude, longitude);
    
    // Display this data on the TFT display
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.fillScreen(TFT_BLACK);
    tft.printf("Heart Rate: %.2f\n", heartRate);
    tft.printf("SpO2: %.2f\n", oxygenLevel);
    tft.printf("Lat: %.6f\n", latitude);
    tft.printf("Lon: %.6f\n", longitude);

    // Send health data via MQTT
    sendHealthData(heartRate, oxygenLevel);
    
    // Send location data via MQTT
    sendLocationData(latitude, longitude);
  } else {
    Serial.println("Waiting for GPS data...");
  }
}

void sendHealthData(float heartRate, float oxygenLevel) {
  // Create JSON payload for health data
  String payload = "{\"heartRate\": " + String(heartRate) + ", \"oxygenLevel\": " + String(oxygenLevel) + "}";

  // Publish health data to the MQTT broker
  if (mqttClient.publish("health/data", payload.c_str())) {
    Serial.println("Health data sent: " + payload);
  } else {
    Serial.println("Failed to send health data");
  }
}

void sendLocationData(double latitude, double longitude) {
  // Create JSON payload for location data
  String payload = "{\"latitude\": " + String(latitude) + ", \"longitude\": " + String(longitude) + "}";

  // Publish location data to the MQTT broker
  if (mqttClient.publish("location/data", payload.c_str())) {
    Serial.println("Location data sent: " + payload);
  } else {
    Serial.println("Failed to send location data");
  }
}

void sendStepCountData(int steps) {
  // Create JSON payload for step count data
  String payload = "{\"steps\": " + String(steps) + "}";

  // Publish step count data to the MQTT broker
  if (mqttClient.publish("steps/data", payload.c_str())) {
    Serial.println("Step count data sent: " + payload);
  } else {
    Serial.println("Failed to send step count data");
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("ClientID", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


