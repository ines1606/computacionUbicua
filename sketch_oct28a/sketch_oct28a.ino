//Libraries
#include <Wire.h>
#include <SPI.h>
#include <HTTPClient.h> //For making HTTP posts
#include <Adafruit_MPU6050.h> //For accelerometer and gyroscope
#include <TinyGPS++.h>  //For gps
#include <MAX30105.h>  //For Oxygen/HeartRate sensors
#include <TFT_eSPI.h> // For the TFT display
#include <BMI270.h>   // Include library for BMI270

// WiFi configuration
const char* ssid = "username";
const char* password = "password";
const char* serverUrl = "http://server.com";

// Sensor initialization
TinyGPSPlus gps;
MAX30105 oxiSensor;
BMI270 bmi;

// Define button pin
const int buttonPin = 25; // Pin for the button
bool lastButtonState = HIGH; // Previous button state

// TFT display configuration
TFT_eSPI tft = TFT_eSPI(); // Create display object

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

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

  delay(1000); // Small delay (1sec)
}

void sendNotification() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    // Create JSON payload for the notification
    String payload = "{\"message\":\"Your child is asking dor help!\"}";

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


//Read GPS data
  while (ss.available() > 0)
   gps.encode(ss.read());
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  
  // Read oxygen and heart rate
  float heartRate = oxiSensor.getHeartRate();
  float oxygenLevel = oxiSensor.getSpO2();
  Serial.printf("Heart Rate: %.2f, SpO2: %.2f\n", heartRate, oxygenLevel);

  // Display this data on the TFT display
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.fillScreen(TFT_BLACK);
  tft.printf("Heart Rate: %.2f\n", heartRate);
  tft.printf("SpO2: %.2f\n", oxygenLevel);
}
