#include <TinyGPS++.h>

// GPS CONF
const int RXPin = 16;          // Pin RX of ESP32 for the GPS
const int TXPin = 17;          // Pin TX of ESP32 for el GPS 
const uint32_t GPSBaud = 9600; 

// Instance declaration
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);        // Monitor serial
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); // Serial2 para el GPS

  Serial.println("Initializing the gps with the esp32...");
}

void loop() {
  
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read()); // Decode data from GPS
    if (gps.location.isUpdated()) {
      Serial.print("Latitud: ");
      Serial.println(gps.location.lat(), 6); // 6 decimals for more accuracy

      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Altitud: ");
      Serial.print(gps.altitude.meters(), 6);
      Serial.println(" m");

      Serial.print("Satélites: ");
      Serial.println(gps.satellites.value());

      Serial.print("Precisión HDOP: ");
      Serial.println(gps.hdop.value());

      Serial.println();
    }
  }
}