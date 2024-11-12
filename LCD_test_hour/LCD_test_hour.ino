#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "time.h"

const char* ssid       = "";
const char* password   = "";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

LiquidCrystal_I2C lcd(0x27, 20, 4);//create an lcd object (ADDRESS display, Size x, Size y)

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %d %B %Y");
  Serial.println(&timeinfo,"%H:%M:%S");
}

void setup() {
  Serial.begin(115200);

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Initialize display
  Wire.begin(26, 27);
  lcd.init();
  Serial.begin(115200);
  Serial.println("\nDisplay LCD");

  Serial.println("We ignite!");
  lcd.backlight();//Turn on the backlight  
  lcd.setCursor (0, 0);//place the cursor on the coordinates (x,y)

  lcd.print(" Display lcd 20x4 ");//display max 20 characters
  lcd.setCursor (0, 1);//place the cursor on the coordinates (x,y)
  lcd.print("*** esp32-wroom****");//display max 20 characters
  lcd.setCursor (0, 2);
  lcd.print("Display test");
  lcd.setCursor (0, 3);
  lcd.print("complete");
  
  delay (4000);
  lcd.clear();
}

void loop() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  // The date
  lcd.setCursor(0, 0); // column 0, row 0
  lcd.print(&timeinfo, "%A");
  lcd.setCursor(0, 1); // column 0, row 1
  lcd.print(&timeinfo, "%d %B %Y");
  
  // The hour
  lcd.setCursor(0, 2);
  lcd.print(&timeinfo, "%H:%M:%S");

  delay(1000);
}