#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "time.h"

const char* ssid       = "";
const char* password   = "";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

LiquidCrystal_I2C lcd(0x27, 20, 4);//crear un objeto lcd (DIRECCIÓN pantalla, Tamaño x, Tamño y)

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
  Wire.setPins(26, 27);
  lcd.init();
  Serial.begin(115200);
  Serial.println("\nPantalla LCD");

  // lcd.begin();//inicializar la pantalla lcd
  Serial.println("Encendemos!");
  lcd.backlight();//Encender la luz de fondo  
  lcd.setCursor (0, 0);//poner el cursor en las coordenadas (x,y)

  lcd.print(" Pantalla lcd 20x4 ");//muestra en la pantalla max 20 caracteres
  lcd.setCursor (0, 1);//poner el cursor en las coordenadas (x,y)
  lcd.print("*** esp32-wroom****");//muestra en la pantalla max 20 caracteres
  lcd.setCursor (0, 2);
  lcd.print("Prueba pantalla");
  lcd.setCursor (0, 3);
  lcd.print("completa");
  
  delay (4000);
  lcd.clear();
}

void loop() {
  // lcd.clear();

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %d %B %Y");
  Serial.println(&timeinfo,"%H:%M:%S");

  // Cambiar para que salga por la pantalla
  lcd.setCursor(0, 0); // column 0, row 0
  lcd.print(&timeinfo, "%A, ");
  lcd.setCursor(0, 1); // column 0, row 1
  lcd.print(&timeinfo, "%d %B %Y");
  
  // Cambiar para que salga por la pantalla
  lcd.setCursor(0, 2);
  lcd.print(&timeinfo, "%H:%M:%S");

  delay(1000);
}
