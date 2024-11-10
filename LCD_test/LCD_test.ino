#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3c, 16, 2);//crear un objeto lcd (DIRECCIÓN pantalla, Tamaño x, Tamño y)

void setup() {
  lcd.init();
  Serial.begin(115200);
  Serial.println("\nPantalla LCD");

  //lcd.begin();//inicializar la pantalla lcd
  Serial.println("Encendemos!");
  lcd.backlight();//Encender la luz de fondo  
  lcd.setCursor (0, 0);//poner el cursor en las coordenadas (x,y)

  lcd.print(" Pantalla lcd 16x2  ");//muestra en la pantalla max 20 caracteres
  lcd.setCursor (0, 1);//poner el cursor en las coordenadas (x,y)
  lcd.print("*** esp32-s****");//muestra en la pantalla max 20 caracteres
  
}

void loop() {
  lcd.backlight();
  delay(5000);
  lcd.setCursor (0,0);//poner el cursor en las coordenadas (x,y)
  lcd.print("Fila 0");//muestra en la pantalla max 20 caracteres
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,1);
  Serial.println("Fila 1");
  lcd.print("Fila 1");
  Serial.println("Apagamos y esperamos");
  delay(5000);//Esperamos 1 segundo antes de repetir el loop
  lcd.clear();
}