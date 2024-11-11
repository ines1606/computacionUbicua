#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);//crear un objeto lcd (DIRECCIÓN pantalla, Tamaño x, Tamño y)

void setup() {
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
  
}

void loop() {
  lcd.backlight();
  delay(5000);
  lcd.clear();
  lcd.setCursor (0,0);//poner el cursor en las coordenadas (x,y)
  Serial.println("Fila 0");
  lcd.print("Fila 0");//muestra en la pantalla max 20 caracteres
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,1);
  Serial.println("Fila 1");
  lcd.print("Fila 1");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,2);
  Serial.println("Fila 2");
  lcd.print("Fila 2");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,3);
  Serial.println("Fila 3");
  lcd.print("Fila 3");
  delay(3000);//Esperamos 5 segundo antes de repetir el loop
  Serial.println("Apagamos y esperamos");
  lcd.clear();
  lcd.noBacklight();
  delay(5000);
}