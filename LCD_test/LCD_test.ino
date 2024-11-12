#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);//create an lcd object (ADDRESS display, Size x, Size y)

void setup() {
  Wire.setPins(26, 27);
  lcd.init();
  Serial.begin(115200);
  Serial.println("\nLCD display");

  Serial.println("We ignite!");
  lcd.backlight();//Turn on the backlight  
  lcd.setCursor (0, 0);//place the cursor on the coordinates (x,y)

  lcd.print(" Display lcd 20x4 ");//display max 20 characters
  lcd.setCursor (0, 1);//place the cursor on the coordinates (x,y)
  lcd.print("*** esp32-wroom****");//display max 20 characters
  lcd.setCursor (0, 2);
  lcd.print("Screen test");
  lcd.setCursor (0, 3);
  lcd.print("complete");
  
}

void loop() {
  lcd.backlight();
  delay(5000);
  lcd.clear();
  lcd.setCursor (0,0);//place the cursor on the coordinates (x,y)
  Serial.println("Row 0");
  lcd.print("Row 0");//display max 20 characters
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,1);
  Serial.println("Row 1");
  lcd.print("Row 1");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,2);
  Serial.println("Row 2");
  lcd.print("Row 2");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,3);
  Serial.println("Row 3");
  lcd.print("Row 3");
  delay(3000);
  Serial.println("We turn off and wait");
  lcd.clear();
  lcd.noBacklight();
  delay(5000); // We wait 5 seconds before repeating the loop
}