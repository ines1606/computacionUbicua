const int buttonPin = 14;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);  // Activate internal pull-up resistor
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    Serial.println("Button pressed");
  } else {
    Serial.println("Button not pressed");
  }

  delay(1000);
}
