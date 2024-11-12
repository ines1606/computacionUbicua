const int botonPin = 14;

void setup() {
  Serial.begin(115200);
  pinMode(botonPin, INPUT_PULLUP);  // Activar resistencia pull-up interna
}

void loop() {
  int estadoBoton = digitalRead(botonPin);

  if (estadoBoton == LOW) {
    Serial.println("Botón presionado");
  } else {
    Serial.println("Botón no presionado");
  }

  delay(200);
}
