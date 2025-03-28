#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    int controlInputInt = Serial.readString().toInt();
    if (controlInputInt == 0)
    {
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  delay(0.1);
}
