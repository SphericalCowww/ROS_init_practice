#define LED_PIN 13
unsigned long intMsgPreviousMillis = 0;
const long intMsgInterval = 1000;
int intMsg;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  unsigned long currentMillis = millis();
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
  if (currentMillis - intMsgPreviousMillis >= intMsgInterval) {
    intMsgPreviousMillis = currentMillis;
    Serial.println(intMsg);
    intMsg++;
  }
  delay(0.1);
  
}
