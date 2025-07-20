int intMsg;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  Serial.println(intMsg);
  intMsg++;
  delay(1000);
};
