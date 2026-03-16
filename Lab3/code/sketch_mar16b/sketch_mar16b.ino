void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(15, INPUT);
  analogSetPinAttenuation(34, ADC_11db);
}

void loop() {
  int raw = analogRead(34);
  Serial.printf("ADC raw: %d  (%.3f V)\n", raw, raw * 3.3f / 4095.0f);
  delay(500);
}
