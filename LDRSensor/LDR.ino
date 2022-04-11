int sensor = PA0;
void setup() {
  Serial.begin(9600);
}
void loop() {
  int val = analogRead(PA0);
  Serial.println(val);
  delay(1000);
}
