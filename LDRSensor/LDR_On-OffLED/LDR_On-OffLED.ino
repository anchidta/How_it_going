//#include <MapleFreeRTOS821.h>
#define LDR_PIN 0
#define LED_PIN 17


void setup() {
  // put your setup code here, to run once:
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int ldrStatus = analogRead(LDR_PIN);

  if (ldrStatus <= 1000) {

    digitalWrite(LED_PIN, LOW);
    Serial.print("Its BRIGHT, Turn off the LED : ");
    Serial.println(ldrStatus);

  } else {

    digitalWrite(LED_PIN, HIGH);
    Serial.print("Its DARK, Turn on the LED : ");
    Serial.println(ldrStatus);

  }
}
