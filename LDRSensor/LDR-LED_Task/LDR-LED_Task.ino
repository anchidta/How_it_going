#include <MapleFreeRTOS821.h>
#define ldrPin 0   //LDR pin on PA0
#define lights 17  //LED pin on PB1

int dayTime;
static void LightsOn(void *pvParameters) {
  for (;;) {
    int ldrStatus = analogRead(ldrPin);

    //Lights are on
    if (ldrStatus <= 1000) {

      digitalWrite(lights, LOW);
      Serial.print("Its BRIGHT, Turn off the LED : ");
      Serial.println(ldrStatus);
      dayTime = HIGH;

    }
    //Lights are off
    else {

      digitalWrite(lights, HIGH);
      Serial.print("Its DARK, Turn on the LED : ");
      Serial.println(ldrStatus);
      dayTime = LOW;

    }
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(ldrPin, INPUT);
  pinMode(lights, OUTPUT);
  Serial.begin(9600);

  xTaskCreate(LightsOn,
              "Working With Interrupt",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  vTaskStartScheduler();
}

void loop() {}
