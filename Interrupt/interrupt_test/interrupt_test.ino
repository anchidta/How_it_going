//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include <MapleFreeRTOS821.h>
#define BOARD_LED_PIN LED_BUILTIN
#define LED_PIN 17
int interruptpin = 20;
int interruptpin2 = 21;
static xSemaphoreHandle sem;
static xSemaphoreHandle sem2;
int state = LOW;
int state2 = LOW;

static void LedTurnOn(void *pvParameters) {
  for (;;) {
    xSemaphoreTake(sem, portMAX_DELAY);
    Serial.println("Change LED State");
    state = !state;
    digitalWrite(BOARD_LED_PIN, state);
  }
}
static void LedTurnOn2(void *pvParameters) {
  for (;;) {
    xSemaphoreTake(sem2, portMAX_DELAY);
    Serial.println("Change LED 2 State");
    state2 = !state2;
    digitalWrite(LED_PIN, state2);
  }
}
void InterruptHandler() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  Serial.println("n Inside Interrupt");
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
}
void InterruptHandler2() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  Serial.println("n Inside Interrupt2");
  xSemaphoreGiveFromISR(sem2, &xHigherPriorityTaskWoken);
}

void setup() {
  // initialize the digital pin as an output:
  Serial.begin(9600);
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(interruptpin, INPUT_PULLUP);
  pinMode(interruptpin2, INPUT_PULLUP);
  Serial.println("n Start Program");
  attachInterrupt(interruptpin, InterruptHandler, RISING);
  attachInterrupt(interruptpin2, InterruptHandler2, RISING);
  sem = xSemaphoreCreateBinary();
  sem2 = xSemaphoreCreateBinary();
  xTaskCreate(LedTurnOn,
              "Working With Interrupt",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  xTaskCreate(LedTurnOn2,
              "Working With Interrupt 2",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              tskIDLE_PRIORITY + 3,
              NULL);
  noInterrupts();
  vTaskStartScheduler();
  interrupts();
}

void loop() {}
