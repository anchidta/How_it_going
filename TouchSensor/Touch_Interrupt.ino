#include <MapleFreeRTOS821.h>
#include <Wire.h>

// Reserved Pin.
int FInA = PB8;
int FInB = PB9;
int fan_interruptpin = PB7;

// Variables Declaration(Semaphore and its state).
static xSemaphoreHandle sem;
int state = LOW;

// FanON function.
static void Fan_On(void *pvParametres) {
  for (;;) {
    xSemaphoreTake(sem, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    Serial.println("Change Fan State");
    state = !state;                         // State Toggle.
    digitalWrite(FInB, state);
  }
}

// InterruptHandle function.
void InterruptHandle() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  Serial.println("Touched");
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}

// Setup function
void setup() {
  // Initialize the digital pin as an input and output.
  Serial.begin(9600);    // Showing result in serial monitor with speed as 9600.
  pinMode(FInA, OUTPUT);
  pinMode(FInB, OUTPUT);
  pinMode(fan_interruptpin, INPUT);
  attachInterrupt(fan_interruptpin, InterruptHandle, RISING); // Interrupt Rising.
  sem = xSemaphoreCreateBinary();
  xTaskCreate(Fan_On,
              "Working with Interrupt",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  noInterrupts();
  vTaskStartScheduler();
  interrupts();
}
void loop() {}
