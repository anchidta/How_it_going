#include <MapleFreeRTOS821.h>
#include <Wire.h>

// Reserved Pin.
int TPin = PB7;
int FInA = PB8;
int FInB = PB9;
int fan_interruptpin = PB2;

// Variables Declaration(Semaphore and its state).
static xSemaphoreHandle sem;
int state = LOW;

// FanInB function.
static void FanInB_On(void *pvParametres) {
  for (;;) {
    xSemaphoreTake(sem, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    Serial.println("Change FanInB State.");
    state = !state;                         // State Toggle.
    digitalWrite(FInB, state);
  }
}

// InterruptHandle function.
void InterruptHandle() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  Serial.println("n Inside Interrupt.");
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}

// Setup function
void setup() {
  // Initialize the digital pin as an input and output.
  Serial.begin(9600);    // Showing result in serial monitor with speed as 9600.
  pinMode(TPin, INPUT);
  pinMode(FInA, OUTPUT);
  pinMode(FInB, OUTPUT);

  Serial.println("n Start Program.");
  int touchval = digitalRead(TPin);
  if (touchval == HIGH) {
    Serial.println("Touched.");  // Showing "Touched" on serial monitor when touchval is HIGH signal.
    attachInterrupt(fan_interruptpin, InterruptHandle, RISING); // Interrupt Rising.
    sem = xSemaphoreCreateBinary();
    xTaskCreate(FanInB_On,
                "Working with Interrupt",
                configMINIMAL_STACK_SIZE + 50,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    noInterrupts();
    vTaskStartScheduler();
    interrupts();
  }
}

void loop() {}
