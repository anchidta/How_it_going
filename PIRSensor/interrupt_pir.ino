#include <MapleFreeRTOS821.h>
#include <Wire.h>

// Reserved Pin.
int Buzzer = PA0;
int Pir = PB6;

// Variables Declaration(Semaphore and its state).
static xSemaphoreHandle sem;

// DETECTED function.
static void Pir_DETECTED(void *pvParametres) {
  for (;;) {
    xSemaphoreTake(sem, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    for (int i = 0; i <50; i++)
    {
    digitalWrite(Buzzer, HIGH);  //ปิดเสียงเตือน
    delay(50);
    digitalWrite(Buzzer, LOW);   //เปิดเสียงเตือน
    delay(50);
    Serial.println("DETECTED");
    }
   
  }
}

// InterruptHandle function.
void InterruptHandle() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  Serial.println("ALERT");
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}

// Setup function
void setup() {
  // Initialize the digital pin as an input and output.
  Serial.begin(9600);    // Showing result in serial monitor with speed as 9600.
  pinMode(Buzzer, OUTPUT);
  pinMode(Pir, INPUT);
  attachInterrupt(Pir, InterruptHandle, RISING); // Interrupt Rising.
  sem = xSemaphoreCreateBinary();
  xTaskCreate(Pir_DETECTED,
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
