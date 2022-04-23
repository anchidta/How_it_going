#include <MapleFreeRTOS821.h>

static void vWifi(void *pvParameters)
{
  for (;;)
  {
    
  }
}
void setup() 
{
  Serial.begin(9600);
  Serial2.begin(9600);
  xTaskCreate(vWifi,
              "Wifi Task",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              configMAX_PRIORITIES - 2,
              NULL);

  vTaskStartScheduler();
}

void loop() {}
