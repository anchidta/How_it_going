#include <MapleFreeRTOS821.h>
#define CurrentSensor PA1
//---------------------------------- Parameters ---------------------------------------------------
float nVPP;   // Voltage measured across resistor
float nCurrThruResistorPP; // Peak Current Measured Through Resistor
float nCurrThruResistorRMS; // RMS current through Resistor
float nCurrentThruWire;     // Actual RMS current in Wire

float getVPP() {
  float result;
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  readValue = analogRead(CurrentSensor);
  // see if you have a new maxValue
  if (readValue > maxValue)
  {
    /*record the maximum sensor value*/
    maxValue = readValue;
  }


  // Convert the digital data to a voltage
  result = (maxValue * 5.0) / 1024.0;

  return result;
}

static void vCurrentSensor(void *pvParameters) {
  for (;;) {
    nVPP = getVPP();
    /*
      Use Ohms law to calculate current across resistor
      and express in mA
    */

    nCurrThruResistorPP = (nVPP / 200.0) * 1000.0;

    /*
      Use Formula for SINE wave to convert
      to RMS
    */

    nCurrThruResistorRMS = nCurrThruResistorPP * 0.707;

    /*
      Current Transformer Ratio is 1000:1...

      Therefore current through 200 ohm resistor
      is multiplied by 1000 to get input current
    */

    nCurrentThruWire = nCurrThruResistorRMS * 1000;
    Serial.print("Volts Peak : ");
    Serial.println(nVPP, 3);


    Serial.print("Current Through Resistor (Peak) : ");
    Serial.print(nCurrThruResistorPP, 3);
    Serial.println(" mA Peak to Peak");

    Serial.print("Current Through Resistor (RMS) : ");
    Serial.print(nCurrThruResistorRMS, 3);
    Serial.println(" mA RMS");

    Serial.print("Current Through Wire : ");
    Serial.print(nCurrentThruWire, 3);
    Serial.println(" mA RMS");

    Serial.println();
    vTaskDelay(1000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(CurrentSensor, INPUT);

  xTaskCreate(vCurrentSensor,
              "Task1",
              configMINIMAL_STACK_SIZE,
              NULL,
              tskIDLE_PRIORITY + 2,
              NULL);
  vTaskStartScheduler();
}

void loop() {}
