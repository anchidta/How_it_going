//---------------------------- Libraries ---------------------------------
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include "DHT.h"

//-------------------------------- Pins --------------------------------
#define BuzzerSensor PA0
#define DhtSensor PA1
//#define Wifi1 PA2
//#define Wifi2 PA3
#define CurrentSensor PA6

#define LdrSensor PB0
#define LEDs PB1
#define BuzzInterruptPin PB6    //PIR sensor
#define FanInterruptPin PB7  //Touch sensor
#define FanInA PB8
#define FanInB PB9

//---------------------------------- Parameters ---------------------------------------------------
#define DHTTYPE DHT11

float nVPP;   // Voltage measured across resistor
float nCurrThruResistorPP; // Peak Current Measured Through Resistor
float nCurrThruResistorRMS; // RMS current through Resistor
float nCurrentThruWire;     // Actual RMS current in Wire

static xSemaphoreHandle SemFan;
static xSemaphoreHandle SemBuzz;

int FanState = LOW;
int BuzzState = LOW;
int dayTime = HIGH;

//------------------------- Functions ----------------------------------
DHT dht(DhtSensor, DHTTYPE);

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

//---------------------------------------- Tasks ------------------------
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

static void vFanOn(void *pvParametres) {
  for (;;) {
    xSemaphoreTake(SemFan, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    Serial.println("Change Fan State");
    FanState = !FanState;                         // State Toggle.
    digitalWrite(FanInB, FanState);
  }
}

static void vBuzzerOn(void *pvParametres) {
  for (;;) {
    xSemaphoreTake(SemBuzz, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    if (!dayTime) {
      //      for (int i = 0; i < 5; i++)
      //      {
      //        digitalWrite(BuzzerSensor, HIGH);  //ปิดเสียงเตือน
      //        delay(50);
      ////        digitalWrite(BuzzerSensor, LOW);   //เปิดเสียงเตือน
      ////        delay(50);
      //        Serial.println("DETECTED");
      //      }
      //      for (int i = 0; i < 5; i++)
      //      {
      //        digitalWrite(BuzzerSensor, HIGH);  //ปิดเสียงเตือน
      //        delay(20);
      ////        digitalWrite(BuzzerSensor, LOW);   //เปิดเสียงเตือน
      ////        delay(20);
      //        Serial.println("DETECTED");
      //      }
      //      for (int i = 0; i < 5; i++)
      //      {
      //        digitalWrite(BuzzerSensor, HIGH);  //ปิดเสียงเตือน
      //        delay(50);
      ////        digitalWrite(BuzzerSensor, LOW);   //เปิดเสียงเตือน
      ////        delay(50);
      //        Serial.println("DETECTED");
      //      }
      //      for (int i = 0; i < 5; i++)
      //      {
      //        digitalWrite(BuzzerSensor, HIGH);  //ปิดเสียงเตือน
      //        delay(20);
      ////        digitalWrite(BuzzerSensor, LOW);   //เปิดเสียงเตือน
      ////        delay(20);
      //        Serial.println("DETECTED");
      //      }
      Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>DETECTED");
      digitalWrite(BuzzerSensor, LOW);
      vTaskDelay(3000);
      digitalWrite(BuzzerSensor, HIGH);

    }
    //    vTaskDelay(1000);
  }
}

static void vLightsOn(void *pvParameters) {
  for (;;) {
    int ldrStatus = analogRead(LdrSensor);

    //Lights are on
    if (ldrStatus <= 1000) {

      digitalWrite(LEDs, LOW);
      Serial.print("Its BRIGHT, Turn off the LED : ");
      Serial.println(ldrStatus);
      dayTime = HIGH;

    }
    //Lights are off
    else {

      digitalWrite(LEDs, HIGH);
      Serial.print("Its DARK, Turn on the LED : ");
      Serial.println(ldrStatus);
      dayTime = LOW;

    }
  }
}

static void vDhtSensor(void *pvParameters) {
  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(false);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.print(F("°F  Heat index: "));
    Serial.print(hic);
    Serial.println(F("°C "));

  }
}
//------------------------------ Interrupt ------------------------------
void InterruptHandleFan() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  //  Serial.println("TOUCHED");
  xSemaphoreGiveFromISR(SemFan, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}

void InterruptHandleBuzz() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  //  Serial.println("ALERT");
  xSemaphoreGiveFromISR(SemBuzz, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}


//-------------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(9600);
  //Pins
  pinMode(BuzzerSensor, OUTPUT);
  pinMode(DhtSensor, INPUT);
  pinMode(CurrentSensor, INPUT);
  pinMode(LdrSensor, INPUT);
  pinMode(BuzzInterruptPin, INPUT);
  pinMode(FanInterruptPin, INPUT);
  pinMode(FanInA, OUTPUT);
  pinMode(FanInB, OUTPUT);
  pinMode(LEDs, OUTPUT);

  digitalWrite(BuzzerSensor, HIGH );

  //Interrupt
  attachInterrupt(FanInterruptPin, InterruptHandleFan, CHANGE);
  SemFan = xSemaphoreCreateBinary();
  attachInterrupt(BuzzInterruptPin, InterruptHandleBuzz, RISING);
  SemBuzz = xSemaphoreCreateBinary();
  //Tasks
  xTaskCreate(vCurrentSensor,
              "Current Sensor Task",
              configMINIMAL_STACK_SIZE,
              NULL,
              configMAX_PRIORITIES - 3,
              NULL);
  xTaskCreate(vFanOn,
              "Fan and Touch Sensor Interrupt",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              configMAX_PRIORITIES,
              NULL);
  xTaskCreate(vBuzzerOn,
              "Buzzer and PIR Sensor Interrupt",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              configMAX_PRIORITIES - 1,
              NULL);
  xTaskCreate(vLightsOn,
              "Turn LEDs on",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              configMAX_PRIORITIES - 2,
              NULL);
  xTaskCreate(vDhtSensor,
              "DHT Sensor Task",
              configMINIMAL_STACK_SIZE + 50,
              NULL,
              configMAX_PRIORITIES - 4,
              NULL);

  noInterrupts();
  vTaskStartScheduler();
  interrupts();
}

void loop() {}
