//---------------------------- Libraries ---------------------------------
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include "DHT.h"
#include "pitches.h"

//-------------------------------- Pins --------------------------------
#define BuzzerSensor PA1
#define DhtSensor PA0
#define CurrentSensor PA6

#define LdrSensor PB0
#define LEDs PB14
#define LEDr PB10
#define LEDy PB11
#define LEDg PB12
#define LEDor PB13
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


DHT dht(DhtSensor, DHTTYPE);

//Params that will be sent to web
float g_Current;
float g_Humidity;
float g_Temperature;
int g_dayTime = HIGH;
int g_fanState = LOW;
int g_buzzState = LOW;

int melody[] = {
  NOTE_FS5, NOTE_FS5, NOTE_D5, NOTE_B4, NOTE_B4, NOTE_E5,
  NOTE_E5, NOTE_E5, NOTE_GS5, NOTE_GS5, NOTE_A5, NOTE_B5,
  NOTE_A5, NOTE_A5, NOTE_A5, NOTE_E5, NOTE_D5, NOTE_FS5,
  NOTE_FS5, NOTE_FS5, NOTE_E5, NOTE_E5, NOTE_FS5, NOTE_E5
};

int durations[] = {
  8, 8, 8, 4, 4, 4,
  4, 5, 8, 8, 8, 8,
  8, 8, 8, 4, 4, 4,
  4, 5, 8, 8, 8, 8
};
int songLength = sizeof(melody) / sizeof(melody[0]);

//------------------------- Functions ----------------------------------

float getVPP() {
  float result;
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  readValue = analogRead(CurrentSensor);
  // see if you have a new maxValue
  if (readValue > maxValue)
  {
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
    nCurrThruResistorPP = (nVPP / 200.0) * 1000.0;
    nCurrThruResistorRMS = nCurrThruResistorPP * 0.707;
    nCurrentThruWire = nCurrThruResistorRMS * 1000;
    g_Current = nCurrentThruWire;
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
    g_fanState = !g_fanState;                         // State Toggle.
    digitalWrite(FanInB, g_fanState);
    if (g_fanState == HIGH) {                    // If Fan ON LED green ON LED orange OFF
      digitalWrite(LEDg, HIGH);
      digitalWrite(LEDor, LOW);
    }
    else {
      digitalWrite(LEDg, LOW);                 // If Fan OFF LED green OFF LED orange ON
      digitalWrite(LEDor, HIGH);
    }

  }
}

static void vBuzzerOn(void *pvParametres) {
  for (;;) {

    xSemaphoreTake(SemBuzz, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    if (!g_dayTime) {

      Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  DETECTED");
      
      for (int thisNote = 0; thisNote < songLength; thisNote++) {
        int duration = 1000 / durations[thisNote];
        tone(BuzzerSensor, melody[thisNote], duration);
        int pause = duration * 0.3;
        delay(pause);
        noTone(BuzzerSensor);
        digitalWrite(LEDr, HIGH);
        delay(50);
        digitalWrite(LEDr, LOW);
        delay(50);
        digitalWrite(LEDy, HIGH);
        delay(50);
        digitalWrite(LEDy, LOW);
        delay(50);
      }

    }
    vTaskDelay(100);

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
      g_dayTime = HIGH;

    }
    //Lights are off
    else {

      digitalWrite(LEDs, HIGH);
      Serial.print("Its DARK, Turn on the LED : ");
      Serial.println(ldrStatus);
      g_dayTime = LOW;

    }
    vTaskDelay(200);

  }
}

static void vDhtSensor(void *pvParameters) {
  for (;;) {

    float h = dht.readHumidity();
    float t = dht.readTemperature(); // Read temperature as Celsius (the default)
    g_Humidity = h;
    g_Temperature = t;
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) ) {
      Serial.println(F("Failed to read from DHT sensor!"));
    }
    else {

      float hic = dht.computeHeatIndex(t, h, false);
      Serial.print(F("Humidity: "));
      Serial.print(h);
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.print(F("°C "));
    }
    vTaskDelay(500);
  }
}

static void vWifi(void *pvParameters)
{
  for (;;)
  {
    k = 0;
    //    Serial.println("Please Refresh your Page");
    while (k < 1000)
    {
      k++;
      while (Serial2.available())
      {
        Serial.println("Serial2.available");
        if (Serial2.find("0,CONNECT"))
        {
          Serial.println("Start Printing");
          Send();
          Serial.println("Done Printing");
          vTaskDelay(1000);
        }
      }
      vTaskDelay(1);
    }
  }
}

//------------------------------ Interrupt ------------------------------
void InterruptHandleFan() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemFan, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}

void InterruptHandleBuzz() {
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemBuzz, &xHigherPriorityTaskWoken);    // Semaphore give token to ISR.
}


//-------------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  dht.begin();
  //  wifi_init();
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
  pinMode(LEDr, OUTPUT);
  pinMode(LEDy, OUTPUT);
  pinMode(LEDg, OUTPUT);
  pinMode(LEDor, OUTPUT);
  
  Serial.println("Hello");

  //Interrupt
  attachInterrupt(FanInterruptPin, InterruptHandleFan, RISING);
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
