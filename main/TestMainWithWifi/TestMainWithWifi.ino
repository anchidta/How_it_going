//---------------------------- Libraries ---------------------------------
#include <MapleFreeRTOS821.h>
#include <Wire.h>
#include "DHT.h"

//-------------------------------- Pins --------------------------------
#define BuzzerSensor PA1
#define DhtSensor PA0
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

String webpage = "";                                 //String variable to store characters
int i = 0, k = 0, x = 0;                                 //integer variables
String readString;                                   //using readString feature to read characters

boolean No_IP = false;                               //boolean variables
String IP = "";                                       //String variable to store data
char temp1 = '0';                                    //character variable

String name = "<p>Circuit Digest</p><p>A community of electrical and electronics students, engineers and makers</p>"; //String with html notations
String data = "<p>Data Received Successfully.....</p>";   //String with html
float data_int = 0.5;

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
bool debug = false;
////------------------------- Functions ----------------------------------

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

void wifi_init()                                //This function contains AT commands that passes to connect_wifi()
{
  connect_wifi("AT", 100);                  //Sends AT command with time(Command for Achknowledgement)
  connect_wifi("AT+CWMODE=3", 100);         //Sends AT command with time (For setting mode of Wifi)
  connect_wifi("AT+CWQAP", 100);           //Sends AT command with time (for Quit AP)
  connect_wifi("AT+RST", 5000);            //Sends AT command with time (For RESETTING WIFI)
  check4IP(5000);
  if (!No_IP)
  {

    Serial.println("Connecting Wifi....");
    connect_wifi("AT+CWJAP=\"IMARC_Net\",\"iostab00\"", 7000);        //provide your WiFi username and password here

  }
  else
  {
  }
  Serial.println("Wifi Connected");
  get_ip();

  connect_wifi("AT+CIPMUX=1", 100);                         //Sends AT command with time (For creating multiple connections)
  connect_wifi("AT+CIPSERVER=1,80", 100);                   //Sends AT command with time (For setting up server with port 80)
}

void connect_wifi(String cmd, int t)                  //This function is for connecting ESP8266 with wifi network by using AT commands
{
  int temp = 0, i = 0;
  while (1)
  {
    Serial.println(cmd);                  //Sends to serial monitor
    Serial2.println(cmd);                 //sends to ESP8266 via serial communication
    while (Serial2.available())
    {
      if (Serial2.find("OK"))
        i = 8;
    }
    delay(t);
    if (i > 5)
      break;
    i++;
  }
  if (i == 8)
    Serial.println("OK");
  else
    Serial.println("Error");
}

void get_ip()                                           //After cheacking ip ,this is a function to get IP address
{
  IP = "";
  char ch = 0;
  while (1)
  {
    Serial2.println("AT+CIFSR");                   //GET IP AT COMMAND
    while (Serial2.available() > 0)
    {
      if (Serial2.find("STAIP,"))                  //This finds the STAIP that is the STATIC IP ADDRESS of ESP8266
      {
        delay(1000);
        Serial.print("IP Address:");
        while (Serial2.available() > 0)
        {
          ch = Serial2.read();                    //Serial2 reads from ESP8266
          if (ch == '+')
            break;
          IP += ch;
        }
      }
      if (ch == '+')
        break;
    }
    if (ch == '+')
      break;
    delay(1000);
  }
  Serial.print(IP);                                //prints IP address in Serial monitor
  Serial.print("Port:");
  Serial.println(80);
}

void check4IP(int t1)                                     //A function to check ip of ESP8266
{
  int t2 = millis();
  while (t2 + t1 > millis())
  {
    while (Serial2.available() > 0)
    {
      if (Serial2.find("WIFI GOT IP"))
      {
        No_IP = true;
      }
    }
  }
}

void sendwebdata(String webPage)                          //This function is used to send webpage datas to the localserver
{
  int ii = 0;
  while (1)
  {
    unsigned int l = webPage.length();
    Serial.print("AT+CIPSEND=0,");
    Serial2.print("AT+CIPSEND=0,");
    Serial.println(l + 2);
    Serial2.println(l + 2);
    vTaskDelay(100);
    Serial.println(webPage);                        //sends webpage data to serial monitor
    Serial2.println(webPage);                       //sends webpage data to serial2 ESP8266
    while (Serial2.available())
    {
      if (Serial2.find("OK"))
      {
        Serial.println("OK");
        ii = 11;
        break;
      }
    }
    if (ii == 11)
      break;
    vTaskDelay(100);
  }
}

void Send()                                        //This function contains data to be sent to local server
{
  webpage = "<h1>Welcome !</h1>";
  sendwebdata(webpage);
  vTaskDelay(100);
  webpage = "<h2>Current :</h2><h3>";
  webpage += g_Current;
  webpage += "</h3>";
  sendwebdata(webpage);
  vTaskDelay(100);
  webpage = "<h2>Humidity :</h2><h3>";
  webpage += g_Humidity;
  webpage += "</h3>";
  sendwebdata(webpage);
  vTaskDelay(100);
  webpage = "<h2>Temperature :</h2><h3>";
  webpage += g_Temperature;
  webpage += "</h3>";
  sendwebdata(webpage);
  vTaskDelay(100);
  Serial2.println("AT+CIPCLOSE=0");                  //Closes the server connection
  Serial.println("CIPCLOSE");
}

//---------------------------------------- Tasks ------------------------
static void vCurrentSensor(void *pvParameters) {
  for (;;) {
    nVPP = getVPP();
    nCurrThruResistorPP = (nVPP / 200.0) * 1000.0;
    nCurrThruResistorRMS = nCurrThruResistorPP * 0.707;
    nCurrentThruWire = nCurrThruResistorRMS * 1000;
    g_Current = nCurrentThruWire;
    if (debug) {
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
    }
    vTaskDelay(1000);
  }
}

static void vFanOn(void *pvParametres) {
  for (;;) {

    xSemaphoreTake(SemFan, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    if (debug) {
      Serial.println("Change Fan State");
    }
    g_fanState = !g_fanState;                         // State Toggle.
    digitalWrite(FanInB, g_fanState);

  }
}

static void vBuzzerOn(void *pvParametres) {
  for (;;) {

    xSemaphoreTake(SemBuzz, portMAX_DELAY);      // Max delay time in board because don't know when interrupt begin
    if (!g_dayTime) {

      Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  DETECTED");
      analogWrite(BuzzerSensor, 230);
      g_buzzState = HIGH;
      vTaskDelay(3000);
      analogWrite(BuzzerSensor, 0);
      g_buzzState = LOW;

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
      if (debug) {
        Serial.print("Its BRIGHT, Turn off the LED : ");

        Serial.println(ldrStatus);
      }
      g_dayTime = HIGH;

    }
    //Lights are off
    else {

      digitalWrite(LEDs, HIGH);
      if (debug) {
        Serial.print("Its DARK, Turn on the LED : ");
        Serial.println(ldrStatus);
      }
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
      if (debug) {
        Serial.print(F("Humidity: "));
        Serial.print(h);
        Serial.print(F("%  Temperature: "));
        Serial.print(t);
        Serial.print(F("°C "));
      }
    }
    vTaskDelay(500);
  }
}

static void vWifi(void *pvParameters)
{
  for (;;)
  {
    //    int incomming = Serial2.read();
    k = 0;
    //        Serial.println("Please Refresh your Page");
    while (k < 1000)
    {
      k++;
      while (Serial2.available())
      {
        Serial.println("Serial2.available");
        //        Serial.write(Serial2.read());
        //        if (Serial2.find("OK"))
        //        {
        Serial.println("Start Printing");
        Send();
        Serial.println("Done Printing");
        vTaskDelay(2000);
        //        }
        //        else{
        //          Serial.print("Nothing found");
        //          }
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
  wifi_init();
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
  xTaskCreate(vWifi,
              "Wifi Task",
              configMINIMAL_STACK_SIZE + 100,
              NULL,
              configMAX_PRIORITIES - 1,
              NULL);

  noInterrupts();
  vTaskStartScheduler();
  interrupts();
}

void loop() {}
