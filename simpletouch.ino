#include <Wire.h>  
// Define connected pin using PB7 and using led on stm32 PC13
int touchpin = PB7;
int ledpin = PC13; 

// Setup digital pin touchpin as input and ledpin as an output
void setup() 
{ 
  Serial.begin(9600); // Showing result in serial monitor with speed as 9600
  pinMode(ledpin, OUTPUT);
  pinMode(touchpin, INPUT);
}

void loop() {
  int touchval = digitalRead(touchpin);
  if (touchval == HIGH) {
    Serial.println("Touched"); // Showing "Touched" on serial monitor when touchval is HIGH signal
    digitalWrite(ledpin, HIGH); // Led on stm32 will blinking when touch sensor detected
  } else {
    digitalWrite(ledpin, LOW); // If touch senser not detected led on stm32 off
  }
  delay(1000); // Waits for 1 second
}
