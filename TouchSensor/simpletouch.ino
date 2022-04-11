#include <Wire.h>  
// Define connected pin using PB7 
int touchpin = PB7;

// Setup digital pin touchpin as input
void setup() 
{ 
  Serial.begin(9600); // Showing result in serial monitor with speed as 9600
  pinMode(touchpin, INPUT);
}

void loop() {
  int touchval = digitalRead(touchpin);
  if (touchval == HIGH) {
    Serial.println("Touched"); // Showing "Touched" on serial monitor when touchval is HIGH signal
  } 
  delay(500); // Waits for 0.5 second
}
