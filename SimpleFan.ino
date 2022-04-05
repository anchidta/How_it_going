#include <Wire.h>  
// Define connected pin using PB8 and PB9
int INA = PB8;
int INB = PB9;

// Setup digital pin INA and INB as an output
void setup() 
{ 
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT); 
} 

void loop() 
{ 
 // Forward
  digitalWrite(INA,LOW);
  digitalWrite(INB,HIGH); 
  delay(2000);              // wait for 2 second
  
  // Stop
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW); 
  delay(5000);             // wait for 5 second

  // Backward
  digitalWrite(INA,HIGH);
  digitalWrite(INB,LOW); 
  delay(2000);            // wait for 2 second

  // Stop
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW); 
  delay(5000);            // wait for 5 second
}
