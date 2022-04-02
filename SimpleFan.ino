//Define connected pin using PB8 and PB9
#define INA = PB8;
#define INB = PB9;

//Setup digital pin INA and INB as an output
void setup() 
{ 
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT); 
} 

void loop() 
{ 
 // LEFT
  digitalWrite(INA,LOW);
  digitalWrite(INB,HIGH); 
  delay(2000);              //wait for 2 second
  
  // STOP
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW); 
  delay(5000);             //wait for 5 second

  // RIGHT
  digitalWrite(INA,HIGH);
  digitalWrite(INB,LOW); 
  delay(2000);            //wait for 2 second

  // STOP
  digitalWrite(INA,LOW);
  digitalWrite(INB,LOW); 
  delay(5000);            //wait for 5 second
}
