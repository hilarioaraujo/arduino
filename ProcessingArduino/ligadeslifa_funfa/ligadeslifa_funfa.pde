//tom 2011 teste bluetooth ligar Rele

#include <SoftwareSerial.h>
#define rxPin 0
#define txPin 1
int ledpin = 13; 
char val; 

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
byte pinState = 0;


void setup() { 
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    pinMode(ledpin, OUTPUT); 
    mySerial.begin(9600);  
} 
void loop() { 

        val = mySerial.read(); 
        mySerial.print(val);  
  
  
  
  
  
  
  switch (val) {
  case 'l':
    digitalWrite(ledpin, HIGH); 
    break;
  case 'd':
     digitalWrite(ledpin, LOW);
    break;
  default: 
     digitalWrite(ledpin, LOW);
}

}
  
  
  
  
  
  
