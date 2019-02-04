//tom 2011 teste bluetooth ligar Rele

#include <SoftwareSerial.h>
#define rxPin 0
#define txPin 1
char val; 

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
byte pinState = 0;


void setup() { 
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    pinMode(22, OUTPUT); 
    mySerial.begin(9600);  
} 
void loop() { 

        val = mySerial.read(); 
        mySerial.print(val);  
  
  
  
  
  
  
  switch (val) {
  case 'on':
    digitalWrite(22, 1); 
    break;
  case 'off':
     digitalWrite(22, 0);
    break;
  default: 
     digitalWrite(22, 0);
}

}
  
  
  
  
  
  
