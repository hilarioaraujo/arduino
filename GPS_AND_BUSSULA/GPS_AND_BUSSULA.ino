#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial GPS_Serial(13, 12); // RX, TX
SoftwareSerial BUSSULA_Serial(10, 11);

Servo Servo_Leme;
Servo Servo_Vela;

String stringGPS = "";
String stringPROA = "";

long previousMillis = 0;
long interval1 = 1000;
long interval2 = 2000;

unsigned long currentMillis = 0;


void setup()  {
  
  GPS_Serial.begin(57600);
  BUSSULA_Serial.begin(19200);   
  
  Servo_Leme.attach(8);
  Servo_Vela.attach(9);
  
  
Serial.begin(57600);  
}

//_________________________________________________________________________________
//_________________________________________________________________________________

void loop()   {
  
   currentMillis = millis();
   
   
   BUSSULA_Serial.begin(19200);
   while(currentMillis - previousMillis < interval1)
  { 
    Servo_Leme.write(15);
    
    currentMillis = millis();    
        if (BUSSULA_Serial.available())
  {
    
    char aux1 = BUSSULA_Serial.read();   
    if (aux1== 'P')
    {
      Serial.print("Proa: ");
      Serial.print(stringPROA);
      Serial.print('\n');
      stringPROA="";
    }
    else
    {   
      stringPROA = String(stringPROA + aux1);
    }    
    if(aux1== 'C')
      stringPROA="";   
    
  
//    Serial.write(BUSSULA_Serial.read()); 

    
  }
  }
  
  BUSSULA_Serial.end();  
  
  
  //______________________________________
  
  GPS_Serial.begin(57600);
  while((currentMillis - previousMillis > interval1)&& (currentMillis - previousMillis < interval2))
 {  
   Servo_Leme.write(150);
  currentMillis = millis();
      
  if (GPS_Serial.available())
  {   
    char c = GPS_Serial.read();   
    if (c== '$')
    {
      Serial.print("Latitude: ");
      Serial.print(stringGPS.substring(17,28));
      Serial.print("  ||  Longitude: ");
      Serial.print(stringGPS.substring(29,41));    
      stringGPS ="";    
      Serial.print('\n');
    }
    else
    {   
      stringGPS = String(stringGPS + c);
    }       
  }
}
GPS_Serial.end();

//_____________________________________________________________________________________________


if((currentMillis - previousMillis > interval1)&& (currentMillis - previousMillis > interval2))
{
previousMillis = currentMillis;
}
}




