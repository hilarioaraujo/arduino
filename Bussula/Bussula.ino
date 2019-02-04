
#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
Serial.begin(57600);
pinMode(12, OUTPUT);
digitalWrite(12, HIGH);
pinMode(9, OUTPUT);
digitalWrite(9, LOW);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(19200);
}

void loop() // run over and over
{
  if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
}

