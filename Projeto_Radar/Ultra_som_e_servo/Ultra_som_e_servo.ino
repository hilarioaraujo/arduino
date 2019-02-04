#define trigPin 19
#define echoPin 18
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
           myservo.attach(22);  // attaches the servo on pin 9 to the servo object
        //a proa do navio serrá no angulo de motor 160/2=80
          // put your setup code here, to run once:
          Serial.begin (9600);
          pinMode(trigPin, OUTPUT);
          pinMode(echoPin, INPUT);
}

void loop() {
            // put your main code here, to run repeatedly:
          //myservo.write(82.5);
         // delay(100000);
          int anguloservo;
          int distancialcd;
          for (pos = 0; pos <= 160; pos += 1) { // goes from 0 degrees to 160 degrees
              // in steps of 1 degree
              myservo.write(pos);              // tell servo to go to position in variable 'pos'
              anguloservo= 7.5 + myservo.read(); // Soma se 7.5 para obter um circulo trigonomético com os angulos definidos como os conhecemos na matemática
              Serial.println("Angulo:");              
              Serial.print(anguloservo);
          long duration, distance; //http://arduino.cc/en/Reference/Long
            digitalWrite(trigPin, LOW);  //seta o pino 12 com um pulso baixo "LOW" ou desligado ou ainda 0
            delayMicroseconds(2); // delay de 2 microssegundos
           
            digitalWrite(trigPin, HIGH); //seta o pino 12 com pulso alto "HIGH" ou ligado ou ainda 1
            delayMicroseconds(10);  //delay de 10 microssegundos
            digitalWrite(trigPin, LOW); //seta o pino 12 com pulso baixo novamente
            duration = pulseIn(echoPin, HIGH);  //pulseIn lê o tempo entre a chamada e o pino entrar em high
            //Esse calculo é baseado em s = v . t, lembrando que o tempo vem dobrado
            //porque é o tempo de ida e volta do ultrassom
            distance = (duration/2) / 29.1;
            if (distance >= 700 || distance <= 0){
               Serial.println("");
              Serial.println("Nada ao alcance!");
             
            }
            else {

              distancialcd = round(distance*119)/400;
               Serial.println("");
               Serial.println("Distancia para lcd:");              
              Serial.print(distancialcd);
              Serial.println("Alvo detetado a:");              
              Serial.print(distance);
              Serial.println(" cm");
              
            }
            delay(150);                       // waits 15ms for the servo to reach the position
  }

   for (pos = 160; pos >= 0; pos -= 1) { // goes from 160 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    anguloservo= 7.5 + myservo.read(); // Soma se 7.5 para obter um circulo trigonomético com os angulos definidos como os conhecemos na matemática
              Serial.println("Angulo:");              
              Serial.print(anguloservo);
   
    long duration, distance; //http://arduino.cc/en/Reference/Long
            digitalWrite(trigPin, LOW);  //seta o pino 12 com um pulso baixo "LOW" ou desligado ou ainda 0
            delayMicroseconds(2); // delay de 2 microssegundos
           
            digitalWrite(trigPin, HIGH); //seta o pino 12 com pulso alto "HIGH" ou ligado ou ainda 1
            delayMicroseconds(10);  //delay de 10 microssegundos
            digitalWrite(trigPin, LOW); //seta o pino 12 com pulso baixo novamente
            duration = pulseIn(echoPin, HIGH);  //pulseIn lê o tempo entre a chamada e o pino entrar em high
            //Esse calculo é baseado em s = v . t, lembrando que o tempo vem dobrado
            //porque é o tempo de ida e volta do ultrassom
            distance = (duration/2) / 29.1;
            if (distance >= 700 || distance <= 0){
              Serial.println("Nada ao alcance!");
             
            }
            else {
               distancialcd = round(distance*119)/400;
               Serial.println("");
               Serial.println("Distancia para lcd:");              
              Serial.print(distancialcd);
              Serial.println("Alvo detetado a:");              
              Serial.print(distance);
              Serial.println(" cm");
            }
             delay(150);                       // waits 15ms for the servo to reach the position
  }
}
