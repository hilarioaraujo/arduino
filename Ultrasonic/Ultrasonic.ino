#define trigPin 19
#define echoPin 18
#define ledRed 8
#define ledBlue 7
 
void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 
void loop() {
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
    Serial.print(distance);
    Serial.println(" cm");
    
  }
  delay(500);
}
