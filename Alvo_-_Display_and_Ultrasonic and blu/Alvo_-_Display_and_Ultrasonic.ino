#define trigPin 13
#define echoPin 12
#define ledRed 8
#define ledBlue 7
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <SoftwareSerial.h>
#define rxPin 0
#define txPin 1
char val; 

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
byte pinState = 0;

#define LCD_ADDR 0x27 // I2C address
#define EN 2 // LCD En (Enable)
#define RW 1 // LCD Rw (Read/write)
#define RS 0 // LCD Rs (Reset)
#define D4 4 // LCD data 0
#define D5 5 // LCD data 1
#define D6 6 // LCD data 2
#define D7 7 // LCD data 3
#define BACKLIGHT_PIN 3
#define BACKLIGHT_POL POSITIVE //ligar ou nao o backlight (luz de fundo). Valor pode ser POSITIVE ou NEGATIVE

#define COLUNAS 16 // quantidade de colunas do LCD
#define LINHAS 2 // quantidade de linhas do LCD

LiquidCrystal_I2C lcd(LCD_ADDR, EN, RW, RS, D4, D5, D6, D7, BACKLIGHT_PIN, BACKLIGHT_POL);

void setup() {
  Serial.begin (9600);
  lcd.begin(COLUNAS,LINHAS);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
    lcd.setCursor(0,0); 
    lcd.print("Nada ao ");
    lcd.setCursor(0,1);
    lcd.print("   alcance"); 
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
    lcd.setCursor(0,0); 
    lcd.print("O alvo esta ");
    lcd.setCursor(0,1);
    lcd.print(distance); 
    lcd.print(" cm"); 
  }
  
  delay(10000);
  lcd.setCursor(0,1);
  lcd.print("            ");
   lcd.setCursor(0,0);
  lcd.print("            ");

case 'off':
     digitalWrite(22, 0);
    break;
    
}
  }

  
  
