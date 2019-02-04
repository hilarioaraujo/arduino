// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A1 // Command/Data goes to Analog 2
#define LCD_WR A4 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A5 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
const float pi = 3.141591;

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Adafruit_TFTLCD tft;

#define trigPin 19 // defenir pin  19 como recetor
#define echoPin 18 // defenir pin  18 como transmissor
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // definir angulo inicial do servo inicial = 0
int alcancemax = 150; // alcance max do sonar em cm
int xlcd;
int ylcd;
int distcircmenor;
int distcircmaior;

void setup() {
          myservo.attach(22);  // ligação do servo ao pin digital 22
          Serial.begin (9600);
          pinMode(trigPin, OUTPUT);
          pinMode(echoPin, INPUT);
          uint16_t identifier = tft.readID();
          identifier=0x9341; // identificar o tftlcd como 0x9341
          tft.begin(identifier); // iniciar o tftlcd
          tft.setRotation(2); // Rodar o tftlcd 180º         
          tft.setCursor(0, 0); // selecionar onde se irá escrever inicialmente
          tft.setTextColor(WHITE); // selecionar a cor branca para o que irá ser escrito 
          tft.setTextSize(2); // definir tamanho de letra como 2
          tft.println("Sonar 2.0");
          tft.setTextSize(1); // definir tamanho de letra como 1
          tft.println("O Software ira"); // Escrever no tftlcd
          tft.println("arrancar dentro");
          tft.println("de momentos.");
          tft.println("");
          tft.println("Software desenvolvido");
          tft.println("por:");
          tft.println("CAD R. ARAUJO");
          tft.println("CAD P. ROCHA");
          tft.println("CAD M. NUNES");
          
          myservo.write(pos); // colocar o servo no angulo inicial
          delay(3000);

          tft.fillScreen(BLACK); // Fazer um reset ao tftlcd, preenchendo-o a cor preta

          
          tft.setCursor(2, 4);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.println("HEAD UP");
          
          tft.setCursor(3, 289);
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.println("Sonar 2.0");
          
          tft.setCursor(170, 289);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.println("Alcance:");
          
          tft.setCursor(170, 299);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(alcancemax); // escreve o valor que o programador determinou como sendo o alcance max do sonar

          tft.setCursor(190, 299);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.println("cm");



          distcircmenor=alcancemax/3;// formula para calcular a distancia real do circulo intermédio em cm
          tft.setCursor(118, 202);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(distcircmenor); // escreve no circulo mais pequeno a distancia real nesse circulo em cm

          distcircmaior=(2*alcancemax)/3;// formula para calcular a distancia real do circulo intermédio em cm
          tft.setCursor(116, 242);
          tft.setTextColor(WHITE);
          tft.setTextSize(1);
          tft.print(distcircmaior);// escreve no circulo mais pequeno a distancia real nesse circulo em cm


          
          
          tft.drawCircle(119, 159, 119, CYAN); // desenha um circulo a azul ciano no centro do tftlcd (119,159) com raio (119) - circulo maior e limitante
          tft.drawCircle(119, 159, 79, CYAN); // desenha um circulo a azul ciano no centro do tftlcd (119,159) com raio (79) - circulo intermédio
          tft.drawCircle(119, 159, 40, CYAN); // desenha um circulo a azul ciano no centro do tftlcd (119,159) com raio (40)- circulo mais pequeno
          tft.fillTriangle(119, 153, 115, 163, 123, 163, WHITE); // desenha um tringulo preenchido para simular o navio;
}

void loop() {

          float anguloservo;
          double distancialcd;
          for (pos = 0; pos <= 160; pos += 1) { // o angulo do servo irá variar de 0 até 160
              myservo.write(pos);              // escrever no servo o angulo fornecido pelo ciclo for
              anguloservo= ((pi*(-7.5 - myservo.read()))/180); // Subtrai se  7.5 para obter um circulo trigonomético com os angulos definidos como os conhecemos na matemática, para este lcd
              
              Serial.print("Angulo para lcd (em rad): ");
              Serial.println(anguloservo);//mostrar no serial o valor do angulo final em radianos
              Serial.print("Angulo para lcd (em deg): ");
              Serial.println(pos+7.5);//mostrar no serial o valor do angulo final em graus
              
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
            
              if (distance >= alcancemax || distance <= 0){ // caso a distancia seja 0 ou maior que o alcanse definido
                  Serial.println("Nada ao alcance!");
                  Serial.println("");
                  distancialcd = 119; // definimos a distancia como sendo o alcanse máximo
                  // através das coordenadas polares podemos saber as coordenadas dos pontos. 
                  // x= a+pcos(angulo), sendo que o a = à abcissa do centro, e p a distancia ao centro
                  // y=b+psen(angulo), sendo que o b = á ordenada do centro, e p a distancia ao centro
                  
                  xlcd= round(119+(distancialcd*cos(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  ylcd= round(159+(distancialcd*sin(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, RED ); // desenha uma linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  
                  delay(100);// compasso de espera de 100 ms para se poder ver a linha vermelha a percorrer os angulos todos
                  
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, BLACK ); // apaga a linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  tft.drawCircle(119, 159, 119, CYAN); // desenha de novo o circulo a azul ciano no centro do tftlcd (119,159) com raio (119) - circulo maior e limitante
                  tft.drawCircle(119, 159, 79, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (79) - circulo intermédio
                  tft.drawCircle(119, 159, 40, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (40)- circulo mais pequeno
                  tft.fillTriangle(119, 153, 115, 163, 123, 163, WHITE); // desenha de novo o tringulo preenchido para simular o navio;
              
              }
              
              else {
                  distancialcd = round((distance*119)/alcancemax); // converte a distancia real em pixeis de acordo com o alcance maximo definido
                  xlcd= round(119+(distancialcd*cos(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  ylcd= round(159+(distancialcd*sin(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, RED ); //desenha uma linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  tft.drawPixel(xlcd, ylcd, GREEN ); // desenha o ponto que detetou o contacto a verde
                  
                  delay(100); // compasso de espera de 100 ms para se poder ver a linha vermelha a percorrer os angulos todos
                  
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, BLACK );
                  tft.drawCircle(119, 159, 119, CYAN); // desenha de novo o circulo a azul ciano no centro do tftlcd (119,159) com raio (119) - circulo maior e limitante
                  tft.drawCircle(119, 159, 79, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (79) - circulo intermédio
                  tft.drawCircle(119, 159, 40, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (40)- circulo mais pequeno
                  tft.drawPixel(xlcd, ylcd, GREEN ); // desenha o ponto que detetou o contacto a verde, serve de redundancia.
                  tft.fillTriangle(119, 153, 115, 163, 123, 163, WHITE); // desenha de novo o tringulo preenchido para simular o navio;
                  
                  Serial.print("Distancia para lcd: ");              
                  Serial.println(distancialcd);//mostrar no serial a distancia que será usada na conversão em coordenadas polares (p)
                  Serial.print("Alvo detetado a: ");              
                  Serial.print(distance); ////mostrar no serial o valor real da distancia do contacto em cm
                  Serial.println(" cm");
                  Serial.println(" ");
              }
          
              delay(50);                       // waits 50ms for the servo to reach the position
        }

        for (pos = 160; pos >= 0; pos -= 1) { // goes from 160 degrees to 0 degrees
                myservo.write(pos);              // tell servo to go to position in variable 'pos'
                anguloservo= ((pi*(-7.5- myservo.read()))/180); // Subtrai se  7.5 para obter um circulo trigonomético com os angulos definidos como os conhecemos na matemática, para este lcd
                
                Serial.print("Angulo:");
                Serial.println(anguloservo);   //mostrar no serial o valor do angulo final em radianos
                Serial.print("Angulo para lcd (em deg): ");
                Serial.println(pos+7.5);          //mostrar no serial o valor do angulo final em graus
                
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
                
              if (distance >= alcancemax || distance <= 0){ // caso a distancia seja 0 ou maior que o alcanse definido
                  Serial.println("Nada ao alcance!");
                  Serial.println(" ");
                  distancialcd = 119; // definimos a distancia como sendo o alcanse máximo
                  // através das coordenadas polares podemos saber as coordenadas dos pontos. 
                  // x= a+pcos(angulo), sendo que o a = à abcissa do centro, e p a distancia ao centro
                  // y=b+psen(angulo), sendo que o b = á ordenada do centro, e p a distancia ao centro
                  
                  xlcd= round(119+(distancialcd*cos(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  ylcd= round(159+(distancialcd*sin(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, RED ); // desenha uma linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  
                  delay(100);// compasso de espera de 100 ms para se poder ver a linha vermelha a percorrer os angulos todos
                  
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, BLACK ); // apaga a linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  tft.drawCircle(119, 159, 119, CYAN); // desenha de novo o circulo a azul ciano no centro do tftlcd (119,159) com raio (119) - circulo maior e limitante
                  tft.drawCircle(119, 159, 79, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (79) - circulo intermédio
                  tft.drawCircle(119, 159, 40, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (40)- circulo mais pequeno
                  tft.fillTriangle(119, 153, 115, 163, 123, 163, WHITE); // desenha de novo o tringulo preenchido para simular o navio;
              

              }
              
              else {
                  distancialcd = round((distance*119)/alcancemax); // converte a distancia real em pixeis de acordo com o alcance maximo definido
                  xlcd= round(119+(distancialcd*cos(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  ylcd= round(159+(distancialcd*sin(anguloservo))); //arrendondamento para converter a distancia  e o angulo do servo para pixel
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, RED ); //desenha uma linha vermelha desde o centro da circunferencia (119,159) até ao ponto (xlcd-1, ylcd-1)
                  tft.drawPixel(xlcd, ylcd, GREEN ); // desenha o ponto que detetou o contacto a verde
                  
                  delay(100); // compasso de espera de 100 ms para se poder ver a linha vermelha a percorrer os angulos todos
                  
                  tft.drawLine(119, 159, xlcd-1, ylcd-1, BLACK );
                  tft.drawCircle(119, 159, 119, CYAN); // desenha de novo o circulo a azul ciano no centro do tftlcd (119,159) com raio (119) - circulo maior e limitante
                  tft.drawCircle(119, 159, 79, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (79) - circulo intermédio
                  tft.drawCircle(119, 159, 40, CYAN); // desenha de novo o circulo azul ciano no centro do tftlcd (119,159) com raio (40)- circulo mais pequeno
                  tft.drawPixel(xlcd, ylcd, GREEN ); // desenha o ponto que detetou o contacto a verde, serve de redundancia.
                  tft.fillTriangle(119, 153, 115, 163, 123, 163, WHITE); // desenha de novo o tringulo preenchido para simular o navio;
                  

                  Serial.print("Distancia para lcd: ");              
                  Serial.println(distancialcd);//mostrar no serial a distancia que será usada na conversão em coordenadas polares (p)
                  Serial.print("Alvo detetado a: ");              
                  Serial.print(distance); ////mostrar no serial o valor real da distancia do contacto em cm
                  Serial.println(" cm");
                  Serial.println(" ");
                }
                delay(50);                       // espera de 50 ms para voltar a começar de novo o ciclo.
      }
}


