#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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

 Serial.begin(9600);
 lcd.begin(COLUNAS,LINHAS);

 lcd.setCursor(0,0); 
 lcd.print("Marinha de ");
 lcd.setCursor(0,1);
 lcd.print("     Guerra"); 
 
}

void loop(){

 // Turn off the display:
  lcd.noDisplay();
  delay(500);
   // Turn on the display:
  lcd.display();
  delay(500);

 
}
