  
// Programa : Teste LCD 20x4 Arduino
// Autor : Arduino e Cia

// Carrega a biblioteca do LCD
#include <LiquidCrystal.h>

// Inicializa o LCD
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

void setup()
{
  // Define o LCD com 20 colunas e 4 linhas
  lcd.begin(20, 4);
  
  // Mostra informacoes no display
  lcd.setCursor(2,0);
  lcd.print("Bom dia sunshine,");
  lcd.setCursor(0,1);
  lcd.print("hoje sonhei contigo!");
  lcd.setCursor(5,2);
  lcd.print("Miss you :)");
    lcd.setCursor(3,3);
  lcd.print("Adoro - te <3");
}

void loop()
{
  // Seu codigo aqui
}
