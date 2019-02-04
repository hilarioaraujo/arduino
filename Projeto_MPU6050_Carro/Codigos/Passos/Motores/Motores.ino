
 /*Pinagem do arduino*/
 
//motor_A
int IN1 = 4 ;
int IN2 = 5 ;
int velocidadeA = 3; //motor da direira
 
//motor_B
int IN3 = 6 ;
int IN4 = 7 ;
int velocidadeB = 8; //motor da esquerda
 
//variavel auxiliar
int velocidade = 0;
 
//Inicializa Pinos
void setup(){
pinMode(IN1,OUTPUT);
pinMode(IN2,OUTPUT);
pinMode(IN3,OUTPUT);
pinMode(IN4,OUTPUT);
pinMode(velocidadeA,OUTPUT);
pinMode(velocidadeB,OUTPUT);
}
 
void loop(){
 
/*Exemplo de velocidades no motor A*/
 
//Sentido 2
digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
 
//Alta
analogWrite(velocidadeA,225);


// 
////Baixa
//analogWrite(velocidadeA,80);
 
/*Exemplo de variacao de velocidade no motor B*/
 
//Sentido 2

//velocidade de 0 a 255
//while (velocidadeB &lt; 255){
//analogWrite(velocidadeB,velocidade);
//velocidade = velocidade + 10;
//delay(50);
//}
////velocidade de 255 a 0
//while (velocidadeB &gt; 0){
//analogWrite(velocidadeB,velocidade);
//velocidade = velocidade - 10;
//delay(50);
//}
}
