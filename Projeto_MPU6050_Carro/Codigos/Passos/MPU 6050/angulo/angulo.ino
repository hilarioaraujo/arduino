//Programa : Teste MPU-6050
//Alteracoes e adaptacoes : Arduino e Cia
//
//Baseado no programa original de JohnChi
 
//Carrega a biblioteca Wire
#include<Wire.h>
 
//Endereco I2C do MPU6050
const int MPU=0x68;  

//Variaveis para armazenar valores dos sensores
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,Roll,Pitch,Yaw;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
   
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);
    
}

void loop()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  
 //Solicita os dados do sensor MPU6050
  Wire.requestFrom(MPU,14,true);  
    
  
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Roll = FunctionsPitchRoll(AcX, AcY, AcZ);
  Pitch = FunctionsPitchRoll(AcY, AcX, AcZ);
  Yaw = FunctionsPitchRoll(AcZ, AcX, AcY); //Calcolo angolo Pitch
 
  //Mostra os valores no serial
  Serial.print("Acel. X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.print(AcZ);
  Serial.print(" | Gir. X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.print(GyZ);
  Serial.print(" | Temp = "); Serial.println(Tmp/340.00+36.53); //converte a temperatuda de F para Cº
  Serial.print("Roll: "); Serial.println(Roll);
  Serial.print("Pitch: "); Serial.println(Pitch);
  Serial.print("Yaw: "); Serial.println(Yaw);
//    

delay(300); //Aguarda 300 ms e reinicia o processo

}
//Função para Calcular, o Pitch e Roll.
double FunctionsPitchRoll(double A, double B, double C){
double DatoA, DatoB, Value;
DatoA = A;
DatoB = (B*B) + (C*C);
DatoB = sqrt(DatoB);

Value = atan2(DatoA, DatoB);
Value = Value * 180/3.14159265 ;

return (int)Value;
}



