

// ================================================================
// ===               SETUP INICIAL              ===
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;
#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
MPU6050 accelgyro(0x68); 

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yawOffset; //offset (deslocamento) do angulo Yaw
double yaw;   //angulo yaw, que irá determinar o angulo em que estará o tanque
int yq=1;
double posic_x, posic_y, posic_z;


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int16_t ax, ay, az,gx, gy, gz;


///////////////////////////////////   Deteção de Interrupção   /////////////////////////////
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


///////////////////////////////////   CONFIGURAÇÃO DA CALIBRACAO   /////////////////////////////
int buffersize=3000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=6;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)



///////////////////////////////////   Variáveis do Sistema de Controlo   /////////////////////////////
int k=1; //constante do sistema de controlo K
int w; // velocidade angular
double vl, vr; // velocidade da lagarta esquerda e direita respetivamente em RPM's dos motores
double vm = 300; //velocidade média do tanque em RPM's do motor.




///////////////////////////////////   Alvos de Navegação   /////////////////////////////
double alvo_x=50;
double alvo_y=50;
double alvo_angulo, erro;



// ================================================================
// ===                      SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    
    //CONFIGURAR O MPU6050
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("A configurar o Sensor MPU6050..."));
    devStatus = mpu.dmpInitialize();

    
    while (Serial.available() && Serial.read()); // empty buffer           
    while (Serial.available() && Serial.read()); // empty buffer again
    Serial.println(accelgyro.testConnection() ? "A conecção ao MPU6050 foi realizada com sucesso" : "Erro na conecção ao MPU6050");
    delay(1000);
    
    // Reset aos deslocamentos do Acel e Gyro
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
  

///////////////////////////////////   CALIBRACAO   /////////////////////////////
   if (state==0){
        Serial.println("\A recolher os de valores do gyro e do acel");
        meansensors();
        state++;
        delay(1000);
    }

  if (state==1) {
        Serial.println("\A calcular os deslocamentos");
        calibration();
        state++;
        delay(1000);
    }

  if (state==2) {
          meansensors();
          Serial.println("\nCalibração concluída!");
          Serial.print("\nDados do Sensor MPU 6050 com os deslocamentos calculados:\t");
          Serial.print(mean_ax); 
          Serial.print("\t");
          Serial.print(mean_ay); 
          Serial.print("\t");
          Serial.print(mean_az); 
          Serial.print("\t");
          Serial.print(mean_gx); 
          Serial.print("\t");
          Serial.print(mean_gy); 
          Serial.print("\t");
          Serial.println(mean_gz);
          Serial.print("Deslocamentos Calculados:\t");
          Serial.print(ax_offset); 
          Serial.print("\t");
          Serial.print(ay_offset); 
          Serial.print("\t");
          Serial.print(az_offset); 
          Serial.print("\t");
          Serial.print(gx_offset); 
          Serial.print("\t");
          Serial.print(gy_offset); 
          Serial.print("\t");
          Serial.println(gz_offset); 
          Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
            
          // Aplicar os deslocamentos de giroscópios, dimensionados para sensibilidade mínima
          mpu.setXAccelOffset(ax_offset);
          mpu.setYAccelOffset(ay_offset);
          mpu.setZAccelOffset(az_offset);    
          mpu.setXGyroOffset(gx_offset);
          mpu.setYGyroOffset(gy_offset);
          mpu.setZGyroOffset(gz_offset);
         
          state++;    
    }


    if (devStatus == 0) {
        //Ligar o MPU 6050 uma vez que já foi configurado
        Serial.println(F("A iniciar o DMP do MPU6050..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("A ativar o pin de interrupção do sensor MPU 6050..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP pronto, á espera do primeira interrupção do arduino"));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("Falha na inicialização do DMP (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);   


}


// ================================================================
// ===                  Programa Principal                   ===
// ================================================================  

void loop() {

     // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO está em sobrecarga!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

//Recebe os dados do acelarometro e do giroscópio com os deslocamentos
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Serial.print("Valores de posicçaõ do Acel e Giro:\t");
        Serial.print(ax); 
        Serial.print("\t");
        Serial.print(ay); 
        Serial.print("\t");
        Serial.print(az); 
        Serial.print("\t");
        Serial.print(gx); 
        Serial.print("\t");
        Serial.print(gy); 
        Serial.print("\t");
        Serial.println(gz); 
        Serial.println("\nOs dados são: acelX acelY acelZ giroX giroY giroZ");
       
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        //Calcular o angulo Yaw no momento
        yaw=ypr[0] * 180/M_PI;

        //Calcular o offSet do Yaw pela 1º vez de acordo com a orientação inicial, isto é angulo 0 será para onde o tanque está orientado
        if (yq==1){
            yawOffset = yaw;
            yq++;
        }
        
        yaw = yaw - yawOffset; //Calcula o angulo yaw com o offset calculado anteriormente
        
        Serial.print("Yaw: ");
        Serial.println(yaw);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);


///////////////////////////////////   CALCULOS PARA O ALVO   /////////////////////////////

        alvo_angulo=atan(alvo_y/alvo_x);
        erro=yaw-alvo_angulo;













        
    }
}









// ================================================================
// ===                  Funções                   ===
// ================================================================  

///////////////////////////////////   Funcões de Calibracao   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}



