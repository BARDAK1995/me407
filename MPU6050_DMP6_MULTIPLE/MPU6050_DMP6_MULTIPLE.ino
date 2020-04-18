
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu1;

MPU6050 mpu2(0x69);

#define OUTPUT_READABLE_YAWPITCHROLL

#define MPU6050_1_INT_PIN 3
#define MPU6050_1_INT digitalPinToInterrupt(MPU6050_1_INT_PIN)

#define MPU6050_2_INT_PIN 2
#define MPU6050_2_INT digitalPinToInterrupt(MPU6050_2_INT_PIN)

bool blinkState = false;
int Pitch_RapidPin=11;
int Roll_RapidPin=12;
int Pitch_CheckPin=10;
int Roll_CheckPin=13;
int Pitch_DirPin=5;
int Roll_DirPin=6;
int levelingDonePin=4;
float pitch_1=0;
float roll_1=0;
float pitch_2=0;
float roll_2=0;

int firstNumberFirst2digits=0;
int firstNumberNEXT2digits=0;
int secondNumberFirst2digits=5;
int secondNumberNEXT2digits=1;

byte pitchTamsayi=0;
byte pitchKusurat=0;
byte rollTamsayi=0;
byte rollKusurat=0;

int i2cCounter=0;

unsigned long currentMillis = 0; 
unsigned long initialMilllis = 0;
unsigned long previousMillis=0;
bool millischeck=false;

// MPU control/status vars
bool dmpReady_1 = false;  // set true if DMP init was successful
bool dmpReady_2 = false;
uint8_t mpuIntStatus_1;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus_2; 
uint8_t devStatus_1 ;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus_2 ;
uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize_2;
uint16_t fifoCount_1;     // count of all bytes currently in FIFO
uint16_t fifoCount_2; 
uint8_t fifoBuffer_1[64]; // FIFO storage buffer
uint8_t fifoBuffer_2[64];


// orientation/motion vars
Quaternion q_1;           // [w, x, y, z]         quaternion container
VectorInt16 aa_1;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_1;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_1;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_1;    // [x, y, z]            gravity vector
float euler_1[3];         // [psi, theta, phi]    Euler angle container
float ypr_1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Quaternion q_2;           // [w, x, y, z]         quaternion container
VectorInt16 aa_2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_2;    // [x, y, z]            gravity vector
float euler_2[3];         // [psi, theta, phi]    Euler angle container
float ypr_2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_1() {
    mpuInterrupt_1 = true;
}
volatile bool mpuInterrupt_2 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_2() {
    mpuInterrupt_2 = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    pinMode(MPU6050_1_INT, INPUT);
    pinMode(MPU6050_2_INT, INPUT);
    pinMode(Pitch_CheckPin,OUTPUT);
    pinMode(Pitch_DirPin,OUTPUT); 
    pinMode(Pitch_RapidPin,OUTPUT);

    pinMode(Roll_CheckPin,OUTPUT);
    pinMode(Roll_DirPin,OUTPUT); 
    pinMode(Roll_RapidPin,OUTPUT);

    pinMode(levelingDonePin,OUTPUT);
    
    digitalWrite(levelingDonePin,LOW);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//    Serial.println(F("Initializing I2C devices..."));
    
    mpu1.initialize();
    mpu1.setRate(9);
    delay(1);
    
    mpu2.initialize();
    mpu2.setRate(9);
//    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    devStatus_1 = mpu1.dmpInitialize();
    delay(1);
    devStatus_2 = mpu2.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(220);
    mpu1.setYGyroOffset(76);
    mpu1.setZGyroOffset(-85);
    mpu1.setZAccelOffset(1788); // 1688 factory default for my test chip

    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788);
    delay(30);
    if (devStatus_1 == 0) {
//        Serial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 7)..."));
        attachInterrupt(MPU6050_1_INT, dmpDataReady_1, RISING);
        mpuIntStatus_1 = mpu1.getIntStatus();

//        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_1 = true;

        // get expected DMP packet size for later comparison
        packetSize_1 = mpu1.dmpGetFIFOPacketSize();
        initialMilllis = millis();
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus_1);
//        Serial.println(F(")"));
    }
    if (devStatus_2 == 0) {
//        Serial.println(F("Enabling DMP..."));
        mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 7)..."));
        attachInterrupt(MPU6050_2_INT, dmpDataReady_2, RISING);
        mpuIntStatus_2 = mpu2.getIntStatus();

//        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_2 = true;

        // get expected DMP packet size for later comparison
        packetSize_2 = mpu2.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus_1);
//        Serial.println(F(")"));
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  currentMillis=millis();
  
    
    if (currentMillis - initialMilllis >= 7000) {
      millischeck=true;
      }
    else{
//      Serial.println("JUST WAIT");
      pitchTamsayi=11;
      pitchKusurat=22;
      rollTamsayi=33;
      rollKusurat=44;
      
    }

    
    while (!mpuInterrupt_1 && fifoCount_1 < packetSize_1&&!mpuInterrupt_2 && fifoCount_2 < packetSize_2) {
        if(currentMillis-previousMillis>50){
          previousMillis=currentMillis;
          Wire.beginTransmission(5);
          Wire.write(pitchTamsayi);//tamsayi
          Wire.write(pitchKusurat);//kusurat
          Wire.write(rollTamsayi);
          Wire.write(rollKusurat);
      
//      if(i2cCounter%4==0) Wire.write(pitchTamsayi);//tamsayi
//      else if(i2cCounter%4==1) Wire.write(pitchKusurat);//kusurat
//      else if(i2cCounter%4==2) Wire.write(rollTamsayi);
//      else if(i2cCounter%4==3){
//        Wire.write(rollKusurat);
//        i2cCounter=-1;
//      }
        Wire.endTransmission();
        i2cCounter++;
    }

      
        if (abs(pitch_1)>0.35){
          digitalWrite(Pitch_RapidPin,LOW);
          }
        else{
          digitalWrite(Pitch_RapidPin,HIGH);
          }
        if (abs(roll_1)>0.4){
          digitalWrite(Roll_RapidPin,LOW);
          }
        else{
          digitalWrite(Roll_RapidPin,HIGH);
          }
        if(pitch_1>0.04){
          digitalWrite(Pitch_CheckPin,HIGH);
          digitalWrite(Pitch_DirPin,HIGH);
          }
        else if(pitch_1<-0.04){
          digitalWrite(Pitch_CheckPin,HIGH);
          digitalWrite(Pitch_DirPin,LOW);
        }
        else{
          digitalWrite(Pitch_CheckPin,LOW);
          digitalWrite(Pitch_DirPin,LOW);
          }
        if(roll_1>0.04){
          digitalWrite(Roll_CheckPin,HIGH);
          digitalWrite(Roll_DirPin,HIGH);
          }
        else if(roll_1<-0.04){
          digitalWrite(Roll_CheckPin,HIGH);
          digitalWrite(Roll_DirPin,LOW);
          }
        else{
          digitalWrite(Roll_CheckPin,LOW);
          digitalWrite(Roll_DirPin,LOW);
        }
        if(abs(roll_1)<0.04&abs(pitch_1)<0.04){
          digitalWrite(levelingDonePin,HIGH);
          }
        else if(!millischeck){
          digitalWrite(levelingDonePin,HIGH);
          }
        else{
          digitalWrite(levelingDonePin,LOW);
          }
      }
    
    // if programming failed, don't try to do anything
    if (mpuInterrupt_1){ 
      // wait for MPU interrupt or extra packet(s) available
      
    // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt_1 = false;
      mpuIntStatus_1 = mpu1.getIntStatus();
      

//      mpuInterrupt_2 = false;
//      mpuIntStatus_2 = mpu2.getIntStatus();
//      // get current FIFO count
//      fifoCount_2 = mpu2.getFIFOCount();
//       // check for overflow (this should never happen unless our code is too inefficient)
//      if ((mpuIntStatus_2 & 0x10) || fifoCount_2 == 1024) {
//        // reset so we can continue cleanly
//        mpu2.resetFIFO();
////        Serial.println(F("FIFO overflow!"));
//      // otherwise, check for DMP data ready interrupt (this should happen frequently)
//      }
      // get current FIFO count
      
      fifoCount_1 = mpu1.getFIFOCount();
       // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus_1 & 0x10) || fifoCount_1 == 1024) {
        // reset so we can continue cleanly
        mpu1.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
       
      } else if (mpuIntStatus_1 & 0x02) {
        // wait for correct available data length, should be a VERY short wait
          while (fifoCount_1 < packetSize_1) fifoCount_1 = mpu1.getFIFOCount();
          // read a packet from FIFO
          mpu1.getFIFOBytes(fifoBuffer_1, packetSize_1);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount_1 -= packetSize_1;
          #ifdef OUTPUT_READABLE_YAWPITCHROLL
              // display Euler angles in degrees
              mpu1.dmpGetQuaternion(&q_1, fifoBuffer_1);
              mpu1.dmpGetGravity(&gravity_1, &q_1);
              mpu1.dmpGetYawPitchRoll(ypr_1, &q_1, &gravity_1);
              pitch_1=ypr_1[1] * 180/M_PI;
              roll_1=ypr_1[2] * 180/M_PI;
              firstNumberFirst2digits=floor(abs(pitch_1));
              firstNumberNEXT2digits=(abs(pitch_1)-firstNumberFirst2digits)*100;
              secondNumberFirst2digits=floor(abs(roll_1));
              secondNumberNEXT2digits=(abs(roll_1)-secondNumberFirst2digits)*100;
              pitchTamsayi=firstNumberFirst2digits;
              pitchKusurat=firstNumberNEXT2digits;
              rollTamsayi=secondNumberFirst2digits;
              rollKusurat=secondNumberNEXT2digits;
//              Serial.print("ypr\t");
//              Serial.print(ypr[0] * 180/M_PI);
//              Serial.print("\t");
//              Serial.print(ypr[1] * 180/M_PI);
//              Serial.print("\t");
//              Serial.println(ypr[2] * 180/M_PI);
          #endif

       }
    }
    if (mpuInterrupt_2){ 
      // wait for MPU interrupt or extra packet(s) available
      
    // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt_2 = false;
      mpuIntStatus_2 = mpu2.getIntStatus();


//      mpuInterrupt_1 = false;
//      mpuIntStatus_1 = mpu1.getIntStatus();
//      // get current FIFO count
//      fifoCount_1 = mpu1.getFIFOCount();
//       // check for overflow (this should never happen unless our code is too inefficient)
//      if ((mpuIntStatus_1 & 0x10) || fifoCount_1 == 1024) {
//        // reset so we can continue cleanly
//        mpu1.resetFIFO();
//      }

      
      // get current FIFO count
      fifoCount_2 = mpu2.getFIFOCount();
       // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus_2 & 0x10) || fifoCount_2 == 1024) {
        // reset so we can continue cleanly
        mpu2.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus_2 & 0x02) {
        // wait for correct available data length, should be a VERY short wait
          while (fifoCount_2 < packetSize_1) fifoCount_2 = mpu2.getFIFOCount();
          // read a packet from FIFO
          mpu2.getFIFOBytes(fifoBuffer_2, packetSize_2);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount_2 -= packetSize_2;
//          #ifdef OUTPUT_READABLE_YAWPITCHROLL
//              // display Euler angles in degrees
//              mpu2.dmpGetQuaternion(&q_2, fifoBuffer_2);
//              mpu2.dmpGetGravity(&gravity_2, &q_2);
//              mpu2.dmpGetYawPitchRoll(ypr_2, &q_2, &gravity_2);
//              pitch_2=ypr_2[1] * 180/M_PI;
//              roll_2=ypr_2[2] * 180/M_PI;
//              firstNumberFirst2digits=floor(abs(pitch_2));
//              firstNumberNEXT2digits=(abs(pitch_2)-firstNumberFirst2digits)*100;
//              secondNumberFirst2digits=floor(abs(roll_2));
//              secondNumberNEXT2digits=(abs(roll_2)-secondNumberFirst2digits)*100;
//              pitchTamsayi=firstNumberFirst2digits;
//              pitchKusurat=firstNumberNEXT2digits;
//              rollTamsayi=secondNumberFirst2digits;
//              rollKusurat=secondNumberNEXT2digits;
//              
//          #endif

       }
    }
    

    
}
