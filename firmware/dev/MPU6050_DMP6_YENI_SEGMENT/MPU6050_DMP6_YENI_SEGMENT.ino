// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MAX7219_DIN           12
#define MAX7219_CS            10
#define MAX7219_CLK           11
enum {  MAX7219_REG_DECODE    = 0x09,  
        MAX7219_REG_INTENSITY = 0x0A,
        MAX7219_REG_SCANLIMIT = 0x0B,
        MAX7219_REG_SHUTDOWN  = 0x0C,
        MAX7219_REG_DISPTEST  = 0x0F };
enum  { OFF = 0,  
        ON  = 1 };
const byte DP = 0b10000000;  
const byte C  = 0b01001110;  
const byte F  = 0b01000111;

String str="11111111"; 

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */




// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


#define MPU6050_INT_PIN 3
#define MPU6050_INT digitalPinToInterrupt(MPU6050_INT_PIN)

bool blinkState = false;
//int Pitch_RapidPin=11;
//int Roll_RapidPin=12;
//int Pitch_CheckPin=10;
//int Roll_CheckPin=13;
//int Pitch_DirPin=5;
//int Roll_DirPin=6;
int levelingDonePin=4;
float pitch=0;
float roll=0;

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
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    pinMode(MAX7219_DIN, OUTPUT);   // serial data-in
    pinMode(MAX7219_CS, OUTPUT);    // chip-select, active low    
    pinMode(MAX7219_CLK, OUTPUT);   // serial clock
    digitalWrite(MAX7219_CS, HIGH);
    resetDisplay(); 
    
    
    pinMode(MPU6050_INT, INPUT);
    
//    pinMode(Pitch_CheckPin,OUTPUT);
//    pinMode(Pitch_DirPin,OUTPUT); 
//    pinMode(Pitch_RapidPin,OUTPUT);
//
//    pinMode(Roll_CheckPin,OUTPUT);
//    pinMode(Roll_DirPin,OUTPUT); 
//    pinMode(Roll_RapidPin,OUTPUT);

    pinMode(levelingDonePin,OUTPUT);
    
    digitalWrite(levelingDonePin,LOW);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    delay(100);
//    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    
    mpu.initialize();
    mpu.setDLPFMode(6); 
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(58);
    mpu.setYGyroOffset(-13);
    mpu.setZGyroOffset(-9);
    mpu.setZAccelOffset(1267); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 7)..."));
        attachInterrupt(MPU6050_INT, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        initialMilllis = millis();
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     
  
  currentMillis=millis();
  
    
    if (currentMillis - initialMilllis >= 20000) {
      millischeck=true;
      }
    else{
      Serial.println("JUST WAIT");
      pitchTamsayi=11;
      pitchKusurat=22;
      rollTamsayi=33;
      rollKusurat=44;
      
    }

    if(currentMillis-previousMillis>150){
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
    
    // if programming failed, don't try to do anything
    if (!dmpReady){
      return;
      } 

    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
//      if (abs(pitch)>0.35){
//        digitalWrite(Pitch_RapidPin,LOW);
//        }
//      else{
//        digitalWrite(Pitch_RapidPin,HIGH);
//        }
//      if (abs(roll)>0.4){
//        digitalWrite(Roll_RapidPin,LOW);
//        }
//      else{
//        digitalWrite(Roll_RapidPin,HIGH);
//        }
//      if(pitch>0.04){
//        digitalWrite(Pitch_CheckPin,HIGH);
//        digitalWrite(Pitch_DirPin,HIGH);
//        }
//       else if(pitch<-0.04){
//        digitalWrite(Pitch_CheckPin,HIGH);
//        digitalWrite(Pitch_DirPin,LOW);
//        }
//       else{
//        digitalWrite(Pitch_CheckPin,LOW);
//        digitalWrite(Pitch_DirPin,LOW);
//        }
//       if(roll>0.04){
//        digitalWrite(Roll_CheckPin,HIGH);
//        digitalWrite(Roll_DirPin,HIGH);
//        }
//       else if(roll<-0.04){
//        digitalWrite(Roll_CheckPin,HIGH);
//        digitalWrite(Roll_DirPin,LOW);
//        }
//       else{
//        digitalWrite(Roll_CheckPin,LOW);
//        digitalWrite(Roll_DirPin,LOW);
//      }
//
//      if(abs(roll)<0.04&abs(pitch)<0.04){
//        digitalWrite(levelingDonePin,HIGH);
//        }
//      else if(!millischeck){
//        digitalWrite(levelingDonePin,HIGH);
//        }
//      else{
//        digitalWrite(levelingDonePin,LOW);
//        }
//    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

       

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch=ypr[1] * 180/M_PI;
            roll=ypr[2] * 180/M_PI;
          
            firstNumberFirst2digits=floor(abs(pitch));
            firstNumberNEXT2digits=(abs(pitch)-firstNumberFirst2digits)*100;
            
            secondNumberFirst2digits=floor(abs(roll));
            secondNumberNEXT2digits=(abs(roll)-secondNumberFirst2digits)*100;

//            Serial.print(firstNumberFirst2digits);
//            Serial.println(firstNumberNEXT2digits);
//            Serial.println("done");
            pitchTamsayi=firstNumberFirst2digits;
            pitchKusurat=firstNumberNEXT2digits;
            rollTamsayi=secondNumberFirst2digits;
            rollKusurat=secondNumberNEXT2digits;
            
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif



        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            pitch=ypr[1] * 180/M_PI;
            roll=ypr[2] * 180/M_PI;
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

    }
}

void displayTime(String timeString)  
{
    set_register(MAX7219_REG_SHUTDOWN, OFF);      // turn off display
    set_register(MAX7219_REG_SCANLIMIT, 7);       // limit to 8 digits
    set_register(MAX7219_REG_DECODE, 0b11111111); // decode all digits

    set_register(1, timeString.charAt(7));
    set_register(2, timeString.charAt(6));
    set_register(3, timeString.charAt(5));
    set_register(4, timeString.charAt(4));
    set_register(5, timeString.charAt(3));
    set_register(6, timeString.charAt(2));
    set_register(7, timeString.charAt(1));
    set_register(8, timeString.charAt(0));

    set_register(MAX7219_REG_SHUTDOWN, ON);       // Turn on display
}
void set_register(byte reg, byte value)  
{
    digitalWrite(MAX7219_CS, LOW);
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, reg);
    shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, value);
    digitalWrite(MAX7219_CS, HIGH);
}
