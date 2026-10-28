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

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL


//#define OUTPUT_TEAPOT
#define MAX7219_DIN           11
#define MAX7219_CS            10
#define MAX7219_CLK           13

// enumerate the MAX7219 registers
// MAX7219 Datasheet, Table 2, page 7
enum {  MAX7219_REG_DECODE    = 0x09,
        MAX7219_REG_INTENSITY = 0x0A,
        MAX7219_REG_SCANLIMIT = 0x0B,
        MAX7219_REG_SHUTDOWN  = 0x0C,
        MAX7219_REG_DISPTEST  = 0x0F
     };

// enumerate the SHUTDOWN modes
// MAX7219 Datasheet, Table 3, page 7
enum  { OFF = 0,
        ON  = 1
      };

const byte DP = 0b10000000;
const byte C  = 0b01001110;
const byte F  = 0b01000111;

#define MPU6050_INT_PIN 3
#define MPU6050_INT digitalPinToInterrupt(MPU6050_INT_PIN)

bool blinkState = false;

int firstNumberFirst2digits = 0;
int firstNumberNEXT2digits = 0;
int secondNumberFirst2digits = 5;
int secondNumberNEXT2digits = 1;

String firstNumberFirst2digitsSTR = "12";
String firstNumberNEXT2digitsSTR = "34";
String secondNumberFirst2digitsSTR = "56";
String secondNumberNEXT2digitsSTR = "78";


float pitch = 0;
float roll = 0;
byte pitchTamsayi = 0;
byte pitchKusurat = 0;
byte rollTamsayi = 0;
byte rollKusurat = 0;

String str = "12345678";

int i2cCounter = 0;

unsigned long currentMillis = 0;
unsigned long initialMilllis = 0;
unsigned long previousMillis = 0;
bool millischeck = false;

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
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



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
  pinMode(MPU6050_INT, INPUT);
  pinMode(MAX7219_DIN, OUTPUT);   // serial data-in
  pinMode(MAX7219_CS, OUTPUT);    // chip-select, active low
  pinMode(MAX7219_CLK, OUTPUT);   // serial clock
  digitalWrite(MAX7219_CS, HIGH);

  resetDisplay();                 // reset the MAX2719 display


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
  //
  mpu.initialize();
  mpu.setDLPFMode(6);
  //    // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  //    // load and configure the DMP
  //    Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  //
  //    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-110);
  mpu.setZGyroOffset(33);
  mpu.setXAccelOffset(1089); 
  mpu.setYAccelOffset(-1775); 
  mpu.setZAccelOffset(1561); // 1688 factory default for my test chip
  //
  //    // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //        Serial.println(F("Enabling DMP..."));
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

  }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  currentMillis = millis();

  // if programming failed, don't try to do anything
  if (!dmpReady) {
    return;
  }
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (currentMillis - previousMillis > 150) {
      previousMillis = currentMillis;
      displayNumbers(str);
      Serial.println(str);
    }
  }
  // wait for MPU interrupt or extra packet(s) available
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  //    // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  //
  //    // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //        // reset so we can continue cleanly
    mpu.resetFIFO();
    //        Serial.println(F("FIFO overflow!"));
    //
    //    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    //        // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    //
    //        // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //
    //        // track FIFO count here in case there is > 1 packet available
    //        // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitch=ypr[1] * 180/M_PI;
    roll=ypr[2] * 180/M_PI;

    //
    firstNumberFirst2digits = floor(abs(pitch));
    firstNumberNEXT2digits = (abs(pitch) - firstNumberFirst2digits) * 100;

    if (firstNumberFirst2digits < 10) {
      firstNumberFirst2digitsSTR = "0" + String(firstNumberFirst2digits);
    }
    else {
      firstNumberFirst2digitsSTR = String(firstNumberFirst2digits);
    }

    if (firstNumberNEXT2digits < 10) {
      firstNumberNEXT2digitsSTR = "0" + String(firstNumberNEXT2digits);
    }
    else {
      firstNumberNEXT2digitsSTR = String(firstNumberNEXT2digits);
    }

    secondNumberFirst2digits = floor(abs(roll));
    secondNumberNEXT2digits = (abs(roll) - secondNumberFirst2digits) * 100;

    if (secondNumberFirst2digits < 10) {
      secondNumberFirst2digitsSTR = "0" + String(secondNumberFirst2digits);
    }
    else {
      secondNumberFirst2digitsSTR = String(secondNumberFirst2digits);
    }

    if (secondNumberNEXT2digits < 10) {
      secondNumberNEXT2digitsSTR = "0" + String(secondNumberNEXT2digits);
    }
    else {
      secondNumberNEXT2digitsSTR = String(secondNumberNEXT2digits);
    }





    //            Serial.println(firstNumberFirst2digits);
    //            Serial.println(firstNumberNEXT2digits);
    //            Serial.println("done");


    //            pitchTamsayi=firstNumberFirst2digits;
    //            pitchKusurat=firstNumberNEXT2digits;
    //            rollTamsayi=secondNumberFirst2digits;
    //            rollKusurat=secondNumberNEXT2digits;

    str = firstNumberFirst2digitsSTR + firstNumberNEXT2digitsSTR + secondNumberFirst2digitsSTR + secondNumberNEXT2digitsSTR;

    //            Serial.print("ypr\t");
    //            Serial.print(ypr[0] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.print(ypr[1] * 180/M_PI);
    //            Serial.print("\t");
    //            Serial.println(ypr[2] * 180/M_PI);
    //        #endif
  }
}



// ... write a value into a max7219 register
//    MAX7219 Datasheet, Table 1, page 6
void set_register(byte reg, byte value)
{
  digitalWrite(MAX7219_CS, LOW);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, reg);
  shiftOut(MAX7219_DIN, MAX7219_CLK, MSBFIRST, value);
  digitalWrite(MAX7219_CS, HIGH);
}


// ... reset the max7219 chip
void resetDisplay()
{
  set_register(MAX7219_REG_SHUTDOWN, OFF);   // turn off display
  set_register(MAX7219_REG_DISPTEST, OFF);   // turn off test mode
  set_register(MAX7219_REG_INTENSITY, 0x0D); // display intensity
}
void displayNumbers(String numberString)
{
  set_register(MAX7219_REG_SHUTDOWN, OFF);  // turn off display
  set_register(MAX7219_REG_SCANLIMIT, 7);   // scan limit 8 digits
  set_register(MAX7219_REG_DECODE, 0b11111111); // decode all digits

  set_register(1, numberString.charAt(7));
  set_register(2, numberString.charAt(6));
  set_register(3, numberString.charAt(5) | DP);
  set_register(4, numberString.charAt(4));
  set_register(5, numberString.charAt(3));
  set_register(6, numberString.charAt(2));
  set_register(7, numberString.charAt(1) | DP);
  set_register(8, numberString.charAt(0));

  set_register(MAX7219_REG_SHUTDOWN, ON);   // Turn on display
}
