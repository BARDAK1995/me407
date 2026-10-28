// defines pins numbers
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

const int stepPin1 = 8; 
const int dirPin1 = 9; 
const int stepPin2 = 10; 
const int dirPin2 = 11;

int buttonPin1=2;
int buttonPin2=3;
int buttonPin3=4;

int toggleState1=0;
int toggleState2=0;
int lastButtonState=1;
long unsigned int lastPress;
unsigned long button_time = 0;  
unsigned long last_button_time = 0; 
volatile int buttonFlag1;
volatile int buttonFlag2;
int debounceTime=50;

int directionBoth=0;
int directionPITCH=0;
int directionROLL=0;
int timeBetweenSteps=2600;

int pitchError=0;
int rollError=0;


void readButtons(){
    if((millis()-lastPress)>debounceTime && buttonFlag1){
        lastPress=millis();
        toggleState1=digitalRead(buttonPin1);
        toggleState1=toggleState1*1;   
        buttonFlag1=0;
        //sendSerial();  
    }
    if((millis()-lastPress)>debounceTime && buttonFlag2){
      lastPress=millis();
      toggleState2=digitalRead(buttonPin2);
      toggleState2=toggleState2*1;
      buttonFlag2=0;
      //sendSerial();
    }
    directionBoth = digitalRead(buttonPin3);   
}
void sendSerial(){
  
    Serial.println();
    Serial.print(toggleState1);
    Serial.print(":");
    Serial.print(toggleState2);
    Serial.print(":"); 
    Serial.print(directionBoth);
}

void ISR_Button1(){
  buttonFlag1=1;
}

void ISR_Button2(){
  buttonFlag2=1;  
}


void setup() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  

  
  // Sets the two pins as Outputs
  pinMode(stepPin1,OUTPUT); 
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);

  pinMode(buttonPin1,INPUT);
  pinMode(buttonPin2,INPUT);
  pinMode(buttonPin3,INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin1),ISR_Button1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin2),ISR_Button2,CHANGE);
  Serial.begin(9600);
}

void getXYZorientation() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.5; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) +1.6; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 2.52; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 1.64; // GyroErrorY ~(2)
  GyroZ = GyroZ + 1.34; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
//  roll = 0.34 * gyroAngleX + 0.66 * accAngleX;
//  pitch = 0.34 * gyroAngleY + 0.66 * accAngleY;

//  roll = gyroAngleX ;
//  pitch =gyroAngleY;

  roll = accAngleX;
  pitch = accAngleY;

  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

 
}

void actuateMotors(){
  
  digitalWrite(dirPin1,directionPITCH); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,directionROLL);
  // Makes 200 pulses for making one full cycle rotation
  
  for(int x = 0; x < toggleState1; x++) {
    digitalWrite(stepPin1,HIGH); 
    delayMicroseconds(timeBetweenSteps); 
    digitalWrite(stepPin1,LOW); 
    delayMicroseconds(timeBetweenSteps);
    readButtons();
  }
  
  for(int x = 0; x < toggleState2; x++) {
    digitalWrite(stepPin2,HIGH);
    delayMicroseconds(timeBetweenSteps);
    digitalWrite(stepPin2,LOW);
    delayMicroseconds(timeBetweenSteps);
    readButtons();
  
  }
}
void loop() {
  
  readButtons();
  
  getXYZorientation();
  readButtons();
  
  if(abs(pitch)>1.8|abs(roll)>1.8){
    Wire.endTransmission(true);
    if(abs(pitch)>2.4){
        toggleState1=abs(pitch)*20;  
      }
    else {
      toggleState1=abs(pitch)*2;
      }

    if(abs(roll)>2.8){
        toggleState2=abs(roll)*20;  
      }
    else {
      toggleState2=abs(roll)*2;
      }
    
    if(pitch>1){ 
      directionPITCH=1;
      }
    else{
      directionPITCH=0;
      }
    if(roll>1){
      directionROLL=1;
      }
    else{
      directionROLL=0;
      }
    actuateMotors();
    delay(750);
    }
  
  
  delay(50);
}
