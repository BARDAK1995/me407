// defines pins numbers

#include <AccelStepper.h>



int inputpin_Pitch_Dir=3;
int inputpin_Pitch_Step=2;
int Output_Pitch_Step=19;
int Output_Pitch_Dir=18;

int inputpin_Roll_Step=6;
int inputpin_Roll_Dir=7;

int Output_Roll_Step=12;
int Output_Roll_Dir=11;


AccelStepper stepperPitch(1, Output_Pitch_Step,Output_Pitch_Dir);
AccelStepper stepperRoll(1, Output_Roll_Step,Output_Roll_Dir);

bool pitch_dir=false;
bool roll_dir=false;

bool pitch_Step=false;
bool roll_Step=false;

bool blinkState = false;



int toggleState1=0;
int toggleState2=0;

//int lastButtonState=1;
//long unsigned int lastPress;
//unsigned long button_time = 0;  
//unsigned long last_button_time = 0; 
//volatile int buttonFlag1;
//volatile int buttonFlag2;
//int debounceTime=50;

int directionBoth=0;
int directionPITCH=0;
int directionROLL=0;



void setup() {
  Serial.begin(9600);
  stepperPitch.setMaxSpeed(1000.0);
  stepperPitch.setSpeed(800);
//  stepperPitch.setAcceleration(500.0);
  stepperPitch.setMinPulseWidth(20); 
  stepperPitch.enableOutputs();

  stepperRoll.setMaxSpeed(1000.0);
  stepperRoll.setSpeed(820);
//  stepperRoll.setAcceleration(500.0);
  stepperRoll.setMinPulseWidth(20); 
  stepperRoll.enableOutputs();


  pinMode(inputpin_Pitch_Dir, INPUT);
  pinMode(inputpin_Pitch_Step, INPUT);
  pinMode(inputpin_Roll_Dir, INPUT);
  pinMode(inputpin_Roll_Step, INPUT);


//  attachInterrupt(digitalPinToInterrupt(buttonPin1),ISR_Button1,CHANGE);
//  attachInterrupt(digitalPinToInterrupt(buttonPin2),ISR_Button2,CHANGE);
}

void getGyroInputs(){
  pitch_dir = digitalRead(inputpin_Pitch_Dir);
  pitch_Step=digitalRead(inputpin_Pitch_Step);
  roll_dir = digitalRead(inputpin_Roll_Dir);
  roll_Step=digitalRead(inputpin_Roll_Step);
  
}

void actuateMotors(){
  
  stepperPitch.setPinsInverted(pitch_dir,false,false);
  stepperRoll.setPinsInverted(roll_dir,false,false);
  
  if(pitch_Step){

    stepperPitch.runSpeed();
    } 
  if(roll_Step){

    stepperRoll.runSpeed();
    } 
  }
void loop() {
  
  getGyroInputs();
  actuateMotors();

}
