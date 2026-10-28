// defines pins numbers
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

int timeBetweenSteps=6000;


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
void loop() {
  
  readButtons();
  
  
  digitalWrite(dirPin1,directionBoth); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,directionBoth);

  
  // Makes 200 pulses for making one full cycle rotation
  
  for(int x = 0; x < toggleState1; x++) {
    digitalWrite(stepPin1,HIGH); 
    delayMicroseconds(timeBetweenSteps); 
    digitalWrite(stepPin1,LOW); 
    delayMicroseconds(timeBetweenSteps);
    readButtons();
  }
  
  //delay(1000); // One second delay
  
  for(int x = 0; x < toggleState2; x++) {
    digitalWrite(stepPin2,HIGH);
    delayMicroseconds(timeBetweenSteps);
    digitalWrite(stepPin2,LOW);
    delayMicroseconds(timeBetweenSteps);
    readButtons();
  }
  //delay(10);
}
