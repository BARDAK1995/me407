
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#include <AccelStepper.h>
AccelStepper stepper(1,2,7);

// Define addresses for radio channels
#define CLIENT_ADDRESS 1   
#define SERVER_ADDRESS 2
RH_NRF24 RadioDriver(9,10);
 
RHReliableDatagram RadioManager(RadioDriver, SERVER_ADDRESS);

uint8_t ReturnMessage[] = "DONE"; 
uint8_t ReturnMessage2[] = "11.11";
// Define the Message Buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
int counter=0;
int turnstep=16;
int turnspeed=1;
int turndirection=1;
int turnAmount=0;
int turndir=0;
float targetposition=0;
float targetAngle=0;
int firstDigit=0;
int secondDigit=0;
int thirdDigit=0;
int fourthDigit=0;
int fifthDigit=0;
int STEP_ENABLE_PIN=8;

void setup()
{
  // Setup Serial Monitor
//  Serial.begin(115200);
  pinMode(STEP_ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_ENABLE_PIN, HIGH);
  // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
//  if (!RadioManager.init())
//    Serial.println("init failed");
  RadioManager.init();
    
} 
 
void loop()
{

  if(counter==0){
    digitalWrite(STEP_ENABLE_PIN, HIGH);
//    Serial.println("xxx");
    }
  else{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    }
    
  if (RadioManager.available())
  {
 // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (RadioManager.recvfromAck(buf, &len, &from))
    {
//      Serial.println(buf[0]-'0');
//      Serial.println(buf[1]-'0');
//      Serial.println(buf[2]-'0');
//      Serial.println(buf[3]-'0');
//      Serial.println(buf[4]-'0');
      processInputs();
      
      // Send a reply back to the originator client, check for error
      targetAngle=360*targetposition/16000;
      dtostrf(targetAngle, 0, 5, ReturnMessage2);
//      Serial.println("targetAngle");
      if (!RadioManager.sendtoWait(ReturnMessage2, sizeof(ReturnMessage2), from)){
//        Serial.println("sendtoWait failed");
        }
//        
        
      actuate();
    
    if(counter==0){
      targetposition=0;

      }
   
    }

  }              
}
void processInputs(){
    
    if(buf[0]=='0'){
      turnstep= buf[1] - '0';
    }
    else{
      turnstep=10*(buf[0] - '0')+buf[1] - '0';
      }
    
      if((buf[4]-'0')==0){
//        Serial.println(counter);
        counter=counter+1;
//        Serial.print("SELECTED MODE ________");
//        Serial.println(counter);
        if(counter==1){
          stepper.setCurrentPosition(0);
          targetposition=0;
        }
        if(counter==2){
          stepper.setCurrentPosition(0);
          targetposition=0;
        }
        else if(counter==3){
          counter=0;
          digitalWrite(STEP_ENABLE_PIN, HIGH);
         
        }
//          Serial.println("RESETED");
          
      }
//      Serial.println(isSelectedMode);
      

    else if(buf[2]-'0'==9){
      turnspeed=4000;
      turnAmount=4000;//90deg    
    }
    else if(buf[2]- '0'==8){
      turnspeed=3000;
      turnAmount=1600*1;//36deg 
    }
    else if(buf[2]- '0'==7){
      turnspeed=2500;
      turnAmount=400*1;//9deg 
    }
    else if(buf[2]- '0'==6){
      turnspeed=2000;
      turnAmount=80*2;//3.8deg 
    }
    else if(buf[2]- '0'==5){
      turnspeed=2000;
      turnAmount=20*2;//0.9deg 
    }
    else if(buf[2]- '0'==4){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==3){
      turnspeed=2000;
      turnAmount=10*1;//0.225deg 
    }
    else if(buf[2]- '0'==2){
      turnspeed=2000;
      turnAmount=5*1;//0.1125deg 
    }
    else if(buf[2]- '0'==1){
      turnspeed=360;
      turnAmount=turnstep*2;//2stepp
    }
    else if(buf[2]- '0'==0){
      turnspeed=360;
      turnAmount=turnstep;//SingleStep
    }
    turndir=buf[3]- '0';
//    Serial.println(turndir);


    if(turndir!=9){
      if(turndir==1){
        targetposition=targetposition+turnAmount;
//        runmotor();
        delay(1);
        }
      else if(turndir==0){
        targetposition=targetposition-turnAmount;
//        runmotor();
        delay(1);
        }
  }

    
    
    
  
  }
void actuate(){
  if(turndir!=9){
      if(turndir==1){
//        targetposition=targetposition+turnAmount;
        runmotor();
        delay(1);
        }
      else if(turndir==0){
//        targetposition=targetposition-turnAmount;
        runmotor();
        delay(1);
        }
  }
}
void runmotor(){
    stepper.setMaxSpeed(turnspeed);
    stepper.setAcceleration(3000);
    stepper.runToNewPosition(targetposition);
//    Serial.println(targetposition);
//    Serial.println(turnspeed);
  }
