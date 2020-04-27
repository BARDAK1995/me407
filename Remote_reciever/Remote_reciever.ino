// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
#include <AccelStepper.h>
AccelStepper stepper(1,6,7);

int turnstep=16;
int turnspeed=1;
int turndirection=1;
int turnAmount=0;
int turndir=0;
long targetposition=0;
// Create Amplitude Shift Keying Object
RH_ASK rf_driver;
String x="12345";
void setup()
{
    // Initialize ASK Object
    rf_driver.init();
    // Setup Serial Monitor
    Serial.begin(9600);
}
 
void loop()
{
    // Set buffer to size of expected message
    uint8_t buf[5];
    uint8_t buflen = sizeof(buf);
    // Check if received packet is correct size
    if (rf_driver.recv(buf, &buflen))
    {
      
      // Message received with valid checksum
//      Serial.print("Message Received: ");
//    Serial.println((char*)buf);
//        x=(char*)buf;
//        Serial.println(x);
    if(buf[0]=='0'){
      turnstep= buf[1] - '0';
    }
    else{
      turnstep=10*(buf[0] - '0')+buf[1] - '0';
      }
//      Serial.println(turnstep);
    }

    if(buf[2]-'0'==9){
      turnspeed=4000;
      turnAmount=4000*2;//180deg    
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
      turnAmount=80*1;//1.8deg 
    }
    else if(buf[2]- '0'==5){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==4){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==3){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==2){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==1){
      turnspeed=2000;
      turnAmount=20*1;//0.45deg 
    }
    else if(buf[2]- '0'==0){
      turnspeed=360;
      turnAmount=turnstep;
    }
    turndir=buf[3]- '0';
//    Serial.println(turndir);
    if(turndir!=9){
      if(turndir==1){
        targetposition=targetposition+turnAmount;
        runmotor();
        delay(200);
        }
      else if(turndir==0){
        targetposition=targetposition-turnAmount;
        runmotor();
        delay(200);
        }
    
      
      
    }
}
void runmotor(){
    stepper.setMaxSpeed(turnspeed);
    stepper.setAcceleration(750);
    stepper.runToNewPosition(targetposition);
    Serial.println(targetposition);
    Serial.println(turnspeed);
    
  
  }
