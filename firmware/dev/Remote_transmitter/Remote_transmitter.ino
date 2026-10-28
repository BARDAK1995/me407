// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h>

int xValue=0;
int yValue=0;

// Create Amplitude Shift Keying Object
RH_ASK rf_driver;
String msgString = "00129";//step(1,..16) speed(1..9) dir(1,2) select(1,0)
#define joyX A0
#define joyY A1
int SW_pin=2;
int turnspeed=1;
int turnstep=16;
int turndirection=0;
int delaytime=150;

void setup()
{
    // Initialize ASK Object
    pinMode(SW_pin, INPUT);
    digitalWrite(SW_pin, HIGH);
    rf_driver.init();
    Serial.begin(9600);
}
 
void loop()
{
    xValue = analogRead(joyX);
    
    if(xValue>800){
      if(turnspeed>0&&turnspeed<9){
      turnspeed++;
      delay(delaytime);

      }
      else if(turnspeed==0){
        if(turnstep==16){
          turnspeed=1;
          }
        else{
          turnstep=turnstep*2;
          delay(delaytime);

          }
        }
    }
    else if(xValue<200){
      if(turnspeed>0){
        turnspeed--;
        delay(delaytime);

        }
      else if(turnspeed==0&turnstep>1){
        turnstep=turnstep/2;
        delay(delaytime);

        }
    }
    yValue = analogRead(joyY);
    
    if(yValue>950){
      turndirection=1;
      delay(delaytime);

    }
    else if(yValue<150){
      turndirection=0;
      delay(delaytime);
 
    }
    else{
      turndirection=9;
      }
    int isSelect=digitalRead(SW_pin);
    
     
//    const char *msg = "Hello World";
//    Serial.print(xValue);
//    Serial.print("\t");
//    Serial.pintln(yValue);
    if(turnstep<10){
      msgString="0"+String(turnstep)+String(turnspeed)+String(turndirection)+String(isSelect);
      }
    else{
      msgString=String(turnstep)+String(turnspeed)+String(turndirection)+String(isSelect);
    }
//    Serial.print(turnstep);
//    Serial.print(turnspeed);
//    Serial.print(turndirection);
//    Serial.println(isSelect);
    char tmp[6];
    msgString.toCharArray(tmp, sizeof(tmp));
//    send(tmp);

     Serial.println(tmp);
    rf_driver.send((uint8_t *)tmp, strlen(tmp));
    rf_driver.waitPacketSent();
    delay(50);
}
