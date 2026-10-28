/*
  nRF24L01+ Joystick Transmitter
  nrf24l01-joy-xmit-demo.ino
  nRF24L01+ Transmitter with Joystick
  Use with Joystick Receiver Demo
  DroneBot Workshop 2018
  https://dronebotworkshop.com
*/
 #include <TM1637Display.h>
const uint8_t OFF[] = {0, 0, 0, 0};
// In this library, the byte order is .GFEDCBA
const uint8_t PLAY[] = {B01110011, B00111000, B01011111, B01101110};
const uint8_t DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};
TM1637Display display(4, 5);





 
// Include RadioHead ReliableDatagram & NRF24 Libraries
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
 
// Include dependant SPI Library 
#include <SPI.h>

String msgString = "00191";//step(1,..16) speed(1..9) dir(1,2) select(1,0)goneezxx

int SW_pin=17;
int turnspeed=1;
int turnstep=16;
int turndirection=0;
int delaytime=30;
int isSelect=0;

int xValue=0;
int yValue=0;

float angleData=0;
int decimalPlace=1;
int angledata2Display=0;
// Define Joystick Connections
#define joyX A1
#define joyY A2

 
// Define addresses for radio channels
#define CLIENT_ADDRESS 1   
#define SERVER_ADDRESS 2
 
// Create an instance of the radio driver
//RH_NRF24 RadioDriver;
RH_NRF24 RadioDriver(7,6);

// Sets the radio driver to NRF24 and the client address to 1
RHReliableDatagram RadioManager(RadioDriver, CLIENT_ADDRESS);
 
// Declare unsigned 8-bit joystick array
uint8_t joystick[3]; 
 
// Define the Message Buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

void countdown() {
  for(int i=1500; i>0; i--) {
    // showNumberDec is a function for displaying numeric values,
    display.showNumberDecEx(1234,0b01000000, true, 4, 0);

    delay(80);
  }
}

void displayDone() {
  display.setSegments(DONE);
  delay(20);
}
void displayPlay() {
  if(decimalPlace==1){
  display.showNumberDecEx(angledata2Display,0b10000000, true, 4, 0);
  delay(20);
  }
  if(decimalPlace==2){
  display.showNumberDecEx(angledata2Display,0b01000000, true, 4, 0);
  delay(20);
  }
  if(decimalPlace==3){
  display.showNumberDecEx(angledata2Display,0b00100000, true, 4, 0);
  delay(20);
  }
  
}


void setup()
{
  display.setBrightness(2);
  // Clear the display
  display.setSegments(OFF);
  // Setup Serial Monitor
  Serial.begin(115200);
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!RadioManager.init())
    Serial.println("init failed");
  if (!RadioManager.init())
    Serial.println("init failed");
}
 
void loop()
{
  readJoyStick();
//  displayPlay();
//  displayDone();
  
  generateMessage();
  if(turndirection!=9){
    sendMessage();
    delay(200);
    }
  if(isSelect==0){
    sendMessage();
    delay(200);
    }
  
    
  delay(5);  // Wait a bit before next transmission
}

void sendMessage(){
  
  char tmp[6];
    msgString.toCharArray(tmp, sizeof(tmp));
  Serial.println(msgString);
//  Serial.println("Sending Joystick data to nrf24_reliable_datagram_server");
//  Serial.println(tmp);
  //Send a message containing Joystick data to manager_server
  if (RadioManager.sendtoWait(tmp, sizeof(tmp), SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (RadioManager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
//      Serial.print("got reply from : 0x");
//      Serial.print(from, HEX);
//      Serial.print(": ");
      Serial.println((char*)buf);
      angleData=(float)atof(buf);
      if(angleData<0){
        angleData=angleData*-1;
        }
      
      if(angleData>360){
        angleData=angleData-360;
        }
      if(angleData<10){
        decimalPlace=1;
        }
      else if(angleData<100){
        decimalPlace=2;
        }
      else if(angleData<360){
        decimalPlace=3;
        }
      Serial.println(angleData);
      angledata2Display=angleData*pow(10, (4-decimalPlace));
      Serial.println(angledata2Display);
      displayPlay();
    }
    else
    {
      Serial.println("No reply, is nrf24_reliable_datagram_server running?");
    }
  }
  else
    Serial.println("sendtoWait failed");
 
  
  
  }
void generateMessage(){
  if(turnstep<10){
      msgString="0"+String(turnstep)+String(turnspeed)+String(turndirection)+String(isSelect);
      }
    else{
      msgString=String(turnstep)+String(turnspeed)+String(turndirection)+String(isSelect);
    }

//   if(turnstep<10){
//      msgString="0"+String(turnstep)+String(turnspeed)+String(turndirection);
//      }
//    else{
//      msgString=String(turnstep)+String(turnspeed)+String(turndirection);
//    }
  
  
  }
void readJoyStick(){
  xValue = analogRead(joyX);
    if(xValue>950){
      if(turnspeed>0&&turnspeed<9){
      turnspeed++;
      delay(delaytime*2);
      }
      else if(turnspeed==0){
        if(turnstep==16){
          turnspeed=1;
          }
        else{
          turnstep=turnstep*2;
          delay(delaytime*2);
          }
        }
    }
    else if(xValue<110){
      if(turnspeed>0){
        turnspeed--;
        delay(delaytime*3);
        }
      else if(turnspeed==0&turnstep>1){
        turnstep=turnstep/2;
        delay(delaytime*3);
        }
    }
    yValue = analogRead(joyY);
    if(yValue>950){
      turndirection=1;
      delay(delaytime);
    }
    else if(yValue<110){
      turndirection=0;
      delay(delaytime);
    }
    else{
      turndirection=9;
      }
    isSelect=digitalRead(SW_pin);
  }
