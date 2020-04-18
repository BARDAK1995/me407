/*
testWholeDisplay.ino
-test each segment in the display
-a simple example using Dean Reading's SevSeg library to light up all 4 digits plus the 4 decimal points on a 4 digit display
-the purpose of this example is to ensure you have the wires all hooked up right for every segment and digit, and to troubleshoot the display and wiring
 to ensure *every* segment and period lights up

By Gabriel Staples 
Website: http://www.ElectricRCAircraftGuy.com
My contact info is available by clicking the "Contact Me" tab at the top of my website.
Written: 1 Oct. 2015
Last Updated: 1 Oct. 2015
*/

/*
LICENSING:
-this *example file* only is modified from Dean Reading's original example, and is in the public domain.

Dean Reading's SevSeg library is as follows:
Copyright 2014 Dean Reading

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at 
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <Wire.h>
#include <SevSeg.h>
long c=0;
int place=0;
long number=0;
long solTamsayi=12;
int solKusurat=34;

unsigned long currentMillis = 0; 
unsigned long initialMilllis = 0;
unsigned long previousMillis=0;
bool millischeck=false;

int sagTamsayi=56;
int sagKusurat=78;

SevSeg sevseg; //Instantiate a seven segment controller object

void setup()
{
  Wire.begin(5);
  Wire.onReceive(receiveEvent);
  Serial.begin(57600);
  
  
  byte numDigits = 8;   
  byte digitPins[] = {2, 3, 4, 5,14,15,16,17}; //Digits: 1,2,3,4 <--put one resistor (ex: 220 Ohms, or 330 Ohms, etc, on each digit pin)
//  byte digitPins[] = {2, 3, 4, 5}; //Digits: 1,2,3,4 <--put one resistor (ex: 220 Ohms, or 330 Ohms, etc, on each digit pin)
  byte segmentPins[] = {6, 7, 8, 9, 10, 11, 12, 13}; //Segments: A,B,C,D,E,F,G,Period

  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins,false,false,true,false);
  sevseg.setBrightness(30); //Note: 100 brightness simply corresponds to a delay of 2000us after lighting each segment. A brightness of 0 
                            //is a delay of 1us; it doesn't really affect brightness as much as it affects update rate (frequency).
                            //Therefore, for a 4-digit 7-segment + pd, COMMON_ANODE display, the max update rate for a "brightness" of 100 is 1/(2000us*8) = 62.5Hz.
                            //I am choosing a "brightness" of 10 because it increases the max update rate to approx. 1/(200us*8) = 625Hz.
                            //This is preferable, as it decreases aliasing when recording the display with a video camera....I think.
}

void loop()
{

  currentMillis=millis();
  if (currentMillis - initialMilllis <= 3000) {
    delay(100);
    return;
  }
  //local vars
  
  c=c+1;
  place=c%3;
  
//  Serial.println(number2);
  number=(100*solTamsayi+solKusurat)*10000+100*sagTamsayi+sagKusurat;
//  Serial.println(number);
  sevseg.setNumber(number,place*4-2);
//  decPlace %= 4; //rollover back to 0 once variable gets to 4; To anyone wondering: the % is called the "modulo" operator; see here for explanation & example: https://www.arduino.cc/en/Reference/Modulo
//  delayMicroseconds(200);
//  delay();
  sevseg.refreshDisplay(); // Must run repeatedly; don't use blocking code (ex: delay()) in the loop() function or this won't work right
}


void receiveEvent(int howMany){
//  while(1<Wire.available()){{//reads the first string part, if any
//    char c=Wire.read();
//    Serial.print(c);
//    }
//  }

  solTamsayi=Wire.read();
  solKusurat=Wire.read();
  sagTamsayi=Wire.read();
  sagKusurat=Wire.read();
  }
