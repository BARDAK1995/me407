
#include <Wire.h>




void setup() {
  // put your setup code here, to run once:
  Wire.begin();
}

byte firstNumberFirst2digits=0;
byte firstNumberSECOND2digits=0;
byte secondNumberFirst2digits=5;
byte secondNumberSECOND2digits=1;

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(5);
//  Wire.write("x isasdad   ");
  Wire.write(firstNumberFirst2digits);
  firstNumberFirst2digits++;
  if(firstNumberFirst2digits==100){
    firstNumberSECOND2digits++;
    firstNumberFirst2digits=0;
    }
  if(firstNumberSECOND2digits==100){
    firstNumberSECOND2digits=0;
    }
    
  Wire.write(firstNumberSECOND2digits);

  Wire.write(secondNumberFirst2digits);
  secondNumberFirst2digits++;
  if(secondNumberFirst2digits==100){
    secondNumberSECOND2digits++;
    secondNumberFirst2digits=0;
    }
  if(secondNumberSECOND2digits==100){
    secondNumberSECOND2digits=0;
    }
  Wire.write(secondNumberSECOND2digits);
  Wire.endTransmission();

  delay(200);  

}
