#define STEP_PIN 8

unsigned long t = 0;
int x=500;

int dir=0;
int randomDelay=120;
void setup() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);
    Serial.begin(9600); 
}

void step(long stepDelay) {
    digitalWrite(STEP_PIN, HIGH);
    
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);

    unsigned long dt = micros() - t;
    if (dt < stepDelay*2) {
      delayMicroseconds(stepDelay*2 - dt);
    }

    t = micros();
}

void loop() {
    DoOtherStuff();
    
    if (x>0) {
        x=x-1;
        Serial.println(x);
        step(10000);
    }
    else{
      x=400;
      randomDelay=randomDelay*5;
      digitalWrite(9, LOW);
      }
}

void DoOtherStuff(){
  delayMicroseconds(randomDelay);
  
   
  
  }
