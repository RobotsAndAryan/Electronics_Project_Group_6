#include <Servo.h> 

// variables for the servos instead of defines
int servo1_locked = 0;    
int servo1_open = 70;    
int servo2_locked = 70;  
int servo2_open = 0;     

int s1Pin = 7;        
int s2Pin = 8;        

// buttons wired to GND
int buttonHatch = A1;     
int buttonLaunch = A2;    

Servo myHatch1; 
Servo myHatch2; 

int currentMasterState = 0;
bool isHatchOpen = false;
unsigned long hatchTimer = 0;
unsigned long launchTimer = 0;
bool hasDeployed = false;

void setup() {
  Serial.begin(115200);   
  Serial1.begin(115200);  // this connects to the other arduino
  
  pinMode(buttonHatch, INPUT_PULLUP);
  pinMode(buttonLaunch, INPUT_PULLUP);
  
  myHatch1.attach(s1Pin);
  myHatch2.attach(s2Pin);
  
  myHatch1.write(servo1_locked);
  myHatch2.write(servo2_locked);

  Serial.println("Motor Controller is ON");
}

void loop() {
  // read what the main brain is doing
  if (Serial1.available() > 0) {
    int newState = Serial1.read();
    
    // if it restarts, reset our variable too
    if (newState == 0 && currentMasterState != 0) {
      hasDeployed = false; 
    }
    currentMasterState = newState;
  }

  // State 0 is when we are just sitting on the ground
  if (currentMasterState == 0) {
    
    // open or close hatch with the button
    if (digitalRead(buttonHatch) == LOW) {
      if (millis() - hatchTimer > 500) {
        
        if (isHatchOpen == false) {
          isHatchOpen = true;
          myHatch1.write(servo1_open);
          myHatch2.write(servo2_open);
          Serial.println("Opened hatches with button");
        } else {
          isHatchOpen = false;
          myHatch1.write(servo1_locked);
          myHatch2.write(servo2_locked);
          Serial.println("Closed hatches with button");
        }
        hatchTimer = millis();
      }
    }

    // Launch button
    if (digitalRead(buttonLaunch) == LOW) {
      if(millis() - launchTimer > 2000) {
        Serial1.write(0xAA); 
        Serial.println("Told main brain to launch!");
        launchTimer = millis();
      }
    }
  } 
  
  else if (currentMasterState == 1 || currentMasterState == 3) {
    // make sure they are locked during drop
    if (hasDeployed == false) {
        myHatch1.write(servo1_locked);
        myHatch2.write(servo2_locked);
    }
  }
  
  else if (currentMasterState == 2) {
    if (hasDeployed == false) {
      Serial.println("Time to open parachutes!");
      myHatch1.write(servo1_open);
      
      // wait a bit so the arduino doesn't crash from too much power draw
      delay(200); 
      
      myHatch2.write(servo2_open);
      hasDeployed = true;
    }
  } 
  
  else if (currentMasterState == 5) {
    // crashed, just leave them open
  }
}