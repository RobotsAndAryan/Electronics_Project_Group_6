#include <Servo.h> 
#include <Wire.h>
#include <Motoron.h>

// --- MIRRORED SERVO KINEMATICS ---
#define SERVO1_LOCKED_ANGLE 0    
#define SERVO1_OPEN_ANGLE 90    
#define SERVO2_LOCKED_ANGLE 90  
#define SERVO2_OPEN_ANGLE 0      

const int servo1_pin = 7;        
const int servo2_pin = 8;        

// --- UI BUTTON PINS (Wired to GND) ---
const int btn_hatch = A1;     
const int btn_launch = A2;    

Servo hatchServo1; 
Servo hatchServo2; 
MotoronI2C mc; 

uint8_t master_state = 0;

bool hatch_is_open = false;
unsigned long last_hatch_toggle = 0;
unsigned long last_launch_press = 0;
bool winch_active = false;
bool parachutes_deployed = false;

void setup() {
  Serial.begin(115200);   // To PC via USB
  Serial1.begin(115200);  // To Arduino 1 via TX/RX (Pins 1/0)
  
  pinMode(btn_hatch, INPUT_PULLUP);
  pinMode(btn_launch, INPUT_PULLUP);
  
  // Attach and Lock Servos
  hatchServo1.attach(servo1_pin);
  hatchServo2.attach(servo2_pin);
  hatchServo1.write(SERVO1_LOCKED_ANGLE);
  hatchServo2.write(SERVO2_LOCKED_ANGLE);

  // Initialize Motoron Winch
  Wire.begin(); 
  mc.setAddress(16); 
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(1, 100); 
  mc.setMaxDeceleration(1, 100);
  
  Serial.println(F("======================================"));
  Serial.println(F(" ARDUINO 2 (MOTOR CONTROLLER) ONLINE"));
  Serial.println(F("======================================"));
}

void loop() {
  // 1. READ DOWNLINK FROM MAIN BRAIN
  if (Serial1.available() > 0) {
    uint8_t new_state = Serial1.read();
    
    // Reset parachute flag if we cycle back to 0
    if (new_state == 0 && master_state != 0) {
      parachutes_deployed = false;
    }
    master_state = new_state;
  }

  // 2. EXECUTE HARDWARE COMMANDS BASED ON STATE
  
  if (master_state == 0) {
    // STATE 0: Pre-Flight UI Active
    
    // Toggle Hatch locally
    if (digitalRead(btn_hatch) == LOW && (millis() - last_hatch_toggle > 500)) {
      hatch_is_open = !hatch_is_open;
      if (hatch_is_open) {
        hatchServo1.write(SERVO1_OPEN_ANGLE);
        hatchServo2.write(SERVO2_OPEN_ANGLE);
        Serial.println(F("[LOCAL UI] Hatches Opened"));
      } else {
        hatchServo1.write(SERVO1_LOCKED_ANGLE);
        hatchServo2.write(SERVO2_LOCKED_ANGLE);
        Serial.println(F("[LOCAL UI] Hatches Locked"));
      }
      last_hatch_toggle = millis();
    }

    // Launch Button - Send 0xAA Uplink to Main Brain
    if (digitalRead(btn_launch) == LOW && (millis() - last_launch_press > 2000)) {
      Serial1.write(0xAA); 
      Serial.println(F("[UPLINK] Sent Launch Command (0xAA) to Main Brain"));
      last_launch_press = millis();
    }
  } 
  
  else if (master_state == 2) {
    // STATE 2: Free-Fall. Deploy Parachutes immediately.
    if (!parachutes_deployed) {
      Serial.println(F("[EXECUTE] State 2 Received: Deploying Parachutes!"));
      hatchServo1.write(SERVO1_OPEN_ANGLE);
      delay(200); // 200ms stagger
      hatchServo2.write(SERVO2_OPEN_ANGLE);
      parachutes_deployed = true;
    }
  } 
  
  else if (master_state == 5) {
    // STATE 5: Recovery. Run winch.
    if (!winch_active) {
      Serial.println(F("[EXECUTE] State 5 Received: Activating Winch!"));
      hatchServo1.write(SERVO1_LOCKED_ANGLE);
      hatchServo2.write(SERVO2_LOCKED_ANGLE);
      mc.setSpeed(1, 80 0); 
      winch_active = true;
      delay(5000);
    }
  }

  // 3. SAFETY SHUTOFF
  // If we leave state 5, kill the winch instantly
  if (master_state != 5 && winch_active) {
    mc.setSpeed(1, 0);
    winch_active = false;
    Serial.println(F("[EXECUTE] Winch deactivated."));
  }
}