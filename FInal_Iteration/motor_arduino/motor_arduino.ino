#include <Servo.h> 

// --- MIRRORED SERVO KINEMATICS ---
#define SERVO1_LOCKED_ANGLE 0    
#define SERVO1_OPEN_ANGLE 70    
#define SERVO2_LOCKED_ANGLE 70  
#define SERVO2_OPEN_ANGLE 0      

const int servo1_pin = 7;        
const int servo2_pin = 8;        

// --- UI BUTTON PINS (Wired to GND) ---
const int btn_hatch = A1;     
const int btn_launch = A2;    

Servo hatchServo1; 
Servo hatchServo2; 

uint8_t master_state = 0;

bool hatch_is_open = false;
unsigned long last_hatch_toggle = 0;
unsigned long last_launch_press = 0;
bool parachutes_deployed = false;

void setup() {
  Serial.begin(115200);   
  Serial1.begin(115200);  // Comm link to Main Brain (Pins 1/0)
  
  pinMode(btn_hatch, INPUT_PULLUP);
  pinMode(btn_launch, INPUT_PULLUP);
  
  hatchServo1.attach(servo1_pin);
  hatchServo2.attach(servo2_pin);
  hatchServo1.write(SERVO1_LOCKED_ANGLE);
  hatchServo2.write(SERVO2_LOCKED_ANGLE);

  Serial.println(F("======================================"));
  Serial.println(F(" MOTOR CONTROLLER ONLINE"));
  Serial.println(F("======================================"));
}

void loop() {
  // 1. READ DOWNLINK FROM MAIN BRAIN
  if (Serial1.available() > 0) {
    uint8_t new_state = Serial1.read();
    
    if (new_state == 0 && master_state != 0) {
      parachutes_deployed = false; // Reset for next flight
    }
    master_state = new_state;
  }

  // 2. EXECUTE HARDWARE COMMANDS
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

    // Launch Button - Send 0xAA Uplink
    if (digitalRead(btn_launch) == LOW && (millis() - last_launch_press > 2000)) {
      Serial1.write(0xAA); 
      Serial.println(F("[UPLINK] Sent Launch Command (0xAA) to Main Brain"));
      last_launch_press = millis();
    }
  } 
  else if (master_state == 1 || master_state == 3) {
    if (!parachutes_deployed) {
        hatchServo1.write(SERVO1_LOCKED_ANGLE);
        hatchServo2.write(SERVO2_LOCKED_ANGLE);
    }
  }
  else if (master_state == 2) {
    if (!parachutes_deployed) {
      Serial.println(F("[EXECUTE] State 2 Received: Deploying Parachutes!"));
      hatchServo1.write(SERVO1_OPEN_ANGLE);
      delay(200); // 200ms stagger to prevent brownout
      hatchServo2.write(SERVO2_OPEN_ANGLE);
      parachutes_deployed = true;
    }
  } 
  else if (master_state == 5) {
    // Touchdown. Leave servos open so chute cords aren't crushed.
  }
}