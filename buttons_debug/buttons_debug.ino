#include <Servo.h> 

// --- MIRRORED SERVO KINEMATICS ---
#define SERVO1_LOCKED_ANGLE 0    
#define SERVO1_OPEN_ANGLE 90    
#define SERVO2_LOCKED_ANGLE 90  
#define SERVO2_OPEN_ANGLE 0      

// --- ACTUATOR PINS ---
const int servo1_pin = 7;        
const int servo2_pin = 8;        

// --- UI BUTTON PINS (Wired to GND) ---
const int btn_hatch = A1;     
const int btn_launch = A2;    

// DUAL SERVO OBJECTS
Servo hatchServo1; 
Servo hatchServo2; 

// Hatch UI State
bool hatch_is_open = false;
unsigned long last_hatch_toggle = 0;

void setup() {
  // 1. VISUAL PROOF OF LIFE (Onboard LED strobes rapidly)
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(50);
    digitalWrite(LED_BUILTIN, LOW); delay(50);
  }

  Serial.begin(115200);
  
  // 2. THE ANTI-HANG TIMEOUT (Wait max 3 seconds for monitor)
  unsigned long boot_time = millis();
  while (!Serial && (millis() - boot_time < 3000)); 
  
  Serial.println(F("\n======================================"));
  Serial.println(F(" ISOLATION TEST: STAGE 0 KINEMATICS"));
  Serial.println(F("======================================"));

  // Initialize Buttons with Internal Pullups
  pinMode(btn_hatch, INPUT_PULLUP);
  pinMode(btn_launch, INPUT_PULLUP);
  
  // Attach and Lock Servos
  Serial.print(F("Attaching Servos and moving to LOCKED position... "));
  hatchServo1.attach(servo1_pin);
  hatchServo2.attach(servo2_pin);
  hatchServo1.write(SERVO1_LOCKED_ANGLE);
  hatchServo2.write(SERVO2_LOCKED_ANGLE);
  Serial.println(F("Done."));
  
  Serial.println(F("\n[READY] Press A1 to toggle hatch. Press A2 to simulate launch."));
}

void loop() {
  
  // 1. TEST HATCH TOGGLE BUTTON (A1)
  if (digitalRead(btn_hatch) == LOW) {
    if (millis() - last_hatch_toggle > 500) { // 500ms debounce
      hatch_is_open = !hatch_is_open;
      
      if (hatch_is_open) {
        hatchServo1.write(SERVO1_OPEN_ANGLE);
        hatchServo2.write(SERVO2_OPEN_ANGLE);
        Serial.println(F("[ACTION] Hatch Button Pressed -> Hatches OPENED (90 deg spread)"));
      } else {
        hatchServo1.write(SERVO1_LOCKED_ANGLE);
        hatchServo2.write(SERVO2_LOCKED_ANGLE);
        Serial.println(F("[ACTION] Hatch Button Pressed -> Hatches LOCKED (0 deg spread)"));
      }
      last_hatch_toggle = millis();
    }
  }

  // 2. TEST MANUAL LAUNCH BUTTON (A2)
  if (digitalRead(btn_launch) == LOW) {
    Serial.println(F("[ACTION] Launch Button Pressed -> SIMULATING LAUNCH SEQUENCE..."));
    delay(1000); 
    Serial.println(F("[READY] Standing by."));
  }
}