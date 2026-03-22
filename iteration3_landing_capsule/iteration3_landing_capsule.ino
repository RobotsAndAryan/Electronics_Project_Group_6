#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Servo.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Motoron.h>
#include <math.h>

#define DESK_TEST_MODE true 

#define MIN_TEMP 10.0           
#define TEMP_HAZARD_OFFSET 6.0  
#define CHUTE_HEIGHT 50.0       
#define TILT_THRESHOLD 35.0     
#define IMPACT_THRESHOLD 300    
#define POST_IMPACT_LOG_MS 5000 
#define SAMPLE_INTERVAL_MS 10   
#define WINCH_RETRACT_MS 20000   

// --- MIRRORED SERVO KINEMATICS ---
#define SERVO1_LOCKED_ANGLE 0    
#define SERVO1_OPEN_ANGLE 90    
#define SERVO2_LOCKED_ANGLE 90  
#define SERVO2_OPEN_ANGLE 0      

// --- EXTERNAL HULL LED PINS ---
const int led_safe = 2;       
const int led_hazard = 3;     
const int led_status = 4;     

// --- SENSOR & ACTUATOR PINS ---
const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int servo1_pin = 7;        
const int servo2_pin = 8;        

// --- PRE-FLIGHT UI BUTTONS ---
const int btn_hatch = A1;     
const int btn_launch = A2;    

// --- TRIPLE FSR ARRAY PINS ---
const int FSR1_pin = A3; 
const int FSR2_pin = A4; 
const int FSR3_pin = A5; 
// --------------

float last_alt = 0;
unsigned long last_time = 0;
float sonar_buffer[5] = {999.0, 999.0, 999.0, 999.0, 999.0};
int sonar_idx = 0;

char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 
IPAddress broadcastIP(255, 255, 255, 255); 
unsigned int localPort = 2390;

WiFiUDP Udp;
GridEYE grideye;

Servo hatchServo1; 
Servo hatchServo2; 

Adafruit_MPU6050 mpu; 
MotoronI2C mc; 

// Independent baselines for all 3 sensors
int fsr1_baseline = 0;
int fsr2_baseline = 0;
int fsr3_baseline = 0;

float dynamic_max_temp = 25.0; 
bool wifiOnline = false;

float filter_pitch = 0.0;
float filter_roll = 0.0;
unsigned long last_udp_tx = 0; 
unsigned long last_debug_print = 0; 

bool hatch_is_open = false;
unsigned long last_hatch_toggle = 0;

// NEW PACKET STRUCTURE: Added 3x uint16_t for FSR data (291 bytes total)
#pragma pack(push, 1) 
struct TelemetryPacket {
  uint32_t timestamp;
  uint8_t state; 
  float altitude;
  float pitch;
  float roll;
  float accel_x;    
  float accel_y;    
  float accel_z; 
  uint16_t fsr1;
  uint16_t fsr2;
  uint16_t fsr3;
  float thermal_grid[64];
} t_packet;
#pragma pack(pop)

void clearI2CBus() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(20);
  if (digitalRead(SDA) == LOW) {
    Serial.println(F("[ERROR] I2C bus locked. Recovering..."));
    pinMode(SCL, OUTPUT);
    for (byte i = 0; i < 9; i++) {
      digitalWrite(SCL, LOW); delayMicroseconds(20);
      digitalWrite(SCL, HIGH); delayMicroseconds(20);
      if (digitalRead(SDA) == HIGH) break;
    }
  }
}

void transmitTelemetry(uint8_t currentState) {
  if (!wifiOnline) return;
  
  static uint8_t last_tx_state = 255;
  
  if (currentState != last_tx_state || (millis() - last_udp_tx >= 50)) {
    t_packet.timestamp = millis();
    t_packet.state = currentState;
    Udp.beginPacket(broadcastIP, localPort);
    Udp.write((uint8_t*)&t_packet, sizeof(TelemetryPacket)); 
    Udp.endPacket();
    
    if (millis() - last_debug_print > 500) {
       Serial.print(F("[TX] -> UDP Packet Sent | State: ")); 
       Serial.println(currentState);
       last_debug_print = millis();
    }
    
    last_udp_tx = millis();
    last_tx_state = currentState;
  }
}

float get_raw_height() {
  digitalWrite(sonar_trig_pin, LOW); delayMicroseconds(5);
  digitalWrite(sonar_trig_pin, HIGH); delayMicroseconds(10);
  digitalWrite(sonar_trig_pin, LOW);
  long duration = pulseIn(sonar_echo_pin, HIGH, 25000); 
  if (duration == 0) return 999.0; 
  float dist = (duration * 0.0343) / 2.0;
  return dist;
}

float read_height_median() {
  sonar_buffer[sonar_idx] = get_raw_height();
  sonar_idx = (sonar_idx + 1) % 5;
  float sorted[5];
  memcpy(sorted, sonar_buffer, sizeof(sonar_buffer));
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (sorted[j] > sorted[j+1]) {
        float temp = sorted[j]; sorted[j] = sorted[j+1]; sorted[j+1] = temp;
      }
    }
  }
  return sorted[2]; 
}

void updateIMU(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  t_packet.accel_x = a.acceleration.x;
  t_packet.accel_y = a.acceleration.y;
  t_packet.accel_z = a.acceleration.z;
  if (dt <= 0) return;
  float accel_roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accel_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  filter_roll = 0.98 * (filter_roll + (g.gyro.x * 180.0 / PI) * dt) + 0.02 * accel_roll;
  filter_pitch = 0.98 * (filter_pitch + (g.gyro.y * 180.0 / PI) * dt) + 0.02 * accel_pitch;
  t_packet.pitch = filter_pitch;
  t_packet.roll = filter_roll;
}

void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000)); 
  
  Serial.println(F("\n======================================"));
  Serial.println(F(" ALRS-007 TRIPLE FSR ARRAY EDITION"));
  Serial.println(F("======================================"));

  clearI2CBus();
  
  hatchServo1.attach(servo1_pin);
  hatchServo2.attach(servo2_pin);
  hatchServo1.write(SERVO1_LOCKED_ANGLE);
  hatchServo2.write(SERVO2_LOCKED_ANGLE);
  
  pinMode(led_safe, OUTPUT);
  pinMode(led_hazard, OUTPUT);
  pinMode(led_status, OUTPUT);
  
  pinMode(btn_hatch, INPUT_PULLUP);
  pinMode(btn_launch, INPUT_PULLUP);
  
  pinMode(sonar_trig_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);
  
  Serial.print(F("[BOOT] Calibrating Triple FSR Array... "));
  long f1_sum = 0, f2_sum = 0, f3_sum = 0;
  for(int i=0; i<50; i++) { 
    f1_sum += analogRead(FSR1_pin); 
    f2_sum += analogRead(FSR2_pin); 
    f3_sum += analogRead(FSR3_pin); 
    delay(10); 
  }
  fsr1_baseline = f1_sum / 50;
  fsr2_baseline = f2_sum / 50;
  fsr3_baseline = f3_sum / 50;
  Serial.println(F("OK."));

  Wire.begin(); 
  
  Serial.print(F("[BOOT] Initializing Motoron (Winch)... "));
  mc.setAddress(16); 
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(1, 100); 
  mc.setMaxDeceleration(1, 100);
  Serial.println(F("OK."));

  Serial.print(F("[BOOT] Initializing IMU... "));
  if (!mpu.begin(0x68, &Wire)) {
      Serial.println(F("FAILED! Check Primary I2C wiring. Halting."));
      while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
  Serial.println(F("OK."));
  
  Serial.print(F("[BOOT] Initializing Thermal Camera... "));
  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) {
    grideye.begin(0x69, Wire);
    Serial.println(F("OK."));
  } else {
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {
        Serial.println(F("\n[FATAL ERROR] GridEYE is at 0x68! Address Collision with MPU6050!"));
        while(1);
    } else {
        Serial.println(F("FAILED! Camera not found on bus. Halting."));
        while(1);
    }
  }
  delay(500); 

  float t_sum = 0; int v_pix = 0;
  for(int frame=0; frame<3; frame++) {
    for(int i=0; i<64; i++){
      float temp = grideye.getPixelTemperature(i);
      if(temp>0.0 && temp<100.0) { t_sum += temp; v_pix++; }
    }
    delay(100); 
  }
  dynamic_max_temp = (t_sum / v_pix) + TEMP_HAZARD_OFFSET;

  Serial.print(F("[BOOT] Starting WiFi AP... "));
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) while (true);
  delay(2000); 
  wifiOnline = true;
  Udp.begin(localPort);
  Serial.println(F("OK."));
  
  for(int i=0; i<3; i++) {
    digitalWrite(led_safe, HIGH); digitalWrite(led_hazard, HIGH); digitalWrite(led_status, HIGH);
    delay(150);
    digitalWrite(led_safe, LOW); digitalWrite(led_hazard, LOW); digitalWrite(led_status, LOW);
    delay(150);
  }

  Serial.println(F("\n[STATUS] ENTERING STATE 0: PRE-FLIGHT STANDBY"));
  
  char in_buf[16];
  unsigned long last_standby = 0;
  
  while (true) {
    if (millis() - last_standby >= 50) { 
      // Continuously read FSRs for live GCS telemetry
      t_packet.fsr1 = analogRead(FSR1_pin);
      t_packet.fsr2 = analogRead(FSR2_pin);
      t_packet.fsr3 = analogRead(FSR3_pin);
      
      for(int i=0; i<64; i++) t_packet.thermal_grid[i] = grideye.getPixelTemperature(i);
      t_packet.altitude = read_height_median();
      updateIMU(0); 
      transmitTelemetry(0); 
      last_standby = millis();
    }
    
    if (digitalRead(btn_hatch) == LOW) {
      if (millis() - last_hatch_toggle > 500) { 
        hatch_is_open = !hatch_is_open;
        if (hatch_is_open) {
          hatchServo1.write(SERVO1_OPEN_ANGLE);
          hatchServo2.write(SERVO2_OPEN_ANGLE);
          Serial.println(F("[STATE 0 UI] Hatches opened."));
        } else {
          hatchServo1.write(SERVO1_LOCKED_ANGLE);
          hatchServo2.write(SERVO2_LOCKED_ANGLE);
          Serial.println(F("[STATE 0 UI] Hatches locked."));
        }
        last_hatch_toggle = millis();
      }
    }

    if (digitalRead(btn_launch) == LOW) {
      Serial.println(F("[CMD] MANUAL LAUNCH PRESSED. Proceeding."));
      break; 
    }
    
    if (Udp.parsePacket()) {
      int len = Udp.read(in_buf, 15);
      in_buf[len] = 0; 
      if (strcmp(in_buf, "LAUNCH") == 0) {
        Serial.println(F("[CMD] REMOTE LAUNCH RECEIVED. Proceeding."));
        break; 
      }
    }
  }
}

bool phase1_recon() {
  Serial.println(F("\n[STATE 1] Thermal Recon Active"));
  
  hatchServo1.write(SERVO1_LOCKED_ANGLE);
  hatchServo2.write(SERVO2_LOCKED_ANGLE);
  
  digitalWrite(led_status, HIGH); 
  digitalWrite(led_safe, LOW);
  digitalWrite(led_hazard, LOW);
  
  for(int i = 0; i < 64; i++){ t_packet.thermal_grid[i] = grideye.getPixelTemperature(i); } 
  t_packet.altitude = read_height_median();
  transmitTelemetry(1);
  
  float center_temps[4] = {t_packet.thermal_grid[27], t_packet.thermal_grid[28], t_packet.thermal_grid[35], t_packet.thermal_grid[36]};
  bool is_safe = true;
  
  for (int i = 0; i < 4; i++){
    if (center_temps[i] < MIN_TEMP || center_temps[i] > dynamic_max_temp){
      is_safe = false;
      break; 
    }
  }
  
  if (!is_safe) {
    Serial.println(F("[ERROR] Hazard detected! Aborting drop."));
    digitalWrite(led_status, LOW);
    digitalWrite(led_hazard, HIGH); 
    delay(5000); 
    digitalWrite(led_hazard, LOW);
    return false;
  }
  
  Serial.println(F("[STATUS] Recon Clear. Payload Safe. Commencing 3-second drop warning..."));
  digitalWrite(led_status, LOW);
  digitalWrite(led_safe, HIGH); 
  delay(5000);
  
  for(int i = 3; i > 0; i--) {
    Serial.print(F("Dropping in ")); Serial.println(i);
    updateIMU(0.01);
    transmitTelemetry(1);
    delay(1000);
  }
  
  digitalWrite(led_safe, LOW); 
  return true;
}

void phase2_descent() {
  Serial.println(F("\n[STATE 2] Free-Fall Descent"));
  digitalWrite(led_status, HIGH); 
  last_time = millis();
  while (true) {
    float alt = read_height_median();
    unsigned long current_time = millis();
    updateIMU((current_time - last_time) / 1000.0);
    t_packet.altitude = alt;
    transmitTelemetry(2); 
    if ((alt < CHUTE_HEIGHT && alt > 5.0) || max(abs(filter_pitch), abs(filter_roll)) > TILT_THRESHOLD) break; 
    last_time = current_time;
    delay(20);
  }
  Serial.println(F("[ACTION] Deploying Dual Parachute Servos (Mirrored & Staggered)!"));
  hatchServo1.write(SERVO1_OPEN_ANGLE);
  delay(200); 
  hatchServo2.write(SERVO2_OPEN_ANGLE);
  delay(500);
  digitalWrite(led_status, LOW);
}

void phase3_impact() {
  Serial.println(F("\n[STATE 3] Impact Armed. Waiting for strike on ANY sensor..."));
  digitalWrite(led_status, HIGH);
  
  bool impact_detected = false;
  
  while (!impact_detected) {
    t_packet.fsr1 = analogRead(FSR1_pin);
    t_packet.fsr2 = analogRead(FSR2_pin);
    t_packet.fsr3 = analogRead(FSR3_pin);
    
    // REDUNDANT LOGICAL OR GATE: Any single sensor crossing threshold triggers impact
    if (t_packet.fsr1 > fsr1_baseline + IMPACT_THRESHOLD ||
        t_packet.fsr2 > fsr2_baseline + IMPACT_THRESHOLD ||
        t_packet.fsr3 > fsr3_baseline + IMPACT_THRESHOLD) {
      impact_detected = true;
    }
    
    updateIMU(0.01);
    transmitTelemetry(3);
    delay(10);
  }
  
  Serial.println(F("\n[STATE 4] IMPACT DETECTED! Logging high-speed crash data..."));
  unsigned long impactTime = millis();
  while (millis() - impactTime < POST_IMPACT_LOG_MS) {
    // Keep reading all 3 FSRs to determine final resting angle
    t_packet.fsr1 = analogRead(FSR1_pin);
    t_packet.fsr2 = analogRead(FSR2_pin);
    t_packet.fsr3 = analogRead(FSR3_pin);
    
    updateIMU(0.01);
    transmitTelemetry(4);
    delay(SAMPLE_INTERVAL_MS);
  }
  Serial.println(F("[STATE 4] Crash logging complete."));
  digitalWrite(led_status, LOW);
}

void phase4_recovery() {
  Serial.println(F("\n[STATE 5] Winch Recovery"));
  digitalWrite(led_status, HIGH);
  unsigned long winchStart = millis();
  
  mc.setSpeed(1, 800); 
  
  while (millis() - winchStart < WINCH_RETRACT_MS) {
    transmitTelemetry(5);
    delay(20);
  }
  
  mc.setSpeed(1, 0); 
  digitalWrite(led_status, LOW);
  Serial.println(F("[STATUS] Recovery complete. System secure."));
}

void loop() {
  if (phase1_recon()) {
    phase2_descent();
    phase3_impact();
    phase4_recovery();
    
    Serial.println(F("\n[SYSTEM] Mission Complete. Resetting in 10 seconds."));
    unsigned long resetStart = millis();
    while(millis() - resetStart < 10000) {
      transmitTelemetry(5);
      delay(100);
    }
    NVIC_SystemReset(); 
  } else {
    Serial.println(F("[SYSTEM] Resetting Recon loop in 3 seconds..."));
    unsigned long waitStart = millis();
    while(millis() - waitStart < 3000) {
      delay(100);
    }
  }
}