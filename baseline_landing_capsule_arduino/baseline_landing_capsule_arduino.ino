#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Servo.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/* --- CONFIGURATION --- */
#define MIN_TEMP 10.0           
#define TEMP_HAZARD_OFFSET 6.0  
#define CHUTE_HEIGHT 50.0       
#define TILT_THRESHOLD 45.0     // Emergency chute deployment angle (Degrees)
#define IMPACT_THRESHOLD 1000   
#define POST_IMPACT_LOG_MS 5000 
#define SAMPLE_INTERVAL_MS 10   
#define WINCH_RETRACT_MS 4000   // Duration to run the winch motor

/* --- SERVO CONFIGURATION (Hatch Release) --- */
#define HATCH_LOCKED_ANGLE 0    
#define HATCH_OPEN_ANGLE 180    // FIXED: Standard servos physically cannot exceed 180 degrees     

/* --- PINOUT --- */
const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int servo_pin = 7;        // Hatch release servo
const int winch_in1 = 8;        // L298N Motor Driver IN1 (Winch)
const int winch_in2 = 9;        // L298N Motor Driver IN2 (Winch)
const int FSR_pin = A0; 
const int phase1_led = 2;       // FIXED: Shifted away from TX pin
const int phase2_led = 3; 
const int phase3_led = 4; 

float last_alt = 0;
unsigned long last_time = 0;

/* --- MEDIAN FILTER BUFFER --- */
float sonar_buffer[5] = {999.0, 999.0, 999.0, 999.0, 999.0};
int sonar_idx = 0;

/* --- Connection to base_station --- */
char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 
IPAddress remoteIP(10,72,31,75); 
unsigned int localPort = 2390;

/* --- OBJECTS --- */
WiFiUDP Udp;
GridEYE grideye;
Servo hatchServo; 
Adafruit_MPU6050 mpu; 

float pixelTable[64]; 
int fsr_baseline = 0;
float dynamic_max_temp = 25.0; 
bool wifiOnline = false;

/* --- I2C BUS RECOVERY FUNCTION --- */
// Forces stuck slave devices to release the SDA line
void clearI2CBus() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(20);
  
  if (digitalRead(SDA) == LOW) {
    Serial.println(F("WARNING: I2C BUS LOCKED. ATTEMPTING RECOVERY..."));
    pinMode(SCL, OUTPUT);
    for (byte i = 0; i < 9; i++) {
      digitalWrite(SCL, LOW);
      delayMicroseconds(20);
      digitalWrite(SCL, HIGH);
      delayMicroseconds(20);
      if (digitalRead(SDA) == HIGH) {
        Serial.println(F("I2C BUS RECOVERED."));
        break;
      }
    }
  }
}

/* --- FILTERED SONAR DRIVER --- */
float get_raw_height() {
  digitalWrite(sonar_trig_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonar_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_trig_pin, LOW);
  long duration = pulseIn(sonar_echo_pin, HIGH, 25000); 
  if (duration == 0) return 999.0; 
  float dist = (duration * 0.0343) / 2.0;
  if (dist < 2.0 || dist > 600.0) return 999.0;
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
        float temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
      }
    }
  }
  return sorted[2]; 
}

/* --- PHASE 1: THERMAL RECON --- */
bool phase1_recon() {
  Serial.println(F("\n>>> PHASE 1: THERMAL SCANNING..."));
  digitalWrite(phase1_led, HIGH);
  for(int i = 0; i < 64; i++){
    pixelTable[i] = grideye.getPixelTemperature(i);
  } 
  if (wifiOnline) {
    Udp.beginPacket(remoteIP, localPort);
    Udp.write((uint8_t*)pixelTable, sizeof(pixelTable)); 
    Udp.endPacket();
  }
  float center_temps[4] = {pixelTable[27], pixelTable[28], pixelTable[35], pixelTable[36]};
  for (int i = 0; i < 4; i++){
    if (center_temps[i] < MIN_TEMP || center_temps[i] > dynamic_max_temp){
      Serial.print(F("ABORT: Hazard detected. Temp: ")); Serial.print(center_temps[i]);
      Serial.print(F("C (Threshold: ")); Serial.print(dynamic_max_temp); Serial.println(F("C)"));
      digitalWrite(phase1_led, LOW);
      return false;
    }
  }
  Serial.println(F("STATUS: Zone Clear."));
  delay(1000);
  digitalWrite(phase1_led, LOW);
  return true;
}

/* --- PHASE 2: DESCENT CONTROL --- */
void phase2_descent() {
  Serial.println(F("\n>>> PHASE 2: DESCENT MONITORING ACTIVE"));
  digitalWrite(phase2_led, HIGH);
  
  last_time = millis();
  last_alt = read_height_median();

  // Initialize filter variables
  float filter_pitch = 0.0;
  float filter_roll = 0.0;

  while (true) {
    float alt = read_height_median();
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; 
    
    float vel = 0.0;
    if (dt > 0) vel = (last_alt - alt) / dt; 

    // Read IMU Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (dt > 0) {
      // Step 1: Calculate absolute angles from Accelerometer
      float accel_roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      float accel_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

      // Step 2: Convert Gyro rad/s to degrees/sec
      float gyro_roll_rate = g.gyro.x * 180.0 / PI;
      float gyro_pitch_rate = g.gyro.y * 180.0 / PI;

      // Step 3: Complementary Filter (Sensor Fusion)
      filter_roll = 0.98 * (filter_roll + gyro_roll_rate * dt) + 0.02 * accel_roll;
      filter_pitch = 0.98 * (filter_pitch + gyro_pitch_rate * dt) + 0.02 * accel_pitch;
    }

    float max_tilt = max(abs(filter_pitch), abs(filter_roll));

    Serial.print(F("Alt(cm): ")); Serial.print(alt);
    Serial.print(F(" | Vel(cm/s): ")); Serial.print(vel);
    Serial.print(F(" | Tilt(deg): ")); Serial.println(max_tilt);
    
    // OVERRIDE LOGIC: Altitude reached OR capsule is tumbling uncontrollably
    if ((alt < CHUTE_HEIGHT && alt > 5.0) || max_tilt > TILT_THRESHOLD) {
      if (max_tilt > TILT_THRESHOLD) {
        Serial.println(F("EMERGENCY OVERRIDE: CRITICAL TILT DETECTED!"));
      }
      break; 
    }
    
    last_alt = alt;
    last_time = current_time;
    delay(30); 
  }

  Serial.println(F("EVENT: DEPLOYING CHUTE!"));
  hatchServo.write(HATCH_OPEN_ANGLE); 
  delay(500); 
  
  Serial.println(F("STATUS: Chute active. Waiting for < 10cm..."));
  int validationCounter = 0;
  while (validationCounter < 3) { 
    float currentAlt = read_height_median();
    if (currentAlt < 15.0 && currentAlt > 1.0) {
      validationCounter++;
    } else {
      validationCounter = 0; 
    }
    delay(20);
  }
  Serial.println(F("STATUS: Proximity threshold met. Arming Impact Sensor."));
  digitalWrite(phase2_led, LOW);
}

/* --- PHASE 3: IMPACT LOGGING --- */
void phase3_impact() {
  Serial.println(F("\n>>> PHASE 3: IMPACT DIAGNOSTICS ARMED"));
  digitalWrite(phase3_led, HIGH);

  while (analogRead(FSR_pin) < (fsr_baseline + IMPACT_THRESHOLD)) { 
    // High-speed polling
  }

  unsigned long impactTime = millis();
  Serial.println(F("IMPACT DETECTED! STREAMING ACCELEROMETER DATA..."));
  Serial.println(F("Time_Rel_ms,FSR_Raw,Accel_X(m/s2),Accel_Y(m/s2),Accel_Z(m/s2)"));

  while (millis() - impactTime < POST_IMPACT_LOG_MS) {
    unsigned long relT = millis() - impactTime;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print(relT); Serial.print(",");
    Serial.print(analogRead(FSR_pin)); Serial.print(",");
    Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print(a.acceleration.y); Serial.print(",");
    Serial.println(a.acceleration.z);
    
    delay(SAMPLE_INTERVAL_MS);
  }

  Serial.println(F("--- DATA STREAM END ---"));
  digitalWrite(phase3_led, LOW);
}

/* --- PHASE 4: RECOVERY/RETRACTION --- */
void phase4_recovery() {
  Serial.println(F("\n>>> PHASE 4: PARACHUTE RETRACTION INITIATED"));
  
  // Power the DC Motor via L298N to reel in the parachute
  digitalWrite(winch_in1, HIGH);
  digitalWrite(winch_in2, LOW);
  
  delay(WINCH_RETRACT_MS);
  
  // Cut power to the winch
  digitalWrite(winch_in1, LOW);
  digitalWrite(winch_in2, LOW);
  
  Serial.println(F("STATUS: Parachute secured. System Safe."));
}

/* --- INITIALIZATION --- */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000)); 

  Serial.println(F("ALRS-007 COMMAND & CONTROL V7.3"));
  analogReadResolution(14); 

  // --- MANUAL I2C BUS RECOVERY ---
  clearI2CBus();

  hatchServo.attach(servo_pin);
  hatchServo.write(HATCH_LOCKED_ANGLE);

  // Winch Motor Pins
  pinMode(winch_in1, OUTPUT);
  pinMode(winch_in2, OUTPUT);
  digitalWrite(winch_in1, LOW);
  digitalWrite(winch_in2, LOW);

  // 1. Calibrate FSR 
  Serial.print(F("CALIBRATING FSR..."));
  long f_sum = 0;
  for(int i=0; i<50; i++) {
    f_sum += analogRead(FSR_pin);
    delay(10);
  }
  fsr_baseline = f_sum / 50;
  Serial.print(F(" Baseline: ")); Serial.println(fsr_baseline);

  // 2. I2C Check & Sensor Init
  Wire.begin();   // <-- PRIMARY BUS (A4/A5) FOR MPU6050
  Wire1.begin();  // <-- SECONDARY BUS (QWIIC) FOR GRIDEYE
  
  // INIT MPU6050 on Primary Bus
  if (!mpu.begin(0x68, &Wire)) { 
    Serial.println(F("FATAL: MPU6050 NOT FOUND ON PRIMARY BUS."));
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("IMU INITIALIZED."));

  // INIT GRIDEYE on Secondary Bus using manual hardware ping
  Wire1.beginTransmission(0x69);
  if (Wire1.endTransmission() == 0) {
    grideye.begin(0x69, Wire1);
  } else {
    Wire1.beginTransmission(0x68);
    if (Wire1.endTransmission() == 0) {
      grideye.begin(0x68, Wire1);
    } else {
      Serial.println(F("FATAL: CAMERA NOT FOUND ON QWIIC BUS."));
      while(1) { delay(10); }
    }
  }
  Serial.println(F("THERMAL CAMERA INITIALIZED."));
  delay(500); 

  // 3. Thermal Auto-Calibration 
  Serial.print(F("CALIBRATING THERMAL BASELINE..."));
  float temp_sum = 0;
  int valid_pixels = 0;
  for(int frame = 0; frame < 3; frame++) {
    for(int i = 0; i < 64; i++){
      float temp = grideye.getPixelTemperature(i);
      if(temp > 0.0 && temp < 100.0) { 
        temp_sum += temp;
        valid_pixels++;
      }
    }
    delay(100); 
  }
  if(valid_pixels > 0) {
    float ambient_avg = temp_sum / (float)valid_pixels;
    dynamic_max_temp = ambient_avg + TEMP_HAZARD_OFFSET;
    Serial.print(F(" Ambient: ")); Serial.print(ambient_avg);
    Serial.print(F("C | Hazard Threshold: ")); Serial.print(dynamic_max_temp); Serial.println(F("C"));
  } else {
    Serial.println(F(" FAILED. Using default 25C threshold."));
    dynamic_max_temp = 25.0;
  }

  // 4. Network Check
  WiFi.begin(ssid, pass);
  unsigned long wifiT = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiT < 5000)) {
    delay(500); Serial.print(".");
  }
  wifiOnline = (WiFi.status() == WL_CONNECTED);
  if(wifiOnline) {
    Udp.begin(localPort);
    Serial.println(F(" WiFi [OK]"));
  } else {
    Serial.println(F(" WiFi [TIMEOUT]"));
  }

  // 5. Pin Config
  pinMode(sonar_trig_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);
  pinMode(phase1_led, OUTPUT);
  pinMode(phase2_led, OUTPUT);
  pinMode(phase3_led, OUTPUT);

  // Pre-fill sonar buffer
  for (int i = 0; i < 5; i++) {
    read_height_median();
    delay(20);
  }

  Serial.println(F("\nSYSTEM ARMED. DROP IN 5s."));
  delay(5000);
}

void loop() {
  if (phase1_recon()) {
    phase2_descent();
    phase3_impact();
    phase4_recovery();
    
    Serial.println(F("\n>>> MISSION SUCCESS. REBOOTING IN 5s..."));
    delay(5000); 
    NVIC_SystemReset(); 
  } else {
    Serial.println(F("RETRYING..."));
    delay(3000);
  }
}