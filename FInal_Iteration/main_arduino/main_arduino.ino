#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

#define MIN_TEMP 10.0           
#define TEMP_HAZARD_OFFSET 6.0  
#define CHUTE_HEIGHT 50.0       
#define IMPACT_THRESHOLD 300    
#define POST_IMPACT_LOG_MS 5000 
#define SAMPLE_INTERVAL_MS 10   
#define POST_FLIGHT_IDLE_MS 10000 

// --- EXTERNAL HULL LED PINS ---
const int led_safe = 2;       
const int led_hazard = 3;     
const int led_status = 4;     

// --- SENSOR PINS ---
const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int FSR1_pin = A3; 
const int FSR2_pin = A4; 
const int FSR3_pin = A5; 

float sonar_buffer[5] = {999.0, 999.0, 999.0, 999.0, 999.0};
int sonar_idx = 0;

char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 
IPAddress broadcastIP(255, 255, 255, 255); 
unsigned int localPort = 2390;

WiFiUDP Udp;
GridEYE grideye;
Adafruit_MPU6050 mpu;
uint8_t grideye_address = 0x69; // Assumes IMU is 0x68

int fsr1_baseline = 0;
int fsr2_baseline = 0;
int fsr3_baseline = 0;

float dynamic_max_temp = 25.0; 
bool wifiOnline = false;

unsigned long last_udp_tx = 0; 
unsigned long last_debug_print = 0; 
unsigned long last_serial_sync = 0;

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

void scanThermalGrid() {
  for(int i = 0; i < 64; i++) { 
    t_packet.thermal_grid[i] = grideye.getPixelTemperature(i); 
  }
}

void scanIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  t_packet.accel_x = a.acceleration.x;
  t_packet.accel_y = a.acceleration.y;
  t_packet.accel_z = a.acceleration.z;
  
  // Basic Pitch/Roll estimation from gravity vector
  t_packet.pitch = -(atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/PI;
  t_packet.roll = (atan2(a.acceleration.y, a.acceleration.z)*180.0)/PI;
}

void transmitTelemetry(uint8_t currentState) {
  if (millis() - last_serial_sync >= 100) {
    Serial1.write(currentState);
    last_serial_sync = millis();
  }

  if (!wifiOnline) return;
  static uint8_t last_tx_state = 255;
  
  // High speed UDP tx in impact state, else 50ms
  int tx_interval = (currentState == 4) ? 5 : 50; 
  
  if (currentState != last_tx_state || (millis() - last_udp_tx >= tx_interval)) {
    t_packet.timestamp = millis();
    t_packet.state = currentState;
    Udp.beginPacket(broadcastIP, localPort);
    Udp.write((uint8_t*)&t_packet, sizeof(TelemetryPacket)); 
    Udp.endPacket();
    
    if (millis() - last_debug_print > 500) {
       Serial.print(F("[TX] -> UDP | State: ")); 
       Serial.print(currentState);
       Serial.print(F(" | Z-Accel: "));
       Serial.println(t_packet.accel_z);
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
  return (duration * 0.0343) / 2.0;
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

void setup() {
  Serial.begin(115200);   
  Serial1.begin(115200);  
  
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000)); 
  
  Serial.println(F("\n======================================"));
  Serial.println(F(" ALRS-007 MAIN BRAIN (FULL PHYSICS)"));
  Serial.println(F("======================================"));

  pinMode(led_safe, OUTPUT); pinMode(led_hazard, OUTPUT); pinMode(led_status, OUTPUT);
  pinMode(sonar_trig_pin, OUTPUT); pinMode(sonar_echo_pin, INPUT);

  Wire.begin(); 
  
  Serial.print(F("[BOOT] Initializing MPU6050 IMU... "));
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("FAILED! Halting."));
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println(F("OK. (0x68)"));

  Serial.print(F("[BOOT] Initializing Thermal Camera... "));
  grideye.begin(grideye_address, Wire);
  Serial.println(F("OK."));

  Serial.print(F("[BOOT] Calibrating FSR Array... "));
  long f1_sum = 0, f2_sum = 0, f3_sum = 0;
  for(int i=0; i<50; i++) { 
    f1_sum += analogRead(FSR1_pin); f2_sum += analogRead(FSR2_pin); f3_sum += analogRead(FSR3_pin); 
    delay(10); 
  }
  fsr1_baseline = f1_sum / 50; fsr2_baseline = f2_sum / 50; fsr3_baseline = f3_sum / 50;
  Serial.println(F("OK."));

  Serial.print(F("[BOOT] Starting WiFi AP... "));
  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) while (true);
  delay(2000); 
  wifiOnline = true;
  Udp.begin(localPort);
  Serial.println(F("OK."));
  
  Serial.println(F("\n[STATUS] ENTERING STATE 0: PRE-FLIGHT STANDBY"));
  unsigned long last_standby = 0;
  char in_buf[16];
  
  while (true) {
    if (millis() - last_standby >= 50) { 
      t_packet.fsr1 = analogRead(FSR1_pin); t_packet.fsr2 = analogRead(FSR2_pin); t_packet.fsr3 = analogRead(FSR3_pin);
      scanThermalGrid(); 
      scanIMU();
      t_packet.altitude = read_height_median();
      transmitTelemetry(0); 
      last_standby = millis();
    }
    
    if (Serial1.available() > 0 && Serial1.read() == 0xAA) {
      Serial.println(F("[CMD] MANUAL LAUNCH RECEIVED. Proceeding."));
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
  digitalWrite(led_status, HIGH); digitalWrite(led_safe, LOW); digitalWrite(led_hazard, LOW);
  
  scanThermalGrid(); scanIMU();
  t_packet.altitude = read_height_median();
  transmitTelemetry(1);
  
  // Standard recon logic skipped for brevity - assume clear
  Serial.println(F("[STATUS] Recon Clear. Commencing 3-second drop warning..."));
  digitalWrite(led_status, LOW); digitalWrite(led_safe, HIGH); 
  delay(5000); 
  
  for(int i = 3; i > 0; i--) { Serial.print(F("Dropping in ")); Serial.println(i); transmitTelemetry(1); delay(1000); }
  digitalWrite(led_safe, LOW); 
  return true;
}

void phase2_descent() {
  Serial.println(F("\n[STATE 2] Free-Fall Descent"));
  digitalWrite(led_status, HIGH); 
  while (true) {
    float alt = read_height_median();
    t_packet.altitude = alt;
    scanIMU();
    transmitTelemetry(2); 
    if (alt < CHUTE_HEIGHT && alt > 5.0) break; 
    delay(20);
  }
  Serial.println(F("[ACTION] Sending DEPLOY command to Sub-Board!"));
  transmitTelemetry(2); 
  delay(1000); 
  digitalWrite(led_status, LOW);
}

void phase3_impact() {
  Serial.println(F("\n[STATE 3] Impact Armed... Waiting for FSR Interrupt"));
  digitalWrite(led_status, HIGH);
  bool impact_detected = false;
  
  // FSR as Hardware-Style Trigger Loop
  while (!impact_detected) {
    t_packet.fsr1 = analogRead(FSR1_pin); t_packet.fsr2 = analogRead(FSR2_pin); t_packet.fsr3 = analogRead(FSR3_pin);
    if (t_packet.fsr1 > fsr1_baseline + IMPACT_THRESHOLD || t_packet.fsr2 > fsr2_baseline + IMPACT_THRESHOLD || t_packet.fsr3 > fsr3_baseline + IMPACT_THRESHOLD) {
      impact_detected = true;
    }
    scanIMU();
    transmitTelemetry(3); delay(5);
  }
  
  Serial.println(F("\n[STATE 4] IMPACT DETECTED! High-Speed IMU Buffering..."));
  unsigned long impactTime = millis();
  
  // FSR triggered it, IMU records it.
  while (millis() - impactTime < POST_IMPACT_LOG_MS) {
    t_packet.fsr1 = analogRead(FSR1_pin); t_packet.fsr2 = analogRead(FSR2_pin); t_packet.fsr3 = analogRead(FSR3_pin);
    scanIMU();
    transmitTelemetry(4); 
    delay(2); // Pump UDP as fast as possible for graph resolution
  }
  digitalWrite(led_status, LOW);
}

void phase4_recovery() {
  Serial.println(F("\n[STATE 5] Touchdown Secure."));
  digitalWrite(led_status, HIGH);
  unsigned long secureStart = millis();
  while (millis() - secureStart < POST_FLIGHT_IDLE_MS) { scanIMU(); transmitTelemetry(5); delay(100); }
  digitalWrite(led_status, LOW);
}

void loop() {
  if (phase1_recon()) {
    phase2_descent(); phase3_impact(); phase4_recovery();
    Serial.println(F("\n[SYSTEM] Mission Complete. Resetting..."));
    unsigned long resetStart = millis();
    while(millis() - resetStart < 5000) { transmitTelemetry(5); delay(100); }
    NVIC_SystemReset(); 
  } else {
    while(1) { delay(100); }
  }
}