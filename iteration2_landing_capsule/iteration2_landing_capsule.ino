#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Servo.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define DESK_TEST_MODE true 

#define MIN_TEMP 10.0           
#define TEMP_HAZARD_OFFSET 6.0  
#define CHUTE_HEIGHT 50.0       
#define TILT_THRESHOLD 45.0     
#define IMPACT_THRESHOLD 300    
#define POST_IMPACT_LOG_MS 5000 
#define SAMPLE_INTERVAL_MS 10   
#define WINCH_RETRACT_MS 4000   
#define HATCH_LOCKED_ANGLE 0    
#define HATCH_OPEN_ANGLE 180    

const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int servo_pin = 7;        
const int winch_in1 = 8;        
const int winch_in2 = 9;        
const int FSR_pin = A0; 
const int phase1_led = 2;       
const int phase2_led = 3; 
const int phase3_led = 4; 

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
Servo hatchServo; 
Adafruit_MPU6050 mpu; 

int fsr_baseline = 0;
float dynamic_max_temp = 25.0; 
bool wifiOnline = false;

float filter_pitch = 0.0;
float filter_roll = 0.0;
unsigned long last_udp_tx = 0; 

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
  float thermal_grid[64];
} t_packet;
#pragma pack(pop)

void clearI2CBus() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delay(20);
  if (digitalRead(SDA) == LOW) {
    Serial.println(F("I2C locked. Recovering..."));
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
  if (millis() - last_udp_tx < 50) return; 
  
  t_packet.timestamp = millis();
  t_packet.state = currentState;
  
  Udp.beginPacket(broadcastIP, localPort);
  Udp.write((uint8_t*)&t_packet, sizeof(TelemetryPacket)); 
  Udp.endPacket();
  
  last_udp_tx = millis();
}

float get_raw_height() {
  digitalWrite(sonar_trig_pin, LOW); delayMicroseconds(5);
  digitalWrite(sonar_trig_pin, HIGH); delayMicroseconds(10);
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
  float gyro_roll_rate = g.gyro.x * 180.0 / PI;
  float gyro_pitch_rate = g.gyro.y * 180.0 / PI;

  filter_roll = 0.98 * (filter_roll + gyro_roll_rate * dt) + 0.02 * accel_roll;
  filter_pitch = 0.98 * (filter_pitch + gyro_pitch_rate * dt) + 0.02 * accel_pitch;
  
  t_packet.pitch = filter_pitch;
  t_packet.roll = filter_roll;
}

bool phase1_recon() {
  Serial.println(F("\nPhase 1: Thermal"));
  digitalWrite(phase1_led, HIGH);
  
  for(int i = 0; i < 64; i++){
    t_packet.thermal_grid[i] = grideye.getPixelTemperature(i);
  } 
  t_packet.altitude = read_height_median();
  updateIMU(0); 
  transmitTelemetry(1); 

  float center_temps[4] = {t_packet.thermal_grid[27], t_packet.thermal_grid[28], t_packet.thermal_grid[35], t_packet.thermal_grid[36]};

  for (int i = 0; i < 4; i++){
    if (center_temps[i] < MIN_TEMP || center_temps[i] > dynamic_max_temp){
      Serial.println(F("Hazard found. Retrying..."));
      digitalWrite(phase1_led, LOW);
      if (DESK_TEST_MODE) delay(2000);
      return false; 
    }
  }
  
  Serial.println(F("Clear."));
  if (DESK_TEST_MODE) delay(3000); else delay(1000);
  
  digitalWrite(phase1_led, LOW);
  return true;
}

void phase2_descent() {
  Serial.println(F("\nPhase 2: Descent"));
  digitalWrite(phase2_led, HIGH);
  last_time = millis();
  
  unsigned long last_print = 0;

  while (true) {
    float alt = read_height_median();
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; 
    
    updateIMU(dt);
    t_packet.altitude = alt;
    transmitTelemetry(2); 

    float max_tilt = max(abs(filter_pitch), abs(filter_roll));
    
    if (DESK_TEST_MODE && (current_time - last_print >= 500)) {
      Serial.print(F("Alt: ")); Serial.print(alt);
      Serial.print(F(" | Tilt: ")); Serial.println(max_tilt);
      last_print = current_time;
    }

    if ((alt < CHUTE_HEIGHT && alt > 5.0) || max_tilt > TILT_THRESHOLD) {
      break; 
    }
    
    last_time = current_time;
    delay(20); 
  }

  Serial.println(F("Deploy chute."));
  hatchServo.write(HATCH_OPEN_ANGLE); 
  delay(500); 
  
  int validationCounter = 0;
  while (validationCounter < 3) { 
    float alt = read_height_median();
    t_packet.altitude = alt;
    transmitTelemetry(2);
    if (alt < 15.0 && alt > 1.0) validationCounter++;
    else validationCounter = 0; 
    delay(20);
  }
  digitalWrite(phase2_led, LOW);
}

void phase3_impact() {
  Serial.println(F("\nPhase 3: Armed"));
  digitalWrite(phase3_led, HIGH);
  
  last_time = millis();
  unsigned long last_fsr_print = millis();
  int target_impact = fsr_baseline + IMPACT_THRESHOLD;

  while (analogRead(FSR_pin) < target_impact) { 
    unsigned long current_time = millis();
    updateIMU((current_time - last_time) / 1000.0);
    last_time = current_time;
    
    if (DESK_TEST_MODE && (current_time - last_fsr_print >= 500)) {
      Serial.print(F("FSR: ")); Serial.println(analogRead(FSR_pin));
      last_fsr_print = current_time;
    }
    
    t_packet.altitude = 0.0; 
    transmitTelemetry(3);
    delay(10);
  }

  unsigned long impactTime = millis();
  Serial.println(F("\nImpact! Logging..."));

  while (millis() - impactTime < POST_IMPACT_LOG_MS) {
    unsigned long current_time = millis();
    updateIMU((current_time - last_time) / 1000.0);
    last_time = current_time;
    
    transmitTelemetry(4); 
    
    if (!DESK_TEST_MODE) {
      Serial.print(millis() - impactTime); Serial.print(",");
      Serial.print(analogRead(FSR_pin)); Serial.print(",");
      Serial.print(t_packet.accel_x); Serial.print(",");
      Serial.print(t_packet.accel_y); Serial.print(",");
      Serial.println(t_packet.accel_z);
    }
    delay(SAMPLE_INTERVAL_MS);
  }
  
  digitalWrite(phase3_led, LOW);
}

void phase4_recovery() {
  Serial.println(F("\nPhase 4: Winch"));
  transmitTelemetry(5); 
  
  digitalWrite(winch_in1, HIGH); digitalWrite(winch_in2, LOW);
  delay(WINCH_RETRACT_MS);
  digitalWrite(winch_in1, LOW); digitalWrite(winch_in2, LOW);
  
  Serial.println(F("Done."));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000)); 

  Serial.println(F("ALRS-007 INIT"));

  clearI2CBus();
  hatchServo.attach(servo_pin);
  hatchServo.write(HATCH_LOCKED_ANGLE);

  pinMode(winch_in1, OUTPUT); pinMode(winch_in2, OUTPUT);
  digitalWrite(winch_in1, LOW); digitalWrite(winch_in2, LOW);

  long f_sum = 0;
  for(int i=0; i<50; i++) { f_sum += analogRead(FSR_pin); delay(10); }
  fsr_baseline = f_sum / 50;

  Wire.begin(); Wire1.begin();  
  if (!mpu.begin(0x68, &Wire)) while (1);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Wire1.beginTransmission(0x69);
  if (Wire1.endTransmission() == 0) grideye.begin(0x69, Wire1);
  else {
    Wire1.beginTransmission(0x68);
    if (Wire1.endTransmission() == 0) grideye.begin(0x68, Wire1);
    else while(1);
  }
  delay(500); 

  float temp_sum = 0; int valid_pixels = 0;
  for(int frame=0; frame<3; frame++) {
    for(int i=0; i<64; i++){
      float temp = grideye.getPixelTemperature(i);
      if(temp>0.0 && temp<100.0) { temp_sum += temp; valid_pixels++; }
    }
    delay(100); 
  }
  if(valid_pixels > 0) dynamic_max_temp = (temp_sum / valid_pixels) + TEMP_HAZARD_OFFSET;
  else dynamic_max_temp = 25.0;

  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) while (true);
  delay(2000); 
  wifiOnline = true;
  Udp.begin(localPort);
  Serial.print(F("AP IP: ")); Serial.println(WiFi.localIP());

  pinMode(sonar_trig_pin, OUTPUT); pinMode(sonar_echo_pin, INPUT);
  pinMode(phase1_led, OUTPUT); pinMode(phase2_led, OUTPUT); pinMode(phase3_led, OUTPUT);

  for (int i = 0; i < 5; i++) { read_height_median(); delay(20); }
  
  Serial.println(F("Waiting for LAUNCH..."));
  char incomingBuffer[16];
  while (true) {
    digitalWrite(LED_BUILTIN, (millis() % 1000) < 200); 
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int len = Udp.read(incomingBuffer, 15);
      incomingBuffer[len] = 0; 
      if (strcmp(incomingBuffer, "LAUNCH") == 0) {
        digitalWrite(LED_BUILTIN, LOW); 
        Serial.println(F("LAUNCHING"));
        break; 
      }
    }
    delay(10);
  }
}

void loop() {
  if (phase1_recon()) {
    phase2_descent();
    phase3_impact();
    phase4_recovery();
    delay(10000);
    NVIC_SystemReset(); 
  } else {
    delay(3000);
  }
}