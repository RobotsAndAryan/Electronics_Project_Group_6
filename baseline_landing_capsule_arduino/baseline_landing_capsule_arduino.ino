#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

/* --- CONFIGURATION --- */
#define MAX_TEMP 25.0
#define MIN_TEMP 10.0
#define CHUTE_HEIGHT 50.0       
#define IMPACT_THRESHOLD 1000   
#define POST_IMPACT_LOG_MS 5000 
#define SAMPLE_INTERVAL_MS 5    

/* --- PINOUT --- */
const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int motor1_pin = 7;
const int FSR_pin = A0; 
const int FLEX_pin = A1; 
const int phase1_led = 1; 
const int phase2_led = 2; 
const int phase3_led = 3; 

/* --- TELEMETRY --- */
char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 
IPAddress remoteIP(192, 168, 1, 100); 
unsigned int localPort = 2390;

/* --- OBJECTS --- */
WiFiUDP Udp;
GridEYE grideye;
float pixelTable[64]; 
int fsr_baseline = 0;
bool wifiOnline = false;

/* --- FILTERED SONAR DRIVER --- */
float read_height() {
  digitalWrite(sonar_trig_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(sonar_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar_trig_pin, LOW);
  
  // 25ms timeout is enough for ~4 meters
  long duration = pulseIn(sonar_echo_pin, HIGH, 25000); 
  
  if (duration == 0) return 999.0; // Return a safe high value on error
  
  float dist = (duration * 0.0343) / 2.0;
  
  // Basic outlier rejection
  if (dist < 2.0 || dist > 400.0) return 999.0;
  
  return dist;
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
    if (center_temps[i] < MIN_TEMP || center_temps[i] > MAX_TEMP){
      Serial.print(F("ABORT: Hazard detected. Temp: ")); Serial.println(center_temps[i]);
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
  
  // 1. Wait for deployment height
  while (true) {
    float alt = read_height();
    if (alt < CHUTE_HEIGHT && alt > 5.0) break; 
    delay(30); 
  }

  // 2. Fire Parachute
  Serial.println(F("EVENT: DEPLOYING CHUTE!"));
  digitalWrite(motor1_pin, HIGH);
  delay(1200); 
  digitalWrite(motor1_pin, LOW);
  
  // 3. WAIT FOR GROUND PROXIMITY (FIXED LOGIC)
  Serial.println(F("STATUS: Chute active. Waiting for < 10cm..."));
  int validationCounter = 0;
  while (validationCounter < 3) { // Must see 3 consecutive low readings
    float currentAlt = read_height();
    if (currentAlt < 15.0 && currentAlt > 1.0) {
      validationCounter++;
    } else {
      validationCounter = 0; // Reset if we see a high/error reading
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

  // Trigger Wait
  while (analogRead(FSR_pin) < (fsr_baseline + IMPACT_THRESHOLD)) { 
    // Super-fast loop for ground contact
  }

  unsigned long impactTime = millis();
  Serial.println(F("IMPACT DETECTED! STREAMING DATA..."));
  Serial.println(F("Time_Rel_ms,FSR_Raw,Flex_Raw"));

  while (millis() - impactTime < POST_IMPACT_LOG_MS) {
    unsigned long relT = millis() - impactTime;
    Serial.print(relT); Serial.print(",");
    Serial.print(analogRead(FSR_pin)); Serial.print(",");
    Serial.println(analogRead(FLEX_pin));
    delay(SAMPLE_INTERVAL_MS);
  }

  Serial.println(F("--- DATA STREAM END ---"));
  digitalWrite(phase3_led, LOW);
}

/* --- INITIALIZATION --- */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000)); 

  Serial.println(F("ALRS-007 COMMAND & CONTROL V3.1"));
  analogReadResolution(14); 

  // 1. Calibrate Hardware Baseline
  Serial.print(F("CALIBRATING FSR..."));
  long f_sum = 0;
  for(int i=0; i<50; i++) {
    f_sum += analogRead(FSR_pin);
    delay(10);
  }
  fsr_baseline = f_sum / 50;
  Serial.print(F(" Baseline: ")); Serial.println(fsr_baseline);

  // 2. Network Check
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

  // 3. I2C Check
  Wire1.begin();
  Wire1.beginTransmission(0x69);
  if (Wire1.endTransmission() != 0) {
    Serial.println(F("FATAL: CAMERA NOT FOUND."));
    while(1);
  }
  grideye.begin(0x69, Wire1);

  // 4. Pin Config
  pinMode(sonar_trig_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);
  pinMode(motor1_pin, OUTPUT);
  pinMode(phase1_led, OUTPUT);
  pinMode(phase2_led, OUTPUT);
  pinMode(phase3_led, OUTPUT);

  Serial.println(F("\nSYSTEM ARMED. DROP IN 5s."));
  delay(5000);
}

void loop() {
  if (phase1_recon()) {
    phase2_descent();
    phase3_impact();
    
    Serial.println(F("\n>>> MISSION SUCCESS. HALTED."));
    while(1); 
  } else {
    Serial.println(F("RETRYING..."));
    delay(3000);
  }
}