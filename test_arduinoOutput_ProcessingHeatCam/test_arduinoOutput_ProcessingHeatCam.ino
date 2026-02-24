#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Wire.h>
#include <Modulino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

#define MAX_TEMP 25.0
#define MIN_TEMP 10.0
#define CHUTE_HEIGHT 50.0 // Deploy chute at 50cm
#define BUFFER_SIZE 500   // Safe memory limit for the R4

float pixelTable[64]; // Use float to capture accurate AMG8833 data
float mean_height = 0.0;

// Pin Definitions
const int sonar_trig_pin = 5;
const int sonar_echo_pin = 6;
const int motor1_pin = 7;
const int motor2_pin = 8;
const int FSR_pin = A0; // Use Arduino constants, not raw integers

const int phase1_pin = 1;
const int phase2_pin = 2;
const int phase3_pin = 3;

// Network Credentials
char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 
unsigned int localPort = 2390; 
IPAddress remoteIP(192, 168, 1, 100); 
unsigned int remotePort = 2390; 

WiFiUDP Udp;
GridEYE grideye;
ModulinoButtons button;

// --- Helper: Read Altitude ---
float read_height() {
  float total_height = 0;
  int valid_reads = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(sonar_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(sonar_trig_pin, HIGH);
    delayMicroseconds(10); // MUST be microseconds
    digitalWrite(sonar_trig_pin, LOW); 

    long duration = pulseIn(sonar_echo_pin, HIGH, 30000); // 30ms timeout

    if (duration > 0) {
      total_height += (duration * 0.0343) / 2.0;
      valid_reads++;
    }
    delay(10); // Short delay between pings to prevent echo overlap
  }

  if (valid_reads == 0) {
    Serial.println("Sonar Error: No valid echoes.");
    return 999.0; // Return safe high altitude on failure
  }

  mean_height = total_height / valid_reads;
  return mean_height;
}

// --- Phase 1: Recon ---
bool phase1() {
  Serial.println("Executing Thermal Scan...");
  
  for(int i = 0; i < 64; i++){
    pixelTable[i] = grideye.getPixelTemperature(i);
  } 

  // Send telemetry
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write((uint8_t*)pixelTable, sizeof(pixelTable)); 
  Udp.endPacket();
  Serial.println("Thermal Array sent via UDP.");

  // Check the center 4 pixels: indices 27, 28, 35, 36
  float center_temps[4] = {pixelTable[27], pixelTable[28], pixelTable[35], pixelTable[36]};

  for (int i = 0; i < 4; i++){
    if (center_temps[i] < MIN_TEMP || center_temps[i] > MAX_TEMP){
      Serial.println("Hazard Detected in Landing Zone!");
      return false;
    }
  }
  return true;
}

// --- Phase 2: Descent ---
bool phase2() {
  Serial.println("Descending... Waiting for Chute Altitude.");
  
  while (read_height() > CHUTE_HEIGHT) {
    Serial.print("Current Altitude: ");
    Serial.println(mean_height);
  }
  
  // Deploy Parachute
  Serial.println("DEPLOYING PARACHUTE!");
  digitalWrite(motor1_pin, HIGH);
  delay(1000); // 1 second is plenty for a DC motor hatch
  digitalWrite(motor1_pin, LOW);

  // Wait until impact is imminent
  while (read_height() > 10.0) {
    Serial.println("Approaching ground...");
  }
  return true;
}

// --- Phase 3: Impact ---
void phase3() {
  int sensor_dataset[BUFFER_SIZE]; 
  int baseline_fsr = analogRead(FSR_pin);
  
  Serial.println("ARMED: Waiting for Impact Spike...");

  // Block until FSR spikes 100 units above baseline (Impact occurs)
  while (analogRead(FSR_pin) < (baseline_fsr + 100)) {
    // Waiting for ground contact
  }

  Serial.println("IMPACT DETECTED! Recording high-speed data...");
  
  // High-speed buffer capture
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sensor_dataset[i] = analogRead(FSR_pin);
    delayMicroseconds(500); // 2kHz sampling rate
  }

  Serial.println("Switch to Serial Plotter to view data.");
  delay(2000);
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print("Flex_Value:");
    Serial.println(sensor_dataset[i]);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  // Initialize WiFi
  Serial.print("Connecting to WiFi...");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(500);
  }
  Serial.println("Connected!");
  Udp.begin(localPort);

  // Initialize hardware
  Wire1.begin(); // Use Wire1 for SparkFun Qwiic AMG8833
  if (!grideye.begin(GridEYE_DEFAULT_ADDRESS, Wire1)) {
    Serial.println("Camera fault. Halting.");
    while(1);
  }
  
  Modulino.begin();
  button.begin();
  analogReadResolution(14); // Critical for high-res impact data

  pinMode(sonar_trig_pin, OUTPUT);
  pinMode(sonar_echo_pin, INPUT);
  pinMode(motor1_pin, OUTPUT);
  pinMode(motor2_pin, OUTPUT);
  pinMode(phase1_pin, OUTPUT);
  pinMode(phase2_pin, OUTPUT);
  pinMode(phase3_pin, OUTPUT);
}

void loop() {
  button.setLeds(1, 0, 0); // Red LED
  Serial.println("To start Phase 1 , Press Button A");

  while (!button.update() || !button.isPressed(0)) {
    // Wait for button press
    delay(50);
  }

  Serial.println("Button A pressed! Initializing Thermal Reconnaissance");
  digitalWrite(phase1_pin, HIGH);
  
  if (phase1()) {
    digitalWrite(phase1_pin, LOW);
    digitalWrite(phase2_pin, HIGH);
    button.setLeds(0, 1, 0); // Green LED
    
    if (phase2()) {
      digitalWrite(phase2_pin, LOW);
      digitalWrite(phase3_pin, HIGH);
      phase3();
      
      Serial.println("Mission Complete. Halting.");
      while(1); // Stop execution after one full drop
    }
  } else {
    Serial.println("Location not suitable for Drop, RELOCATE!!");
    button.setLeds(1, 0, 0); // Back to red
    delay(2000); // Wait before allowing a restart
  }
}
