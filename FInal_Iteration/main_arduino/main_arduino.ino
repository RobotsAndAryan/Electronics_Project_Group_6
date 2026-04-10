#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

// threshold variables
float minTemp = 10.0;           
float tempHazardOffset = 6.0;  
float chuteHeight = 50.0;       
int impactThreshold = 300;    
int postCrashTime = 5000; 
int idleTime = 10000; 

// LED pins on the outside
int ledSafe = 2;       
int ledHazard = 3;     
int ledStatus = 4;     

// sensor pins
int trigPin = 5;
int echoPin = 6;
int fsr1Pin = A3; 
int fsr2Pin = A4; 
int fsr3Pin = A5; 

// array to hold previous sonar readings for filtering
float sonarArray[5] = {999.0, 999.0, 999.0, 999.0, 999.0};
int sIndex = 0;

char mySSID[] = "capsule_wifi";
char myPass[] = "sxpn2655"; 
IPAddress targetIP(255, 255, 255, 255); // broadcast 
unsigned int udpPort = 2390;

WiFiUDP myUDP;
GridEYE thermalCam;
Adafruit_MPU6050 myIMU;
int camAddress = 0x69; // IMU is 0x68 so we made camera 69

int baseFSR1 = 0;
int baseFSR2 = 0;
int baseFSR3 = 0;

float dynamicMaxTemp = 25.0; 
bool isWifiConnected = false;

unsigned long lastUdpSend = 0; 
unsigned long lastDebugPrint = 0; 
unsigned long lastSerialSync = 0;

// this struct has to match the python code exactly
#pragma pack(push, 1) 
struct CapsuleData {
  uint32_t time_ms;
  uint8_t currentState; 
  float currentAlt;
  float pitchAngle;      
  float rollAngle;       
  float ax;    
  float ay;    
  float az;    
  uint16_t force1;
  uint16_t force2;
  uint16_t force3;
  float thermalPixels[64];
} sensorData;
#pragma pack(pop)

void updateThermal() {
  // loop through all 64 pixels
  for(int i=0; i<64; i++) { 
    sensorData.thermalPixels[i] = thermalCam.getPixelTemperature(i); 
  }
}

void updateIMU() {
  sensors_event_t a, g, temp;
  myIMU.getEvent(&a, &g, &temp);
  
  sensorData.ax = a.acceleration.x;
  sensorData.ay = a.acceleration.y;
  sensorData.az = a.acceleration.z;
  
  // calculate pitch and roll using gravity maths
  sensorData.pitchAngle = -(atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/PI;
  sensorData.rollAngle = (atan2(a.acceleration.y, a.acceleration.z)*180.0)/PI;
}

void sendTelemetryData(uint8_t stateNow) {
  // talk to the motor controller via serial
  if (millis() - lastSerialSync >= 100) {
    Serial1.write(stateNow);
    lastSerialSync = millis();
  }

  if (isWifiConnected == false) {
    return;
  }
  
  static uint8_t previousState = 255;
  
  // send data faster if we are crashing
  int sendDelay;
  if (stateNow == 4) {
    sendDelay = 5;
  } else {
    sendDelay = 50;
  }
  
  if (stateNow != previousState || (millis() - lastUdpSend >= sendDelay)) {
    sensorData.time_ms = millis();
    sensorData.currentState = stateNow;
    
    myUDP.beginPacket(targetIP, udpPort);
    myUDP.write((uint8_t*)&sensorData, sizeof(CapsuleData)); 
    myUDP.endPacket();
    
    // debug print every half second
    if (millis() - lastDebugPrint > 500) {
       Serial.print("Sending UDP... State: "); 
       Serial.print(stateNow);
       Serial.print(" | Z-Accel: ");
       Serial.println(sensorData.az);
       lastDebugPrint = millis();
    }
    lastUdpSend = millis();
    previousState = stateNow;
  }
}

float getSonarDistance() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) {
    return 999.0; // out of range
  }
  return (duration * 0.0343) / 2.0;
}

float getFilteredAltitude() {
  sonarArray[sIndex] = getSonarDistance();
  sIndex = sIndex + 1;
  if (sIndex > 4) {
    sIndex = 0;
  }
  
  // copy array so we dont mess up the original one
  float sorted[5];
  for(int k=0; k<5; k++){
    sorted[k] = sonarArray[k];
  }
  
  // basic bubble sort to find the median
  for (int i=0; i<4; i++) {
    for (int j=0; j<4-i; j++) {
      if (sorted[j] > sorted[j+1]) {
        float tempVal = sorted[j]; 
        sorted[j] = sorted[j+1]; 
        sorted[j+1] = tempVal;
      }
    }
  }
  return sorted[2]; // middle value
}

void setup() {
  Serial.begin(115200);   
  Serial1.begin(115200);  
  
  delay(2000); // give serial time to start
  
  Serial.println("---------------------------------");
  Serial.println(" ALRS-007 Main Computer Starting");
  Serial.println("---------------------------------");

  pinMode(ledSafe, OUTPUT); 
  pinMode(ledHazard, OUTPUT); 
  pinMode(ledStatus, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);

  Wire.begin(); 
  
  Serial.print("Starting IMU... ");
  if (!myIMU.begin(0x68, &Wire)) {
    Serial.println("Failed! Check wiring.");
    while (1){} // stuck here
  }
  myIMU.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println("Done.");

  Serial.print("Starting Thermal Camera... ");
  thermalCam.begin(camAddress, Wire);
  Serial.println("Done.");

  Serial.print("Getting FSR baseline values... ");
  long f1 = 0;
  long f2 = 0;
  long f3 = 0;
  for(int i=0; i<50; i++) { 
    f1 = f1 + analogRead(fsr1Pin); 
    f2 = f2 + analogRead(fsr2Pin); 
    f3 = f3 + analogRead(fsr3Pin); 
    delay(10); 
  }
  baseFSR1 = f1 / 50; 
  baseFSR2 = f2 / 50; 
  baseFSR3 = f3 / 50;
  Serial.println("Done.");

  Serial.print("Setting up wifi access point... ");
  if (WiFi.beginAP(mySSID, myPass) != WL_AP_LISTENING) {
    while (true); // stuck
  }
  delay(2000); 
  isWifiConnected = true;
  myUDP.begin(udpPort);
  Serial.println("Wifi is ready.");
  
  Serial.println("System is in STATE 0 (Standby). Waiting for launch command...");
  unsigned long standbyTimer = 0;
  char packetBuffer[16];
  
  while (true) {
    if (millis() - standbyTimer >= 50) { 
      sensorData.force1 = analogRead(fsr1Pin); 
      sensorData.force2 = analogRead(fsr2Pin); 
      sensorData.force3 = analogRead(fsr3Pin);
      updateThermal(); 
      updateIMU();
      sensorData.currentAlt = getFilteredAltitude();
      
      sendTelemetryData(0); 
      standbyTimer = millis();
    }
    
    // check if physical button was pressed on motor board
    if (Serial1.available() > 0) {
      if (Serial1.read() == 0xAA) {
        Serial.println("Launch button pressed!");
        break;
      }
    }
    // check if UDP command was sent from python
    if (myUDP.parsePacket()) {
      int packetLen = myUDP.read(packetBuffer, 15);
      packetBuffer[packetLen] = 0; 
      String received = String(packetBuffer);
      if (received == "LAUNCH") {
        Serial.println("Remote launch command received!");
        break; 
      }
    }
  }
}

bool doReconPhase() {
  Serial.println("--- STATE 1: Thermal Scan ---");
  digitalWrite(ledStatus, HIGH); 
  digitalWrite(ledSafe, LOW); 
  digitalWrite(ledHazard, LOW);
  
  updateThermal(); 
  updateIMU();
  sensorData.currentAlt = getFilteredAltitude();
  sendTelemetryData(1);
  
  Serial.println("Ground looks safe. Dropping in 3 seconds...");
  digitalWrite(ledStatus, LOW); 
  digitalWrite(ledSafe, HIGH); 
  delay(5000); 
  
  for(int i=3; i>0; i--) { 
    Serial.print(i); 
    Serial.println("..."); 
    sendTelemetryData(1); 
    delay(1000); 
  }
  digitalWrite(ledSafe, LOW); 
  return true;
}

void doDescentPhase() {
  Serial.println("--- STATE 2: Falling ---");
  digitalWrite(ledStatus, HIGH); 
  
  while (true) {
    float alt = getFilteredAltitude();
    sensorData.currentAlt = alt;
    updateIMU();
    sendTelemetryData(2); 
    
    if (alt < chuteHeight && alt > 5.0) {
      break; 
    }
    delay(20);
  }
  
  Serial.println("Deploying parachutes now!");
  sendTelemetryData(2); 
  delay(1000); 
  digitalWrite(ledStatus, LOW);
}

void doImpactPhase() {
  Serial.println("--- STATE 3: Armed, waiting to hit ground ---");
  digitalWrite(ledStatus, HIGH);
  bool crashed = false;
  
  // wait until the FSR gets pressed hard
  while (crashed == false) {
    sensorData.force1 = analogRead(fsr1Pin); 
    sensorData.force2 = analogRead(fsr2Pin); 
    sensorData.force3 = analogRead(fsr3Pin);
    
    if (sensorData.force1 > baseFSR1 + impactThreshold || 
        sensorData.force2 > baseFSR2 + impactThreshold || 
        sensorData.force3 > baseFSR3 + impactThreshold) {
      crashed = true;
    }
    updateIMU();
    sendTelemetryData(3); 
    delay(5);
  }
  
  Serial.println("--- STATE 4: HIT THE GROUND ---");
  Serial.println("Recording IMU data really fast...");
  unsigned long timeHit = millis();
  
  while (millis() - timeHit < postCrashTime) {
    sensorData.force1 = analogRead(fsr1Pin); 
    sensorData.force2 = analogRead(fsr2Pin); 
    sensorData.force3 = analogRead(fsr3Pin);
    updateIMU();
    sendTelemetryData(4); 
    delay(2); // delay 2ms so we get lots of data points for the graph
  }
  digitalWrite(ledStatus, LOW);
}

void doRecoveryPhase() {
  Serial.println("--- STATE 5: Done ---");
  digitalWrite(ledStatus, HIGH);
  unsigned long timer = millis();
  while (millis() - timer < idleTime) { 
    updateIMU(); 
    sendTelemetryData(5); 
    delay(100); 
  }
  digitalWrite(ledStatus, LOW);
}

void loop() {
  if (doReconPhase()) {
    doDescentPhase(); 
    doImpactPhase(); 
    doRecoveryPhase();
    
    Serial.println("Mission is over. Restarting system...");
    unsigned long t = millis();
    while(millis() - t < 5000) { 
      sendTelemetryData(5); 
      delay(100); 
    }
    NVIC_SystemReset(); // resets the arduino
  } else {
    while(1) { delay(100); }
  }
}