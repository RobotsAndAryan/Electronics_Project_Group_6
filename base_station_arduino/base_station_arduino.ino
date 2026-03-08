/* --- ALRS-007 BASE STATION RECEIVER (ACCESS POINT MODE) --- */

#include <WiFiS3.h>
#include <WiFiUdp.h>

/* --- CONFIGURATION --- */
char ssid[] = "capsule_wifi"; // The network name this board will CREATE
char pass[] = "sxpn2655";     // The password (must be at least 8 chars)
unsigned int localPort = 2390;

/* --- OBJECTS & BUFFERS --- */
WiFiUDP Udp;
byte packetBuffer[256]; 
float thermalGrid[64];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println(F("\n======================================="));
  Serial.println(F("   ALRS-007 BASE STATION INITIALIZED   "));
  Serial.println(F("======================================="));

  // 1. Create the WiFi Access Point
  Serial.print(F("Creating isolated WiFi network: "));
  Serial.println(ssid);
  
  int status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println(F("FATAL: Creating Access Point failed. Halting."));
    while (true); 
  }
  
  delay(2000); // Allow AP to stabilize
  
  Serial.println(F("\n[!] NETWORK HOSTED SUCCESSFULLY."));
  Serial.print(F("Base Station IP Address: "));
  IPAddress myIP = WiFi.localIP();
  Serial.println(myIP);

  // 2. Start UDP Listener
  Udp.begin(localPort);
  Serial.print(F("Listening for incoming telemetry on UDP port: "));
  Serial.println(localPort);
  Serial.println(F("Awaiting Capsule Connection & Drop Sequence...\n"));
}

void loop() {
  int packetSize = Udp.parsePacket();
  
  if (packetSize) {
    Udp.read(packetBuffer, 256);
    
    if (packetSize == 256) {
      // Decode raw bytes to floats
      memcpy(thermalGrid, packetBuffer, 256);
      
      float max_temp = -999.0;
      float center_sum = thermalGrid[27] + thermalGrid[28] + thermalGrid[35] + thermalGrid[36];
      float center_avg = center_sum / 4.0;

      // Clear terminal (ANSI escape codes)
      Serial.write(27);
      Serial.print("[2J"); 
      Serial.write(27);
      Serial.print("[H");
      
      Serial.print(F("--- CAPSULE TELEMETRY RECEIVED FROM: "));
      Serial.print(Udp.remoteIP());
      Serial.println(F(" ---"));

      // Print 8x8 Grid
      for (int i = 0; i < 64; i++) {
        if (thermalGrid[i] > max_temp) {
          max_temp = thermalGrid[i];
        }
        Serial.print(thermalGrid[i], 1);
        Serial.print(F("\t"));
        if ((i + 1) % 8 == 0) {
          Serial.println();
        }
      }
      
      Serial.println(F("---------------------------------------------"));
      Serial.print(F("Center Target Average: ")); 
      Serial.print(center_avg, 2); 
      Serial.println(F(" C"));
      Serial.print(F("Maximum Grid Temp:     ")); 
      Serial.print(max_temp, 2); 
      Serial.println(F(" C"));
      Serial.println(F("---------------------------------------------\n"));
      
    } else {
      Serial.print(F("[!] WARNING: Malformed packet received. Size: "));
      Serial.println(packetSize);
    }
  }
}