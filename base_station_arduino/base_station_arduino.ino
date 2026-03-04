/* * ALRS-007 BASE STATION RECEIVER 
 * Hardware: Arduino Uno R4 WiFi (or compatible)
 */

#include <WiFiS3.h>
#include <WiFiUdp.h>

/* --- CONFIGURATION --- */
char ssid[] = "capsule_wifi"; // MUST match the network the capsule is on
char pass[] = "sxpn2655";
unsigned int localPort = 2390;

/* --- OBJECTS & BUFFERS --- */
WiFiUDP Udp;
byte packetBuffer[256]; // 64 floats * 4 bytes = 256 bytes expected
float thermalGrid[64];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println(F("\n======================================="));
  Serial.println(F("   ALRS-007 BASE STATION INITIALIZED   "));
  Serial.println(F("======================================="));

  // 1. Connect to the network
  Serial.print(F("Connecting to WiFi network: "));
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(F("\nWiFi connected."));
  Serial.print(F("Base Station IP Address: "));
  Serial.println(WiFi.localIP());

  // CRITICAL: If this IP address does not exactly match the 'remoteIP' 
  // hardcoded in your capsule code, you will receive nothing.

  // 2. Start UDP Listener
  Udp.begin(localPort);
  Serial.print(F("Listening for incoming telemetry on UDP port: "));
  Serial.println(localPort);
  Serial.println(F("Awaiting Capsule Drop Sequence...\n"));
}

void loop() {
  // Check if a UDP packet has arrived
  int packetSize = Udp.parsePacket();
  
  if (packetSize) {
    // Read the packet into our raw byte buffer
    Udp.read(packetBuffer, 256);
    
    // Validate packet size to prevent memory corruption
    if (packetSize == 256) {
      
      // Memory Copy: Cast the 256 raw bytes directly into the 64-element float array
      memcpy(thermalGrid, packetBuffer, 256);
      
      // Process the data
      float max_temp = -999.0;
      float center_sum = 0;
      
      // Calculate center average (Indices 27, 28, 35, 36)
      center_sum = thermalGrid[27] + thermalGrid[28] + thermalGrid[35] + thermalGrid[36];
      float center_avg = center_sum / 4.0;

      // Clear terminal (works on most ANSI serial monitors)
      Serial.write(27);
      Serial.print("[2J"); 
      Serial.write(27);
      Serial.print("[H");
      
      Serial.print(F("--- CAPSULE TELEMETRY RECEIVED FROM: "));
      Serial.print(Udp.remoteIP());
      Serial.println(F(" ---"));

      // Print the 8x8 Grid
      for (int i = 0; i < 64; i++) {
        // Track the hottest pixel in the frame
        if (thermalGrid[i] > max_temp) {
          max_temp = thermalGrid[i];
        }
        
        // Print with exactly 1 decimal point for alignment
        Serial.print(thermalGrid[i], 1);
        Serial.print(F("\t"));
        
        // New line every 8 pixels to form the grid
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
      Serial.print(packetSize);
      Serial.println(F(" bytes. Expected 256."));
    }
  }
}