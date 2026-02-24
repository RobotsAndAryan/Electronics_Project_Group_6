#include <WiFiS3.h>
#include <WiFiUdp.h>

char ssid[] = "capsule_wifi";
char pass[] = "sxpn2655"; 

unsigned int localPort = 2390; 
WiFiUDP Udp;
float receivedArray[64]; 

void setup() {
  // Use 115200 for the R4.
  Serial.begin(115200);
  
  // LED Heartbeat - if this doesn't light up, the board isn't even reaching setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(2000); // Give the Serial Monitor time to open
  Serial.println("\n--- ALRS-007 BASE STATION BOOT ---");

  // Force AP Mode
  Serial.print("Creating AP: ");
  Serial.println(ssid);
  
  int status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("AP ERROR: Check if your board has the latest WiFi firmware.");
    while (1) {
      // Blink SOS if AP fails
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW); delay(100);
    }
  }

  Serial.print("Base IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.println("Listening for Capsule...");
}

void loop() {
  // Blink LED to show loop is running
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Packet Received! Size: ");
    Serial.println(packetSize);

    if (packetSize == sizeof(receivedArray)) {
      Udp.read((uint8_t*)receivedArray, sizeof(receivedArray));
      
      Serial.println("\n[THERMAL DATA]");
      for (int i = 0; i < 64; i++) {
        Serial.print(receivedArray[i], 1);
        Serial.print((i + 1) % 8 == 0 ? "\n" : "\t");
      }
    }
  }
}