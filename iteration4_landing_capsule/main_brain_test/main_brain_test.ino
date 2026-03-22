uint8_t current_state = 0;
unsigned long last_tx_time = 0;
unsigned long state_timer = 0;

void setup() {
  Serial.begin(115200);   // To PC via USB
  Serial1.begin(115200);  // To Arduino 2 via TX/RX (Pins 1/0)
  
  while (!Serial && millis() < 3000); 

  Serial.println(F("======================================"));
  Serial.println(F(" ARDUINO 1 (MAIN BRAIN) - TX/RX TEST"));
  Serial.println(F("======================================"));
  Serial.println(F("[STATE 0] Waiting for Uplink (0xAA) from Motor Controller..."));
}

void loop() {
  // 1. LISTEN FOR UPLINK FROM MOTOR CONTROLLER
  if (Serial1.available() > 0) {
    byte incoming = Serial1.read();
    if (incoming == 0xAA && current_state == 0) {
      Serial.println(F("\n[UPLINK RECEIVED] Manual Launch Triggered!"));
      Serial.println(F("[ACTION] Transitioning to State 2 (Parachute Descent)"));
      current_state = 2; 
      state_timer = millis();
    }
  }

  // 2. SIMULATE THE MISSION TIMELINE
  // After 4 seconds in free-fall, simulate impact and go to Winch Recovery
  if (current_state == 2 && (millis() - state_timer > 4000)) {
    Serial.println(F("\n[SIMULATION] Impact detected. Transitioning to State 5 (Winch Recovery)"));
    current_state = 5;
    state_timer = millis();
  }
  
  // After 6 seconds of winching, reset back to standby
  if (current_state == 5 && (millis() - state_timer > 6000)) {
    Serial.println(F("\n[SIMULATION] Recovery complete. Resetting to State 0."));
    current_state = 0;
    Serial.println(F("[STATE 0] Waiting for Uplink (0xAA) from Motor Controller..."));
  }

  // 3. DOWNLINK HEARTBEAT (10Hz)
  if (millis() - last_tx_time > 100) {
    Serial1.write(current_state);
    last_tx_time = millis();
  }
}