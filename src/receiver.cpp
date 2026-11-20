#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t senderAddress[] = {0x20,0x9B,0xA9,0x61,0xAA,0x14}; // Sender MAC Address

// Structure to receive telemetry data from the rocket
typedef struct {
  float altitude;
  float temperature;
  float pressure;
  float orientationRoll;
  float orientationPitch;
  float orientationYaw;
  float velocityZ; 
  float gyroX;
  float gyroY;
  float gyroZ;
} Struct_message;

Struct_message dataRocket;

// Structure to send ejector command to the rocket
typedef struct {
  bool ejectorCommand; 
} Command_message;

Command_message incomingCmd;
esp_now_peer_info_t peerInfo;

bool newData = false; // Flag to indicate new incoming data

// Rocket flight states
enum flightState {
  FlightStanby = 0,
  Apogee = 1,
  EjectionCharge = 2
};
flightState rocketState = FlightStanby;

// Variables for apogee detection, auto-ejection, and manual override
float maxAltitude = 0.0;
bool isEjectTriggered = false;   // Locks state once Ejection (manual or auto) fires
bool apogeeDetected = false;     // Marks when Apogee is first reached
unsigned long apogeeTime = 0;    // Timestamp when Apogee occurred

// Helper function to fire ejector via ESP-NOW and lock state
void triggerEjection() {
  if (!isEjectTriggered) {
    incomingCmd.ejectorCommand = true;
    esp_now_send(senderAddress, (uint8_t *)&incomingCmd, sizeof(incomingCmd));
    isEjectTriggered = true;
  }
  rocketState = EjectionCharge; // Force state to EjectionCharge (2)
}

// Callback function executed when data is received via ESP-NOW
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&dataRocket, data, sizeof(dataRocket)); 
  newData = true; 
}

void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Starting...");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback function
  esp_now_register_recv_cb(onDataReceived);

  // Register sender as a peer
  memcpy(peerInfo.peer_addr, senderAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Check for manual commands from Serial Studio (Fast & Non-Blocking)
  static String rxBuffer = "";
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) {
        if (rxBuffer.equalsIgnoreCase("eject")) {
          // Manual override: fire immediately regardless of current state
          triggerEjection();
        } 
        else if (rxBuffer.equalsIgnoreCase("reset")) {
          // Reset all states, timers, and baseline altitude
          isEjectTriggered = false;
          apogeeDetected = false;
          apogeeTime = 0;
          maxAltitude = dataRocket.altitude;
          rocketState = FlightStanby;
        }
      }
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }

  //  Process real telemetry data and forward to Serial Studio
  if (newData) {
    // Run automatic state detection only if ejection has not been triggered yet
    if (!isEjectTriggered) {
      // Track peak altitude
      if (dataRocket.altitude > maxAltitude) {
        maxAltitude = dataRocket.altitude;
      }

      // Detect Apogee (climbed > 5m, then dropped >= 2m OR descending < -1 m/s)
      if (!apogeeDetected) {
        if ((maxAltitude > 5.0 && (maxAltitude - dataRocket.altitude) >= 2.0) || 
            (maxAltitude > 5.0 && dataRocket.velocityZ < -1.0)) {
          apogeeDetected = true;
          apogeeTime = millis(); // Record exact time Apogee occurred
          rocketState = Apogee;
        } else {
          rocketState = FlightStanby;
        }
      } 
      // Auto-trigger Ejection Charge 1.5 seconds (1500 ms) after Apogee
      else {
        if (millis() - apogeeTime >= 1500) {
          triggerEjection();
        }
      }
    }

    // Print 11 CSV values formatted for Serial Studio
    Serial.print("/*");
    Serial.print(dataRocket.altitude, 2); Serial.print(",");
    Serial.print(dataRocket.temperature, 2); Serial.print(",");
    Serial.print(dataRocket.pressure, 4); Serial.print(","); 
    Serial.print(dataRocket.orientationRoll, 2); Serial.print(",");
    Serial.print(dataRocket.orientationPitch, 2); Serial.print(",");
    Serial.print(dataRocket.orientationYaw, 2); Serial.print(",");
    Serial.print(dataRocket.velocityZ, 2); Serial.print(",");
    Serial.print(dataRocket.gyroX, 2); Serial.print(",");
    Serial.print(dataRocket.gyroY, 2); Serial.print(",");
    Serial.print(dataRocket.gyroZ, 2); Serial.print(",");
    Serial.print(rocketState); // Prints 0, 1, or 2
    Serial.println("*/");

    newData = false; 
  } 
}