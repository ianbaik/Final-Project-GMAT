#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t senderAddress[] = {0x68,0x09,0x47,0x48,0x7C,0x00}; // Sender MAC Address

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

typedef struct {
  bool ejectorCommand; 
} Command_message;

Command_message incomingCmd;
esp_now_peer_info_t peerInfo;

bool newData = false;

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

// Simulation variables (Remove these 2 lines for real flight code)
float simAltitude = 0.0;
bool isGoingUp = true;

// Helper function to fire ejector via ESP-NOW and lock state
void triggerEjection() {
  if (!isEjectTriggered) {
    incomingCmd.ejectorCommand = true;
    esp_now_send(senderAddress, (uint8_t *)&incomingCmd, sizeof(incomingCmd));
    isEjectTriggered = true;
  }
  rocketState = EjectionCharge; // Force state to 2
}

void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&dataRocket, data, sizeof(dataRocket)); 
  newData = true; 
}

void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Ready...");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  memcpy(peerInfo.peer_addr, senderAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // 1. Check commands from Serial Studio (Non-Blocking Manual Override & Reset)
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
          // Reset all states and timers
          isEjectTriggered = false;
          apogeeDetected = false;
          apogeeTime = 0;
          maxAltitude = 0.0;
          simAltitude = 0.0;
          isGoingUp = true;
          rocketState = FlightStanby;
        }
      }
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }

  // 2. Smooth flight simulation (Remove this block when using real Sender data)
  static unsigned long lastSimTime = 0;
  if (millis() - lastSimTime > 200) { 
    if (isGoingUp) {
      simAltitude += 2.5;
      dataRocket.velocityZ = 12.5;
      if (simAltitude >= 50.0) {
        isGoingUp = false;
      }
    } else {
      simAltitude -= 2.0;
      dataRocket.velocityZ = -10.0;
      if (simAltitude <= 0.0) {
        simAltitude = 0.0;
      }
    }

    dataRocket.altitude = simAltitude; 
    dataRocket.temperature = random(250, 350) / 10.0; 
    dataRocket.pressure = 1.0; 
    dataRocket.orientationRoll = random(-30, 30);
    dataRocket.orientationPitch = random(-30, 30);
    dataRocket.orientationYaw = random(-180, 180);
    dataRocket.gyroX = random(-10, 10);
    dataRocket.gyroY = random(-10, 10);
    dataRocket.gyroZ = random(-10, 10);
    
    newData = true; 
    lastSimTime = millis();
  }

  // 3. Automatic Flight State Machine & Telemetry Output
  if (newData) {
    if (!isEjectTriggered) {
      // Track peak altitude
      if (dataRocket.altitude > maxAltitude) {
        maxAltitude = dataRocket.altitude;
      }

      // Step A: Detect Apogee (climbed > 5m, then dropped >= 2m OR descending < -1 m/s)
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
      // Step B: Auto-trigger Ejection Charge 1.5 seconds (1500 ms) after Apogee
      else {
        if (millis() - apogeeTime >= 1500) {
          triggerEjection(); // Automatically sends ESP-NOW command & sets state to 2
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