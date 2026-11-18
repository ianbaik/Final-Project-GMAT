#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>

uint8_t senderAddress[] = {0x20,0x9B,0xA9,0x61,0xAA,0x14}; // Sender MAC Address

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

bool newData=false; // Flag to indicate if new data has been received

//fungction to handle incoming data
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    mempcpy(&dataRocket, data, sizeof(dataRocket)); // Copy the received data into the Struct_message struct
    newData = true; // Set the flag to indicate new data has been received
}

void setup() {
  Serial.begin(115200);
  Serial.println("Receiver Starting...");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function
  esp_now_register_recv_cb(onDataReceived);

  // Add the sender as a peer
  memcpy(peerInfo.peer_addr, senderAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if(newData) {
    // Print the received data to Serial Monitor
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
    Serial.print(dataRocket.gyroZ, 2);
    Serial.println("*/");

    newData = false; 
  } 

  // fiture to command ejector from serial monitor
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 
    if(command.equalsIgnoreCase("eject")) {
      incomingCmd.ejectorCommand = true;
      esp_now_send(senderAddress, (uint8_t *)&incomingCmd, sizeof(incomingCmd));
    }
  }
}

