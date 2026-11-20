#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>

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

bool newData = false; // Flag to indicate if new data has been received

// --- TAMBAHAN: Variabel Lampu LED ---
int statusEjection = 0; // 0 = Mati, 1 = Menyala

// function to handle incoming data
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // --- PERBAIKAN: Typo mempcpy menjadi memcpy ---
    memcpy(&dataRocket, data, sizeof(dataRocket)); 
    newData = true; 
}

void setup(){
    Serial.begin(115200);
    Serial.println("Receiver Dummy Aktif...");

    // --- TAMBAHAN: Setup ESP-NOW agar tombol Eject tidak error ---
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

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

void loop(){
  // --- MULAI BLOK SIMULASI DATA ---
  static unsigned long lastSimTime = 0;
  if (millis() - lastSimTime > 200) { 
    dataRocket.altitude = random(0, 1000); 
    dataRocket.temperature = random(250, 500) / 10.0; 
    dataRocket.pressure = 1.0; 
    dataRocket.orientationRoll = random(-180, 180);
    dataRocket.orientationPitch = random(-180, 180);
    dataRocket.orientationYaw = random(-180, 180);
    dataRocket.velocityZ = random(-50, 50);
    dataRocket.gyroX = random(-10, 10);
    dataRocket.gyroY = random(-10, 10);
    dataRocket.gyroZ = random(-10, 10);
    
    newData = true; 
    lastSimTime = millis();
  }
  // --- AKHIR BLOK SIMULASI DATA ---

  // Kode untuk print ke Serial Studio
  if(newData) {
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
    Serial.print(dataRocket.gyroZ, 2); Serial.print(","); // Tambahan koma
    
    // --- TAMBAHAN: Print Data ke-11 ---
    Serial.print(statusEjection); 
    
    Serial.print("*/");
    Serial.println();

    newData = false; 
  } 

  // Fitur Tombol Eject Manual
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 
    if(command.equalsIgnoreCase("eject")) {
      incomingCmd.ejectorCommand = true;
      esp_now_send(senderAddress, (uint8_t *)&incomingCmd, sizeof(incomingCmd));
      
      // --- TAMBAHAN: Nyalakan lampu LED di Serial Studio ---
      statusEjection = 1; 
    }
  }
}