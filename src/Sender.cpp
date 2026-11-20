#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t receiverAddress[] = {0x68, 0x09, 0x47, 0x48, 0x7C, 0x00}; // Receiver MAC Address
const int ejectPin = 25; // Pin for the ejector mechanism

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

typedef struct {
  bool ejectorCommand; 
} Command_message;

Struct_message dataRocket;
Command_message incomingCmd;
esp_now_peer_info_t peerInfo;

Adafruit_BMP280 bmp; // I2C

const int MPU_addr = 0x68; // I2C address of the MPU-6050

int16_t AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ; // Raw sensor variables

// Variables for velocity, orientation, and environment
float velocityZ = 0.0;
float orientationRoll = 0.0, orientationPitch = 0.0, orientationYaw = 0.0;
float altitude = 0.0, pressure = 0.0, temperature = 0.0;
float baseAltitude = 0.0;

// Calibration offset variables (Fixed typo on accelOffsetZ)
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

// Function prototypes
void setupMPU6050();
void readMPU6050();
void calibrateSensors();
void onCommandReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

unsigned long timeNow = 0;
bool triggerEject = false;
unsigned long timeDelay=0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Rocket Sender...");
  
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Set ejector pin as output and ensure it starts LOW
  pinMode(ejectPin, OUTPUT);
  digitalWrite(ejectPin, LOW); 

  // Initialize I2C communication 
  Wire.begin(21, 22); 
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("BMP280 sensor found!");
  }

  setupMPU6050();
  Serial.println("MPU6050 sensor found!");
  Serial.println("Keep the rocket upright and completely still during calibration...");
  
  // Run full auto-calibration for Gyro, Accel, and Base Altitude
  calibrateSensors();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent); 

  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(onCommandReceived); 
}

void loop() {
  if (millis() - timeNow >= 500) { // Update every 500 ms (dt = 0.5s)
    timeNow = millis();
    
    // Read BMP280 sensor
    altitude = bmp.readAltitude(1013.25) - baseAltitude;
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 101325.0; // Convert Pa to atm

    // Read & apply calibration offsets to MPU6050
    readMPU6050();
    float gyroX_calibrated = GyroX - gyroOffsetX;
    float gyroY_calibrated = GyroY - gyroOffsetY;
    float gyroZ_calibrated = GyroZ - gyroOffsetZ;

    float accelX_calibrated = AccelX - accelOffsetX;
    float accelY_calibrated = AccelY - accelOffsetY;
    float accelZ_calibrated = AccelZ - accelOffsetZ;
    
    // Convert calibrated raw accelerometer data to g's (±16g range -> 2048 LSB/g)
    float accelX_g = accelX_calibrated / 2048.0;  
    float accelY_g = accelY_calibrated / 2048.0;  
    float accelZ_g = accelZ_calibrated / 2048.0;  

    // Convert raw gyroscope data to degrees per second (±250 dps range -> 131 LSB/dps)
    float gyroZ_dps = gyroZ_calibrated / 131.0;

    // Calculate orientation angles
    orientationRoll = atan2(accelY_g, accelZ_g) * 180.0 / PI;
    orientationPitch = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * 180.0 / PI;
    orientationYaw += gyroZ_dps * 0.5; 

    // Calculate net vertical acceleration & integrate for Velocity Z
    // Subtract 1.0g for gravity (assumes Z-axis points straight UP toward nose cone)
    float accelZ_ms2 = (accelZ_g - 1.0) * 9.80665; 

    // Deadband filter to prevent drift when stationary
    if (abs(accelZ_ms2) < 0.25) {
      accelZ_ms2 = 0.0;
    }

    velocityZ += accelZ_ms2 * 0.5; // dt = 0.5s

    // Print debug telemetry to Serial Monitor
    Serial.println("---- Data Sensor Rocket ----");
    Serial.print("BMP280  | Alt: "); Serial.print(altitude); Serial.print(" m | Temp: ");
    Serial.print(temperature); Serial.print(" C | Press: "); Serial.print(pressure, 4); Serial.println(" atm");

    Serial.print("MPU6050 | Roll: "); Serial.print(orientationRoll); Serial.print(" | Pitch: ");
    Serial.print(orientationPitch); Serial.print(" | Yaw: "); Serial.print(orientationYaw);
    Serial.print(" | VelZ: "); Serial.print(velocityZ); Serial.println(" m/s");

    // Package and send telemetry via ESP-NOW
    dataRocket.altitude = altitude;
    dataRocket.temperature = temperature;   
    dataRocket.pressure = pressure;
    dataRocket.orientationRoll = orientationRoll;
    dataRocket.orientationPitch = orientationPitch;
    dataRocket.orientationYaw = orientationYaw;
    dataRocket.velocityZ = velocityZ;
    dataRocket.gyroX = gyroX_calibrated;
    dataRocket.gyroY = gyroY_calibrated;
    dataRocket.gyroZ = gyroZ_calibrated;

    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &dataRocket, sizeof(dataRocket));
    if (result != ESP_OK) {
      Serial.println("Error sending telemetry data");
    }
  }

  // Execute Ejector Charge when command is received from Ground Station
  if (triggerEject) {
    Serial.println("Ejector Active! Firing for 2 seconds...");
    digitalWrite(ejectPin, HIGH);
     if(millis()-timeDelay>50000){
       digitalWrite(ejectPin, LOW);
       Serial.println("Ejector OFF");
       triggerEject = false;
       incomingCmd.ejectorCommand = false;
    }
  }
}

void setupMPU6050() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);

  // Set accelerometer range to ±16g (Sensitivity: 2048 LSB/g)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // ACCEL_CONFIG register 
  Wire.write(0x18);  // ±16g
  Wire.endTransmission(true);
}

void calibrateSensors() {
  const int calibrationSamples = 300;
  int32_t gyroXTotal = 0, gyroYTotal = 0, gyroZTotal = 0;
  int32_t accelXTotal = 0, accelYTotal = 0, accelZTotal = 0;
  float altTotal = 0.0;

  delay(500); // Wait 0.5s for hand vibrations to settle

  for (int sample = 0; sample < calibrationSamples; sample++) {
    readMPU6050();
    gyroXTotal += GyroX;
    gyroYTotal += GyroY;
    gyroZTotal += GyroZ;

    accelXTotal += AccelX;
    accelYTotal += AccelY;
    accelZTotal += AccelZ;

    altTotal += bmp.readAltitude(1013.25);
    delay(5); // Total calibration time ~1.5 seconds
  }

  // Calculate average Gyro offsets (target = 0)
  gyroOffsetX = static_cast<float>(gyroXTotal) / calibrationSamples;
  gyroOffsetY = static_cast<float>(gyroYTotal) / calibrationSamples;
  gyroOffsetZ = static_cast<float>(gyroZTotal) / calibrationSamples;

  // Calculate average Accel offsets
  //  2048.0 equals 1g at ±16g sensitivity. Move '- 2048.0' if X or Y points UP instead of Z.
  accelOffsetX = static_cast<float>(accelXTotal) / calibrationSamples;
  accelOffsetY = static_cast<float>(accelYTotal) / calibrationSamples;
  accelOffsetZ = (static_cast<float>(accelZTotal) / calibrationSamples) - 2048.0;

  // Set initial ground altitude as 0.0m reference, accounting for 0.86m payload height above ground
  baseAltitude = (altTotal / calibrationSamples)-0.86;

  Serial.println("Calibration complete!");
  Serial.print("Base Altitude: "); Serial.print(baseAltitude); Serial.println(" m");
}

void readMPU6050() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starting register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AccelX = Wire.read() << 8 | Wire.read();
  AccelY = Wire.read() << 8 | Wire.read();
  AccelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature registers
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
}

void onCommandReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&incomingCmd, data, sizeof(incomingCmd));
  if (incomingCmd.ejectorCommand == true) {
    triggerEject = true;
    timeDelay=millis();
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}