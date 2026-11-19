#include <Arduino.h>
#include <Wire.h>
#include <AdaFruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t receiverAddress[]={0x68, 0x09, 0x47, 0x48, 0x7C, 0x00};  // Receiver MAC Address
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

Struct_message dataRocket; // Create a variable of type Struct_message to hold the sensor data
Command_message incomingCmd; // Create a variable of type Command_message to hold the incoming command data
esp_now_peer_info_t peerInfo; // Create a variable to hold the peer information

Adafruit_BMP280 bmp; // I2C

const int MPU_addr=0x68; // I2C address of the MPU-6050

int16_t AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ; // Variables raw data

// Variables velocity and orientation
float velocityZ;
float orientationRoll, orientationPitch, orientationYaw;
float altitude, pressure, temperature; // Variables for altitude and pressure
float baseAltitude; // Variable to store the base altitude
float gyroOffsetX, gyroOffsetY, gyroOffsetZ; // Variabel for calibration gyro

// prototype function 
void setupMPU6050();
void readMPU6050();
void calibrateGyro();
void onCommandReceived();
void onDataSent();

unsigned long timeNow=0; // Variable to store the last time data was read

bool triggerEject = false;

void setup() {
    Serial.begin(115200);
    Serial.println("Mulai Testing");
    
    WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Set the ejector pin as output
    pinMode(ejectPin, OUTPUT);
    digitalWrite(ejectPin, LOW); 

    // Initialize I2C communication 
    Wire.begin(21,22); 
    if(!bmp.begin(0x76)) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while(1);
    } else {
        Serial.println("BMP280 sensor found!");
    }
    setupMPU6050(); // Set up Sensor MPU6050
    Serial.println("MPU6050 sensor found!");
    Serial.println("Keep the rocket still while calibrating the gyro...");
    calibrateGyro();
    baseAltitude = 0.86; // height from the ground to the BMP sensor's height from the ground

    // initialize ESP-NOW
    if(esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(onDataSent); 

    memcpy(peerInfo.peer_addr, receiverAddress, 6); // Set the receiver's MAC address
    peerInfo.channel = 0; // Use the current channel
    peerInfo.encrypt = false; // No encryption

    if(esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    esp_now_register_recv_cb(onCommandReceived); 
}

void loop() {
    if(millis() - timeNow >= 500) { // Update every 500 ms
        timeNow = millis();
        // Read data from BMP280 sensor
        altitude = bmp.readAltitude(1011.0)-baseAltitude; // Adjust sea level pressure as needed
        temperature = bmp.readTemperature();
        pressure = bmp.readPressure()/101325.0; // Convert to atm
    
        readMPU6050(); // Read data from MPU6050 sensor
        float gyroX_calibrated = GyroX - gyroOffsetX;
        float gyroY_calibrated = GyroY - gyroOffsetY;
        float gyroZ_calibrated = GyroZ - gyroOffsetZ;
        
        // Convert raw accelerometer data to g's 
        float accelX_g = AccelX / 2048.0;  
        float accelY_g = AccelY / 2048.0;  
        float accelZ_g = AccelZ / 2048.0;  

        float gyroZ_dps = gyroZ_calibrated / 131.0; // convert raw gyroscope data to degrees per second (dps)

        // Calculate roll, pitch, and yaw angles using accelerometer and gyroscope data
        orientationRoll = atan2(accelY_g, accelZ_g) * 180.0 / PI;
        orientationPitch = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * 180.0 / PI;
        orientationYaw += gyroZ_dps * 0.5; 

        float accelZ_ms2 =(accelZ_g-1.0) * 9.80665; // calculate accelaration vertical in m/s^2 (subtracting 1g for gravity)

        if(abs(accelZ_ms2)<0.2) accelZ_ms2=0; // filter out small values to reduce noise

        velocityZ += accelZ_ms2 * 0.5; // integrate acceleration to get velocity (assuming 0.5s interval)

        // Print the raw data to Serial Monitor
        Serial.println("----Data Sensor Rocket---");
        Serial.print("BMP280 |");
        Serial.print(" Altitude: "); Serial.print(altitude); Serial.print(" m |");
        Serial.print(" Temperature: "); Serial.print(temperature); Serial.print(" °C |");   
        Serial.print(" Pressure: "); Serial.print(pressure); Serial.println(" atm |");

        Serial.print("MPU6050 |");
        Serial.print("Roll: "); Serial.print(orientationRoll); Serial.print(" ° |");
        Serial.print("Pitch: "); Serial.print(orientationPitch); Serial.print(" ° |");  
        Serial.print("Yaw: "); Serial.print(orientationYaw); Serial.print(" ° |");  
        Serial.print("Velocity Z: "); Serial.print(velocityZ); Serial.println(" m/s |");

        // Package Data into Struct_message and send via ESP-NOW
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
        if (result == ESP_OK) {
            Serial.println("Sent with success");
        } else {
            Serial.println("Error sending the data");
        }
    }
    if(triggerEject){
        Serial.println("Ejector Active! firing for 2 seconds");
        digitalWrite(ejectPin, HIGH);
        delay(2000); 
        digitalWrite(ejectPin, LOW);
        Serial.println("Ejector OFF");
        triggerEject = false;
        incomingCmd.ejectorCommand = false;
    }
}
void setupMPU6050() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    // Set the accelerometer range to ± 16g 
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x1C);  // ACCEL_CONFIG register 
    Wire.write(0x18);  // set to ± 16g
    Wire.endTransmission(true);
}

void calibrateGyro() {
    const int calibrationSamples = 200;
    int32_t gyroXTotal = 0;
    int32_t gyroYTotal = 0;
    int32_t gyroZTotal = 0;

    for (int sample = 0; sample < calibrationSamples; sample++) {
        readMPU6050();
        gyroXTotal += GyroX;
        gyroYTotal += GyroY;
        gyroZTotal += GyroZ;
        delay(5);
    }

    gyroOffsetX = static_cast<float>(gyroXTotal) / calibrationSamples;
    gyroOffsetY = static_cast<float>(gyroYTotal) / calibrationSamples;
    gyroOffsetZ = static_cast<float>(gyroZTotal) / calibrationSamples;

    Serial.print("Gyro calibration complete (raw offsets): ");
    Serial.print(gyroOffsetX); Serial.print(", ");
    Serial.print(gyroOffsetY); Serial.print(", ");
    Serial.println(gyroOffsetZ);
}

void readMPU6050() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AccelX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AccelY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AccelZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read(); Wire.read();  // Skip MPU6050 temperature registers
    GyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void onCommandReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    memcpy(&incomingCmd, data, sizeof(incomingCmd)); // Copy the received data into the command_message struct
    if (incomingCmd.ejectorCommand==true) {
        triggerEject = true;
    }
}
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }