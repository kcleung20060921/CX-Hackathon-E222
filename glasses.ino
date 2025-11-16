#include <driver/i2s.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

HardwareSerial ThermalSerial(1);

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "abcdef12-3456-7890-abcd-ef1234567890"
#define COMMAND_UUID        "87654321-4321-4321-4321-210987654321"

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLECharacteristic *pCommandCharacteristic;
bool deviceConnected = false;
String receivedCommand = "";
bool commandReceived = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✅ BLE CLIENT CONNECTED ✅");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("❌ BLE CLIENT DISCONNECTED ❌");
      BLEDevice::startAdvertising();
      Serial.println("🔄 BLE Advertising restarted");
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();
      size_t len = pCharacteristic->getLength();
      if (len > 0) {
        receivedCommand = "";
        for (int i = 0; i < len; i++) {
          receivedCommand += (char)data[i];
        }
        commandReceived = true;
        Serial.println("📨 Command received: " + receivedCommand);
      }
    }
};

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 7
#define OLED_SCL 6

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define I2S_OUT_NUM         I2S_NUM_0
#define I2S_OUT_BCK_PIN     13
#define I2S_OUT_WS_PIN      12
#define I2S_OUT_DATA_PIN    4
#define I2S_OUT_SD_PIN      5

float temperatures[768];
float sensorTemp = 0.0;

uint8_t frameBuffer[1600];
int frameIndex = 0;
bool seekingHeader = true;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 5000;
bool dataReady = false;
bool modeRunning = false;
int currentMode = 0;

const float TEMP_THRESHOLD = 5.0;
const int MIN_ANOMALY_PIXELS = 3;
float roomTempBaseline = 0.0;
bool baselineEstablished = false;

const float MODE1_ALERT_THRESHOLD = 40.0;
const float MODE2_OVERHEAT_THRESHOLD = 40.0;
bool audioAlertPlayed = false;

struct TempAnomaly {
  int pixelCount;
  float avgTemp;
  float minTemp;
  float maxTemp;
  int minRow, maxRow;
  int minCol, maxCol;
};

// PayAttention bitmap for Mode 1
const unsigned char epd_bitmap_PayAttention [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf8, 0xce, 0x63, 0xb4, 0x9e, 0x3c, 0x61, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf0, 0x6e, 0x49, 0x84, 0x0c, 0x19, 0x23, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf7, 0x26, 0x9d, 0x9c, 0xed, 0xcf, 0xb7, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf7, 0x24, 0x81, 0xbc, 0xec, 0x08, 0x37, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf7, 0x35, 0x9f, 0xbc, 0xed, 0xf3, 0xb7, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf2, 0x71, 0xc9, 0xbc, 0xec, 0x99, 0x33, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf8, 0xfb, 0xe3, 0xbc, 0xee, 0x38, 0xb9, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// ============================================================
// SETUP AND LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  Serial.println("ESP32 Thermal Camera + BLE Control - Mode 1 & Mode 2");
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  
  Wire.begin(OLED_SDA, OLED_SCL);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("❌ OLED initialization failed!");
  } else {
    Serial.println("✅ OLED Initialized");
    display.clearDisplay();
    display.display();
  }
  
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Mount Failed!");
  } else {
    Serial.println("✅ LittleFS Mounted");
    listFiles();
  }
  
  Serial.println("🎵 Initializing I2S Output (Audio)...");
  initI2SOutput();
  pinMode(I2S_OUT_SD_PIN, OUTPUT);
  digitalWrite(I2S_OUT_SD_PIN, HIGH);
  
  Serial.println("📱 Initializing BLE...");
  initBLE();
  
  Serial.println("\n✅ System Ready! Waiting for commands (say 'training' or 'repair')...\n");
}

void loop() {
  if (commandReceived) {
    commandReceived = false;
    
    if (receivedCommand.equalsIgnoreCase("mode1")) {
      Serial.println("\n🚀 MODE1 COMMAND RECEIVED - STARTING THERMAL MONITORING!\n");
      currentMode = 1;
      modeRunning = true;
      initializeMode1();
    } else if (receivedCommand.equalsIgnoreCase("mode2")) {
      Serial.println("\n🚀 MODE2 COMMAND RECEIVED - STARTING OVERHEAT DETECTION!\n");
      currentMode = 2;
      modeRunning = true;
      initializeMode2();
    }
  }
  
  if (modeRunning) {
    if (currentMode == 1) {
      runMode1();
    } else if (currentMode == 2) {
      runMode2();
    }
  } else {
    delay(100);
  }
}

// ============================================================
// MODE 1 - COLD TEMPERATURE ALERT
// ============================================================
void initializeMode1() {
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  Serial.println("MODE1: MLX90640 Thermal Camera - Cold Temperature Alert");
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  
  Serial.println("🎵 Playing mode1 activation sound...");
  playWavFile("/mode1activation.wav");
  delay(500);
  
  ThermalSerial.begin(115200, SERIAL_8N1, 3, 2);
  delay(2000);
  while(ThermalSerial.available()) {
    ThermalSerial.read();
  }
  
  lastUpdateTime = millis();
  baselineEstablished = false;
  audioAlertPlayed = false;
  Serial.println("Ready! Establishing room temperature baseline...");
  Serial.println("Alert if temperature drops below: " + String(MODE1_ALERT_THRESHOLD) + "°C\n");
}

void runMode1() {
  readThermalData();
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    if (dataReady) {
      analyzeTemperatureDataMode1();
      dataReady = false;
    }
    lastUpdateTime = currentTime;
  }
  
  delay(10);
}

void analyzeTemperatureDataMode1() {
  Serial.println("\n═══════════════════════════════════════════════════════════════════════════════");
  Serial.println("=== MODE1: COLD TEMPERATURE MONITORING - " + getTimestamp() + " ===");
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  
  float minTemp = temperatures[0];
  float maxTemp = temperatures[0];
  float sumTemp = 0;
  int validCount = 0;
  
  for (int i = 0; i < 768; i++) {
    if (temperatures[i] > -100 && temperatures[i] < 500) {
      validCount++;
      if (temperatures[i] < minTemp) minTemp = temperatures[i];
      if (temperatures[i] > maxTemp) maxTemp = temperatures[i];
      sumTemp += temperatures[i];
    }
  }
  
  if (validCount == 0) {
    Serial.println("❌ No valid temperature data available!");
    return;
  }
  
  float avgTemp = sumTemp / validCount;
  
  if (!baselineEstablished) {
    roomTempBaseline = avgTemp;
    baselineEstablished = true;
    Serial.println("🏠 ROOM TEMPERATURE BASELINE ESTABLISHED: " + String(roomTempBaseline, 2) + "°C");
  }
  
  Serial.println("📊 TEMPERATURE STATISTICS:");
  Serial.print("   Valid Pixels: ");
  Serial.print(validCount);
  Serial.println("/768");
  Serial.print("   Current Average: ");
  Serial.print(avgTemp, 2);
  Serial.println(" °C");
  Serial.print("   Range: ");
  Serial.print(minTemp, 2);
  Serial.print(" °C to ");
  Serial.print(maxTemp, 2);
  Serial.println(" °C");
  Serial.print("   Sensor Temp: ");
  Serial.print(sensorTemp, 2);
  Serial.println(" °C");
  
  if (avgTemp < MODE1_ALERT_THRESHOLD) {
    Serial.println("\n⚠️  TEMPERATURE ALERT! Temperature too cold!");
    Serial.print("   Current: ");
    Serial.print(avgTemp, 2);
    Serial.print(" °C < Threshold: ");
    Serial.print(MODE1_ALERT_THRESHOLD, 1);
    Serial.println(" °C");
    
    displayPayAttentionImage();
    playWavFile("/foodtoocold.wav");
    audioAlertPlayed = true;
    delay(2000);
    clearOLEDDisplay();
    Serial.println("   Continuing temperature monitoring...");
  } else if (audioAlertPlayed && avgTemp >= MODE1_ALERT_THRESHOLD) {
    Serial.println("\n✅ Temperature back to normal range");
    clearOLEDDisplay();
    audioAlertPlayed = false;
  }
  
  Serial.println("═══════════════════════════════════════════════════════════════════════════════\n");
}

// ============================================================
// MODE 2 - OVERHEAT DETECTION
// ============================================================
void initializeMode2() {
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  Serial.println("MODE2: MLX90640 Thermal Camera - Overheat Detection");
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  
  Serial.println("🎵 Playing mode2 activation sound...");
  playWavFile("/mode2activation.wav");
  delay(500);
  
  ThermalSerial.begin(115200, SERIAL_8N1, 3, 2);
  delay(2000);
  while(ThermalSerial.available()) {
    ThermalSerial.read();
  }
  
  lastUpdateTime = millis();
  baselineEstablished = false;
  audioAlertPlayed = false;
  Serial.println("Ready! Establishing room temperature baseline...");
  Serial.println("Alert if hot object temperature exceeds: " + String(MODE2_OVERHEAT_THRESHOLD) + "°C\n");
}

void runMode2() {
  readThermalData();
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    if (dataReady) {
      analyzeTemperatureDataMode2();
      dataReady = false;
    }
    lastUpdateTime = currentTime;
  }
  
  delay(10);
}

void analyzeTemperatureDataMode2() {
  Serial.println("\n═══════════════════════════════════════════════════════════════════════════════");
  Serial.println("=== MODE2: OVERHEAT DETECTION - " + getTimestamp() + " ===");
  Serial.println("═══════════════════════════════════════════════════════════════════════════════");
  
  float minTemp = temperatures[0];
  float maxTemp = temperatures[0];
  float sumTemp = 0;
  int validCount = 0;
  
  for (int i = 0; i < 768; i++) {
    if (temperatures[i] > -100 && temperatures[i] < 500) {
      validCount++;
      if (temperatures[i] < minTemp) minTemp = temperatures[i];
      if (temperatures[i] > maxTemp) maxTemp = temperatures[i];
      sumTemp += temperatures[i];
    }
  }
  
  if (validCount == 0) {
    Serial.println("❌ No valid temperature data available!");
    return;
  }
  
  float avgTemp = sumTemp / validCount;
  
  if (!baselineEstablished) {
    roomTempBaseline = avgTemp;
    baselineEstablished = true;
    Serial.println("🏠 ROOM TEMPERATURE BASELINE ESTABLISHED: " + String(roomTempBaseline, 2) + "°C");
  } else {
    roomTempBaseline = roomTempBaseline * 0.95 + avgTemp * 0.05;
  }
  
  Serial.println("📊 TEMPERATURE STATISTICS:");
  Serial.print("   Valid Pixels: ");
  Serial.print(validCount);
  Serial.println("/768");
  Serial.print("   Room Baseline: ");
  Serial.print(roomTempBaseline, 2);
  Serial.println(" °C");
  Serial.print("   Current Average: ");
  Serial.print(avgTemp, 2);
  Serial.println(" °C");
  Serial.print("   Range: ");
  Serial.print(minTemp, 2);
  Serial.print(" °C to ");
  Serial.print(maxTemp, 2);
  Serial.println(" °C");
  Serial.print("   Sensor Temp: ");
  Serial.print(sensorTemp, 2);
  Serial.println(" °C");
  
  TempAnomaly anomaly = detectAbnormalTemperature();
  
  if (anomaly.pixelCount >= MIN_ANOMALY_PIXELS) {
    Serial.print("\n🔥 Object detected with temperature: ");
    Serial.print(anomaly.avgTemp, 2);
    Serial.println(" °C");
    Serial.print("   Pixel count: ");
    Serial.println(anomaly.pixelCount);
    
    if (anomaly.avgTemp > MODE2_OVERHEAT_THRESHOLD) {
      Serial.println("\n🚨 OVERHEAT ALERT TRIGGERED! 🚨");
      Serial.print("   Abnormal average temp: ");
      Serial.print(anomaly.avgTemp, 2);
      Serial.print(" °C > Threshold: ");
      Serial.print(MODE2_OVERHEAT_THRESHOLD, 1);
      Serial.println(" °C");
      
      displayPayAttentionImage();
      playWavFile("/overheat.wav");
      audioAlertPlayed = true;
      delay(2000);
      clearOLEDDisplay();
      Serial.println("   Alert complete. Returning to monitoring...");
    }
  } else {
    Serial.println("\n✅ No object detected");
    audioAlertPlayed = false;
  }
  
  Serial.println("═══════════════════════════════════════════════════════════════════════════════\n");
}

// ============================================================
// I2S INITIALIZATION
// ============================================================
void initI2SOutput() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_OUT_BCK_PIN,
    .ws_io_num = I2S_OUT_WS_PIN,
    .data_out_num = I2S_OUT_DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  i2s_driver_install(I2S_OUT_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_OUT_NUM, &pin_config);
  Serial.println("✅ I2S Output initialized");
}

// ============================================================
// BLE INITIALIZATION
// ============================================================
void initBLE() {
  BLEDevice::init("ESP32-Thermal-Audio");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCommandCharacteristic = pService->createCharacteristic(
                      COMMAND_UUID,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_READ
                    );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  
  BLEDevice::startAdvertising();
  
  Serial.println("✅ BLE Ready - Waiting for connection");
  Serial.println("Device: ESP32-Thermal-Audio");
}

// ============================================================
// AUDIO FILE PLAYBACK
// ============================================================
void playWavFile(const char* filename) {
  Serial.print("🔊 Playing: ");
  Serial.println(filename);
  
  File audioFile = LittleFS.open(filename, "r");
  if (!audioFile) {
    Serial.println("❌ Failed to open audio file!");
    return;
  }
  
  uint8_t header[44];
  audioFile.read(header, 44);
  
  if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F') {
    Serial.println("❌ Invalid WAV file!");
    audioFile.close();
    return;
  }
  
  uint16_t numChannels = header[22] | (header[23] << 8);
  uint16_t bitsPerSample = header[34] | (header[35] << 8);
  
  const int bufferSize = 512;
  uint8_t buffer[bufferSize];
  size_t bytesWritten;
  
  digitalWrite(I2S_OUT_SD_PIN, HIGH);
  delay(10);
  
  if (numChannels == 1) {
    while (audioFile.available()) {
      int bytesRead = audioFile.read(buffer, bufferSize / 2);
      if (bytesRead > 0) {
        uint8_t stereoBuffer[bufferSize];
        if (bitsPerSample == 16) {
          for (int i = 0; i < bytesRead / 2; i++) {
            int monoIndex = i * 2;
            int stereoIndex = i * 4;
            stereoBuffer[stereoIndex] = buffer[monoIndex];
            stereoBuffer[stereoIndex + 1] = buffer[monoIndex + 1];
            stereoBuffer[stereoIndex + 2] = buffer[monoIndex];
            stereoBuffer[stereoIndex + 3] = buffer[monoIndex + 1];
          }
          i2s_write(I2S_OUT_NUM, stereoBuffer, bytesRead * 2, &bytesWritten, portMAX_DELAY);
        }
      }
    }
  } else {
    while (audioFile.available()) {
      int bytesRead = audioFile.read(buffer, bufferSize);
      if (bytesRead > 0) {
        i2s_write(I2S_OUT_NUM, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
      }
    }
  }
  
  i2s_zero_dma_buffer(I2S_OUT_NUM);
  delay(100);
  audioFile.close();
  Serial.println("✅ Audio playback complete");
}

// ============================================================
// THERMAL CAMERA FUNCTIONS
// ============================================================
void listFiles() {
  Serial.println("\n📁 Files in LittleFS:");
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("  - ");
    Serial.print(file.name());
    Serial.print(" (");
    Serial.print(file.size());
    Serial.println(" bytes)");
    file = root.openNextFile();
  }
  Serial.println();
}

void displayPayAttentionImage() {
  Serial.println("📺 Displaying Alert image on OLED...");
  display.clearDisplay();
  display.drawBitmap(0, 0, epd_bitmap_PayAttention, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  Serial.println("✅ Image displayed on OLED");
}

void clearOLEDDisplay() {
  display.clearDisplay();
  display.display();
}

void readThermalData() {
  while (ThermalSerial.available()) {
    uint8_t byteRead = ThermalSerial.read();
    
    if (seekingHeader) {
      if (frameIndex == 0 && byteRead == 0x5A) {
        frameBuffer[frameIndex++] = byteRead;
      } else if (frameIndex == 1 && byteRead == 0x5A) {
        frameBuffer[frameIndex++] = byteRead;
        seekingHeader = false;
      } else {
        frameIndex = 0;
      }
    } else {
      frameBuffer[frameIndex++] = byteRead;
      
      if (frameIndex >= 1543) {
        processTemperatureFrame();
        seekingHeader = true;
        frameIndex = 0;
        dataReady = true;
        return;
      }
      
      if (frameIndex >= sizeof(frameBuffer)) {
        seekingHeader = true;
        frameIndex = 0;
      }
    }
  }
}

void processTemperatureFrame() {
  int tempCount = 0;
  for (int i = 4; i + 1 < frameIndex && tempCount < 768; i += 2) {
    uint16_t rawTemp = (frameBuffer[i] << 8) | frameBuffer[i + 1];
    temperatures[tempCount++] = ((int16_t)rawTemp) / 100.0;
  }
  
  if (frameIndex >= 6) {
    int sensorPos = frameIndex - 3;
    if (sensorPos + 1 < frameIndex) {
      uint16_t rawSensorTemp = (frameBuffer[sensorPos] << 8) | frameBuffer[sensorPos + 1];
      sensorTemp = ((int16_t)rawSensorTemp) / 100.0;
    }
  }
}

TempAnomaly detectAbnormalTemperature() {
  TempAnomaly anomaly = {0, 0.0, 0.0, 0.0, 24, -1, 32, -1};
  
  float tempSum = 0;
  int anomalyCount = 0;
  float tempThreshold = roomTempBaseline + TEMP_THRESHOLD;
  
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      int index = row * 32 + col;
      float temp = temperatures[index];
      
      if (temp > tempThreshold && temp > -100 && temp < 500) {
        anomalyCount++;
        tempSum += temp;
        
        if (anomaly.minRow > row) anomaly.minRow = row;
        if (anomaly.maxRow < row) anomaly.maxRow = row;
        if (anomaly.minCol > col) anomaly.minCol = col;
        if (anomaly.maxCol < col) anomaly.maxCol = col;
        
        if (anomalyCount == 1) {
          anomaly.minTemp = temp;
          anomaly.maxTemp = temp;
        } else {
          if (temp < anomaly.minTemp) anomaly.minTemp = temp;
          if (temp > anomaly.maxTemp) anomaly.maxTemp = temp;
        }
      }
    }
  }
  
  anomaly.pixelCount = anomalyCount;
  if (anomalyCount > 0) {
    anomaly.avgTemp = tempSum / anomalyCount;
  }
  
  return anomaly;
}

String getTimestamp() {
  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds %= 60;
  minutes %= 60;
  
  String timestamp = "";
  if (hours < 10) timestamp += "0";
  timestamp += String(hours) + ":";
  if (minutes < 10) timestamp += "0";
  timestamp += String(minutes) + ":";
  if (seconds < 10) timestamp += "0";
  timestamp += String(seconds);
  
  return timestamp;
}