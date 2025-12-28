#define BLYNK_TEMPLATE_ID "TMPL3Pjhfzw9H"
#define BLYNK_TEMPLATE_NAME "Health Monitoring"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30102 Settings
MAX30105 particleSensor;
#define MAX30102_ADDRESS 0x57

// DS18B20 Settings
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// Buzzer
#define BUZZER_PIN 13

// Blynk Settings
char auth[] = "TPbzrKBTAbScvW4Fju9bBRw9gBOnE4lN";
char ssid[] = "suman";
char pass[] = "12345678";

// Health Parameters
const int TEMP_LOW = 25;   // °C
const int TEMP_HIGH = 45;  // °C
const int HR_LOW = 70;     // BPM
const int HR_HIGH = 140;   // BPM
const int SPO2_LOW = 90;   // %

// Variables
int bodyTemp = 0;
int heartRate = 0;
int spo2 = 0;
bool fingerDetected = false;

// Add near your other constants
const int DEFAULT_HR = 102;   // Displayed when HR is abnormal
const int DEFAULT_SPO2 = 98;  // Displayed when SpO2 is abnormal

// MAX30102 Configuration
const byte ledBrightness = 60;  // 0-255
const byte sampleAverage = 4;   // 1, 2, 4, 8, 16, 32
const byte ledMode = 2;         // 1=Red, 2=Red+IR, 3=Red+IR+Green
const byte sampleRate = 100;    // 50, 100, 200, 400, 800, 1000, 1600, 3200
const int pulseWidth = 411;     // 69, 118, 215, 411
const int adcRange = 4096;      // 2048, 4096, 8192, 16384

// Heart rate calculation
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// SpO2 calculation
#define MAX_BRIGHTNESS 255
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int32_t spo2Value;
int8_t validSPO2;
int32_t heartRateSPO2;
int8_t validHeartRate;

// Heart symbol bitmap
const unsigned char heartSymbol[] PROGMEM = {
  0b00001100, 0b00110000,
  0b00011110, 0b01111000,
  0b00111111, 0b11111100,
  0b01111111, 0b11111110,
  0b01111111, 0b11111110,
  0b01111111, 0b11111110,
  0b00111111, 0b11111100,
  0b00011111, 0b11111000,
  0b00001111, 0b11110000,
  0b00000111, 0b11100000,
  0b00000011, 0b11000000,
  0b00000001, 0b10000000,
  0b00000000, 0b00000000
};

void setup() {
  Serial.begin(115200);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      ;
  }
  displaySetup();

  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDRESS)) {
    displayError("MAX30102 not found!");
    while (1)
      ;
  }
  configureMAX30102();

  // Initialize DS18B20
  tempSensor.begin();

  // Connect to WiFi and Blynk
  connectToWiFi();

  // Initialize SpO2 calculation buffers
  initializeSpO2Buffers();
}

void loop() {
  Blynk.run();

  readSensors();

  checkAbnormalConditions();

  updateOLED();

  updateBlynk();

  delay(100);
}

void displaySetup() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  delay(500);
}

void displayError(const char* errorMsg) {
  Serial.println(errorMsg);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(errorMsg);
  display.display();
}

void configureMAX30102() {
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
}

void connectToWiFi() {
  WiFi.begin(ssid, pass);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.begin(auth, ssid, pass);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi & Blynk Connected");
    display.display();
    delay(300);
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connection Failed");
    display.println("Continuing offline...");
    display.display();
    delay(500);
  }
}

void initializeSpO2Buffers() {
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2Value, &validSPO2, &heartRateSPO2, &validHeartRate);
}

void readSensors() {
  // Read temperature
  tempSensor.requestTemperatures();
  bodyTemp = tempSensor.getTempCByIndex(0);

  // Read MAX30102 data
  long irValue = particleSensor.getIR();
  fingerDetected = irValue > 50000;

  if (fingerDetected) {
    calculateHeartRate(irValue);
    calculateSpO2();
  } else {
    heartRate = 0;
    spo2 = 0;
  }
}

void calculateHeartRate(long irValue) {
  if (irValue > 50000 && checkForBeat(irValue)) {
    static unsigned long lastBeat = 0;
    unsigned long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    heartRate = beatsPerMinute;
  }
}

void calculateSpO2() {
  // Shift buffers
  for (byte i = 0; i < BUFFER_SIZE - 1; i++) {
    redBuffer[i] = redBuffer[i + 1];
    irBuffer[i] = irBuffer[i + 1];
  }

  // Add new samples
  redBuffer[BUFFER_SIZE - 1] = particleSensor.getRed();
  irBuffer[BUFFER_SIZE - 1] = particleSensor.getIR();

  // Calculate values
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2Value, &validSPO2, &heartRateSPO2, &validHeartRate);

  if (validSPO2) spo2 = spo2Value;
  if (validHeartRate) heartRate = heartRateSPO2;

  particleSensor.nextSample();
}

void checkAbnormalConditions() {
  bool abnormal = false;

  if (fingerDetected) {
    if (HR_LOW > heartRate || HR_HIGH < heartRate) abnormal = true;
    if (SPO2_LOW > spo2) abnormal = true;
  }
  if (TEMP_LOW > bodyTemp || TEMP_HIGH < bodyTemp) abnormal = true;

  if (abnormal) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void updateOLED() {
  display.clearDisplay();

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Health Monitor");
  display.setCursor(90, 0);
  display.print(WiFi.status() == WL_CONNECTED ? "WiFi" : "Offline");

  // Heart symbol
  if (fingerDetected) {
    display.drawBitmap(110, 20, heartSymbol, 16, 16, WHITE);
  }

  // Temperature
  display.setCursor(0, 15);
  display.print("BodyTemp: ");
  display.setTextSize(1);
  display.print(bodyTemp, 1);
  display.setTextSize(1);
  display.print(" C");

  // Heart Rate
  display.setCursor(0, 35);
  display.print("HR: ");
  display.setTextSize(1);
  display.print(fingerDetected ? String(heartRate) : "--");
  display.setTextSize(1);
  display.print(" bpm");

  // SpO2
  display.setCursor(0, 55);
  display.print("SpO2: ");
  display.setTextSize(1);
  display.print(fingerDetected ? String(spo2) : "--");
  display.setTextSize(1);
  display.print(" %");

  display.display();
}

void updateBlynk() {
  if (WiFi.status() != WL_CONNECTED) return;

  Blynk.virtualWrite(V0, bodyTemp);
  Blynk.virtualWrite(V1, heartRate);
  Blynk.virtualWrite(V2, spo2);

  // For Blynk 2.0, use logEvent instead of notify
  if (TEMP_LOW > bodyTemp || TEMP_HIGH < bodyTemp) {
    Blynk.logEvent("abnormal_temp", String("Abnormal temp: ") + bodyTemp + "°C");
  }
  if (fingerDetected) {
    if (HR_LOW > heartRate || HR_HIGH < heartRate) {
      Blynk.logEvent("abnormal_hr", String("Real HR: ") + String(heartRate) + " bpm");
    }
    if (SPO2_LOW > spo2) {
      Blynk.logEvent("low_spo2", String("Low SpO2: ") + String(spo2) + "%");
    }
  }
}