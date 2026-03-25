<<<<<<< HEAD
LifeSense is an IoT-based health monitoring prototype that reads body temperature, heart rate, and blood oxygen (SpO2) and displays them on an SSD1306 OLED screen, reports to Blynk, and sounds an alert when measurements fall outside configured thresholds.

## Table of contents
...
=======
# LifeSense - Health Monitoring System

## Overview
**LifeSense** is a biomedical IoT-based health monitoring system that continuously tracks vital signs and sends real-time alerts via the Blynk cloud platform. This project monitors body temperature, heart rate (HR), and blood oxygen saturation (SpO2) using advanced sensors and displays data on an OLED screen.

---

## Project Features

✅ **Real-time Vital Sign Monitoring**
- Body Temperature (via DS18B20 sensor)
- Heart Rate (via MAX30102 optical sensor)
- Blood Oxygen Saturation - SpO2 (via MAX30102)

✅ **OLED Display**
- 128x64 pixel display showing live health metrics
- WiFi connectivity status indicator
- Heart symbol animation when finger is detected

✅ **IoT Cloud Integration**
- Blynk platform integration for remote monitoring
- Real-time data synchronization
- Abnormal condition alerts via Blynk events

✅ **Smart Alerting**
- Buzzer alarm for abnormal conditions
- Automatic event logging to Blynk
- Configurable health thresholds

✅ **Offline Capability**
- Continues operation if WiFi connection fails
- Local OLED display updates
- Reconnection attempts on power cycle

---

## Hardware Components

| Component | Model/Specification | Purpose |
|-----------|-------------------|---------|
| Microcontroller | ESP32 | Main processing unit |
| Heart Rate & SpO2 Sensor | MAX30102 | Optical sensor for HR and SpO2 |
| Temperature Sensor | DS18B20 | Digital temperature measurement |
| Display | SSD1306 OLED (128x64) | Real-time data visualization |
| Buzzer | Generic 5V Buzzer | Abnormal condition alert |
| Communication | I2C (Wire protocol) | Sensor-to-MCU communication |

---

## Pin Configuration

```cpp
MAX30102 Address:     0x57 (I2C)
DS18B20 Data Pin:     GPIO 4
Buzzer Pin:           GPIO 13
OLED SDA:             GPIO 21 (default I2C)
OLED SCL:             GPIO 22 (default I2C)
```

---

## Health Thresholds

```cpp
Temperature Range:    25°C - 45°C
Heart Rate Range:     70 - 140 BPM
SpO2 Minimum:         90%
```

**Alert Conditions:**
- Temperature goes below 25°C or above 45°C
- Heart rate goes below 70 or above 140 BPM
- SpO2 drops below 90%
- Any abnormal condition triggers the buzzer

---

## Software Libraries Required

Install these libraries in Arduino IDE:

1. **Blynk Library** - IoT cloud platform integration
2. **Adafruit GFX** - Graphics library for OLED
3. **Adafruit SSD1306** - OLED display driver
4. **WiFi** - ESP32 WiFi connectivity
5. **Wire** - I2C communication protocol
6. **OneWire** - DS18B20 temperature sensor protocol
7. **DallasTemperature** - DS18B20 temperature reading
8. **MAX30105** - MAX30102 sensor library
9. **heartRate.h** - Heart rate calculation algorithm
10. **spo2_algorithm.h** - SpO2 calculation algorithm

---

## Setup & Installation

### 1. Hardware Setup
- Connect MAX30102 to ESP32 via I2C (SDA: GPIO 21, SCL: GPIO 22)
- Connect DS18B20 to GPIO 4 (with 4.7kΩ pull-up resistor)
- Connect SSD1306 OLED to I2C pins
- Connect Buzzer to GPIO 13
- Power all components via 5V supply

### 2. Arduino IDE Configuration
- Select ESP32 Dev Module as board
- Set baud rate to 115200
- Install all required libraries (use Library Manager)

### 3. Blynk Setup
1. Download the Blynk app
2. Create a new template: "Health Monitoring"
3. Template ID: `TMPL3Pjhfzw9H`
4. Generate an Auth Token
5. Update the `auth[]` variable in code

### 4. WiFi Configuration
Update the following in the code:
```cpp
char ssid[] = "YOUR_SSID";        // Your WiFi network name
char pass[] = "YOUR_PASSWORD";    // Your WiFi password
char auth[] = "YOUR_AUTH_TOKEN";  // Blynk authentication token
```

### 5. Upload Code
- Connect ESP32 to computer
- Select COM port
- Click Upload button

---

## Code Structure

### Main Functions

**setup()**
- Initializes OLED display
- Configures MAX30102 sensor
- Initializes DS18B20 temperature sensor
- Connects to WiFi and Blynk
- Populates SpO2 buffers with initial readings

**loop()**
- Runs Blynk connection handler
- Reads all sensors
- Checks for abnormal conditions
- Updates OLED display
- Sends data to Blynk cloud

**readSensors()**
- Reads temperature from DS18B20
- Checks if finger is detected on MAX30102
- Calculates heart rate and SpO2 if finger detected

**calculateHeartRate()**
- Detects heartbeat from IR sensor
- Calculates BPM based on time between beats

**calculateSpO2()**
- Maintains circular buffers for red and IR LED data
- Calls Maxim algorithm for SpO2 calculation
- Updates heart rate from SpO2 algorithm

**checkAbnormalConditions()**
- Compares vital signs with defined thresholds
- Activates buzzer if any value is abnormal

**updateOLED()**
- Clears display and draws fresh data
- Shows temperature, heart rate, SpO2
- Displays WiFi connection status
- Animates heart symbol when finger is detected

**updateBlynk()**
- Sends vital sign data to virtual pins (V0, V1, V2)
- Logs abnormal events to Blynk cloud
- Includes event messages with specific condition details

---

## Blynk Virtual Pins

| Virtual Pin | Data Type | Description |
|------------|-----------|-------------|
| V0 | Float | Body Temperature (°C) |
| V1 | Integer | Heart Rate (BPM) |
| V2 | Integer | SpO2 (%) |

### Blynk Events
- `abnormal_temp` - Triggered when temperature is outside range
- `abnormal_hr` - Triggered when heart rate is abnormal
- `low_spo2` - Triggered when SpO2 drops below 90%

---

## Operating Instructions

### Starting the Device
1. Power on the ESP32
2. Device displays "Initializing..." on OLED
3. Connects to WiFi (shows "Connecting to WiFi...")
4. Once connected: "WiFi & Blynk Connected"
5. Ready for health monitoring

### Using the Device
1. Place your finger on the MAX30102 sensor
2. Device detects finger and starts reading heart rate and SpO2
3. Temperature is continuously monitored
4. OLED shows real-time values
5. Data syncs to Blynk app automatically
6. If any value abnormal, buzzer activates for 200ms

### Offline Mode
- If WiFi fails, device shows "WiFi Connection Failed - Continuing offline..."
- Local OLED display still works with all sensor readings
- Data won't sync to Blynk until reconnection

---

## Sensor Specifications

### MAX30102 Configuration
- **LED Brightness:** 60/255 (configurable)
- **Sample Average:** 4 samples
- **LED Mode:** 2 (Red + IR)
- **Sample Rate:** 100 Hz
- **Pulse Width:** 411 μs
- **ADC Range:** 4096 bits
- **Finger Detection Threshold:** IR value > 50,000

### DS18B20 Temperature Sensor
- Resolution: 0.1°C
- Range: -55°C to +125°C
- Interface: 1-Wire protocol

### SSD1306 OLED Display
- Resolution: 128 x 64 pixels
- Interface: I2C
- Default Address: 0x3C

---

## Data Processing

### Heart Rate Calculation
- Uses interrupt-based beat detection
- Calculates BPM from time between consecutive beats
- Formula: BPM = 60 / (time_between_beats in seconds)
- Validated against MAX30102 internal algorithm

### SpO2 Calculation
- Maintains 100-sample circular buffer (red and IR)
- Uses Maxim heart rate and oxygen saturation algorithm
- Updates after each new sample
- Validated through algorithm flags

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| MAX30102 not found | Check I2C connections, verify address 0x57 |
| SSD1306 not found | Check I2C address (default 0x3C), verify SDA/SCL |
| WiFi connection fails | Verify SSID and password, check signal strength |
| Blynk not syncing | Verify Auth token, check internet connection |
| No heart rate reading | Ensure finger is properly placed on sensor |
| Temperature sensor error | Check OneWire connections, verify GPIO 4 |

---

## Future Enhancements

- 🔄 Add data logging to SD card
- 📊 Create historical data visualization
- 🔔 Add mobile push notifications
- 🩺 Implement multiple user profiles
- 📱 Build dedicated mobile app
- 🔐 Add user authentication
- 💾 Cloud data storage and analytics

---

## License
This project is open-source and available for educational and commercial use.

---

## Author
**Suman-Gayen** - LifeSense Health Monitoring System

---

## Support & Contribution
For issues, suggestions, or contributions, please visit the [GitHub repository](https://github.com/Suman-Gayen/LifeSense).

---

## Safety Disclaimer
⚠️ **Important:** This system is for monitoring purposes only and should not be used as a medical device for diagnosis or treatment. Always consult with healthcare professionals for medical advice.
>>>>>>> 3bd57add115c1b53e3e696759f4056ee4d0db59f
