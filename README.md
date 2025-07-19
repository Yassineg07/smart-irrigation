# Smart Irrigation System

IoT-based automatic irrigation system using STM32F407, ESP8266 WiFi, and environmental sensors with web dashboard control.

![alt text](<Screenshot 1.png>)
![alt text](<Screenshot 2.png>)

## Features

- **Automatic Irrigation**: Activates watering when soil humidity ≤ 50%, stops when ≥ 80%
- **Manual Override**: Remote control via web dashboard
- **Real-time Monitoring**: Temperature/humidity updates every 2 seconds
- **SMS Alerts**: Temperature threshold notifications (33°C) with 5-minute cooldown
- **Web Dashboard**: Live data and controls accessible from any device
- **Dual Mode Operation**: Seamless AUTO/MANUAL mode switching

## Hardware Components

| Component | Purpose | Connection Notes |
|-----------|---------|------------------|
| **STM32F407VGT6** | Main controller | STM32F407G-DISC1 board |
| **ESP8266** | WiFi connectivity | UART2 (PA2/PA3) at 115200 baud |
| **DHT11** | Temperature/humidity sensor | PA8 with voltage divider |
| **SIM800L** | GSM/SMS module | UART4 (PA0/PA1) at 9600 baud |
| **Water Pump** | Irrigation actuator | PA15 via L298N H-bridge |
| **LED Indicator** | Visual status | PD15 (onboard LED) |

## Circuit Connections

### DHT11 Sensor (with Voltage Protection)
```
DHT11 VCC  → STM32 5V (or external battery)
DHT11 GND  → STM32 GND
DHT11 Data → Voltage Divider → STM32 PA8

Voltage Divider (5V → 3.3V):
5V ----[10kΩ]----+-----> STM32 PA8 + DHT11 Data
                 |
              [20kΩ]
                 |
               GND
```

### ESP8266 WiFi Module
```
ESP8266 VCC → 3.3V
ESP8266 GND → GND
ESP8266 TX  → STM32 PA3 (UART2_RX)
ESP8266 RX  → STM32 PA2 (UART2_TX)
```

### SIM800L GSM Module
```
SIM800L VCC → 4V Battery (separate power supply)
SIM800L GND → Common GND
SIM800L TX  → Voltage Divider → STM32 PA1 (UART4_RX)
SIM800L RX  → STM32 PA0 (UART4_TX)

TX Voltage Divider (5V → 3.3V):
SIM800L TX ----[10kΩ]----+-----> STM32 PA1
                         |
                      [20kΩ]
                         |
                       GND
```

### Motor Control

**Demo Setup (using onboard LED for visual indication):**
- STM32 PD15 → Onboard Blue LED (visual indicator)

**Production Setup (real water pump with L298N H-Bridge):**
```
Power Supply:
12V/24V PSU (+) → L298N VCC
12V/24V PSU (-) → L298N GND

Control Connections:
STM32 PA15 → L298N IN1 (Motor control signal)
STM32 GND → L298N GND (Common ground)
STM32 5V  → L298N +5V (Logic power, if needed)

Motor Connections:
L298N OUT1 → Water Pump Motor (+)
L298N OUT2 → Water Pump Motor (-)

L298N Configuration:
- Connect ENA to 5V (already done with jumper) for full speed (or to PWM pin for speed control)
- Leave IN2 disconnected or tied to GND
- IN1 controls motor ON/OFF via STM32 PA15
```

### USB OTG (Optional)
- For debugging and monitoring system status via USB CDC

## Software Setup

### 1. Configure Network Settings

**Required Configuration Changes in `Core/Src/main.c`:**

Edit the following defines (located around **lines 40-66**):

```c
/* WiFi Configuration - Lines 40-41 */
#define WIFI_SSID                   "Your_WiFi_Network"      // Replace with your WiFi name
#define WIFI_PASSWORD               "Your_WiFi_Password"     // Replace with your WiFi password

/* MQTT Configuration - Lines 44-46 */
#define MQTT_BROKER_IP              "192.168.1.100"         // Replace with your MQTT broker IP
#define MQTT_BROKER_PORT            1883                     // Default MQTT port (usually no change needed)
#define MQTT_CLIENT_ID              "STM32_Irrigation_System" // Unique client ID (optional to change)

/* SMS Configuration - Lines 66-67 */
#define SMS_PHONE_NUMBER            "+1234567890"            // Replace with your phone number (+country code)
#define SMS_COOLDOWN_TIME           300000                   // 5 minutes (change if needed)
```

**Required Configuration Changes in `websocket/server.js`:**

Edit the following lines (located around **lines 8-25**):

```javascript
/* Line 8: Web server port */
const port = 3000;                                          // Change if port 3000 is occupied

/* Line 17: Network interface log message */
console.log(`- Network: http://YOUR_LOCAL_IP:${port}`);     // Replace YOUR_LOCAL_IP with your computer's IP

/* Line 25: MQTT broker connection */
const mqttClient = mqtt.connect('mqtt://YOUR_MQTT_BROKER_IP'); // Replace YOUR_MQTT_BROKER_IP with broker IP
```

**Configuration Summary:**
- **WiFi Credentials**: Update your network name and password
- **MQTT Broker IP**: Use your computer's local IP address (where Mosquitto runs)
- **Phone Number**: Include country code (e.g., +1 for US, +33 for France, +216 for Tunisia)
- **Dashboard IP**: Must match MQTT broker IP for proper communication

### 2. MQTT Broker Setup
Install Mosquitto MQTT broker:

**Windows:**
- Download from [mosquitto.org](https://mosquitto.org/download/)
- Install and run as service

**Ubuntu/Linux:**
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

**Docker:**
```bash
docker run -d -p 1883:1883 --name mosquitto eclipse-mosquitto
```

### 3. Web Dashboard Setup

> [!IMPORTANT]
> **Windows users:** Double-click `start_dashboard.bat` for easy setup!

**Other Setup Methods:**

**Option A: Automated Setup (Recommended)**
```bash
cd websocket
npm install
npm run setup    # Automated configuration script
npm start
```

**Option B: Manual Setup**
```bash
cd websocket
npm install
# Edit server.js and replace YOUR_MQTT_BROKER_IP with your broker IP
npm start
```

### 4. STM32 Programming
1. Open project in STM32CubeIDE
2. Update network configuration in main.c
3. Build and flash to STM32F407
4. Monitor via USB CDC (optional)

## System Operation

### Automatic Mode Logic
- **Motor ON**: When soil humidity ≤ 50% (dry soil needs watering)
- **Motor OFF**: When soil humidity ≥ 80% (soil sufficiently wet)
- **Hysteresis**: Prevents rapid on/off cycling between 50-80%

### Manual Mode
- Direct motor control via web dashboard
- Overrides automatic humidity-based control
- Instant response to user commands

### SMS Alerts
- Triggered when temperature ≥ 33°C
- 5-minute cooldown between alerts
- Includes system status in message

### Web Dashboard Access
Open browser and navigate to: `http://[MQTT_BROKER_IP]:3000`

## MQTT Communication

| Topic | Direction | Data Format |
|-------|-----------|-------------|
| `irrigation/sensor_data` | STM32 → Dashboard | `T25.5H67.2` |
| `irrigation/motor_status` | STM32 → Dashboard | `ON` / `OFF` |
| `irrigation/mode_status` | STM32 → Dashboard | `AUTO` / `MANUAL` |
| `irrigation/system_status` | STM32 → Dashboard | `T25.5H67.2MOFFDAUTO` |
| `irrigation/mode` | Dashboard → STM32 | `AUTO` / `MANUAL` |
| `irrigation/motor` | Dashboard → STM32 | `ON` / `OFF` |
| `irrigation/status_request` | Dashboard → STM32 | `REQUEST` |
| `irrigation/sms_alert` | STM32 → Dashboard | `SENT:Alert message` |

## Custom Drivers

This project features **custom-made, general-purpose drivers** developed to be completely independent from the main application code. These drivers are designed for reusability and can be easily integrated into any STM32 project without modifications:

- **esp8266.h/c**: Complete ESP8266 WiFi and MQTT communication driver
- **dht11.h/c**: DHT11 temperature/humidity sensor interface driver  
- **motor.h/c**: Generic motor control abstraction driver
- **sim800l.h/c**: Full SIM800L GSM/SMS functionality driver

**Key Features:**
- ✅ **Project Independent**: No dependency on main application code
- ✅ **Plug & Play**: Simply include headers and initialize
- ✅ **Well Documented**: Clear function definitions and usage examples
- ✅ **Modular Design**: Each driver handles its own hardware abstraction
- ✅ **Reusable**: Perfect for other STM32 projects requiring these peripherals

These drivers can be copied to any STM32 project and used immediately without code modifications, making them valuable building blocks for IoT and embedded applications.

## Customization

### SMS Cooldown Timer Duration

**STM32 Configuration (main.c):**
```c
/* SMS Configuration - Line ~67 */
#define SMS_COOLDOWN_TIME           300000  // 5 minutes in milliseconds

// Examples:
#define SMS_COOLDOWN_TIME           60000   // 1 minute
#define SMS_COOLDOWN_TIME           180000  // 3 minutes  
#define SMS_COOLDOWN_TIME           600000  // 10 minutes
```

**Dashboard Timer (index.html):**
```javascript
// Line ~701 - SMS sent handler
this.startCooldownTimer(300);  // 300 seconds = 5 minutes

// Examples:
this.startCooldownTimer(60);   // 1 minute
this.startCooldownTimer(180);  // 3 minutes
this.startCooldownTimer(600);  // 10 minutes
```

### Temperature Alert Threshold

**STM32 Temperature Threshold:**
```c
/* SMS Temperature Threshold - Line ~59 */
#define TEMPERATURE_THRESHOLD       33.0f   // Alert when ≥ 33°C

// Examples:
#define TEMPERATURE_THRESHOLD       30.0f   // Alert at 30°C
#define TEMPERATURE_THRESHOLD       35.0f   // Alert at 35°C
#define TEMPERATURE_THRESHOLD       40.0f   // Alert at 40°C
```

**Dashboard Red Color Alert:**
```javascript
// Line ~513 - Temperature threshold for red color
this.temperatureThreshold = 33.0;  // Should match STM32 threshold

// Line ~602 - Temperature alert styling condition
if (data.temperature >= 33.0 || data.alertActive) {
    temperatureElement.classList.add('temperature-alert');
}

// Examples: Change both values to match your threshold
this.temperatureThreshold = 30.0;
if (data.temperature >= 30.0 || data.alertActive) {
```

### Humidity Control Thresholds

**Irrigation Control Points:**
```c
/* Humidity Thresholds - Lines ~60-61 */
#define HUMIDITY_THRESHOLD_LOW      70.0f  // Turn motor ON when humidity ≤ 70%
#define HUMIDITY_THRESHOLD_HIGH     80.0f  // Turn motor OFF when humidity ≥ 80%

// Examples:
// More frequent watering (60-75%):
#define HUMIDITY_THRESHOLD_LOW      60.0f
#define HUMIDITY_THRESHOLD_HIGH     75.0f

// Less frequent watering (40-90%):  
#define HUMIDITY_THRESHOLD_LOW      40.0f
#define HUMIDITY_THRESHOLD_HIGH     90.0f
```

**Note:** Always maintain 10-20% gap between LOW and HIGH thresholds to prevent rapid motor cycling.

## Troubleshooting

### Common Issues
**WiFi Connection Failed:**
- Check ESP8266 3.3V power supply
- Verify WiFi credentials
- Ensure strong signal strength

**No Sensor Readings:**
- Verify DHT11 connections and voltage divider
- Check 5V power supply to sensor
- Allow 2-second sensor warm-up time

**MQTT Not Connecting:**
- Verify broker IP address and port (1883)
- Check firewall settings
- Ensure broker is running

**SMS Not Sending:**
- Check SIM800L 4V power supply
- Verify SIM card and network registration
- Confirm phone number format (+country code)

### Debug Monitoring
- Use USB CDC for real-time system logs
- Monitor MQTT traffic with MQTT Explorer
- Check browser console for dashboard errors

## Future Enhancements

Planned upgrades and features:
- Motor speed control panel
- Configurable humidity/temperature thresholds
- Energy consumption monitoring
- Irrigation scheduling system
- Multiple sensor zones
- Data logging and analytics
- Mobile app integration

## Technical Specifications

- **MCU**: STM32F407VGT6 (168MHz ARM Cortex-M4F)
- **WiFi**: ESP8266 (802.11 b/g/n, 2.4GHz)
- **Sensor**: DHT11 (0-50°C, 20-90% RH, ±2°C/±5% accuracy)
- **Communication**: UART, USB CDC, WiFi, MQTT, GSM
- **Update Rate**: 2-second sensor intervals
- **Power**: 5V (STM32), 3.3V (ESP8266), 4V (SIM800L)

## License

MIT [LICENSE](LICENSE) - Open source project for educational and commercial use.

## Support

For issues or questions:
- Check troubleshooting section above
- Review debug output via USB CDC
- Create issue on GitHub repository
or contact me [Yassineg07](mailto:gharbiyasine040@gmail.com).


