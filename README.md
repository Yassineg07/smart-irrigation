# Smart Irrigation System

An intelligent IoT-based irrigation system built with STM32F407 microcontroller, ESP8266 WiFi module, and environmental sensors. The system provides automated watering based on environmental conditions and remote monitoring through a web dashboard.

## Features

- **Automated Irrigation**: Automatically triggers watering when temperature exceeds configurable threshold (default: 30°C)
- **Manual Control**: Remote manual override via responsive web dashboard
- **Real-time Monitoring**: Continuous temperature and humidity sensing with 2-second update intervals
- **Web Dashboard**: Modern, responsive interface with live data visualization
- **MQTT Communication**: Reliable IoT messaging protocol for device communication
- **SMS Alerts**: Optional SMS notifications via GSM module (SIM800L)
- **Multi-mode Operation**: Seamless switching between automatic and manual modes

## Hardware Requirements

- **STM32F407VGT6** microcontroller (main controller)
- **ESP8266** WiFi module (wireless communication)
- **DHT11** temperature and humidity sensor
- **Water pump/motor** (irrigation actuator)
- **SIM800L GSM module** (optional - for SMS alerts)
- **Power supply** (5V/3.3V depending on components)
- **Connecting wires and breadboard/PCB**

## Pin Configuration

| Component | STM32 Pin | Description |
|-----------|-----------|-------------|
| DHT11 Data | PA8 | Temperature/humidity sensor |
| Motor Control | PD15 | Water pump relay control |
| ESP8266 UART | UART2 (PA2/PA3) | WiFi module communication |
| SIM800L UART | UART4 (PA0/PA1) | GSM module communication |

## Software Architecture

### Embedded Firmware (STM32)
- **STM32 HAL Libraries**: Hardware abstraction layer
- **Custom Drivers**: ESP8266, DHT11, Motor, SIM800L
- **MQTT Client**: Bi-directional communication
- **Sensor Management**: Environmental data collection
- **Control Logic**: Automated irrigation decisions

### Web Dashboard (Node.js)
- **Express.js Server**: HTTP server for web interface
- **WebSocket Server**: Real-time bidirectional communication
- **MQTT Client**: Bridge between web interface and STM32
- **Responsive UI**: Mobile-friendly dashboard design

## MQTT Topic Structure

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `irrigation/sensor_data` | STM32 → Dashboard | Environmental sensor readings |
| `irrigation/motor_status` | STM32 → Dashboard | Water pump status updates |
| `irrigation/mode_status` | STM32 → Dashboard | System mode (AUTO/MANUAL) |
| `irrigation/system_status` | STM32 → Dashboard | Complete system state |
| `irrigation/mode` | Dashboard → STM32 | Mode change commands |
| `irrigation/motor` | Dashboard → STM32 | Manual pump control |
| `irrigation/status_request` | Dashboard → STM32 | System status requests |
| `irrigation/sms_alert` | STM32 → Dashboard | SMS notification status |

## Installation and Setup

### 1. Hardware Assembly
1. Connect DHT11 sensor data pin to STM32 PA8
2. Connect motor control relay to STM32 PD15
3. Wire ESP8266 module to STM32 UART2 (PA2/PA3)
4. Connect SIM800L GSM module to STM32 UART4 (PA0/PA1) - Optional
5. Ensure proper power distribution (3.3V for ESP8266, 5V for motor)

### 2. Network Configuration
Create a `config.h` file in `Core/Inc/` with your network settings:
```c
#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID                   "Your_WiFi_Network"
#define WIFI_PASSWORD               "Your_WiFi_Password"

// MQTT Broker Configuration
#define MQTT_BROKER_IP              "192.168.1.100"  // Your MQTT broker IP
#define MQTT_BROKER_PORT            1883
#define MQTT_CLIENT_ID              "STM32_Irrigation_System"

// SMS Configuration (Optional)
#define SMS_PHONE_NUMBER            "+1234567890"  // Your phone number
#define SMS_COOLDOWN_TIME           300000  // 5 minutes in milliseconds

#endif
```

### 3. MQTT Broker Setup
Install and configure an MQTT broker on your network:

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

**Windows:**
Download and install from [Eclipse Mosquitto](https://mosquitto.org/download/)

**Docker (Cross-platform):**
```bash
docker run -it -p 1883:1883 eclipse-mosquitto
```

### 4. Dashboard Setup
1. Navigate to the `websocket` directory
2. Install Node.js dependencies:
   ```bash
   npm install
   ```
3. Update `server.js` with your MQTT broker IP
4. Start the dashboard:
   ```bash
   npm start
   ```
   Or use the provided batch file on Windows: `start_dashboard.bat`

### 5. STM32 Programming
1. Open the project in STM32CubeIDE
2. Update configuration in `main.c` or include your `config.h`
3. Build and flash the firmware to STM32F407
4. Monitor debug output via USB CDC interface

## Usage Guide

### Accessing the Dashboard
1. Ensure your device is connected to the same WiFi network
2. Open a web browser and navigate to `http://[BROKER_IP]:3000`
3. The dashboard will display real-time sensor data and system status

### Operating Modes
- **AUTO Mode**: System automatically irrigates when temperature exceeds threshold
- **MANUAL Mode**: Direct control of irrigation pump via dashboard

### System Monitoring
- Temperature and humidity readings update every 2 seconds
- Motor status shows current pump operation
- System logs provide activity history
- SMS alerts notify of important events (if GSM module is connected)
## Dashboard Features

- 🌡️ **Environmental Monitoring**: Real-time temperature and humidity display
- 💧 **Pump Status**: Visual indicator of water pump operation
- 🎛️ **Control Interface**: Mode switching and manual override controls
- 📊 **System Dashboard**: Current status, logs, and activity history
- 📱 **Responsive Design**: Optimized for desktop, tablet, and mobile devices
- 🔔 **Notification System**: SMS alerts for system events (optional)

## Troubleshooting

### Common Issues

**STM32 Connection Problems:**
- Verify UART connections and baud rate settings (115200 bps)
- Check WiFi credentials and network connectivity
- Ensure MQTT broker is accessible from STM32's network
- Monitor debug messages via USB CDC interface

**Dashboard Access Issues:**
- Confirm Node.js is properly installed (version 12.0 or higher)
- Check if MQTT broker is running and accessible
- Verify firewall settings allow port 3000 and 1883
- Ensure device is on the same network as the STM32

**WiFi Connectivity Problems:**
- Verify ESP8266 power supply (stable 3.3V)
- Check WiFi signal strength at installation location
- Ensure network credentials are correct
- Verify MQTT broker allows client connections

**Sensor Reading Issues:**
- Check DHT11 sensor connections and power supply
- Verify sensor is not damaged or expired
- Ensure adequate sensor warm-up time (2 seconds)
- Check for electromagnetic interference

### Debug Tips
1. Use STM32's USB CDC interface for real-time debugging
2. Monitor MQTT traffic using tools like MQTT Explorer
3. Check browser developer console for JavaScript errors
4. Verify all hardware connections with multimeter

## Technical Specifications

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | STM32F407VGT6 (168MHz ARM Cortex-M4F) |
| **WiFi Module** | ESP8266 (802.11 b/g/n, 2.4GHz) |
| **Sensor** | DHT11 (Temperature: 0-50°C, Humidity: 20-90% RH) |
| **Communication** | UART, USB CDC, WiFi, MQTT |
| **Update Rate** | 2 seconds for sensor readings |
| **Temperature Threshold** | 30°C (user configurable) |
| **Power Supply** | 5V DC (with 3.3V regulation for ESP8266) |
| **Operating Temperature** | -10°C to +70°C |

## Project Structure

```
smart-irrigation/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main application header
│   │   ├── esp8266.h           # ESP8266 WiFi driver
│   │   ├── dht11.h             # DHT11 sensor driver
│   │   ├── motor.h             # Motor control driver
│   │   └── sim800l.h           # SIM800L GSM driver
│   └── Src/
│       ├── main.c              # Main application logic
│       ├── esp8266.c           # ESP8266 implementation
│       ├── dht11.c             # DHT11 sensor implementation
│       ├── motor.c             # Motor control implementation
│       └── sim800l.c           # SIM800L GSM implementation
├── websocket/
│   ├── server.js               # Node.js web server
│   ├── package.json            # Node.js dependencies
│   ├── start_dashboard.bat     # Windows startup script
│   └── public/
│       └── index.html          # Web dashboard interface
├── Drivers/                    # STM32 HAL drivers
├── USB_DEVICE/                 # USB CDC implementation
├── .gitignore                  # Git ignore rules
└── README.md                   # This file
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/new-feature`)
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For questions, issues, or contributions:
- Create an issue on GitHub
- Check the troubleshooting section above
- Review the technical documentation in source files

## Acknowledgments

- STMicroelectronics for STM32 HAL libraries
- ESP8266 community for WiFi module documentation
- Node.js and npm communities for web technologies
- MQTT.org for IoT communication protocol
