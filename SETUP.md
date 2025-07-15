# Smart Irrigation System - Quick Setup Guide

## Prerequisites
- STM32CubeIDE installed
- Node.js (v12.0 or higher) installed
- MQTT broker (Mosquitto recommended)
- Hardware components assembled

## Step 1: Configure Network Settings

### Option A: Using Configuration Template
1. Copy `config.h.template` to `Core/Inc/config.h`
2. Edit `config.h` with your settings:
   - WiFi SSID and password
   - MQTT broker IP address
   - Phone number (if using SMS alerts)

### Option B: Direct Configuration
Edit the following defines in `Core/Src/main.c`:
```c
#define WIFI_SSID                   "Your_WiFi_Network"
#define WIFI_PASSWORD               "Your_WiFi_Password"
#define MQTT_BROKER_IP              "192.168.1.100"
#define SMS_PHONE_NUMBER            "+1234567890"
```

## Step 2: Setup MQTT Broker

### Ubuntu/Debian:
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

### Windows:
Download and install from: https://mosquitto.org/download/

### Docker:
```bash
docker run -it -p 1883:1883 eclipse-mosquitto
```

## Step 3: Configure Dashboard

### Automatic Setup:
```bash
cd websocket
npm install
npm run setup
```

### Manual Setup:
1. Edit `server.js` and replace `YOUR_MQTT_BROKER_IP` with your broker IP
2. Save the file

## Step 4: Build and Flash STM32

1. Open project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to STM32F407 (F11)
4. Monitor debug output via USB CDC

## Step 5: Start Dashboard

### Windows:
Double-click `start_dashboard.bat`

### Linux/Mac:
```bash
cd websocket
npm start
```

## Step 6: Access Dashboard

Open your web browser and navigate to:
`http://[YOUR_BROKER_IP]:3000`

## Hardware Connections

| Component | STM32 Pin | Notes |
|-----------|-----------|-------|
| DHT11 Data | PA8 | 3.3V/5V compatible |
| Motor Control | PD15 | Use relay for high current |
| ESP8266 TX | PA3 (UART2_RX) | 3.3V logic level |
| ESP8266 RX | PA2 (UART2_TX) | 3.3V logic level |
| SIM800L TX | PA1 (UART4_RX) | Optional, for SMS |
| SIM800L RX | PA0 (UART4_TX) | Optional, for SMS |

## Troubleshooting

### STM32 Not Connecting to WiFi:
- Check ESP8266 power supply (3.3V)
- Verify WiFi credentials
- Ensure strong WiFi signal

### Dashboard Not Loading:
- Check MQTT broker is running
- Verify IP address configuration
- Check firewall settings

### No Sensor Data:
- Verify DHT11 connections
- Check sensor power supply
- Allow 2-second warm-up time

## Support

For issues, check:
1. Serial debug output from STM32
2. MQTT broker logs
3. Browser developer console
4. Project GitHub issues

## Security Note

Never commit sensitive information like WiFi passwords or phone numbers to version control. Use the `config.h` file (which is in `.gitignore`) for sensitive data.
