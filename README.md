# Smart Agriculture System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![STM32](https://img.shields.io/badge/STM32-F407VG-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407vg.html)
[![IoT](https://img.shields.io/badge/IoT-ThingSpeak-brightgreen.svg)](https://thingspeak.com)

An IoT-based smart agriculture system built on the STM32F407VG microcontroller. This project automates agricultural monitoring and irrigation control through sensor integration, wireless connectivity, and remote alerts.

## Features

- **Real-time Environmental Monitoring**: Temperature and humidity sensing with DHT11 sensors
- **Automated Irrigation Control**: Soil moisture-based water pump activation
- **Cloud Data Synchronization**: Regular uploads to ThingSpeak IoT platform
- **Mobile Alerts**: SMS notifications via GSM when environmental thresholds are exceeded
- **Low Power Operation**: Optimized for extended field deployment

## System Architecture

The system consists of an STM32F407VG microcontroller interfaced with sensors, communication modules, and actuators to create a complete agricultural monitoring and control solution.

### Hardware Components

- STM32F407VG Microcontroller
- DHT11 Temperature & Humidity Sensor
- ESP8266 WiFi Module
- SIM800L GSM Module
- DC Water Pump with Motor Driver
- Power Supply System

### Connectivity

- **WiFi**: Real-time data transmission to ThingSpeak cloud
- **GSM**: SMS alerts for critical conditions and remote system status

## Getting Started

### Prerequisites

- STM32CubeIDE or compatible IDE
- STM32 ST-LINK utility
- Active ThingSpeak account
- Active SIM card for the SIM800L module

### Hardware Setup

1. Connect DHT11 sensor to the designated GPIO pin
2. Connect ESP8266 module to USART2 (TX/RX)
3. Connect SIM800L module to UART4 (TX/RX)
4. Connect motor driver input pins to the designated GPIO pins
5. Ensure proper power supply to all components

### Configuration

1. Update WiFi credentials in `main.c`:
   ```c
   #define WIFI_SSID      "Your_SSID"
   #define WIFI_PASSWORD  "Your_Password"
   ```

2. Set your ThingSpeak API key:
   ```c
   #define THINGSPEAK_API_KEY  "Your_ThingSpeak_API_Key"
   ```

3. Configure the SMS alert recipient:
   ```c
   #define PHONE_NUMBER  "+1234567890"
   ```

4. Adjust thresholds according to your requirements:
   ```c
   #define TEMP_THRESHOLD      30.0  // °C
   #define HUM_THRESHOLD       80.0  // %
   #define MOTOR_ON_THRESHOLD  40.0  // % (for soil moisture)
   ```

### Building and Flashing

1. Open the project in STM32CubeIDE
2. Build the project using the hammer icon
3. Connect your STM32 board via ST-LINK
4. Flash the program using the "Run" button

## System Operation

The system operates in a continuous cycle:

1. **Sensing Phase**: Reads temperature and humidity data from DHT11 sensor
2. **Data Processing**: Analyzes sensor data against defined thresholds
3. **Communication**: Uploads data to ThingSpeak cloud at defined intervals
4. **Decision Making**: Controls irrigation pump based on humidity levels
5. **Alerting**: Sends SMS notifications when conditions exceed safe thresholds

## Future Enhancements

- Solar power integration for off-grid operation
- Soil pH sensing capabilities
- Mobile application for remote monitoring and control
- Machine learning for predictive irrigation scheduling
- Multiple sensor node support for larger field coverage

## Detailed Setup Instructions

For detailed hardware connections, firmware configuration, and troubleshooting tips, please see the [SETUP.md](SETUP.md) file.

## Contributing

Contributions to improve the Smart Agriculture System are welcome. Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- STMicroelectronics for the HAL libraries
- ThingSpeak for the IoT platform services
- All contributors to the open-source components used in this project