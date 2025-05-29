# Project Setup and Installation

This document provides detailed instructions for setting up and installing the Smart Agriculture System.

## Hardware Connections

### DHT11 Temperature & Humidity Sensor
- Connect the VCC pin to 3.3V
- Connect the GND pin to ground
- Connect the DATA pin to the designated GPIO pin (PA10 in the default configuration)

### ESP8266 WiFi Module
- Connect VCC to 3.3V
- Connect GND to ground
- Connect TX to RX of USART2 (PA3)
- Connect RX to TX of USART2 (PA2)
- Connect CH_PD (or EN) to 3.3V

### SIM800L GSM Module
- Connect VCC to regulated 4.0-4.2V (important: do not connect directly to 5V)
- Connect GND to ground
- Connect TX to RX of UART4 (PA1)
- Connect RX to TX of UART4 (PA0)

### Motor Driver
- Connect IN1 to PB10
- Connect IN2 to PB11
- Connect motor power supply according to your specific motor driver requirements

## Firmware Setup

1. Clone this repository
2. Open the project in STM32CubeIDE
3. Configure your WiFi credentials, ThingSpeak API key, and phone number in `main.c`
4. Compile and flash to your STM32F407VG board

## ThingSpeak Configuration

1. Create a free ThingSpeak account at [thingspeak.com](https://thingspeak.com/)
2. Create a new channel with the following fields:
   - Field 1: Temperature (°C)
   - Field 2: Humidity (%)
3. Copy your Write API Key and replace the placeholder in `main.c`

## Testing and Verification

After setup, you should:
1. See serial output on UART4 (115200 baud) with sensor readings and status updates
2. Observe data appearing in your ThingSpeak channel
3. Receive SMS alerts when temperature or humidity exceeds thresholds
4. See the motor activate when humidity falls below the threshold

## Troubleshooting

Common issues and solutions:

1. **No WiFi connection**:
   - Check power supply to ESP8266
   - Verify UART connections
   - Confirm WiFi credentials are correct
   
2. **No data uploads to ThingSpeak**:
   - Verify internet connectivity
   - Check API key correctness
   - Ensure proper ThingSpeak channel configuration
   
3. **No SMS alerts**:
   - Verify SIM card is active and has credit
   - Check power supply to SIM800L
   - Confirm network registration status

4. **Motor not activating**:
   - Check connections to motor driver
   - Verify power supply to motor
   - Confirm threshold settings in `main.c`
