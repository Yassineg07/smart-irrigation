const express = require('express');
const WebSocket = require('ws');
const mqtt = require('mqtt');
const path = require('path');

// Initialize Express for serving HTML
const app = express();
const port = 3000;

// Serve static files from 'public' folder
app.use(express.static(path.join(__dirname, 'public')));

// Start HTTP server
const server = app.listen(port, '0.0.0.0', () => {
    console.log(`Smart Irrigation Dashboard running at:`);
    console.log(`- Local: http://localhost:${port}`);
    console.log(`- Network: http://YOUR_LOCAL_IP:${port}`);
    console.log(`STM32 handles SMS alerts automatically, Dashboard shows notifications`);
    console.log(`Temperature threshold: 33°C (handled by STM32)`);
    console.log(`SMS cooldown: 5 minutes (handled by STM32)`);
    console.log(`Access from any device on your WiFi network!`);
});

// Initialize WebSocket server
const wss = new WebSocket.Server({ server });

// Connect to MQTT Broker
const mqttClient = mqtt.connect('mqtt://YOUR_MQTT_BROKER_IP'); 

// Store connected WebSocket clients
const clients = new Set();

// Store system state
let systemState = {
    temperature: 0,
    humidity: 0,
    motorState: 'OFF',
    mode: 'AUTO',
    lastUpdate: new Date().toISOString()
};

// WebSocket Connection Handler
wss.on('connection', (ws) => {
    console.log('New WebSocket client connected');
    clients.add(ws);

    // Send initial connection message with current system state
    ws.send(JSON.stringify({
        type: 'connection',
        status: 'connected',
        data: systemState
    }));

    // Request current status from STM32
    mqttClient.publish('irrigation/status_request', 'get_status');
    console.log('Requested current system status from STM32');

    // Handle incoming messages from browser
    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            
            if (data.type === 'mode_control') {
                // Forward mode command to MQTT
                mqttClient.publish('irrigation/mode', data.mode);
                console.log(`Mode command: ${data.mode}`);
            } else if (data.type === 'motor_control') {
                // Forward motor command to MQTT (only works in manual mode)
                mqttClient.publish('irrigation/motor', data.state);
                console.log(`Motor command: ${data.state}`);
            }
        } catch (error) {
            console.error('Invalid WebSocket message:', error);
        }
    });

    // Handle client disconnection
    ws.on('close', () => {
        clients.delete(ws);
        console.log('WebSocket client disconnected');
    });
});

// MQTT Message Handler (for receiving STM32 updates)
mqttClient.on('connect', () => {
    console.log('Connected to MQTT broker');
    
    // Subscribe to all irrigation topics
    mqttClient.subscribe('irrigation/sensor_data');
    mqttClient.subscribe('irrigation/motor_status');
    mqttClient.subscribe('irrigation/mode_status');
    mqttClient.subscribe('irrigation/system_status');
    mqttClient.subscribe('irrigation/sms_alert');
    
    console.log('Subscribed to MQTT topics for irrigation system');
});

mqttClient.on('message', (topic, message) => {
    const messageStr = message.toString();
    console.log(`MQTT message received - Topic: ${topic}, Message: ${messageStr}`);
    
    try {
        if (topic === 'irrigation/sensor_data') {
            // Parse sensor data: "T25.5H60.2" (ultra-simple format)
            const tempMatch = messageStr.match(/T([\d.]+)/);
            const humMatch = messageStr.match(/H([\d.]+)/);
            
            if (tempMatch && humMatch) {
                systemState.temperature = parseFloat(tempMatch[1]);
                systemState.humidity = parseFloat(humMatch[1]);
                systemState.lastUpdate = new Date().toISOString();
                
                // Broadcast to all WebSocket clients
                broadcastToClients({
                    type: 'sensor_data',
                    data: {
                        temperature: systemState.temperature,
                        humidity: systemState.humidity,
                        lastUpdate: systemState.lastUpdate
                    }
                });
            }
            
        } else if (topic === 'irrigation/motor_status') {
            // Update motor state
            systemState.motorState = messageStr;
            
            broadcastToClients({
                type: 'motor_status',
                state: systemState.motorState
            });
            
        } else if (topic === 'irrigation/mode_status') {
            // Update mode state
            systemState.mode = messageStr;
            
            broadcastToClients({
                type: 'mode_status',
                mode: systemState.mode
            });
            
        } else if (topic === 'irrigation/system_status') {
            // Full system status update: "T25.5H60.2MONDDAUTO" 
            const tempMatch = messageStr.match(/T([\d.]+)/);
            const humMatch = messageStr.match(/H([\d.]+)/);
            const motorMatch = messageStr.match(/M([A-Z]+)/);
            const modeMatch = messageStr.match(/D([A-Z]+)/);
            
            if (tempMatch) systemState.temperature = parseFloat(tempMatch[1]);
            if (humMatch) systemState.humidity = parseFloat(humMatch[1]);
            if (motorMatch) systemState.motorState = motorMatch[1];
            if (modeMatch) systemState.mode = modeMatch[1];
            
            systemState.lastUpdate = new Date().toISOString();
            
            broadcastToClients({
                type: 'system_status',
                data: systemState
            });
        } else if (topic === 'irrigation/sms_alert') {
            // Handle SMS alert notifications from STM32
            // Formats: "SENDING:message", "SENT:message", "FAILED:message", "COOLDOWN:message", etc.
            const parts = messageStr.split(':');
            if (parts.length >= 2) {
                const status = parts[0].toLowerCase();
                const message = parts.slice(1).join(':');
                
                console.log(`SMS Alert from STM32: ${status.toUpperCase()} - ${message}`);
                
                broadcastToClients({
                    type: 'sms_alert',
                    status: status,
                    message: message,
                    timestamp: new Date().toISOString()
                });
            }
        }
        
    } catch (error) {
        console.error('Error parsing MQTT message:', error);
    }
});

// Function to broadcast to all connected WebSocket clients
function broadcastToClients(data) {
    clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(data));
        }
    });
}

// Handle MQTT connection errors
mqttClient.on('error', (error) => {
    console.error('MQTT connection error:', error);
});

// Handle WebSocket server errors
wss.on('error', (error) => {
    console.error('WebSocket server error:', error);
});
