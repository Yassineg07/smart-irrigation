#!/usr/bin/env node

/**
 * Setup script for Smart Irrigation Dashboard
 * This script helps configure the dashboard with your specific settings
 */

const fs = require('fs');
const path = require('path');
const readline = require('readline');

const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

console.log('🌱 Smart Irrigation Dashboard Setup');
console.log('=====================================\n');

function askQuestion(question) {
    return new Promise((resolve) => {
        rl.question(question, (answer) => {
            resolve(answer);
        });
    });
}

async function setup() {
    try {
        console.log('Please provide your network configuration:\n');
        
        const mqttBrokerIP = await askQuestion('Enter your MQTT broker IP address (e.g., 192.168.1.100): ');
        const mqttPort = await askQuestion('Enter MQTT broker port (default: 1883): ') || '1883';
        const webPort = await askQuestion('Enter web dashboard port (default: 3000): ') || '3000';
        
        // Read the current server.js file
        const serverPath = path.join(__dirname, 'server.js');
        let serverContent = fs.readFileSync(serverPath, 'utf8');
        
        // Replace placeholder values
        serverContent = serverContent.replace(/YOUR_MQTT_BROKER_IP/g, mqttBrokerIP);
        serverContent = serverContent.replace(/YOUR_LOCAL_IP/g, mqttBrokerIP);
        serverContent = serverContent.replace(/const port = 3000;/g, `const port = ${webPort};`);
        
        // Write the updated file
        fs.writeFileSync(serverPath, serverContent);
        
        console.log('\n✅ Dashboard configuration updated successfully!');
        console.log(`📊 Dashboard will be available at: http://${mqttBrokerIP}:${webPort}`);
        console.log(`🔌 MQTT broker: ${mqttBrokerIP}:${mqttPort}`);
        console.log('\nTo start the dashboard, run: npm start');
        console.log('Or double-click start_dashboard.bat (Windows)\n');
        
    } catch (error) {
        console.error('❌ Setup failed:', error.message);
    } finally {
        rl.close();
    }
}

if (require.main === module) {
    setup();
}
