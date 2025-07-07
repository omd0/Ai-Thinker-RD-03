/*
    Name:       RD-03 PlatformIO Example
    Created:    2024
    Author:     PlatformIO Example
    Description: Example for Ai-Thinker RD-03 24Ghz FMCW radar sensor
*/

#include <Arduino.h>
#include <Ai-Thinker-RD-03.h>

// Function declarations
void processSingleTarget();
void processMultiTarget();
void printStatus();

// RD-03D Radar sensor instance
AiThinker_RD_03D radar;

// Pin definitions
#define RADAR_RX_PIN 26
#define RADAR_TX_PIN 27

// Timing variables
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("RD-03D Radar Sensor - PlatformIO Example");
    Serial.println("=========================================");
    
    // Initialize radar sensor
    if (!radar.begin(Serial1, RADAR_RX_PIN, RADAR_TX_PIN)) {
        Serial.println("Failed to initialize radar sensor!");
        return;
    }
    
    Serial.println("Radar sensor initialized successfully");
    
    // Configure radar parameters
    Serial.println("Configuring radar parameters...");
    
    // Set timeouts
    radar.setFrameTimeout(500);
    radar.setInterCommandDelay(100);
    
    // Get sensor information
    Serial.print("Firmware Version: ");
    Serial.println(radar.getFirmwareVersion());
    
    Serial.print("Serial Number: ");
    Serial.println(radar.getSerialNumber());
    
    // Configure detection parameters
    if (radar.setDetectionRange(20, 800)) {
        Serial.println("Detection range set: 20-800 cm");
    }
    
    if (radar.setSensitivity(128)) {
        Serial.println("Sensitivity set to 128");
    }
    
    if (radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY)) {
        Serial.println("Output format set to binary");
    }
    
    // Set to multi-target mode for demonstration
    if (radar.setMultiTargetMode()) {
        Serial.println("Set to multi-target mode");
    }
    
    // Exit configuration mode
    if (radar.exitConfigMode()) {
        Serial.println("Exited configuration mode");
    }
    
    Serial.println("Setup complete. Starting detection...");
    Serial.println();
}

void loop() {
    // Read radar data
    int bytesRead = radar.read();
    
    if (bytesRead > 0 && radar.frameReady()) {
        frameCount++;
        lastFrameTime = millis();
        
        // Process the received frame
        switch (radar.getFrameType()) {
            case AiThinker_RD_03D::TARGET_DATA:
                processSingleTarget();
                break;
                
            case AiThinker_RD_03D::MULTI_TARGET_DATA:
                processMultiTarget();
                break;
                
            case AiThinker_RD_03D::DEBUG_DATA:
                Serial.println("Debug data received");
                break;
                
            case AiThinker_RD_03D::ACK_FRAME:
                Serial.println("Configuration ACK received");
                break;
                
            default:
                Serial.println("Unknown frame type");
                break;
        }
    }
    
    // Print status every 10 seconds
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 10000) {
        printStatus();
        lastStatusTime = millis();
    }
    
    // Small delay to prevent overwhelming the serial output
    delay(10);
}

void processSingleTarget() {
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0) {
        AiThinker_RD_03D::TargetInfo target;
        if (radar.getTargetInfo(0, target)) {
            Serial.printf("Single Target - ID: %d, Distance: %d cm, Angle: %d°, Velocity: %d cm/s, Energy: %d\n",
                         target.targetId,
                         target.distance,
                         target.angle,
                         target.velocity,
                         target.energy);
        }
    } else {
        Serial.println("No target detected");
    }
}

void processMultiTarget() {
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0) {
        Serial.printf("Multi-target detected: %d targets\n", targetCount);
        
        for (uint8_t i = 0; i < targetCount; i++) {
            AiThinker_RD_03D::TargetInfo target;
            if (radar.getTargetInfo(i, target)) {
                Serial.printf("  Target %d - ID: %d, Distance: %d cm, Angle: %d°, Velocity: %d cm/s, Energy: %d\n",
                             i,
                             target.targetId,
                             target.distance,
                             target.angle,
                             target.velocity,
                             target.energy);
            }
        }
    } else {
        Serial.println("No targets detected");
    }
}

void printStatus() {
    Serial.printf("=== Status Report ===\n");
    Serial.printf("Total frames received: %lu\n", frameCount);
    Serial.printf("Last frame: %lu ms ago\n", millis() - lastFrameTime);
    Serial.printf("Current mode: %d\n", radar.getCurrentMode());
    
    // Print configuration
    uint16_t minDist, maxDist;
    if (radar.getDetectionRange(minDist, maxDist)) {
        Serial.printf("Detection range: %d-%d cm\n", minDist, maxDist);
    }
    Serial.printf("Sensitivity: %d\n", radar.getSensitivity());
    Serial.printf("Output format: %d\n", radar.getOutputFormat());
    Serial.println("=====================");
} 