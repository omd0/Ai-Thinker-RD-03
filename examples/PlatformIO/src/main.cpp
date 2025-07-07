/*
    Name:       RD-03D PlatformIO Example with STM32-style Data Frame
    Created:    2024
    Author:     PlatformIO Example
    Description: Example for Ai-Thinker RD-03D 24Ghz FMCW radar sensor
                 Using STM32-style data frame structure from blog implementation
*/

#include <Arduino.h>
#include <Ai-Thinker-RD-03.h>

// Function declarations
void processSingleTarget();
void processMultiTarget();
void printStatus();
void demonstrateRadarDataFrame();
void demonstrateCommandFrames();

// RD-03D Radar sensor instance
AiThinker_RD_03D radar;

// Pin definitions
#define RADAR_RX_PIN 26
#define RADAR_TX_PIN 27

// Timing variables
unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;

// STM32-style data frame variables (from blog implementation)
uint8_t RX_BUF[64] = {0};      // 缓存数组 - Buffer array
uint8_t RX_count = 0;          // 计数位 - Count position
uint8_t RX_temp;               // 缓存字符 - Buffer character

uint8_t Radar_1 = 0;           // 目标1标志位 - Target 1 flag
uint8_t Radar_2 = 0;           // 目标2标志位 - Target 2 flag
uint8_t Radar_3 = 0;           // 目标3标志位 - Target 3 flag

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("RD-03D Radar Sensor - PlatformIO Example with STM32 Data Frame");
    Serial.println("================================================================");
    
    // Initialize radar sensor
    if (!radar.begin(Serial1, RADAR_RX_PIN, RADAR_TX_PIN)) {
        Serial.println("Failed to initialize radar sensor!");
        return;
    }
    
    Serial.println("Radar sensor initialized successfully");
    
    // Demonstrate the STM32-style data frame structure
    demonstrateRadarDataFrame();
    
    // Demonstrate command frames
    demonstrateCommandFrames();
    
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

void demonstrateRadarDataFrame() {
    Serial.println("=== STM32-Style Radar Data Frame Demonstration ===");
    
    // Initialize the data frame structure as per blog implementation
    memset(RX_BUF, 0, sizeof(RX_BUF));
    RX_count = 0;
    
    // Reset radar flags
    Radar_1 = Radar_2 = Radar_3 = 0;
    
    Serial.println("Radar data frame initialized:");
    Serial.printf("  RX_BUF size: %d bytes\n", sizeof(RX_BUF));
    Serial.printf("  RX_count: %d\n", RX_count);
    Serial.printf("  Radar flags: R1=%d, R2=%d, R3=%d\n", Radar_1, Radar_2, Radar_3);
    Serial.println();
}

void demonstrateCommandFrames() {
    Serial.println("=== Command Frame Demonstration ===");
    
    // Display Single Target Detection Command from library
    Serial.print("Single Target Detection CMD: ");
    const uint8_t* singleCmd = AiThinker_RD_03D::getSingleTargetCommand();
    for (int i = 0; i < AiThinker_RD_03D::getCommandSize(); i++) {
        Serial.printf("0x%02X ", singleCmd[i]);
    }
    Serial.println();
    
    // Display Multi Target Detection Command from library
    Serial.print("Multi Target Detection CMD:  ");
    const uint8_t* multiCmd = AiThinker_RD_03D::getMultiTargetCommand();
    for (int i = 0; i < AiThinker_RD_03D::getCommandSize(); i++) {
        Serial.printf("0x%02X ", multiCmd[i]);
    }
    Serial.println();
    Serial.println();
}

void processSingleTarget() {
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0) {
        // Update radar flags as per STM32 implementation
        Radar_1 = 1;
        Radar_2 = 0;
        Radar_3 = 0;
        
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
        // Reset radar flags
        Radar_1 = Radar_2 = Radar_3 = 0;
        Serial.println("No target detected");
    }
}

void processMultiTarget() {
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0) {
        // Update radar flags as per STM32 implementation
        Radar_1 = (targetCount >= 1) ? 1 : 0;
        Radar_2 = (targetCount >= 2) ? 1 : 0;
        Radar_3 = (targetCount >= 3) ? 1 : 0;
        
        Serial.printf("Multi-target detected: %d targets (R1=%d, R2=%d, R3=%d)\n", 
                     targetCount, Radar_1, Radar_2, Radar_3);
        
        for (uint8_t i = 0; i < targetCount && i < 3; i++) {
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
        // Reset radar flags
        Radar_1 = Radar_2 = Radar_3 = 0;
        Serial.println("No targets detected");
    }
}

void printStatus() {
    Serial.printf("=== Status Report ===\n");
    Serial.printf("Total frames received: %lu\n", frameCount);
    Serial.printf("Last frame: %lu ms ago\n", millis() - lastFrameTime);
    Serial.printf("Current mode: %d\n", radar.getCurrentMode());
    
    // Print STM32-style radar flags
    Serial.printf("Radar flags: R1=%d, R2=%d, R3=%d\n", Radar_1, Radar_2, Radar_3);
    
    // Print configuration
    uint16_t minDist, maxDist;
    if (radar.getDetectionRange(minDist, maxDist)) {
        Serial.printf("Detection range: %d-%d cm\n", minDist, maxDist);
    }
    Serial.printf("Sensitivity: %d\n", radar.getSensitivity());
    Serial.printf("Output format: %d\n", radar.getOutputFormat());
    Serial.println("=====================");
} 