/*
  RD-03D Cross-Platform Example for PlatformIO
  Demonstrates automatic platform detection and cross-platform compatibility
  
  This example shows how to:
  - Automatically detect platform and configure serial interface
  - Initialize the RD-03D radar sensor on any supported platform
  - Switch between single and multi-target detection modes
  - Parse target coordinates (X, Y) and calculate distance/angle
  - Display target information including velocity
  - Handle different frame types and error conditions
  
  Hardware Connections:
  ESP32:
  - RD-03D TX -> GPIO 16 (RX)
  - RD-03D RX -> GPIO 17 (TX)
  
  ESP8266:
  - RD-03D TX -> GPIO 13 (RX) 
  - RD-03D RX -> GPIO 15 (TX)
  
  STM32:
  - RD-03D TX -> PA10 (USART1_RX)
  - RD-03D RX -> PA9  (USART1_TX)
  
  Common:
  - RD-03D VCC -> 3.3V
  - RD-03D GND -> GND
  
  Created: 2024
  By: Ai-Thinker RD-03D Library
*/

#include <Arduino.h>
#include "Ai-Thinker-RD-03.h"

// Create RD-03D instance
AiThinker_RD_03D radar;

// Platform-specific pin definitions and serial selection
#if defined(ARDUINO_ARCH_ESP32)
  const int RX_PIN = 16;
  const int TX_PIN = 17;
  #define RADAR_SERIAL Serial2
#elif defined(ARDUINO_ARCH_ESP8266)
  const int RX_PIN = 13;
  const int TX_PIN = 15;
  #define RADAR_SERIAL Serial
#elif defined(ARDUINO_ARCH_STM32)
  const int RX_PIN = PA10;
  const int TX_PIN = PA9;
  #define RADAR_SERIAL Serial1
#else
  // Default configuration for other platforms
  const int RX_PIN = 16;
  const int TX_PIN = 17;
  #define RADAR_SERIAL Serial1
#endif

// Detection mode
bool multiTargetMode = true;

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // Print every second

// Function declarations
void printTargetData();
void printStatistics();
void analyzeTarget(const AiThinker_RD_03D::TargetInfo& target);
void switchDetectionMode();
void configureRadar();

// Statistics
unsigned long frameCount = 0;
unsigned long errorCount = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("RD-03D Cross-Platform Example for PlatformIO");
  Serial.println("==============================================");
  
  // Display platform information
  Serial.print("Platform: ");
  Serial.println(AiThinker_RD_03D::getPlatformName());
  Serial.print("Hardware Serial Support: ");
  Serial.println(AiThinker_RD_03D::hasHardwareSerial() ? "Yes" : "No");
  Serial.print("Software Serial Support: ");
  Serial.println(AiThinker_RD_03D::hasSoftwareSerial() ? "Yes" : "No");
  Serial.println();
  
  // Initialize radar sensor with platform-appropriate configuration
  Serial.print("Initializing radar on ");
  Serial.print(AiThinker_RD_03D::getPlatformName());
  Serial.println("...");
  
  if (radar.begin(RADAR_SERIAL, RX_PIN, TX_PIN)) {
    Serial.print("RD-03D initialized successfully on ");
    Serial.println(AiThinker_RD_03D::getPlatformName());
  } else {
    Serial.println("Failed to initialize RD-03D");
    while (1) {
      delay(1000);
    }
  }
  
  // Configure tracking parameters for better ghost target filtering
  Serial.println("Configuring tracking parameters...");
  radar.setMaxTrackDistance(500);  // Filter targets beyond 5 meters
  radar.setMaxTrackVelocity(25);   // Filter fast-moving targets (good for stationary users)
  radar.setTrackingEnabled(true);  // Enable tracking algorithm
  
  Serial.print("Max tracking distance: ");
  Serial.print(radar.getMaxTrackDistance());
  Serial.println(" cm");
  
  Serial.print("Max tracking velocity: ");
  Serial.print(radar.getMaxTrackVelocity());
  Serial.println(" cm/s");
  
  Serial.print("Tracking enabled: ");
  Serial.println(radar.isTrackingEnabled() ? "Yes" : "No");
  
  // Set detection mode
  if (multiTargetMode) {
    Serial.println("Setting multi-target detection mode...");
    if (radar.setMultiTargetMode()) {
      Serial.println("Multi-target mode activated");
    } else {
      Serial.println("Failed to set multi-target mode");
    }
  } else {
    Serial.println("Setting single target detection mode...");
    if (radar.setSingleTargetMode()) {
      Serial.println("Single target mode activated");
    } else {
      Serial.println("Failed to set single target mode");
    }
  }
  
  Serial.println("\nStarting detection with tracking...");
  Serial.println("Format: Target[ID] - Distance: Xcm, Angle: Y°, Velocity: Z cm/s");
  Serial.println("Note: Only confirmed tracks (seen for 3+ frames) are displayed");
  Serial.println("================================================================");
}

void loop() {
  // Read radar data
  int bytesRead = radar.read();
  
  // Check if new frame is available
  if (radar.frameReady()) {
    frameCount++;
    
    // Get frame type
    AiThinker_RD_03D::FrameType frameType = radar.getFrameType();
    
    // Print data at regular intervals
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      printTargetData();
      printStatistics();
      lastPrintTime = currentTime;
    }
  }
  
  // Small delay to prevent overwhelming the serial output
  delay(10);
}

void printTargetData() {
  uint8_t targetCount = radar.getTargetCount();
  
  if (targetCount == 0) {
    Serial.println("No targets detected");
    return;
  }
  
  Serial.print("Detected ");
  Serial.print(targetCount);
  Serial.print(" target(s) at ");
  Serial.print(millis());
  Serial.println("ms:");
  
  for (int i = 0; i < targetCount; i++) {
    AiThinker_RD_03D::TargetInfo target;
    if (radar.getTargetInfo(i, target)) {
      Serial.print("  Target[");
      Serial.print(target.targetId);
      Serial.print("] - ");
      
      // Print coordinates
      Serial.print("X: ");
      Serial.print(target.x);
      Serial.print("cm, Y: ");
      Serial.print(target.y);
      Serial.print("cm, ");
      
      // Print calculated distance and angle
      Serial.print("Distance: ");
      Serial.print(target.distance);
      Serial.print("cm, Angle: ");
      Serial.print(target.angle);
      Serial.print("°, ");
      
      // Print velocity
      Serial.print("Velocity: ");
      Serial.print(target.velocity);
      Serial.println(" cm/s");
      
      // Additional target analysis
      analyzeTarget(target);
    }
  }
  Serial.println();
}

void analyzeTarget(const AiThinker_RD_03D::TargetInfo& target) {
  // Analyze target movement patterns
  static int16_t lastX = 0, lastY = 0;
  static unsigned long lastTime = 0;
  
  unsigned long currentTime = millis();
  if (lastTime > 0) {
    unsigned long timeDiff = currentTime - lastTime;
    if (timeDiff > 0) {
      int16_t deltaX = target.x - lastX;
      int16_t deltaY = target.y - lastY;
      
      // Calculate movement direction
      if (abs(deltaX) > 5 || abs(deltaY) > 5) { // Threshold to avoid noise
        Serial.print("    Movement: ");
        if (deltaX > 0) Serial.print("Right ");
        else if (deltaX < 0) Serial.print("Left ");
        if (deltaY > 0) Serial.print("Forward ");
        else if (deltaY < 0) Serial.print("Backward ");
        Serial.println();
      }
    }
  }
  
  lastX = target.x;
  lastY = target.y;
  lastTime = currentTime;
}

void printStatistics() {
  Serial.print("Statistics - Frames: ");
  Serial.print(frameCount);
  Serial.print(", Errors: ");
  Serial.print(errorCount);
  Serial.print(", Success Rate: ");
  if (frameCount > 0) {
    float successRate = ((float)(frameCount - errorCount) / frameCount) * 100.0;
    Serial.print(successRate, 1);
    Serial.print("%");
  } else {
    Serial.print("N/A");
  }
  Serial.println();
  Serial.println("----------------------------------------");
}

// Optional: Function to demonstrate mode switching
void switchDetectionMode() {
  multiTargetMode = !multiTargetMode;
  
  if (multiTargetMode) {
    Serial.println("Switching to multi-target mode...");
    if (radar.setMultiTargetMode()) {
      Serial.println("Multi-target mode activated");
    }
  } else {
    Serial.println("Switching to single target mode...");
    if (radar.setSingleTargetMode()) {
      Serial.println("Single target mode activated");
    }
  }
}

// Optional: Function to demonstrate configuration
void configureRadar() {
  Serial.println("Configuring radar parameters...");
  
  // Set detection range (20cm to 500cm)
  if (radar.setDetectionRange(20, 500)) {
    Serial.println("Detection range set to 20-500cm");
  }
  
  // Set sensitivity (0-255, higher = more sensitive)
  if (radar.setSensitivity(128)) {
    Serial.println("Sensitivity set to 128");
  }
  
  // Set output format to binary
  if (radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY)) {
    Serial.println("Output format set to binary");
  }
} 