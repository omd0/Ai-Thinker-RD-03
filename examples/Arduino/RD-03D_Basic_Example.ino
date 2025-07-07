/*
  RD-03D Basic Example - Cross Platform Compatible
  Simple example for beginners showing basic RD-03D functionality
  
  This example demonstrates:
  - Automatic platform detection
  - Cross-platform serial initialization
  - Reading target distance and angle
  - Simple output formatting
  
  Hardware Connections:
  ESP32/ESP8266:
  - RD-03D TX -> GPIO 16 (RX)
  - RD-03D RX -> GPIO 17 (TX)
  
  Arduino Uno/Nano:
  - RD-03D TX -> Pin 2 (SoftwareSerial RX)
  - RD-03D RX -> Pin 3 (SoftwareSerial TX)
  
  Arduino Mega:
  - RD-03D TX -> Pin 19 (Serial1 RX)
  - RD-03D RX -> Pin 18 (Serial1 TX)
  
  Common:
  - RD-03D VCC -> 3.3V
  - RD-03D GND -> GND
  
  Created: 2024
  By: Ai-Thinker RD-03D Library
*/

#include "Ai-Thinker-RD-03.h"

// Create RD-03D instance
AiThinker_RD_03D radar;

// Platform-specific serial setup
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  // ESP32/ESP8266 - Use Hardware Serial2 or Serial1
  #define RADAR_SERIAL Serial2
  #define RX_PIN 16
  #define TX_PIN 17
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  // Arduino Mega - Use Hardware Serial1
  #define RADAR_SERIAL Serial1
  #define RX_PIN 19  // Pin 19 is Serial1 RX
  #define TX_PIN 18  // Pin 18 is Serial1 TX
#elif defined(ARDUINO_ARCH_AVR)
  // Arduino Uno/Nano - Use SoftwareSerial
  #include <SoftwareSerial.h>
  SoftwareSerial radarSerial(2, 3); // RX, TX
  #define RADAR_SERIAL radarSerial
  #define RX_PIN 2
  #define TX_PIN 3
#else
  // Generic platform - try Serial1 if available
  #define RADAR_SERIAL Serial1
  #define RX_PIN -1  // Platform-specific
  #define TX_PIN -1  // Platform-specific
#endif

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("RD-03D Cross-Platform Example");
  Serial.println("==============================");
  
  // Display platform information
  Serial.print("Platform: ");
  Serial.println(AiThinker_RD_03D::getPlatformName());
  Serial.print("Hardware Serial Support: ");
  Serial.println(AiThinker_RD_03D::hasHardwareSerial() ? "Yes" : "No");
  Serial.print("Software Serial Support: ");
  Serial.println(AiThinker_RD_03D::hasSoftwareSerial() ? "Yes" : "No");
  Serial.println();
  
  // Initialize radar sensor based on platform
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  // ESP platforms - use hardware serial with pin specification
  if (radar.begin(RADAR_SERIAL, RX_PIN, TX_PIN)) {
    Serial.println("RD-03D initialized with Hardware Serial (ESP)");
  }
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  // Arduino Mega - use hardware serial
  if (radar.begin(RADAR_SERIAL, RX_PIN, TX_PIN)) {
    Serial.println("RD-03D initialized with Hardware Serial (Mega)");
  }
#elif defined(ARDUINO_ARCH_AVR)
  // Arduino Uno/Nano - use software serial
  if (radar.begin(RADAR_SERIAL)) {
    Serial.println("RD-03D initialized with Software Serial (Uno/Nano)");
  }
#else
  // Generic platform
  if (radar.begin(RADAR_SERIAL, RX_PIN, TX_PIN)) {
    Serial.println("RD-03D initialized with default configuration");
  }
#endif
  else {
    Serial.println("Failed to initialize RD-03D");
    while (1) {
      delay(1000);
    }
  }
  
  // Set to single target mode for simplicity
  if (radar.setSingleTargetMode()) {
    Serial.println("Single target mode activated");
  }
  
  Serial.println();
  Serial.println("Starting detection...");
  Serial.println("Distance | Angle | Velocity");
  Serial.println("---------|-------|---------");
}

void loop() {
  // Read radar data
  radar.read();
  
  // Check if new frame is available
  if (radar.frameReady()) {
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0) {
      // Get first target information
      uint16_t distance = radar.getTargetDistance(0);
      int16_t angle = radar.getTargetAngle(0);
      int16_t velocity = radar.getTargetVelocity(0);
      
      // Print formatted output
      Serial.print(distance);
      Serial.print(" cm    | ");
      Serial.print(angle);
      Serial.print("°     | ");
      Serial.print(velocity);
      Serial.println(" cm/s");
    } else {
      Serial.println("No target detected");
    }
  }
  
  delay(500); // Update every 500ms
} 