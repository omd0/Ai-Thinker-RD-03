/*
  RD-03D Data Frame Example
  Demonstrates the new data frame parsing capabilities of the Ai-Thinker RD-03D library
  
  This example shows how to:
  - Initialize the RD-03D radar sensor
  - Switch between single and multi-target detection modes
  - Parse target coordinates (X, Y) and calculate distance/angle
  - Display target information including velocity
  
  Hardware Connections:
  - RD-03D TX -> Arduino RX (Pin 2)
  - RD-03D RX -> Arduino TX (Pin 3)
  - RD-03D VCC -> 3.3V
  - RD-03D GND -> GND
  
  Created: 2024
  By: Ai-Thinker RD-03D Library
*/

#include "Ai-Thinker-RD-03.h"

// Create RD-03D instance
AiThinker_RD_03D radar;

// Pin definitions
const int RX_PIN = 2;
const int TX_PIN = 3;

// Detection mode
bool multiTargetMode = true;

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // Print every second

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("RD-03D Data Frame Example");
  Serial.println("==========================");
  
  // Initialize radar sensor
  if (radar.begin(Serial2, RX_PIN, TX_PIN)) {
    Serial.println("RD-03D initialized successfully");
  } else {
    Serial.println("Failed to initialize RD-03D");
    while (1) {
      delay(1000);
    }
  }
  
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
  
  Serial.println("\nStarting detection...");
  Serial.println("Format: Target[ID] - Distance: Xcm, Angle: Y°, Velocity: Z cm/s");
  Serial.println("================================================================");
}

void loop() {
  // Read radar data
  int bytesRead = radar.read();
  
  // Check if new frame is available
  if (radar.frameReady()) {
    unsigned long currentTime = millis();
    
    // Print data at regular intervals
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      printTargetData();
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
    }
  }
  Serial.println();
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