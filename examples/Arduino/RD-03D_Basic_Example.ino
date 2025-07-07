/*
  RD-03D Basic Example
  Simple example for beginners showing basic RD-03D functionality
  
  This example demonstrates:
  - Basic sensor initialization
  - Reading target distance and angle
  - Simple output formatting
  
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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("RD-03D Basic Example");
  Serial.println("====================");
  
  // Initialize radar sensor
  if (radar.begin(Serial2, 2, 3)) {
    Serial.println("RD-03D initialized successfully");
  } else {
    Serial.println("Failed to initialize RD-03D");
    while (1) {
      delay(1000);
    }
  }
  
  // Set to single target mode for simplicity
  if (radar.setSingleTargetMode()) {
    Serial.println("Single target mode activated");
  }
  
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