# Basic Examples

This guide provides simple, easy-to-understand examples for getting started with the Ai-Thinker RD-03D radar library.

## 📋 Prerequisites

Before running these examples, ensure you have:
- Installed the radar library (see [Installation Guide](../installation.md))
- Connected the hardware (see [Hardware Setup](../hardware-setup.md))
- Selected the correct ESP32 board in your IDE

## 🚀 Example 1: Basic Target Detection

The simplest example that detects and displays target information.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Basic Target Detection");
    
    // Initialize radar
    if (radar.begin(Serial2, 16, 17)) {
        Serial.println("Radar initialized successfully");
    } else {
        Serial.println("Failed to initialize radar");
        while (1) delay(1000);
    }
    
    // Set to multi-target mode
    radar.setMultiTargetMode();
    Serial.println("Multi-target mode activated");
}

void loop() {
    // Read radar data
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        if (targetCount == 0) {
            Serial.println("No targets detected");
        } else {
            Serial.print("Detected ");
            Serial.print(targetCount);
            Serial.println(" target(s):");
            
            for (int i = 0; i < targetCount; i++) {
                AiThinker_RD_03D::TargetInfo target;
                if (radar.getTargetInfo(i, target)) {
                    Serial.print("  Target ");
                    Serial.print(i + 1);
                    Serial.print(": ");
                    Serial.print(target.distance);
                    Serial.print("cm, ");
                    Serial.print(target.angle);
                    Serial.print("°, ");
                    Serial.print(target.velocity);
                    Serial.println(" cm/s");
                }
            }
        }
        Serial.println();
    }
    
    delay(100); // Small delay to prevent overwhelming output
}
```

**Expected Output:**
```
RD-03D Basic Target Detection
Radar initialized successfully
Multi-target mode activated
Detected 1 target(s):
  Target 1: 150cm, 45°, 5 cm/s

No targets detected

Detected 1 target(s):
  Target 1: 148cm, 44°, 3 cm/s
```

## 🎯 Example 2: Single Target Mode

Example using single target detection mode for simpler applications.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Single Target Detection");
    
    // Initialize radar
    radar.begin(Serial2, 16, 17);
    
    // Set to single target mode
    radar.setSingleTargetMode();
    Serial.println("Single target mode activated");
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        if (targetCount > 0) {
            // Get the closest target
            uint16_t distance = radar.getTargetDistance(0);
            int16_t angle = radar.getTargetAngle(0);
            int16_t velocity = radar.getTargetVelocity(0);
            
            Serial.print("Target detected: ");
            Serial.print(distance);
            Serial.print("cm away at ");
            Serial.print(angle);
            Serial.print("° moving at ");
            Serial.print(velocity);
            Serial.println(" cm/s");
            
            // Simple presence detection
            if (distance < 200) {
                Serial.println("*** PERSON DETECTED ***");
            }
        } else {
            Serial.println("No target detected");
        }
    }
    
    delay(200);
}
```

## 📊 Example 3: Distance Monitoring

Monitor distance to the nearest target with alerts.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

// Distance zones
const uint16_t ZONE_CLOSE = 100;   // 1m
const uint16_t ZONE_MEDIUM = 300;  // 3m
const uint16_t ZONE_FAR = 500;     // 5m

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Distance Monitoring");
    
    radar.begin(Serial2, 16, 17);
    radar.setMultiTargetMode();
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        if (targetCount > 0) {
            // Find closest target
            uint16_t closestDistance = 9999;
            for (int i = 0; i < targetCount; i++) {
                uint16_t distance = radar.getTargetDistance(i);
                if (distance < closestDistance) {
                    closestDistance = distance;
                }
            }
            
            // Determine zone
            String zone;
            if (closestDistance < ZONE_CLOSE) {
                zone = "CLOSE";
            } else if (closestDistance < ZONE_MEDIUM) {
                zone = "MEDIUM";
            } else if (closestDistance < ZONE_FAR) {
                zone = "FAR";
            } else {
                zone = "OUT OF RANGE";
            }
            
            Serial.print("Closest target: ");
            Serial.print(closestDistance);
            Serial.print("cm (");
            Serial.print(zone);
            Serial.println(")");
            
            // Zone-specific actions
            if (closestDistance < ZONE_CLOSE) {
                Serial.println("⚠️  WARNING: Target very close!");
            }
        } else {
            Serial.println("No targets in range");
        }
    }
    
    delay(500);
}
```

## 🔄 Example 4: Movement Detection

Detect and analyze target movement patterns.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

// Movement tracking variables
int16_t lastDistance = 0;
int16_t lastAngle = 0;
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Movement Detection");
    
    radar.begin(Serial2, 16, 17);
    radar.setMultiTargetMode();
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        if (targetCount > 0) {
            AiThinker_RD_03D::TargetInfo target;
            if (radar.getTargetInfo(0, target)) {
                unsigned long currentTime = millis();
                
                if (lastTime > 0) {
                    unsigned long timeDiff = currentTime - lastTime;
                    if (timeDiff > 0) {
                        // Calculate movement
                        int16_t distanceChange = target.distance - lastDistance;
                        int16_t angleChange = target.angle - lastAngle;
                        
                        // Determine movement direction
                        String direction = "";
                        if (abs(distanceChange) > 5) {
                            if (distanceChange > 0) {
                                direction += "Moving AWAY ";
                            } else {
                                direction += "Moving TOWARD ";
                            }
                        }
                        
                        if (abs(angleChange) > 5) {
                            if (angleChange > 0) {
                                direction += "RIGHT ";
                            } else {
                                direction += "LEFT ";
                            }
                        }
                        
                        if (direction.length() > 0) {
                            Serial.print("Movement: ");
                            Serial.println(direction);
                        }
                        
                        // Detect rapid movement
                        if (abs(target.velocity) > 50) {
                            Serial.println("🚨 RAPID MOVEMENT DETECTED!");
                        }
                    }
                }
                
                // Update tracking variables
                lastDistance = target.distance;
                lastAngle = target.angle;
                lastTime = currentTime;
                
                // Display current status
                Serial.print("Position: ");
                Serial.print(target.distance);
                Serial.print("cm, ");
                Serial.print(target.angle);
                Serial.print("°, Speed: ");
                Serial.print(target.velocity);
                Serial.println(" cm/s");
            }
        } else {
            Serial.println("No target detected");
        }
    }
    
    delay(200);
}
```

## 🎛️ Example 5: Configuration Demo

Demonstrates various configuration options.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Configuration Demo");
    
    // Initialize radar
    radar.begin(Serial2, 16, 17);
    
    // Configure detection range
    radar.setDetectionRange(50, 400);  // 50cm to 4m
    Serial.println("Detection range: 50cm - 4m");
    
    // Configure sensitivity
    radar.setSensitivity(150);  // Medium-high sensitivity
    Serial.println("Sensitivity: 150");
    
    // Set output format
    radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY);
    Serial.println("Output format: Binary");
    
    // Configure tracking
    radar.setTrackingEnabled(true);
    radar.setMaxTrackDistance(400);  // 4m max
    radar.setMaxTrackVelocity(30);   // 30 cm/s max
    Serial.println("Tracking enabled with custom parameters");
    
    // Set detection mode
    radar.setMultiTargetMode();
    Serial.println("Multi-target mode activated");
    
    Serial.println("\nConfiguration complete. Starting detection...");
    Serial.println("=============================================");
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        Serial.print("Targets: ");
        Serial.print(targetCount);
        
        if (targetCount > 0) {
            Serial.print(" | Closest: ");
            Serial.print(radar.getTargetDistance(0));
            Serial.print("cm");
        }
        
        Serial.println();
    }
    
    delay(1000); // Update every second
}
```

## 🔧 Example 6: Error Handling

Robust example with proper error handling and status monitoring.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

// Status tracking
unsigned long frameCount = 0;
unsigned long errorCount = 0;
unsigned long lastStatusTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Error Handling Demo");
    
    // Initialize with error checking
    if (!radar.begin(Serial2, 16, 17)) {
        Serial.println("❌ Failed to initialize radar");
        Serial.println("Check wiring and power supply");
        while (1) {
            delay(1000);
            Serial.println("Retrying initialization...");
            if (radar.begin(Serial2, 16, 17)) {
                Serial.println("✅ Radar initialized successfully");
                break;
            }
        }
    } else {
        Serial.println("✅ Radar initialized successfully");
    }
    
    // Configure radar
    radar.setMultiTargetMode();
    radar.setTrackingEnabled(true);
    
    Serial.println("Starting detection with error monitoring...");
}

void loop() {
    // Read radar data
    int bytesRead = radar.read();
    
    if (bytesRead > 0) {
        if (radar.frameReady()) {
            frameCount++;
            
            // Process frame
            AiThinker_RD_03D::FrameType frameType = radar.getFrameType();
            uint8_t targetCount = radar.getTargetCount();
            
            // Display frame information
            Serial.print("Frame #");
            Serial.print(frameCount);
            Serial.print(" (");
            Serial.print(frameType);
            Serial.print("): ");
            Serial.print(targetCount);
            Serial.println(" targets");
            
            // Process targets
            for (int i = 0; i < targetCount; i++) {
                AiThinker_RD_03D::TargetInfo target;
                if (radar.getTargetInfo(i, target)) {
                    Serial.print("  Target ");
                    Serial.print(i + 1);
                    Serial.print(": ");
                    Serial.print(target.distance);
                    Serial.print("cm, ");
                    Serial.print(target.angle);
                    Serial.print("°, ");
                    Serial.print(target.velocity);
                    Serial.println(" cm/s");
                } else {
                    Serial.println("  Error: Could not get target info");
                    errorCount++;
                }
            }
        }
    } else {
        // No data received
        Serial.println("No radar data received");
        errorCount++;
    }
    
    // Status report every 10 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastStatusTime > 10000) {
        Serial.println("\n=== STATUS REPORT ===");
        Serial.print("Frames processed: ");
        Serial.println(frameCount);
        Serial.print("Errors encountered: ");
        Serial.println(errorCount);
        
        if (frameCount > 0) {
            float successRate = ((float)(frameCount - errorCount) / frameCount) * 100.0;
            Serial.print("Success rate: ");
            Serial.print(successRate, 1);
            Serial.println("%");
        }
        
        Serial.print("Tracking enabled: ");
        Serial.println(radar.isTrackingEnabled() ? "Yes" : "No");
        Serial.print("Max distance: ");
        Serial.print(radar.getMaxTrackDistance());
        Serial.println("cm");
        Serial.println("===================\n");
        
        lastStatusTime = currentTime;
    }
    
    delay(100);
}
```

## 📈 Example 7: Data Logging

Example that logs radar data for analysis.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

// Logging variables
unsigned long startTime = 0;
unsigned long logInterval = 1000; // Log every second

void setup() {
    Serial.begin(115200);
    Serial.println("RD-03D Data Logging");
    
    radar.begin(Serial2, 16, 17);
    radar.setMultiTargetMode();
    radar.setTrackingEnabled(true);
    
    startTime = millis();
    
    // Print CSV header
    Serial.println("Timestamp,TargetCount,Target1_Distance,Target1_Angle,Target1_Velocity,Target2_Distance,Target2_Angle,Target2_Velocity");
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        unsigned long currentTime = millis();
        
        // Log data at regular intervals
        if (currentTime - startTime >= logInterval) {
            uint8_t targetCount = radar.getTargetCount();
            
            // Start CSV line
            Serial.print(currentTime);
            Serial.print(",");
            Serial.print(targetCount);
            
            // Log target data
            for (int i = 0; i < 2; i++) { // Log up to 2 targets
                if (i < targetCount) {
                    AiThinker_RD_03D::TargetInfo target;
                    if (radar.getTargetInfo(i, target)) {
                        Serial.print(",");
                        Serial.print(target.distance);
                        Serial.print(",");
                        Serial.print(target.angle);
                        Serial.print(",");
                        Serial.print(target.velocity);
                    } else {
                        Serial.print(",0,0,0");
                    }
                } else {
                    Serial.print(",0,0,0");
                }
            }
            
            Serial.println();
            startTime = currentTime;
        }
    }
    
    delay(50);
}
```

## 🎯 Example 8: Simple Presence Detection

Minimal example for basic presence detection.

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

const uint16_t PRESENCE_THRESHOLD = 200; // 2m
bool presenceDetected = false;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial2, 16, 17);
    radar.setSingleTargetMode();
    radar.setTrackingEnabled(true);
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        bool currentPresence = (targetCount > 0 && radar.getTargetDistance(0) < PRESENCE_THRESHOLD);
        
        // Only report state changes
        if (currentPresence != presenceDetected) {
            presenceDetected = currentPresence;
            
            if (presenceDetected) {
                Serial.println("👤 PRESENCE DETECTED");
            } else {
                Serial.println("❌ NO PRESENCE");
            }
        }
    }
    
    delay(500);
}
```

## 📚 Next Steps

After trying these basic examples:

1. **Experiment with parameters** - Try different sensitivity and range settings
2. **Test tracking features** - Compare results with and without tracking enabled
3. **Explore advanced examples** - Check out the [Advanced Examples](advanced-examples.md)
4. **Build your own application** - Use these examples as starting points

## 🐛 Troubleshooting

### Common Issues

**No targets detected:**
- Check wiring connections
- Verify power supply (3.3V)
- Try increasing sensitivity
- Ensure radar is not blocked

**Erratic readings:**
- Enable tracking: `radar.setTrackingEnabled(true)`
- Adjust tracking parameters
- Check for interference sources

**Compilation errors:**
- Ensure ESP32 board is selected
- Check library installation
- Verify include statement

### Getting Help

- Check the [FAQ](../faq.md) for common solutions
- Review [Troubleshooting Guide](../troubleshooting.md)
- Open an issue on [GitHub](https://github.com/omd0/Ai-Thinker-RD-03/issues) 