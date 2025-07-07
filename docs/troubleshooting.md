# Troubleshooting Guide

Comprehensive guide to diagnose and fix common issues with the Ai-Thinker RD-03D radar library.

## 🔍 Quick Diagnostic Checklist

Before diving into specific issues, run through this checklist:

- [ ] ESP32 board is selected in Arduino IDE
- [ ] Library is properly installed
- [ ] Hardware connections are secure
- [ ] Power supply is 3.3V and stable
- [ ] Serial monitor baud rate is 115200
- [ ] No conflicting libraries installed
- [ ] Radar sensor is not damaged

## 🚨 Common Issues and Solutions

### Issue 1: Library Won't Compile

#### Symptoms
```
fatal error: 'Ai-Thinker-RD-03.h' file not found
```

#### Solutions

**1. Check Library Installation**
```bash
# Verify library is in correct location
# Arduino IDE: ~/Documents/Arduino/libraries/Ai-Thinker-RD-03/
# PlatformIO: ~/.platformio/lib/Ai-Thinker-RD-03/
```

**2. Restart Arduino IDE**
- Close Arduino IDE completely
- Reopen Arduino IDE
- Check Sketch → Include Library → Ai-Thinker-RD-03

**3. Manual Installation**
```cpp
// Download ZIP from GitHub
// Sketch → Include Library → Add .ZIP Library
// Select the downloaded ZIP file
```

**4. Check ESP32 Board**
- Tools → Board → ESP32 Arduino → ESP32 Dev Module
- Ensure ESP32 board package is installed

### Issue 2: No Radar Data Received

#### Symptoms
- Serial monitor shows "No targets detected" continuously
- No data messages from radar
- Radar sensor appears unresponsive

#### Solutions

**1. Check Hardware Connections**
```cpp
// Test basic connectivity
void setup() {
    Serial.begin(115200);
    Serial.println("Testing radar connection...");
    
    if (radar.begin(Serial2, 16, 17)) {
        Serial.println("✅ Radar initialized");
    } else {
        Serial.println("❌ Radar initialization failed");
    }
}
```

**2. Verify Power Supply**
- Measure voltage at radar VCC pin (should be 3.3V)
- Check for voltage drops under load
- Ensure stable power supply

**3. Test Different Pins**
```cpp
// Try alternative GPIO pins
radar.begin(Serial2, 18, 19);  // GPIO 18, 19
radar.begin(Serial1, 21, 22);  // GPIO 21, 22
```

**4. Check Serial Communication**
```cpp
// Test UART communication
void setup() {
    Serial.begin(115200);
    Serial2.begin(256000, SERIAL_8N1, 16, 17);
    
    // Send test command
    uint8_t testCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
    Serial2.write(testCmd, 12);
}
```

### Issue 3: Erratic or Unstable Readings

#### Symptoms
- Distance readings jump wildly
- Multiple ghost targets appearing
- Inconsistent target detection
- Velocity readings are impossible

#### Solutions

**1. Enable Tracking Algorithm**
```cpp
// Enable advanced filtering
radar.setTrackingEnabled(true);
radar.setMaxTrackDistance(500);  // 5m max
radar.setMaxTrackVelocity(25);   // 25 cm/s max
```

**2. Adjust Sensitivity**
```cpp
// Reduce sensitivity for less noise
radar.setSensitivity(100);  // Lower value = less sensitive
```

**3. Check for Interference**
- Move away from metal objects
- Check for nearby electronic devices
- Ensure stable mounting
- Add ferrite beads to data lines

**4. Improve Power Supply**
```cpp
// Add decoupling capacitors
// Use regulated 3.3V supply
// Ensure adequate current capacity
```

### Issue 4: Ghost Targets

#### Symptoms
- Multiple false targets detected
- Targets appearing and disappearing
- Impossible target positions
- Inconsistent target count

#### Solutions

**1. Enable and Configure Tracking**
```cpp
// Comprehensive tracking setup
radar.setTrackingEnabled(true);
radar.setMaxTrackDistance(300);  // 3m for indoor use
radar.setMaxTrackVelocity(15);   // 15 cm/s for stationary
```

**2. Adjust Detection Range**
```cpp
// Limit detection range
radar.setDetectionRange(50, 400);  // 50cm to 4m
```

**3. Environmental Optimization**
- Remove metal objects from detection area
- Ensure proper sensor mounting
- Check for multipath reflections
- Optimize sensor height and angle

**4. Advanced Filtering**
```cpp
// Custom tracking parameters for noisy environments
#define MIN_TRACK_AGE 5          // Require 5 frames for confirmation
#define MAX_FRAMES_MISSING 3     // Faster cleanup
```

### Issue 5: Serial Monitor Issues

#### Symptoms
- No output in serial monitor
- Garbled or corrupted data
- Serial connection drops

#### Solutions

**1. Check Serial Settings**
- Baud rate: 115200
- Line ending: Both NL & CR
- Port: Correct ESP32 port

**2. Test Serial Communication**
```cpp
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Serial test - if you see this, serial is working");
}

void loop() {
    Serial.println("Heartbeat: " + String(millis()));
    delay(1000);
}
```

**3. Check USB Connection**
- Try different USB cable
- Test different USB port
- Check for driver issues

### Issue 6: Performance Issues

#### Symptoms
- Slow response time
- High CPU usage
- Memory problems
- Frequent crashes

#### Solutions

**1. Optimize Processing Loop**
```cpp
void loop() {
    // Read radar data
    if (radar.read() > 0 && radar.frameReady()) {
        // Process data efficiently
        uint8_t targetCount = radar.getTargetCount();
        
        // Only process if targets detected
        if (targetCount > 0) {
            // Process targets
        }
    }
    
    // Add appropriate delay
    delay(50);  // 20Hz update rate
}
```

**2. Reduce Serial Output**
```cpp
// Only print when data changes
static uint8_t lastTargetCount = 0;
uint8_t currentTargetCount = radar.getTargetCount();

if (currentTargetCount != lastTargetCount) {
    Serial.print("Targets: ");
    Serial.println(currentTargetCount);
    lastTargetCount = currentTargetCount;
}
```

**3. Memory Management**
```cpp
// Check available memory
Serial.print("Free heap: ");
Serial.println(ESP.getFreeHeap());
```

## 🔧 Advanced Troubleshooting

### Using Debug Tools

#### 1. Frame Dumping
```cpp
// Dump raw frame data for analysis
if (radar.frameReady()) {
    radar.dumpLastFrame("Raw Frame");
    Serial.print("Frame length: ");
    Serial.println(radar.getFrameLength());
}
```

#### 2. Performance Monitoring
```cpp
// Monitor processing performance
unsigned long startTime = micros();
radar.read();
unsigned long endTime = micros();
Serial.print("Processing time: ");
Serial.print(endTime - startTime);
Serial.println(" microseconds");
```

#### 3. Error Counting
```cpp
// Track error rates
static unsigned long frameCount = 0;
static unsigned long errorCount = 0;

frameCount++;
if (!radar.frameReady()) {
    errorCount++;
}

if (frameCount % 100 == 0) {
    float errorRate = (float)errorCount / frameCount * 100.0;
    Serial.print("Error rate: ");
    Serial.print(errorRate, 1);
    Serial.println("%");
}
```

### Hardware Testing

#### 1. Continuity Testing
```cpp
// Test connections with multimeter
// VCC to 3.3V: Should show 3.3V
// GND to GND: Should show 0V
// TX/RX: Should show varying voltage
```

#### 2. Signal Analysis
```cpp
// Use oscilloscope to check signals
// TX line: Should show data pulses
// RX line: Should show command signals
// Power: Should be stable 3.3V
```

#### 3. Power Supply Testing
```cpp
// Monitor power supply under load
// Check for voltage drops
// Verify current capacity
// Test with different power sources
```

## 📊 Diagnostic Tools

### Built-in Diagnostics
```cpp
// Comprehensive diagnostic sketch
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    Serial.println("=== RD-03D Diagnostic Tool ===");
    
    // Test 1: Library version
    Serial.print("Library version: ");
    Serial.println(AI_THINKER_RD_03D_LIB_VERSION);
    
    // Test 2: Memory check
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
    
    // Test 3: Radar initialization
    if (radar.begin(Serial2, 16, 17)) {
        Serial.println("✅ Radar initialized");
    } else {
        Serial.println("❌ Radar initialization failed");
        return;
    }
    
    // Test 4: Configuration
    radar.setTrackingEnabled(true);
    radar.setMultiTargetMode();
    Serial.println("✅ Configuration applied");
    
    Serial.println("Starting diagnostic loop...");
}

void loop() {
    static unsigned long frameCount = 0;
    static unsigned long errorCount = 0;
    
    // Read radar data
    int bytesRead = radar.read();
    
    if (bytesRead > 0) {
        if (radar.frameReady()) {
            frameCount++;
            
            // Display frame info
            Serial.print("Frame #");
            Serial.print(frameCount);
            Serial.print(": ");
            Serial.print(radar.getTargetCount());
            Serial.println(" targets");
            
            // Process targets
            for (int i = 0; i < radar.getTargetCount(); i++) {
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
    } else {
        errorCount++;
        if (errorCount % 10 == 0) {
            Serial.print("No data received. Errors: ");
            Serial.println(errorCount);
        }
    }
    
    // Status report every 100 frames
    if (frameCount % 100 == 0 && frameCount > 0) {
        Serial.println("\n=== STATUS REPORT ===");
        Serial.print("Frames: ");
        Serial.println(frameCount);
        Serial.print("Errors: ");
        Serial.println(errorCount);
        Serial.print("Success rate: ");
        float successRate = ((float)(frameCount - errorCount) / frameCount) * 100.0;
        Serial.print(successRate, 1);
        Serial.println("%");
        Serial.println("===================\n");
    }
    
    delay(100);
}
```

### External Tools

#### 1. Serial Monitor
- **Arduino IDE**: Built-in serial monitor
- **PlatformIO**: `pio device monitor`
- **Third-party**: PuTTY, Tera Term, CoolTerm

#### 2. Logic Analyzer
- **Saleae Logic**: Professional analysis
- **PulseView**: Open-source alternative
- **Arduino Logic Analyzer**: Low-cost option

#### 3. Oscilloscope
- **Digital scope**: For signal analysis
- **Analog scope**: For basic testing
- **USB scope**: Portable option

## 🆘 Getting Help

### Before Asking for Help

1. **Run the diagnostic tool** above
2. **Check this troubleshooting guide**
3. **Verify hardware connections**
4. **Test with minimal code**
5. **Document the issue clearly**

### Information to Provide

When reporting issues, include:

- **Hardware**: ESP32 board model, radar sensor version
- **Software**: Library version, Arduino/PlatformIO version
- **Code**: Minimal example that reproduces the issue
- **Symptoms**: Clear description of the problem
- **Expected**: What should happen
- **Actual**: What actually happens
- **Diagnostics**: Output from diagnostic tool

### Support Channels

- **GitHub Issues**: [https://github.com/omd0/Ai-Thinker-RD-03/issues](https://github.com/omd0/Ai-Thinker-RD-03/issues)
- **Documentation**: Check [FAQ](faq.md) and [API Reference](api-reference.md)
- **Community**: ESP32 and Arduino forums

---

**Remember**: Most issues can be resolved by checking connections, enabling tracking, and adjusting parameters. Start with the basic solutions before moving to advanced troubleshooting. 