# Ai-Thinker-RD-03D
Arduino library for the RD-03D radar module for multi-target detection and tracking by Ai-Thinker

PLEASE NOTE: This is a refactored version of the original RD-03 library to support the RD-03D module. The original author has no affiliation with any manufacturer developing or selling radar modules.

## Overview

The RD-03D is a 24GHz Frequency Modulated Continuous Wave (FMCW) radar sensor with advanced multi-target detection capabilities. Unlike the original RD-03, the RD-03D can:

- **Multi-target tracking**: Detect and track up to 8 targets simultaneously
- **Enhanced measurements**: Provides distance, angle, and velocity for each target
- **Higher baud rate**: Uses 256000 baud for faster data transmission
- **Improved accuracy**: Better target resolution and tracking algorithms

## Key Features

- **Multi-target detection**: Track up to 8 targets simultaneously
- **Distance measurement**: 20cm to 800cm range
- **Angle measurement**: ±180° angular detection
- **Velocity measurement**: Target speed in cm/s
- **Configurable sensitivity**: Adjustable detection sensitivity
- **Multiple output formats**: Binary and ASCII data formats
- **Real-time tracking**: Continuous target monitoring

## Hardware Requirements

- ESP32 development board
- RD-03D radar module
- Serial connection (UART)

## Pin Connections

| RD-03D Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| 5V         | 5V        | Power supply |
| GND        | GND       | Ground       |
| TX         | GPIO 26   | Radar TX → ESP32 RX |
| RX         | GPIO 27   | Radar RX ← ESP32 TX |

## Quick Start

```cpp
#include <Ai-Thinker-RD-03.h>

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    
    // Initialize radar with 256000 baud
    radar.begin(Serial1, 26, 27);
    
    // Configure radar
    radar.setDetectionRange(20, 800);  // 20cm to 800cm
    radar.setSensitivity(128);         // Medium sensitivity
    radar.setMultiTargetMode();        // Enable multi-target detection
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        
        for (uint8_t i = 0; i < targetCount; i++) {
            uint16_t distance = radar.getTargetDistance(i);
            int16_t angle = radar.getTargetAngle(i);
            int16_t velocity = radar.getTargetVelocity(i);
            
            Serial.printf("Target %d: %d cm, %d°, %d cm/s\n", 
                         i, distance, angle, velocity);
        }
    }
}
```

## API Reference

### Initialization
- `begin(HardwareSerial& serial, int rxPin, int txPin)` - Initialize radar
- `setFrameTimeout(unsigned long timeout)` - Set frame timeout
- `setInterCommandDelay(unsigned long delay)` - Set command delay

### Configuration
- `enterConfigMode()` / `exitConfigMode()` - Configuration mode control
- `setSingleTargetMode()` / `setMultiTargetMode()` / `setDebugMode()` - Mode selection
- `setDetectionRange(uint16_t min, uint16_t max)` - Set detection range
- `setSensitivity(uint8_t sensitivity)` - Set sensitivity (0-255)
- `setOutputFormat(OutputFormat format)` - Set output format

### Data Reading
- `read()` - Read incoming data
- `frameReady()` - Check if frame is available
- `getFrameType()` - Get current frame type

### Target Information
- `getTargetCount()` - Get number of detected targets
- `getTargetInfo(uint8_t index, TargetInfo& target)` - Get target details
- `getTargetDistance(uint8_t index)` - Get target distance
- `getTargetAngle(uint8_t index)` - Get target angle
- `getTargetVelocity(uint8_t index)` - Get target velocity
- `getTargetEnergy(uint8_t index)` - Get target signal energy

## Data Structures

### TargetInfo
```cpp
struct TargetInfo {
    uint8_t targetId;     // Target ID (0-255)
    uint16_t distance;    // Distance in cm
    int16_t angle;        // Angle in degrees (-180 to 180)
    int16_t velocity;     // Velocity in cm/s
    uint8_t energy;       // Signal energy (0-255)
    uint8_t status;       // Target status
};
```

## Examples

The library includes several examples:

- `RD-03D_test.ino` - Basic multi-target detection example
- PlatformIO examples in the `examples/PlatformIO/` directory

## Configuration

### Detection Range
```cpp
// Set detection range from 20cm to 800cm
radar.setDetectionRange(20, 800);
```

### Sensitivity
```cpp
// Set sensitivity (0-255, higher = more sensitive)
radar.setSensitivity(128);
```

### Output Format
```cpp
// Set to binary format for faster processing
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY);

// Or ASCII format for human-readable output
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_ASCII);
```

## Troubleshooting

### Common Issues

1. **No data received**: Check baud rate (must be 256000)
2. **Incorrect readings**: Verify pin connections
3. **Configuration failures**: Ensure proper command sequence

### Debug Output

Enable debug output by setting the log level:
```cpp
#define _LOG_LEVEL 6  // Enable debug messages
```

## Version History

- **v1.0.0** - Initial RD-03D support with multi-target detection
- **v0.7.0** - Original RD-03 version (by Dejan Gjorgjevikj)

## References

- [RD-03D Documentation](https://aithinker.blog.csdn.net/article/details/133338984)
- Original RD-03 library by Dejan Gjorgjevikj

## License

This library is distributed in the hope that it will be useful, but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE either express or implied.

Released into the public domain.
Released under LGPL-2.1 see https://github.com/Gjorgjevikj/... for full license
