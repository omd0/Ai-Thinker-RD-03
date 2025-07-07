# RD-03 to RD-03D Refactoring Summary

This document summarizes the changes made to refactor the original RD-03 library to support the RD-03D radar module.

## Overview

The RD-03D is an enhanced version of the RD-03 radar module with significant improvements in capabilities and performance. This refactoring updates the library to support these new features while maintaining compatibility with the existing codebase structure.

## Key Differences Between RD-03 and RD-03D

### Hardware Differences
- **Baud Rate**: RD-03 uses 115200 baud, RD-03D uses 256000 baud
- **Multi-target Support**: RD-03D can track up to 8 targets simultaneously vs single target for RD-03
- **Enhanced Measurements**: RD-03D provides distance, angle, and velocity for each target
- **Improved Accuracy**: Better target resolution and tracking algorithms

### Protocol Differences
- **Frame Structure**: RD-03D uses different frame headers and data formats
- **Command Set**: Updated command structure for RD-03D specific features
- **Data Formats**: Support for both binary and ASCII output formats

## Refactoring Changes

### 1. Header File (`src/Ai-Thinker-RD-03.h`)

#### Class Name Change
- `AiThinker_RD_03` → `AiThinker_RD_03D`

#### New Enums and Constants
```cpp
// Updated frame types for RD-03D
enum FrameType : uint8_t {
    TARGET_DATA = 0x01,        // Single target data
    MULTI_TARGET_DATA = 0x02,  // Multi-target data
    DEBUG_DATA = 0x03,         // Debug data
    // ... other types
};

// RD-03D specific commands
enum RadarCommand : uint8_t {
    SET_SINGLE_TARGET_MODE = 0x01,
    SET_MULTI_TARGET_MODE = 0x02,
    SET_DETECTION_RANGE = 0x05,
    SET_SENSITIVITY = 0x07,
    // ... other commands
};

// New modes for RD-03D
enum RadarMode : uint8_t {
    SINGLE_TARGET_MODE = 0x01,
    MULTI_TARGET_MODE = 0x02,
    DEBUG_MODE = 0x03
};
```

#### New Data Structures
```cpp
// Target information structure
struct TargetInfo {
    uint8_t targetId;     // Target ID (0-255)
    uint16_t distance;    // Distance in cm
    int16_t angle;        // Angle in degrees (-180 to 180)
    int16_t velocity;     // Velocity in cm/s
    uint8_t energy;       // Signal energy (0-255)
    uint8_t status;       // Target status
};

// Multi-target frame structure
struct MultiTargetFrame {
    uint8_t header;                   // 0xAA
    uint8_t frameType;                // 0x02 for multi-target
    uint8_t dataLength;               // Length of data
    uint8_t targetCount;              // Number of targets
    TargetInfo targets[8];            // Up to 8 targets
    uint8_t checksum;                 // Checksum
    uint8_t trailer;                  // 0x55
};
```

#### New API Methods
```cpp
// Multi-target support
uint8_t getTargetCount() const;
bool getTargetInfo(uint8_t index, TargetInfo& target) const;
uint16_t getTargetDistance(uint8_t index = 0) const;
int16_t getTargetAngle(uint8_t index = 0) const;
int16_t getTargetVelocity(uint8_t index = 0) const;
uint8_t getTargetEnergy(uint8_t index = 0) const;

// Configuration methods
bool setDetectionRange(uint16_t minDist, uint16_t maxDist);
bool getDetectionRange(uint16_t& minDist, uint16_t& maxDist);
bool setSensitivity(uint8_t sensitivity);
uint8_t getSensitivity();
bool setOutputFormat(OutputFormat format);
OutputFormat getOutputFormat();
```

### 2. Implementation File (`src/Ai-Thinker-RD-03.cpp`)

#### Baud Rate Update
```cpp
// RD-03D uses 256000 baud rate
radarUART->begin(256000, SERIAL_8N1, rxPin, txPin);
```

#### Frame Processing
- Updated frame parsing to handle RD-03D specific frame formats
- Added support for multi-target data processing
- Implemented new checksum calculation method
- Updated frame validation logic

#### Command Processing
- Implemented new RD-03D command structure
- Added support for configuration commands
- Updated acknowledgment frame handling

### 3. Library Properties (`library.properties`)

#### Updated Metadata
```properties
name=Ai-Thinker-RD-03D
version=1.0.0
sentence=Arduino library for the Ai-Thinker RD-03D 24Ghz FMCW radar sensor with multi-target detection
paragraph=This sensor is a Frequency Modulated Continuous Wave radar with multi-target detection capability...
```

### 4. Examples

#### New Arduino Example (`examples/Arduino/RD-03D_test.ino`)
- Demonstrates multi-target detection
- Shows configuration of RD-03D specific parameters
- Includes mode switching functionality
- Provides comprehensive target information display

#### Updated PlatformIO Example (`examples/PlatformIO/src/main.cpp`)
- Refactored for RD-03D capabilities
- Updated pin assignments and configuration
- Enhanced error handling and status reporting

### 5. Documentation (`README.md`)

#### Comprehensive Updates
- Updated overview and features for RD-03D
- Added hardware requirements and pin connections
- Included quick start guide with code examples
- Added API reference and troubleshooting section
- Updated version history and references

## Migration Guide

### For Existing RD-03 Users

1. **Update Library**: Replace the old library with the RD-03D version
2. **Update Class Name**: Change `AiThinker_RD_03` to `AiThinker_RD_03D`
3. **Update Baud Rate**: Change from 115200 to 256000
4. **Update API Calls**: Use new multi-target methods instead of single target
5. **Test Configuration**: Verify all parameters work with RD-03D

### Code Migration Example

#### Before (RD-03)
```cpp
AiThinker_RD_03 radar;
radar.begin(Serial1, 26, 27);  // 115200 baud
uint16_t distance = radar.targetDistance();
```

#### After (RD-03D)
```cpp
AiThinker_RD_03D radar;
radar.begin(Serial1, 26, 27);  // 256000 baud
radar.setMultiTargetMode();
uint8_t targetCount = radar.getTargetCount();
for (uint8_t i = 0; i < targetCount; i++) {
    uint16_t distance = radar.getTargetDistance(i);
    int16_t angle = radar.getTargetAngle(i);
    int16_t velocity = radar.getTargetVelocity(i);
}
```

## Testing and Validation

### Recommended Test Scenarios
1. **Single Target Detection**: Verify basic target detection works
2. **Multi-target Detection**: Test with multiple targets simultaneously
3. **Configuration**: Test all configuration parameters
4. **Performance**: Verify 256000 baud communication
5. **Error Handling**: Test with invalid configurations

### Known Limitations
- Requires ESP32 architecture (little-endian systems)
- Limited to 8 simultaneous targets
- Some advanced features may require firmware updates

## Future Enhancements

### Potential Improvements
1. **Additional Target Information**: Support for more target properties
2. **Advanced Filtering**: Target filtering and classification
3. **Data Logging**: Built-in data recording capabilities
4. **Web Interface**: Web-based configuration and monitoring
5. **Cloud Integration**: IoT connectivity features

## References

- [RD-03D Documentation](https://aithinker.blog.csdn.net/article/details/133338984)
- Original RD-03 library by Dejan Gjorgjevikj
- Ai-Thinker RD-03D datasheet and specifications

## Conclusion

This refactoring successfully updates the library to support the RD-03D radar module while maintaining the familiar API structure. The enhanced multi-target detection capabilities and improved performance make the RD-03D a significant upgrade over the original RD-03 module. 