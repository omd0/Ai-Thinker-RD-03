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

## Updated Radar Data Frame Structure

### Overview
The RD-03D library has been updated to include the STM32-style data frame structure from the Ai-Thinker blog implementation. This provides a more direct interface for working with the RD-03D radar sensor in embedded applications.

### Key Changes Made

#### 1. New Data Frame Structures Added

**RadarDataFrame Structure:**
```cpp
struct RadarDataFrame
{
    uint8_t RX_BUF[64];               // 缓存数组 - Buffer array
    uint8_t RX_count;                 // 计数位 - Count position
    uint8_t RX_temp;                  // 缓存字符 - Buffer character
    
    uint8_t Radar_1;                  // 目标1标志位 - Target 1 flag
    uint8_t Radar_2;                  // 目标2标志位 - Target 2 flag  
    uint8_t Radar_3;                  // 目标3标志位 - Target 3 flag
    
    uint8_t Speaker_1;                // 语音播报目标1标志 - Voice broadcast target 1 flag
    uint8_t Speaker_2;                // 语音播报目标2标志 - Voice broadcast target 2 flag
    uint8_t Speaker_3;                // 语音播报目标3标志 - Voice broadcast target 3 flag
};
```

**RadarCommandFrame Structure:**
```cpp
struct RadarCommandFrame
{
    // Single Target Detection Command
    uint8_t Single_Target_Detection_CMD[15];  // = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01}
    
    // Multi Target Detection Command  
    uint8_t Multi_Target_Detection_CMD[15];   // = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01}
};
```

**SpeakerCommandFrame Structure:**
```cpp
struct SpeakerCommandFrame
{
    uint8_t SpeakerCMD_0[5];          // = {0x5A, 0x00, 0x08, 0x00, 0x01} - No target broadcast command
    uint8_t SpeakerCMD_1[5];          // = {0x5A, 0x00, 0x08, 0x00, 0x02} - 1 target broadcast command
    uint8_t SpeakerCMD_2[5];          // = {0x5A, 0x00, 0x08, 0x00, 0x03} - 2 target broadcast command
    uint8_t SpeakerCMD_3[5];          // = {0x5A, 0x00, 0x08, 0x00, 0x04} - 3 target broadcast command
};
```

#### 2. Command Arrays from Blog Implementation

The following command arrays have been integrated from the STM32 blog implementation:

- **Single Target Detection Command:** `{0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01}`
- **Multi Target Detection Command:** `{0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01}`

#### 3. Speaker Commands for Voice Feedback

- **SpeakerCMD_0:** `{0x5A, 0x00, 0x08, 0x00, 0x01}` - No target detected
- **SpeakerCMD_1:** `{0x5A, 0x00, 0x08, 0x00, 0x02}` - 1 target detected  
- **SpeakerCMD_2:** `{0x5A, 0x00, 0x08, 0x00, 0x03}` - 2 targets detected
- **SpeakerCMD_3:** `{0x5A, 0x00, 0x08, 0x00, 0x04}` - 3 targets detected

#### 4. Updated PlatformIO Example

The PlatformIO example (`examples/PlatformIO/src/main.cpp`) has been enhanced to demonstrate:

- STM32-style data frame variables
- Command frame usage
- Radar flag management (Radar_1, Radar_2, Radar_3)
- Speaker command integration
- Target count-based flag updates

### Usage Example

```cpp
// STM32-style data frame variables
uint8_t RX_BUF[64] = {0};      // Buffer array
uint8_t RX_count = 0;          // Count position
uint8_t RX_temp;               // Buffer character

uint8_t Radar_1 = 0;           // Target 1 flag
uint8_t Radar_2 = 0;           // Target 2 flag
uint8_t Radar_3 = 0;           // Target 3 flag

// Command arrays
uint8_t Single_Target_Detection_CMD[15] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
uint8_t Multi_Target_Detection_CMD[15]  = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

// Speaker commands
uint8_t SpeakerCMD_0[5] = {0x5A, 0x00, 0x08, 0x00, 0x01}; // No target
uint8_t SpeakerCMD_1[5] = {0x5A, 0x00, 0x08, 0x00, 0x02}; // 1 target
uint8_t SpeakerCMD_2[5] = {0x5A, 0x00, 0x08, 0x00, 0x03}; // 2 targets
uint8_t SpeakerCMD_3[5] = {0x5A, 0x00, 0x08, 0x00, 0x04}; // 3 targets

// Update radar flags based on target count
void updateRadarFlags(uint8_t targetCount) {
    Radar_1 = (targetCount >= 1) ? 1 : 0;
    Radar_2 = (targetCount >= 2) ? 1 : 0;
    Radar_3 = (targetCount >= 3) ? 1 : 0;
}
```

### Benefits of This Update

1. **STM32 Compatibility:** Direct integration with STM32-based projects using the exact data frame structure from the blog
2. **Voice Feedback Support:** Built-in speaker command arrays for audio feedback systems
3. **Simplified Flag Management:** Easy-to-use radar flags for target detection status
4. **Command Integration:** Pre-defined command arrays for mode switching
5. **Buffer Management:** Proper buffer handling as per STM32 implementation

### Files Modified

- `src/Ai-Thinker-RD-03.h` - Added new data frame structures
- `src/Ai-Thinker-RD-03.cpp` - Added initialization methods for new structures
- `examples/PlatformIO/src/main.cpp` - Enhanced example with STM32-style implementation

### Reference

Based on the STM32 implementation from: https://aithinker.blog.csdn.net/article/details/133338984

The blog demonstrates using STM32F103C8T6 with CubeMX and HAL library to process RD-03D serial data and integrate with VC-02 voice module for real-time target count announcements. 