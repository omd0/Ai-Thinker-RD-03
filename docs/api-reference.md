# API Reference

Complete API documentation for the Ai-Thinker RD-03D radar library.

## 📋 Table of Contents

- [Class Overview](#class-overview)
- [Constructor](#constructor)
- [Initialization](#initialization)
- [Configuration Methods](#configuration-methods)
- [Detection Control](#detection-control)
- [Data Reading](#data-reading)
- [Target Data Access](#target-data-access)
- [Tracking Configuration](#tracking-configuration)
- [Utility Methods](#utility-methods)
- [Data Structures](#data-structures)
- [Enumerations](#enumerations)

## 🏗️ Class Overview

```cpp
class AiThinker_RD_03D
```

Main class for interfacing with the Ai-Thinker RD-03D radar sensor.

## 🔧 Constructor

### AiThinker_RD_03D()
Creates a new radar instance with default settings.

```cpp
AiThinker_RD_03D radar;
```

**Default Settings:**
- Frame timeout: 200ms
- Inter-command delay: 100ms
- Mode: Single target
- Output format: Binary
- Tracking: Enabled
- Max distance: 500cm
- Max velocity: 25cm/s

## 🚀 Initialization

### begin()
Initializes the radar sensor with UART communication.

```cpp
bool begin(HardwareSerial& rSerial, int rxPin, int txPin, int rxBufferSize = 2048);
```

**Parameters:**
- `rSerial`: Hardware serial instance (e.g., Serial2)
- `rxPin`: ESP32 GPIO pin for receiving data
- `txPin`: ESP32 GPIO pin for transmitting data
- `rxBufferSize`: Size of receive buffer (default: 2048)

**Returns:** `true` if initialization successful, `false` otherwise

**Example:**
```cpp
if (radar.begin(Serial2, 16, 17)) {
    Serial.println("Radar initialized successfully");
} else {
    Serial.println("Failed to initialize radar");
}
```

## ⚙️ Configuration Methods

### Detection Range

#### setDetectionRange()
Sets the minimum and maximum detection range.

```cpp
bool setDetectionRange(uint16_t minDist, uint16_t maxDist);
```

**Parameters:**
- `minDist`: Minimum detection distance in cm (20-800)
- `maxDist`: Maximum detection distance in cm (20-800)

**Returns:** `true` if successful, `false` otherwise

**Example:**
```cpp
radar.setDetectionRange(20, 500);  // 20cm to 5m
```

#### getDetectionRange()
Gets the current detection range settings.

```cpp
bool getDetectionRange(uint16_t& minDist, uint16_t& maxDist);
```

**Parameters:**
- `minDist`: Reference to store minimum distance
- `maxDist`: Reference to store maximum distance

**Returns:** `true` if successful, `false` otherwise

### Sensitivity

#### setSensitivity()
Sets the radar sensitivity level.

```cpp
bool setSensitivity(uint8_t sensitivity);
```

**Parameters:**
- `sensitivity`: Sensitivity level (0-255, higher = more sensitive)

**Returns:** `true` if successful, `false` otherwise

**Example:**
```cpp
radar.setSensitivity(128);  // Medium sensitivity
```

#### getSensitivity()
Gets the current sensitivity setting.

```cpp
uint8_t getSensitivity();
```

**Returns:** Current sensitivity level (0-255)

### Output Format

#### setOutputFormat()
Sets the data output format.

```cpp
bool setOutputFormat(OutputFormat format);
```

**Parameters:**
- `format`: Output format (FORMAT_ASCII or FORMAT_BINARY)

**Returns:** `true` if successful, `false` otherwise

**Example:**
```cpp
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY);
```

#### getOutputFormat()
Gets the current output format.

```cpp
OutputFormat getOutputFormat();
```

**Returns:** Current output format

## 🎯 Detection Control

### Mode Selection

#### setSingleTargetMode()
Enables single target detection mode.

```cpp
bool setSingleTargetMode();
```

**Returns:** `true` if successful, `false` otherwise

#### setMultiTargetMode()
Enables multi-target detection mode.

```cpp
bool setMultiTargetMode();
```

**Returns:** `true` if successful, `false` otherwise

#### getCurrentMode()
Gets the current detection mode.

```cpp
RadarMode getCurrentMode();
```

**Returns:** Current mode (SINGLE_TARGET_MODE or MULTI_TARGET_MODE)

### Configuration Mode

#### enterConfigMode()
Enters configuration mode for advanced settings.

```cpp
bool enterConfigMode();
```

**Returns:** `true` if successful, `false` otherwise

#### exitConfigMode()
Exits configuration mode.

```cpp
bool exitConfigMode();
```

**Returns:** `true` if successful, `false` otherwise

#### isInConfigMode()
Checks if currently in configuration mode.

```cpp
bool isInConfigMode() const;
```

**Returns:** `true` if in config mode, `false` otherwise

## 📊 Data Reading

### read()
Reads and processes incoming radar data.

```cpp
int read();
```

**Returns:** Number of bytes read

**Example:**
```cpp
int bytesRead = radar.read();
if (bytesRead > 0 && radar.frameReady()) {
    // Process new data
}
```

### frameReady()
Checks if a complete frame is available for processing.

```cpp
bool frameReady() const;
```

**Returns:** `true` if frame available, `false` otherwise

### getFrameType()
Gets the type of the last received frame.

```cpp
FrameType getFrameType() const;
```

**Returns:** Frame type (TARGET_DATA, MULTI_TARGET_DATA, etc.)

## 🎯 Target Data Access

### getTargetCount()
Gets the number of detected targets.

```cpp
uint8_t getTargetCount() const;
```

**Returns:** Number of targets (0-8)

### getTargetInfo()
Gets detailed information about a specific target.

```cpp
bool getTargetInfo(uint8_t index, TargetInfo& target) const;
```

**Parameters:**
- `index`: Target index (0 to targetCount-1)
- `target`: Reference to store target information

**Returns:** `true` if successful, `false` otherwise

**Example:**
```cpp
AiThinker_RD_03D::TargetInfo target;
if (radar.getTargetInfo(0, target)) {
    Serial.print("Target distance: ");
    Serial.println(target.distance);
}
```

### Individual Target Properties

#### getTargetDistance()
Gets the distance of a specific target.

```cpp
uint16_t getTargetDistance(uint8_t index = 0) const;
```

**Parameters:**
- `index`: Target index (default: 0)

**Returns:** Distance in cm

#### getTargetAngle()
Gets the angle of a specific target.

```cpp
int16_t getTargetAngle(uint8_t index = 0) const;
```

**Parameters:**
- `index`: Target index (default: 0)

**Returns:** Angle in degrees (-180 to +180)

#### getTargetVelocity()
Gets the velocity of a specific target.

```cpp
int16_t getTargetVelocity(uint8_t index = 0) const;
```

**Parameters:**
- `index`: Target index (default: 0)

**Returns:** Velocity in cm/s

#### getTargetEnergy()
Gets the signal energy of a specific target.

```cpp
uint8_t getTargetEnergy(uint8_t index = 0) const;
```

**Parameters:**
- `index`: Target index (default: 0)

**Returns:** Signal energy (0-255)

## 🎯 Tracking Configuration

### setMaxTrackDistance()
Sets the maximum distance for track filtering.

```cpp
void setMaxTrackDistance(uint16_t distance);
```

**Parameters:**
- `distance`: Maximum tracking distance in cm

**Example:**
```cpp
radar.setMaxTrackDistance(500);  // 5 meters
```

### getMaxTrackDistance()
Gets the maximum tracking distance.

```cpp
uint16_t getMaxTrackDistance() const;
```

**Returns:** Maximum tracking distance in cm

### setMaxTrackVelocity()
Sets the maximum velocity for track filtering.

```cpp
void setMaxTrackVelocity(uint16_t velocity);
```

**Parameters:**
- `velocity`: Maximum tracking velocity in cm/s

**Example:**
```cpp
radar.setMaxTrackVelocity(25);  // 25 cm/s
```

### getMaxTrackVelocity()
Gets the maximum tracking velocity.

```cpp
uint16_t getMaxTrackVelocity() const;
```

**Returns:** Maximum tracking velocity in cm/s

### setTrackingEnabled()
Enables or disables the tracking algorithm.

```cpp
void setTrackingEnabled(bool enabled);
```

**Parameters:**
- `enabled`: `true` to enable tracking, `false` to disable

**Example:**
```cpp
radar.setTrackingEnabled(true);  // Enable ghost target filtering
```

### isTrackingEnabled()
Checks if tracking is enabled.

```cpp
bool isTrackingEnabled() const;
```

**Returns:** `true` if tracking enabled, `false` otherwise

## 🛠️ Utility Methods

### Timeout Configuration

#### setFrameTimeout()
Sets the frame timeout duration.

```cpp
void setFrameTimeout(unsigned long timeout);
```

**Parameters:**
- `timeout`: Timeout in milliseconds

#### getFrameTimeout()
Gets the current frame timeout.

```cpp
unsigned long getFrameTimeout() const;
```

**Returns:** Frame timeout in milliseconds

### Command Delay

#### setInterCommandDelay()
Sets the delay between commands.

```cpp
void setInterCommandDelay(unsigned long delay);
```

**Parameters:**
- `delay`: Delay in milliseconds

#### getInterCommandDelay()
Gets the current inter-command delay.

```cpp
unsigned long getInterCommandDelay() const;
```

**Returns:** Inter-command delay in milliseconds

### Buffer Management

#### clearRxBuffer()
Clears the receive buffer.

```cpp
void clearRxBuffer();
```

#### getFrameLength()
Gets the length of the last received frame.

```cpp
int getFrameLength() const;
```

**Returns:** Frame length in bytes

### Debug Methods

#### dumpLastFrame()
Dumps the last received frame for debugging.

```cpp
void dumpLastFrame(String label = "", Stream& dumpStream = Serial) const;
```

**Parameters:**
- `label`: Optional label for the dump
- `dumpStream`: Output stream (default: Serial)

#### dumpFrame()
Dumps a specific frame for debugging.

```cpp
static void dumpFrame(const uint8_t* buff, int len, String pre = "", String post = "", Stream& dumpStream = Serial);
```

**Parameters:**
- `buff`: Frame buffer
- `len`: Frame length
- `pre`: Prefix string
- `post`: Postfix string
- `dumpStream`: Output stream

## 📊 Data Structures

### TargetInfo
Contains information about a detected target.

```cpp
struct TargetInfo {
    uint8_t targetId;     // Target ID (0-255)
    int16_t x;           // X-coordinate in cm
    int16_t y;           // Y-coordinate in cm
    uint16_t distance;   // Distance in cm
    int16_t angle;       // Angle in degrees
    int16_t velocity;    // Velocity in cm/s
    uint8_t energy;      // Signal energy (0-255)
    uint8_t status;      // Target status
};
```

### ConfigParams
Contains configuration parameters.

```cpp
struct ConfigParams {
    uint16_t minDistance;    // Minimum detection distance (cm)
    uint16_t maxDistance;    // Maximum detection distance (cm)
    uint8_t sensitivity;     // Sensitivity level (0-255)
    uint8_t outputFormat;    // Output format
};
```

## 🔢 Enumerations

### FrameType
Types of radar frames.

```cpp
enum FrameType : uint8_t {
    UNIDENTIFIED_FRAME = 0x00,
    TARGET_DATA = 0x01,
    MULTI_TARGET_DATA = 0x02,
    DEBUG_DATA = 0x03,
    UNKNOWN_DATA_FORMAT = 0x0f,
    ACK_FRAME = 0xff,
    UNKNOWN_FRAME = 0xfe
};
```

### RadarMode
Detection modes.

```cpp
enum RadarMode : uint8_t {
    SINGLE_TARGET_MODE = 0x01,    // Single target detection
    MULTI_TARGET_MODE = 0x02,     // Multi-target detection
    DEBUG_MODE = 0x03             // Debug mode with raw data
};
```

### OutputFormat
Data output formats.

```cpp
enum OutputFormat : uint8_t {
    FORMAT_ASCII = 0x01,          // ASCII format output
    FORMAT_BINARY = 0x02          // Binary format output
};
```

### RadarCommand
Configuration commands.

```cpp
enum RadarCommand : uint8_t {
    SET_SINGLE_TARGET_MODE = 0x01,
    SET_MULTI_TARGET_MODE = 0x02,
    READ_FIRMWARE_VERSION = 0x03,
    READ_SERIAL_NUMBER = 0x04,
    SET_DETECTION_RANGE = 0x05,
    READ_DETECTION_RANGE = 0x06,
    SET_SENSITIVITY = 0x07,
    READ_SENSITIVITY = 0x08,
    SET_OUTPUT_FORMAT = 0x09,
    READ_OUTPUT_FORMAT = 0x0A,
    ENTER_CONFIG_MODE = 0xFE,
    EXIT_CONFIG_MODE = 0xFF
};
```

## 📝 Usage Examples

### Basic Usage
```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial2, 16, 17);
    radar.setMultiTargetMode();
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        uint8_t targetCount = radar.getTargetCount();
        for (int i = 0; i < targetCount; i++) {
            AiThinker_RD_03D::TargetInfo target;
            if (radar.getTargetInfo(i, target)) {
                Serial.print("Target ");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(target.distance);
                Serial.println(" cm");
            }
        }
    }
}
```

### Advanced Usage with Tracking
```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    radar.begin(Serial2, 16, 17);
    
    // Configure tracking
    radar.setTrackingEnabled(true);
    radar.setMaxTrackDistance(500);
    radar.setMaxTrackVelocity(25);
    
    radar.setMultiTargetMode();
}

void loop() {
    if (radar.read() > 0 && radar.frameReady()) {
        // Process tracked targets
        uint8_t targetCount = radar.getTargetCount();
        Serial.print("Confirmed targets: ");
        Serial.println(targetCount);
    }
}
``` 