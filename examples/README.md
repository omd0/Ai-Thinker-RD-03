# RD-03D Library Examples

This directory contains example projects demonstrating the Ai-Thinker RD-03D radar sensor library with the new data frame parsing implementation.

## Examples Overview

### Arduino Example
- **File**: `Arduino/RD-03D_DataFrame_Example.ino`
- **Description**: Basic Arduino example showing single and multi-target detection
- **Features**:
  - Initialize RD-03D sensor
  - Switch between detection modes
  - Parse target coordinates (X, Y)
  - Calculate distance and angle
  - Display velocity information

### PlatformIO Example
- **File**: `PlatformIO/src/main.cpp`
- **Description**: Advanced ESP32 example with additional features
- **Features**:
  - All Arduino example features
  - Movement pattern analysis
  - Statistics tracking
  - Configuration management
  - Error handling

## Hardware Setup

### Required Components
- Ai-Thinker RD-03D radar sensor
- Arduino/ESP32 development board
- Jumper wires
- 3.3V power supply

### Wiring Diagram
```
RD-03D Sensor    Arduino/ESP32
VCC        -->   3.3V
GND        -->   GND
TX         -->   RX (Pin 2 for Arduino, Pin 16 for ESP32)
RX         -->   TX (Pin 3 for Arduino, Pin 17 for ESP32)
```

## Data Frame Format

The new implementation parses raw data frames with the following structure:

### Frame Header
- Header: `0xAA`
- Frame Type: `0x01` (single target) or `0x02` (multi-target)
- Target Count: Number of detected targets (1-3)
- Trailer: `0x55`

### Target Data (6 bytes per target)
- X-coordinate: 2 bytes (signed, in cm)
- Y-coordinate: 2 bytes (signed, in cm)
- Velocity: 2 bytes (signed, in cm/s)

### Calculated Values
- Distance: Calculated from X and Y coordinates using Pythagorean theorem
- Angle: Calculated using atan2(Y, X) in degrees

## Usage Instructions

### Arduino IDE
1. Install the RD-03D library in Arduino IDE
2. Open `Arduino/RD-03D_DataFrame_Example.ino`
3. Connect hardware according to wiring diagram
4. Upload and monitor serial output

### PlatformIO
1. Navigate to `examples/PlatformIO/`
2. Run `pio run --target upload`
3. Monitor output with `pio device monitor`

## Expected Output

```
RD-03D Data Frame Example
==========================
RD-03D initialized successfully
Multi-target mode activated

Starting detection...
Format: Target[ID] - Distance: Xcm, Angle: Y°, Velocity: Z cm/s
==============================================================

Detected 2 target(s) at 1234ms:
  Target[0] - X: 150cm, Y: 200cm, Distance: 250cm, Angle: 53°, Velocity: 25 cm/s
  Target[1] - X: -100cm, Y: 300cm, Distance: 316cm, Angle: 108°, Velocity: -15 cm/s
```

## Configuration Options

### Detection Modes
- **Single Target**: Detects one target with highest signal strength
- **Multi-Target**: Detects up to 3 targets simultaneously

### Parameters
- **Detection Range**: 20cm to 800cm (configurable)
- **Sensitivity**: 0-255 (higher = more sensitive)
- **Output Format**: Binary or ASCII

## Troubleshooting

### Common Issues
1. **No data received**: Check wiring and baud rate (256000)
2. **Incorrect readings**: Ensure sensor is properly mounted and oriented
3. **Communication errors**: Verify power supply and signal levels

### Debug Information
Enable debug output by setting `_LOG_LEVEL` to 2 or higher in the library header.

## Advanced Features

### Movement Analysis
The PlatformIO example includes movement pattern analysis:
- Direction detection (left/right, forward/backward)
- Velocity tracking
- Position history

### Statistics
- Frame count and error rate
- Success rate calculation
- Performance monitoring

## API Reference

### Main Functions
- `radar.begin(serial, rx_pin, tx_pin)`: Initialize sensor
- `radar.read()`: Read and parse data frames
- `radar.frameReady()`: Check if new data is available
- `radar.getTargetCount()`: Get number of detected targets
- `radar.getTargetInfo(index, target)`: Get target information

### Target Information Structure
```cpp
struct TargetInfo {
    uint8_t targetId;     // Target ID
    int16_t x;           // X-coordinate (cm)
    int16_t y;           // Y-coordinate (cm)
    uint16_t distance;   // Calculated distance (cm)
    int16_t angle;       // Calculated angle (degrees)
    int16_t velocity;    // Velocity (cm/s)
    uint8_t energy;      // Signal energy
    uint8_t status;      // Target status
};
```

## License

This example code is provided under the same license as the main library. 