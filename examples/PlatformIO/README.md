# RD-03D Radar Sensor - PlatformIO Example

This example demonstrates how to use the Ai-Thinker RD-03D radar sensor with PlatformIO and ESP32.

## Features

- **Multi-target detection**: Track up to 8 targets simultaneously
- **Real-time data processing**: Continuous target monitoring
- **Configurable parameters**: Adjustable detection range and sensitivity
- **Multiple output formats**: Support for binary and ASCII data

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

## Configuration

The example is configured for:
- **Baud rate**: 256000 (RD-03D specific)
- **Detection range**: 20cm to 800cm
- **Sensitivity**: 128 (medium)
- **Mode**: Multi-target detection
- **Output format**: Binary

## Building and Uploading

1. **Install PlatformIO**: Make sure you have PlatformIO installed
2. **Clone the repository**: Include the RD-03D library
3. **Build the project**:
   ```bash
   pio run
   ```
4. **Upload to ESP32**:
   ```bash
   pio run --target upload
   ```
5. **Monitor serial output**:
   ```bash
   pio device monitor
   ```

## Expected Output

The example will output:
- Radar initialization status
- Configuration parameters
- Real-time target detection data
- Periodic status reports

Example output:
```
RD-03D Radar Sensor - PlatformIO Example
=========================================
Radar sensor initialized successfully
Configuring radar parameters...
Firmware Version: RD-03D v1.0
Serial Number: 12345678
Detection range set: 20-800 cm
Sensitivity set to 128
Output format set to binary
Set to multi-target mode
Exited configuration mode
Setup complete. Starting detection...

Multi-target detected: 2 targets
  Target 0 - ID: 1, Distance: 150 cm, Angle: 45°, Velocity: 25 cm/s, Energy: 128
  Target 1 - ID: 2, Distance: 300 cm, Angle: -30°, Velocity: -15 cm/s, Energy: 95
```

## Customization

### Changing Detection Range
```cpp
// Set detection range from 50cm to 600cm
radar.setDetectionRange(50, 600);
```

### Adjusting Sensitivity
```cpp
// Set high sensitivity (0-255, higher = more sensitive)
radar.setSensitivity(200);
```

### Switching Modes
```cpp
// Single target mode
radar.setSingleTargetMode();

// Multi-target mode
radar.setMultiTargetMode();

// Debug mode
radar.setDebugMode();
```

## Troubleshooting

### Common Issues

1. **No data received**: 
   - Check baud rate (must be 256000)
   - Verify pin connections
   - Ensure proper power supply

2. **Configuration failures**:
   - Check if radar is responding to commands
   - Verify serial connection
   - Try resetting the radar module

3. **Incorrect readings**:
   - Calibrate detection range
   - Adjust sensitivity settings
   - Check for interference

### Debug Output

Enable debug output by modifying `platformio.ini`:
```ini
build_flags = 
    -D_LOG_LEVEL=6  ; Enable debug messages
```

## API Reference

The example demonstrates the main RD-03D API functions:

- `begin()` - Initialize radar
- `setDetectionRange()` - Configure detection range
- `setSensitivity()` - Set sensitivity level
- `setMultiTargetMode()` - Enable multi-target detection
- `read()` - Read incoming data
- `getTargetCount()` - Get number of targets
- `getTargetInfo()` - Get target details

For complete API documentation, see the main library README.

## License

This example is part of the RD-03D library and follows the same license terms. 