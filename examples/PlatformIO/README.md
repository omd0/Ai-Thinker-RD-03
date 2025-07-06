# RD-03 PlatformIO Example

This is a PlatformIO example for the Ai-Thinker RD-03 24Ghz FMCW radar sensor.

## Features

- **Operating Mode**: Basic target detection with range information
- **Reporting Mode**: Energy data from different distance gates
- **Debugging Mode**: Doppler data visualization
- **Automatic Mode Switching**: Cycles through all three modes every 10 seconds
- **LED Feedback**: Built-in LED indicates target presence

## Hardware Requirements

- ESP32 board (ESP32-S3, ESP32-C3, or standard ESP32)
- Ai-Thinker RD-03 radar module
- Jumper wires

## Connections

| RD-03 Module | ESP32 Pin |
|--------------|-----------|
| VCC          | 3.3V      |
| GND          | GND       |
| RX           | GPIO 17   |
| TX           | GPIO 16   |

## Pin Configuration

The example uses the following pins:
- **RADAR_RX_PIN**: GPIO 16 (connected to RD-03 TX)
- **RADAR_TX_PIN**: GPIO 17 (connected to RD-03 RX)
- **BUILTIN_LED_PIN**: GPIO 2 (built-in LED)

## Installation

1. Clone or download this example
2. Open the project in PlatformIO
3. Select your target board from the available environments:
   - `esp32-s3-devkitm-1` for ESP32-S3
   - `esp32dev` for standard ESP32
   - `esp32-c3-devkitm-1` for ESP32-C3
4. Build and upload the project

## Usage

### Serial Monitor Output

The example provides detailed output through the serial monitor:

```
RD-03 PlatformIO Example Starting...
Radar initialized successfully

=== RD-03 Radar Sensor Information ===
Firmware: v1.0.0
Protocol Version: 1, Buffer Size: 256

Setting mode to OPERATING_MODE... SUCCESS
Current Mode: OPERATING_MODE

=== Starting Main Loop ===
Target: Human, Range: 150 cm - LED ON
Target: None, Range: -1 - LED OFF
```

### Operating Modes

1. **Operating Mode** (Default):
   - Detects targets and provides range information
   - Controls LED based on target presence
   - Output: `Target: Human, Range: 150 cm - LED ON`

2. **Reporting Mode**:
   - Provides energy data from different distance gates
   - Useful for analyzing radar sensitivity
   - Output: `Target: 1, Distance: 150 cm, E0:1234, E1:5678, ...`

3. **Debugging Mode**:
   - Visual representation of doppler data
   - Shows movement patterns in real-time
   - Output: ASCII visualization of radar data

### Mode Switching

The example automatically cycles through all three modes every 10 seconds for demonstration purposes. In a real application, you would typically choose one mode based on your requirements.

## Customization

### Changing Pins

Modify the pin definitions at the top of `main.cpp`:

```cpp
#define RADAR_RX_PIN 16  // Change to your desired RX pin
#define RADAR_TX_PIN 17  // Change to your desired TX pin
#define BUILTIN_LED_PIN 2 // Change to your desired LED pin
```

### Disabling Auto Mode Switching

To use only one mode, comment out the mode switching code in the `loop()` function:

```cpp
// Comment out this section to disable auto mode switching
/*
static unsigned long lastModeChange = 0;
if (millis() - lastModeChange > 10000) {
    // ... mode switching code
}
*/
```

### Adjusting Timing

Modify the delay values to change the update rate:

```cpp
delay(100); // Change this value to adjust loop frequency
```

## Troubleshooting

### Common Issues

1. **"Failed to initialize radar"**:
   - Check wiring connections
   - Verify power supply (3.3V)
   - Ensure correct pin assignments

2. **No serial output**:
   - Check monitor speed (should be 115200)
   - Verify USB connection
   - Try different USB cable

3. **Inconsistent readings**:
   - Ensure stable power supply
   - Check for electromagnetic interference
   - Verify module is properly mounted

### Debug Information

The example includes comprehensive debug output. Monitor the serial console for:
- Firmware version information
- Protocol version and buffer size
- Mode change confirmations
- Target detection status

## Library Information

This example uses the [Ai-Thinker-RD-03 library](https://github.com/Gjorgjevikj/Ai-Thinker-RD-03) by Dejan Gjorgjevikj.

## License

This example is provided as-is for educational purposes. The RD-03 library is licensed under GPL-3.0. 