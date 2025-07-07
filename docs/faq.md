# Frequently Asked Questions (FAQ)

Common questions and answers about the Ai-Thinker RD-03D radar library.

## 🔧 General Questions

### Q: What is the Ai-Thinker RD-03D?
**A:** The RD-03D is a 24GHz Frequency Modulated Continuous Wave (FMCW) radar sensor that can detect multiple targets simultaneously, measuring distance, angle, and velocity. It's designed for human presence detection and basic tracking applications.

### Q: What platforms does this library support?
**A:** This library is designed for ESP32 development boards using the Arduino framework. It works with both Arduino IDE and PlatformIO.

### Q: What's the difference between this library and others?
**A:** This library includes advanced target tracking algorithms that eliminate ghost targets and provide stable readings, making it ideal for reliable human presence detection.

### Q: Is this library free to use?
**A:** Yes, this library is released under the LGPL-2.1 license, which allows free use in both personal and commercial projects.

## 🚀 Getting Started

### Q: How do I install the library?
**A:** See the [Installation Guide](installation.md) for detailed instructions. You can install via Arduino IDE (Sketch → Include Library → Add .ZIP Library) or PlatformIO (add to lib_deps).

### Q: What hardware do I need?
**A:** You need:
- ESP32 development board
- Ai-Thinker RD-03D radar sensor
- Jumper wires
- USB cable for programming

### Q: How do I connect the radar to ESP32?
**A:** Connect as follows:
- RD-03D VCC → ESP32 3.3V
- RD-03D GND → ESP32 GND
- RD-03D TX → ESP32 GPIO 16
- RD-03D RX → ESP32 GPIO 17

### Q: What's the minimum code to get started?
**A:** Here's the simplest example:
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
        Serial.print("Targets: ");
        Serial.println(targetCount);
    }
    delay(100);
}
```

## 🎯 Target Detection

### Q: How many targets can the radar detect?
**A:** The radar can detect up to 3 targets simultaneously in multi-target mode, or 1 target in single target mode.

### Q: What's the detection range?
**A:** The radar can detect targets from 20cm to 800cm (8 meters), configurable via `setDetectionRange()`.

### Q: How accurate are the measurements?
**A:** Typical accuracy is:
- Distance: ±2cm
- Angle: ±1°
- Velocity: ±5cm/s

### Q: What's the update rate?
**A:** The radar can provide updates up to 50Hz, though the actual rate depends on your processing code.

### Q: Why do I see ghost targets?
**A:** Ghost targets are false detections caused by noise, multipath reflections, or interference. Enable the tracking algorithm with `radar.setTrackingEnabled(true)` to eliminate them.

## 🔄 Tracking Algorithm

### Q: What is the tracking algorithm?
**A:** The tracking algorithm filters raw radar data to eliminate ghost targets and provide stable readings. It uses range/velocity gating, track confirmation, and data smoothing.

### Q: How do I enable tracking?
**A:** Simply call:
```cpp
radar.setTrackingEnabled(true);
```

### Q: What are the default tracking parameters?
**A:** Default parameters are:
- Max distance: 500cm
- Max velocity: 25cm/s
- Track confirmation: 3 frames
- Track deletion: 5 frames missing

### Q: How do I customize tracking parameters?
**A:** Use these methods:
```cpp
radar.setMaxTrackDistance(300);  // 3m max
radar.setMaxTrackVelocity(15);   // 15 cm/s max
```

### Q: Should I use tracking for all applications?
**A:** Yes, for most applications. Tracking eliminates ghost targets and provides stable readings. Only disable it if you need raw, unfiltered data for debugging.

## ⚙️ Configuration

### Q: What detection modes are available?
**A:** Two modes:
- **Single Target**: Detects closest target only
- **Multi-Target**: Detects up to 3 targets simultaneously

### Q: How do I change detection mode?
**A:** Use these methods:
```cpp
radar.setSingleTargetMode();   // Single target
radar.setMultiTargetMode();    // Multi-target
```

### Q: How do I adjust sensitivity?
**A:** Use `setSensitivity()` with values 0-255:
```cpp
radar.setSensitivity(128);  // Medium sensitivity
radar.setSensitivity(200);  // High sensitivity
radar.setSensitivity(50);   // Low sensitivity
```

### Q: What output formats are supported?
**A:** Two formats:
- **Binary**: More efficient, recommended
- **ASCII**: Human-readable, for debugging

### Q: How do I set the output format?
**A:** Use:
```cpp
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY);
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_ASCII);
```

## 🔧 Troubleshooting

### Q: The radar isn't detecting anything
**A:** Check:
1. Wiring connections (VCC, GND, TX, RX)
2. Power supply (3.3V required)
3. Sensor orientation (face forward)
4. Detection range settings
5. Sensitivity settings

### Q: I'm getting erratic readings
**A:** Try:
1. Enable tracking: `radar.setTrackingEnabled(true)`
2. Adjust tracking parameters
3. Check for interference sources
4. Ensure stable power supply
5. Move away from metal objects

### Q: The library won't compile
**A:** Check:
1. ESP32 board is selected in Arduino IDE
2. Library is properly installed
3. Include statement is correct
4. No conflicting libraries

### Q: Serial monitor shows no data
**A:** Check:
1. Baud rate is set to 115200
2. Correct serial port is selected
3. Radar is properly initialized
4. Wiring connections are secure

### Q: I'm getting too many false detections
**A:** Try:
1. Enable tracking algorithm
2. Reduce sensitivity
3. Adjust tracking parameters
4. Check for environmental interference
5. Ensure proper sensor mounting

## 📊 Performance

### Q: How much memory does the library use?
**A:** Approximately:
- Library code: ~8KB
- Runtime data: ~2KB
- Tracking algorithm: ~1KB

### Q: What's the processing latency?
**A:** Typical processing times:
- Frame parsing: < 1ms
- Track updates: < 0.1ms per track
- Complete cycle: < 2ms

### Q: How does tracking affect performance?
**A:** The tracking algorithm adds minimal overhead:
- CPU: < 5% additional usage
- Memory: ~1KB additional
- Latency: < 1ms additional

### Q: Can I use multiple radar sensors?
**A:** Yes, create multiple instances:
```cpp
AiThinker_RD_03D radar1;
AiThinker_RD_03D radar2;

radar1.begin(Serial2, 16, 17);
radar2.begin(Serial1, 18, 19);
```

## 🎯 Applications

### Q: What applications is this radar suitable for?
**A:** Common applications include:
- Human presence detection
- Occupancy sensing
- Security systems
- Gesture recognition
- Robotics navigation
- Smart home automation

### Q: Can I use this for outdoor applications?
**A:** Yes, but consider:
- Weather protection for the sensor
- Adjust tracking parameters for outdoor conditions
- Higher velocity thresholds for moving targets
- Extended detection range settings

### Q: Is this suitable for industrial use?
**A:** Yes, with considerations:
- Temperature range: -40°C to +85°C
- Humidity protection may be needed
- EMI shielding in industrial environments
- Regular calibration may be required

### Q: Can I integrate this with other sensors?
**A:** Yes, the library is designed for easy integration:
- Use with PIR sensors for confirmation
- Combine with ultrasonic sensors
- Integrate with camera systems
- Connect to IoT platforms

## 🔮 Advanced Features

### Q: Can I access raw radar data?
**A:** Yes, by disabling tracking:
```cpp
radar.setTrackingEnabled(false);
```

### Q: How do I debug radar issues?
**A:** Use debug methods:
```cpp
radar.dumpLastFrame("Debug");  // Dump frame data
Serial.print(radar.getFrameLength());  // Check frame size
```

### Q: Can I save radar data for analysis?
**A:** Yes, log data to SD card or serial:
```cpp
// Log to CSV format
Serial.print(millis());
Serial.print(",");
Serial.print(radar.getTargetCount());
Serial.print(",");
Serial.println(radar.getTargetDistance(0));
```

### Q: How do I calibrate the radar?
**A:** The radar is factory calibrated, but you can:
- Adjust sensitivity for your environment
- Fine-tune tracking parameters
- Use known distance targets for verification

## 📚 Support

### Q: Where can I find more examples?
**A:** Check:
- [Basic Examples](examples/basic-examples.md)
- [Advanced Examples](examples/advanced-examples.md)
- [PlatformIO Examples](examples/platformio-examples.md)
- GitHub repository examples folder

### Q: How do I report bugs?
**A:** Open an issue on GitHub:
1. Go to [GitHub Issues](https://github.com/omd0/Ai-Thinker-RD-03/issues)
2. Click "New Issue"
3. Provide detailed information about the problem

### Q: Can I contribute to the library?
**A:** Yes! Contributions are welcome:
1. Fork the repository
2. Make your changes
3. Submit a pull request
4. See [Contributing Guidelines](../CONTRIBUTING.md)

### Q: Where can I get help?
**A:** Try these resources:
- [Troubleshooting Guide](troubleshooting.md)
- [API Reference](api-reference.md)
- GitHub Issues
- Community forums

## 🔗 Related Links

- [Installation Guide](installation.md)
- [API Reference](api-reference.md)
- [Troubleshooting Guide](troubleshooting.md)
- [GitHub Repository](https://github.com/omd0/Ai-Thinker-RD-03)
- [Ai-Thinker Official Documentation](https://aithinker.blog.csdn.net/article/details/133338984)

---

**Still have questions?** Open an issue on GitHub or check the troubleshooting guide for more specific solutions. 