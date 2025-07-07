# Ai-Thinker RD-03D Radar Library Documentation

Welcome to the comprehensive documentation for the Ai-Thinker RD-03D 24GHz FMCW radar sensor library. This library provides advanced target detection and tracking capabilities for Arduino and PlatformIO projects.

## 📚 Documentation Sections

### Getting Started
- [Installation Guide](installation.md) - How to install and set up the library
- [Platform Support](platform-support.md) - Cross-platform compatibility and automatic detection
- [Hardware Setup](hardware-setup.md) - Platform-specific wiring and connections

### API Reference
- [Library API Reference](api-reference.md) - Complete API documentation
- [Data Structures](data-structures.md) - Target information and configuration structures
- [Configuration Options](configuration.md) - All available configuration parameters

### Advanced Features
- [Target Tracking Algorithm](tracking-algorithm.md) - Advanced ghost target filtering
- [Detection Modes](detection-modes.md) - Single vs multi-target detection
- [Error Handling](error-handling.md) - Troubleshooting and debugging

### Examples & Tutorials
- [Basic Examples](examples/basic-examples.md) - Simple usage examples
- [Advanced Examples](examples/advanced-examples.md) - Complex implementations
- [PlatformIO Examples](examples/platformio-examples.md) - PlatformIO specific examples

### Troubleshooting
- [Common Issues](troubleshooting.md) - Solutions to frequent problems
- [Performance Optimization](performance.md) - Tips for optimal performance
- [FAQ](faq.md) - Frequently asked questions

## 🚀 Quick Overview

The Ai-Thinker RD-03D is a 24GHz Frequency Modulated Continuous Wave (FMCW) radar sensor that provides:

- **Multi-target detection** up to 8 meters
- **Distance measurement** with centimeter accuracy
- **Angle detection** for target positioning
- **Velocity measurement** for moving targets
- **Advanced tracking algorithm** to eliminate ghost targets
- **Configurable sensitivity** and detection range

## 🔧 Key Features

### Target Tracking
- **Ghost Target Elimination**: Advanced filtering removes false detections
- **Track Confirmation**: Only reports consistently detected targets
- **Data Smoothing**: Stable measurements using exponential moving average
- **Configurable Parameters**: Customizable distance and velocity thresholds

### Detection Capabilities
- **Range**: 20cm to 800cm (configurable)
- **Accuracy**: ±2cm distance, ±1° angle
- **Update Rate**: Up to 50Hz
- **Targets**: Up to 3 simultaneous targets

### Hardware Support
- **ESP32**: Full support with multiple UARTs and custom pins
- **ESP8266**: Hardware + software serial support
- **Arduino AVR**: Uno/Nano/Mega with automatic serial selection
- **STM32**: Multiple hardware UARTs with pin mapping
- **SAMD**: Arduino Zero/MKR board support
- **Cross-Platform**: Automatic platform detection and configuration

## 📋 Requirements

- **Hardware**: Any supported Arduino-compatible board (ESP32, ESP8266, Arduino Uno/Nano/Mega, STM32, SAMD)
- **Software**: Arduino IDE 1.8+ or PlatformIO
- **Library**: This radar library with automatic platform detection
- **Connections**: UART communication (hardware or software serial)

## 🎯 Use Cases

- **Presence Detection**: Detect human presence in rooms
- **Gesture Recognition**: Track hand movements
- **Security Systems**: Intrusion detection
- **Smart Home**: Occupancy sensing
- **Robotics**: Obstacle avoidance and navigation
- **IoT Applications**: Connected sensor networks

## 📖 Getting Started

1. **Install the Library**: See [Installation Guide](installation.md)
2. **Check Platform Support**: Review [Platform Support](platform-support.md) for your board
3. **Connect Hardware**: Follow [Hardware Setup](hardware-setup.md) for your specific platform
4. **Run Examples**: Try the [Basic Examples](examples/basic-examples.md)
5. **Configure Tracking**: Learn about [Target Tracking](tracking-algorithm.md)

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](../CONTRIBUTING.md) for details.

## 📄 License

This library is released under the LGPL-2.1 license. See [LICENSE](../LICENSE) for details.

## 🔗 Links

- [GitHub Repository](https://github.com/omd0/Ai-Thinker-RD-03)
- [Ai-Thinker Official Documentation](https://aithinker.blog.csdn.net/article/details/133338984)
- [Issue Tracker](https://github.com/omd0/Ai-Thinker-RD-03/issues)

---

**Need Help?** Check our [FAQ](faq.md) or [Troubleshooting Guide](troubleshooting.md) for common solutions. 