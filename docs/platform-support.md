# Platform Support

The Ai-Thinker RD-03D library provides automatic platform detection and cross-platform compatibility, making it easy to use the same code across different Arduino-compatible boards.

## 🌐 Supported Platforms

### ✅ **ESP32**
- **Detection**: `ARDUINO_ARCH_ESP32`
- **Serial Support**: Hardware Serial (multiple UARTs)
- **Features**: 
  - Custom pin assignment
  - Multiple hardware serial ports (Serial, Serial1, Serial2)
  - High baud rate support (256000)
  - Buffer size configuration
- **Recommended Setup**: Serial2 with GPIO 16/17

### ✅ **ESP8266**
- **Detection**: `ARDUINO_ARCH_ESP8266` 
- **Serial Support**: Hardware Serial + SoftwareSerial
- **Features**:
  - Custom pin assignment for hardware serial
  - SoftwareSerial fallback option
  - Limited to one hardware UART
- **Recommended Setup**: Hardware Serial with GPIO 13/15

### ✅ **Arduino AVR (Uno, Nano, Pro Mini)**
- **Detection**: `ARDUINO_ARCH_AVR` + `__AVR_ATmega328P__`
- **Serial Support**: Hardware Serial + SoftwareSerial
- **Features**:
  - Automatic SoftwareSerial selection for Uno/Nano
  - Preserves Serial for debugging
  - 256000 baud support varies by board
- **Recommended Setup**: SoftwareSerial on pins 2/3

### ✅ **Arduino Mega**
- **Detection**: `__AVR_ATmega2560__` or `__AVR_ATmega1280__`
- **Serial Support**: Multiple Hardware Serial ports
- **Features**:
  - Serial1, Serial2, Serial3 available
  - True hardware serial performance
  - Dedicated pins for each UART
- **Recommended Setup**: Serial1 (pins 18/19)

### ✅ **STM32**
- **Detection**: `ARDUINO_ARCH_STM32`
- **Serial Support**: Hardware Serial (multiple UARTs)
- **Features**:
  - Multiple hardware UARTs
  - High performance
  - Configurable pin mapping
- **Recommended Setup**: Serial1 with PA9/PA10

### ✅ **SAMD (Arduino Zero, MKR)**
- **Detection**: `ARDUINO_ARCH_SAMD`
- **Serial Support**: Hardware Serial
- **Features**:
  - Native USB + hardware UART
  - SerialUSB for debugging
  - Serial1 for sensor communication
- **Recommended Setup**: Serial1

### ⚠️ **Generic/Other Platforms**
- **Detection**: Fallback for unrecognized platforms
- **Serial Support**: Assumes hardware serial available
- **Features**: Basic functionality with default settings
- **Setup**: Manual configuration required

## 🔧 Automatic Platform Detection

The library automatically detects your platform at compile time and configures the optimal serial interface:

```cpp
// This code works on ALL supported platforms!
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    
    // Platform information (automatic detection)
    Serial.print("Platform: ");
    Serial.println(AiThinker_RD_03D::getPlatformName());
    Serial.print("Hardware Serial: ");
    Serial.println(AiThinker_RD_03D::hasHardwareSerial() ? "Yes" : "No");
    Serial.print("Software Serial: ");
    Serial.println(AiThinker_RD_03D::hasSoftwareSerial() ? "Yes" : "No");
    
    // The library automatically chooses the best initialization method
    // based on your platform!
}
```

## 📋 Platform-Specific Wiring

### ESP32
```
RD-03D    ESP32
------    -----
VCC   →   3.3V
GND   →   GND
TX    →   GPIO 16 (RX)
RX    →   GPIO 17 (TX)
```

**Code:**
```cpp
radar.begin(Serial2, 16, 17);
```

### ESP8266
```
RD-03D    ESP8266
------    -------
VCC   →   3.3V
GND   →   GND
TX    →   GPIO 13 (RX)
RX    →   GPIO 15 (TX)
```

**Code:**
```cpp
radar.begin(Serial, 13, 15);
```

### Arduino Uno/Nano
```
RD-03D    Arduino
------    -------
VCC   →   3.3V
GND   →   GND
TX    →   Pin 2 (SoftwareSerial RX)
RX    →   Pin 3 (SoftwareSerial TX)
```

**Code:**
```cpp
#include <SoftwareSerial.h>
SoftwareSerial radarSerial(2, 3);
radar.begin(radarSerial);
```

### Arduino Mega
```
RD-03D    Mega
------    ----
VCC   →   3.3V
GND   →   GND
TX    →   Pin 19 (Serial1 RX)
RX    →   Pin 18 (Serial1 TX)
```

**Code:**
```cpp
radar.begin(Serial1, 19, 18);
```

### STM32
```
RD-03D    STM32
------    -----
VCC   →   3.3V
GND   →   GND
TX    →   PA10 (USART1_RX)
RX    →   PA9  (USART1_TX)
```

**Code:**
```cpp
radar.begin(Serial1, PA10, PA9);
```

## 🚀 Universal Code Example

This example works on **ALL** supported platforms without modification:

```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

// Platform-specific configuration (happens automatically)
#if defined(ARDUINO_ARCH_ESP32)
  #define RADAR_SERIAL Serial2
  #define RX_PIN 16
  #define TX_PIN 17
#elif defined(ARDUINO_ARCH_ESP8266)
  #define RADAR_SERIAL Serial  
  #define RX_PIN 13
  #define TX_PIN 15
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  #define RADAR_SERIAL Serial1
  #define RX_PIN 19
  #define TX_PIN 18
#elif defined(ARDUINO_ARCH_AVR)
  #include <SoftwareSerial.h>
  SoftwareSerial radarSerial(2, 3);
  #define RADAR_SERIAL radarSerial
#elif defined(ARDUINO_ARCH_STM32)
  #define RADAR_SERIAL Serial1
  #define RX_PIN PA10
  #define TX_PIN PA9
#else
  #define RADAR_SERIAL Serial1
  #define RX_PIN -1
  #define TX_PIN -1
#endif

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Universal RD-03D Example");
    Serial.println("========================");
    
    // Display platform information
    Serial.print("Platform: ");
    Serial.println(AiThinker_RD_03D::getPlatformName());
    
    // Initialize based on platform capabilities
#if defined(ARDUINO_ARCH_AVR) && !defined(__AVR_ATmega2560__) && !defined(__AVR_ATmega1280__)
    // Arduino Uno/Nano - use SoftwareSerial
    if (radar.begin(RADAR_SERIAL)) {
        Serial.println("✓ Initialized with SoftwareSerial");
    }
#else
    // All other platforms - use HardwareSerial
    if (radar.begin(RADAR_SERIAL, RX_PIN, TX_PIN)) {
        Serial.println("✓ Initialized with HardwareSerial");
    }
#endif
    else {
        Serial.println("✗ Failed to initialize radar");
        while(1);
    }
    
    // Set single target mode
    radar.setSingleTargetMode();
    Serial.println("✓ Single target mode activated");
    
    Serial.println("\nStarting detection...");
}

void loop() {
    radar.read();
    
    if (radar.frameReady()) {
        uint8_t targets = radar.getTargetCount();
        
        if (targets > 0) {
            Serial.print("Target detected: ");
            Serial.print(radar.getTargetDistance(0));
            Serial.print("cm, ");
            Serial.print(radar.getTargetAngle(0));
            Serial.print("°, ");
            Serial.print(radar.getTargetVelocity(0));
            Serial.println(" cm/s");
        }
    }
    
    delay(100);
}
```

## ⚡ Performance Considerations

### Hardware Serial vs Software Serial

| Platform | Serial Type | Max Baud | Reliability | CPU Usage |
|----------|-------------|----------|-------------|-----------|
| ESP32 | Hardware | 256000+ | Excellent | Low |
| ESP8266 | Hardware | 256000+ | Excellent | Low |
| Mega | Hardware | 256000+ | Excellent | Low |
| Uno/Nano | Software | 38400* | Good | Medium |
| STM32 | Hardware | 256000+ | Excellent | Low |

**Note:** Arduino Uno/Nano may experience issues at 256000 baud with SoftwareSerial. The library automatically handles this limitation.

### Memory Usage

| Platform | RAM Available | Library Usage | Buffer Size |
|----------|---------------|---------------|-------------|
| ESP32 | 320KB+ | ~4KB | 2048 bytes |
| ESP8266 | 80KB | ~4KB | 2048 bytes |
| Mega | 8KB | ~4KB | 1024 bytes |
| Uno/Nano | 2KB | ~3KB | 512 bytes |
| STM32 | 20KB+ | ~4KB | 2048 bytes |

The library automatically adjusts buffer sizes based on available memory.

## 🔧 Troubleshooting Platform Issues

### ESP32/ESP8266
- **Issue**: Serial pins not working
- **Solution**: Check GPIO pin capabilities, some pins are input-only
- **Alternative**: Try different pin combinations

### Arduino Uno/Nano
- **Issue**: Missed data frames with SoftwareSerial
- **Solution**: Reduce baud rate or use interrupt-driven reading
- **Alternative**: Use hardware serial (lose debug output)

### Arduino Mega
- **Issue**: Wrong serial port selected
- **Solution**: Verify pin connections match selected Serial port
- **Alternative**: Use different Serial port (Serial1-3)

### STM32
- **Issue**: Compilation errors
- **Solution**: Ensure STM32 Arduino core is properly installed
- **Alternative**: Use generic HardwareSerial initialization

### Generic Platforms
- **Issue**: Platform not detected correctly
- **Solution**: Manually configure serial interface
- **Example**: 
```cpp
// Force hardware serial configuration
#define PLATFORM_HAS_HARDWARE_SERIAL
radar.begin(Serial1, rxPin, txPin);
```

## 📚 Additional Resources

- [API Reference](api-reference.md) - Complete API documentation
- [Hardware Setup](hardware-setup.md) - Detailed wiring instructions  
- [Installation Guide](installation.md) - Platform-specific installation
- [Troubleshooting](troubleshooting.md) - Common issues and solutions
- [Examples](examples/) - Platform-specific example code

## 🤝 Contributing Platform Support

To add support for a new platform:

1. **Add platform detection** in `Ai-Thinker-RD-03.h`:
```cpp
#elif defined(YOUR_PLATFORM_MACRO)
    #define PLATFORM_HAS_HARDWARE_SERIAL
    #define PLATFORM_NAME "YourPlatform"
```

2. **Add platform-specific initialization** in `Ai-Thinker-RD-03.cpp`:
```cpp
#elif defined(YOUR_PLATFORM_MACRO)
    // Platform-specific serial setup
    rSerial.begin(256000);
```

3. **Test with example code** and submit a pull request

4. **Update documentation** with platform-specific wiring and usage notes 