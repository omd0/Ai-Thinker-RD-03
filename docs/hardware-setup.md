# Hardware Setup Guide

Complete guide for setting up the Ai-Thinker RD-03D radar sensor with any supported Arduino-compatible platform. The library automatically detects your platform and configures the optimal connection method.

## 📋 Required Components

### Essential Components
- **Arduino-Compatible Board** (ESP32, ESP8266, Arduino Uno/Nano/Mega, STM32, etc.)
- **Ai-Thinker RD-03D Radar Sensor**
- **Jumper Wires** (4 pieces minimum)
- **USB Cable** (for programming)
- **Breadboard** (optional, for prototyping)

### Platform-Specific Requirements

#### ESP32/ESP8266
- **Hardware Serial**: Multiple UARTs available
- **Pin Flexibility**: Custom pin assignment supported
- **Power**: 3.3V compatible

#### Arduino Uno/Nano
- **SoftwareSerial**: Required for sensor communication
- **Pin Limitation**: Uses digital pins 2/3 by default
- **Power**: 3.3V level shifter may be needed

#### Arduino Mega
- **Hardware Serial**: Serial1-3 available
- **Pin Options**: Multiple UART pin choices
- **Power**: 3.3V compatible

#### STM32
- **Hardware Serial**: Multiple UARTs available
- **Pin Mapping**: Platform-specific pin assignments
- **Power**: 3.3V compatible

### Optional Components
- **3.3V Power Supply** (if not using USB power)
- **Protective Case** (for outdoor use)
- **Mounting Hardware** (screws, brackets)
- **Extension Cables** (for remote mounting)

## 🔌 Platform-Specific Pin Connections

The library automatically detects your platform and configures the optimal connection method. Choose the wiring diagram for your specific board:

### ESP32 Wiring

```
ESP32 DevKit          RD-03D Radar Sensor
┌─────────────┐      ┌─────────────┐
│             │      │             │
│ 3.3V ──────┼──────┤ VCC         │
│             │      │             │
│ GND ───────┼──────┤ GND         │
│             │      │             │
│ GPIO 16 ───┼──────┤ TX          │
│             │      │             │
│ GPIO 17 ───┼──────┤ RX          │
│             │      │             │
└─────────────┘      └─────────────┘
```

**Code:** `radar.begin(Serial2, 16, 17);`

### ESP8266 Wiring

```
ESP8266 NodeMCU       RD-03D Radar Sensor
┌─────────────┐      ┌─────────────┐
│             │      │             │
│ 3.3V ──────┼──────┤ VCC         │
│             │      │             │
│ GND ───────┼──────┤ GND         │
│             │      │             │
│ GPIO 13 ───┼──────┤ TX          │
│             │      │             │
│ GPIO 15 ───┼──────┤ RX          │
│             │      │             │
└─────────────┘      └─────────────┘
```

**Code:** `radar.begin(Serial, 13, 15);`

### Arduino Uno/Nano Wiring

```
Arduino Uno           RD-03D Radar Sensor
┌─────────────┐      ┌─────────────┐
│             │      │             │
│ 3.3V ──────┼──────┤ VCC         │
│             │      │             │
│ GND ───────┼──────┤ GND         │
│             │      │             │
│ Pin 2 ─────┼──────┤ TX          │
│             │      │             │
│ Pin 3 ─────┼──────┤ RX          │
│             │      │             │
└─────────────┘      └─────────────┘
```

**Code:** 
```cpp
#include <SoftwareSerial.h>
SoftwareSerial radarSerial(2, 3);
radar.begin(radarSerial);
```

### Arduino Mega Wiring

```
Arduino Mega          RD-03D Radar Sensor
┌─────────────┐      ┌─────────────┐
│             │      │             │
│ 3.3V ──────┼──────┤ VCC         │
│             │      │             │
│ GND ───────┼──────┤ GND         │
│             │      │             │
│ Pin 19 ────┼──────┤ TX          │
│             │      │             │
│ Pin 18 ────┼──────┤ RX          │
│             │      │             │
└─────────────┘      └─────────────┘
```

**Code:** `radar.begin(Serial1, 19, 18);`

### STM32 Wiring

```
STM32 Board           RD-03D Radar Sensor
┌─────────────┐      ┌─────────────┐
│             │      │             │
│ 3.3V ──────┼──────┤ VCC         │
│             │      │             │
│ GND ───────┼──────┤ GND         │
│             │      │             │
│ PA10 ──────┼──────┤ TX          │
│             │      │             │
│ PA9 ───────┼──────┤ RX          │
│             │      │             │
└─────────────┘      └─────────────┘
```

**Code:** `radar.begin(Serial1, PA10, PA9);`

### Universal Pin Mapping Table

| Platform | VCC | GND | RX Pin | TX Pin | Serial Interface | Code Example |
|----------|-----|-----|--------|--------|------------------|--------------|
| ESP32 | 3.3V | GND | GPIO 16 | GPIO 17 | Serial2 | `radar.begin(Serial2, 16, 17)` |
| ESP8266 | 3.3V | GND | GPIO 13 | GPIO 15 | Serial | `radar.begin(Serial, 13, 15)` |
| Uno/Nano | 3.3V | GND | Pin 2 | Pin 3 | SoftwareSerial | `radar.begin(radarSerial)` |
| Mega | 3.3V | GND | Pin 19 | Pin 18 | Serial1 | `radar.begin(Serial1, 19, 18)` |
| STM32 | 3.3V | GND | PA10 | PA9 | Serial1 | `radar.begin(Serial1, PA10, PA9)` |

### Alternative Pin Configurations

#### Using Different GPIO Pins
You can use different GPIO pins by modifying the code:

```cpp
// Example with different pins
radar.begin(Serial2, 18, 19);  // GPIO 18, 19
radar.begin(Serial1, 21, 22);  // GPIO 21, 22
```

#### Using Different Serial Ports
```cpp
// Using Serial1 instead of Serial2
radar.begin(Serial1, 16, 17);

// Using Serial with software serial (not recommended)
// Requires additional library
```

## 🔧 Step-by-Step Setup

### Step 1: Prepare Components
1. **Inspect the radar sensor**
   - Check for physical damage
   - Verify all pins are straight
   - Ensure no debris on the sensor face

2. **Prepare ESP32**
   - Insert into breadboard (if using)
   - Ensure stable mounting
   - Check USB connection

### Step 2: Power Connections
1. **Connect VCC**
   - Connect ESP32 3.3V to RD-03D VCC
   - Use red wire for easy identification
   - Ensure secure connection

2. **Connect Ground**
   - Connect ESP32 GND to RD-03D GND
   - Use black wire for easy identification
   - Double-check connection

### Step 3: Data Connections
1. **Connect TX (Radar → ESP32)**
   - Connect RD-03D TX to ESP32 GPIO 16
   - Use green wire for data transmission
   - Ensure proper orientation

2. **Connect RX (ESP32 → Radar)**
   - Connect ESP32 GPIO 17 to RD-03D RX
   - Use blue wire for commands
   - Verify connection

### Step 4: Verify Connections
1. **Visual Inspection**
   - Check all wires are secure
   - Ensure no loose connections
   - Verify correct pin mapping

2. **Continuity Test** (optional)
   - Use multimeter to test connections
   - Verify no short circuits
   - Check for proper resistance

## 📐 Mounting Considerations

### Sensor Orientation
- **Face Forward**: Ensure radar sensor faces the detection area
- **Level Mounting**: Mount sensor horizontally for best performance
- **Clear Line of Sight**: Avoid obstructions in front of sensor

### Height and Angle
- **Recommended Height**: 1.5-2.5 meters above ground
- **Detection Angle**: ±60° horizontal, ±30° vertical
- **Mounting Angle**: 0° (horizontal) for general use

### Environmental Factors
- **Temperature**: -40°C to +85°C operating range
- **Humidity**: Protect from direct moisture
- **Interference**: Keep away from metal objects and EMI sources

## 🔍 Testing the Setup

### Universal Connection Test
```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Testing radar connections...");
    Serial.print("Platform: ");
    Serial.println(AiThinker_RD_03D::getPlatformName());
    
    bool success = false;
    
    // Platform-specific initialization
    #if defined(ARDUINO_ARCH_ESP32)
        success = radar.begin(Serial2, 16, 17);
    #elif defined(ARDUINO_ARCH_ESP8266)
        success = radar.begin(Serial, 13, 15);
    #elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        success = radar.begin(Serial1, 19, 18);
    #elif defined(ARDUINO_ARCH_AVR)
        #include <SoftwareSerial.h>
        SoftwareSerial radarSerial(2, 3);
        success = radar.begin(radarSerial);
    #elif defined(ARDUINO_ARCH_STM32)
        success = radar.begin(Serial1, PA10, PA9);
    #else
        success = radar.begin(Serial1, -1, -1);
    #endif
    
    if (success) {
        Serial.println("✅ Radar connected successfully");
    } else {
        Serial.println("❌ Radar connection failed");
        Serial.println("Check wiring and power supply");
    }
}

void loop() {
    // Simple data test
    if (radar.read() > 0) {
        Serial.println("✅ Data received from radar");
    } else {
        Serial.println("❌ No data from radar");
    }
    delay(1000);
}
```

### Expected Results
- **Success**: "Radar connected successfully" + periodic data messages
- **Failure**: "Radar connection failed" or "No data from radar"

## 🐛 Troubleshooting

### Common Connection Issues

#### No Power to Radar
**Symptoms:**
- Radar sensor not responding
- No data received

**Solutions:**
1. Check 3.3V connection
2. Verify power supply stability
3. Test with multimeter
4. Try different power source

#### No Data Reception
**Symptoms:**
- "No data from radar" messages
- Zero target detections

**Solutions:**
1. Check TX/RX connections
2. Verify GPIO pin assignments
3. Test with different pins
4. Check for loose connections

#### Erratic Data
**Symptoms:**
- Inconsistent readings
- Frequent disconnections

**Solutions:**
1. Check wire quality
2. Reduce wire length
3. Add ferrite beads
4. Check for interference

### Advanced Troubleshooting

#### Using Oscilloscope
1. **Check Power Supply**
   - Measure 3.3V at radar VCC pin
   - Verify stable voltage (no ripple)

2. **Check Data Lines**
   - Monitor TX line for data pulses
   - Check RX line for command signals
   - Verify proper signal levels

#### Using Logic Analyzer
1. **Capture UART Signals**
   - Monitor TX/RX communication
   - Verify baud rate (256000)
   - Check data format

2. **Analyze Frame Structure**
   - Verify frame headers
   - Check data integrity
   - Identify timing issues

## 🔧 Advanced Configurations

### Multiple Radar Sensors
```cpp
AiThinker_RD_03D radar1;
AiThinker_RD_03D radar2;

void setup() {
    // First radar on Serial2
    radar1.begin(Serial2, 16, 17);
    
    // Second radar on Serial1
    radar2.begin(Serial1, 18, 19);
}
```

### Remote Mounting
For long-distance connections:
- Use shielded cables
- Add signal amplifiers if needed
- Consider RS485 conversion for long runs
- Implement error checking

### Power Supply Options

#### USB Power (Recommended)
- **Pros**: Simple, stable, regulated
- **Cons**: Limited to USB cable length
- **Use Case**: Development and testing

#### External 3.3V Supply
- **Pros**: Independent power, longer runs
- **Cons**: Additional components needed
- **Use Case**: Production installations

#### Battery Power
- **Pros**: Portable, wireless
- **Cons**: Limited runtime, voltage regulation
- **Use Case**: Mobile applications

## 📊 Performance Optimization

### Wire Quality
- **Use stranded wire** for flexibility
- **Keep wires short** to minimize interference
- **Use twisted pairs** for data lines
- **Add shielding** for noisy environments

### Power Supply Quality
- **Use regulated 3.3V supply**
- **Add decoupling capacitors**
- **Ensure adequate current capacity**
- **Minimize voltage ripple**

### Environmental Considerations
- **Avoid metal objects** near sensor
- **Minimize electromagnetic interference**
- **Ensure proper ventilation**
- **Protect from moisture**

## 🔒 Safety Considerations

### Electrical Safety
- **Never exceed 3.3V** on radar sensor
- **Use proper grounding**
- **Avoid short circuits**
- **Disconnect power before modifications**

### Physical Safety
- **Secure mounting** to prevent falls
- **Protect from impact**
- **Ensure proper ventilation**
- **Follow local regulations**

### RF Safety
- **24GHz radiation** is non-ionizing
- **Power levels are very low**
- **Safe for continuous operation**
- **Follow manufacturer guidelines**

## 📚 Additional Resources

### Datasheets
- [Ai-Thinker RD-03D Datasheet](https://aithinker.blog.csdn.net/article/details/133338984)
- [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

### Tools
- **Multimeter**: For voltage and continuity testing
- **Oscilloscope**: For signal analysis
- **Logic Analyzer**: For digital signal debugging
- **Power Supply**: For stable 3.3V testing

### Community Support
- [GitHub Issues](https://github.com/omd0/Ai-Thinker-RD-03/issues)
- [ESP32 Forums](https://esp32.com/)
- [Arduino Forums](https://forum.arduino.cc/)

---

**Need Help?** If you're still having issues after following this guide, check the [Troubleshooting Guide](troubleshooting.md) or open an issue on GitHub. 