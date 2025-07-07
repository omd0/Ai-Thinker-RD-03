# Installation Guide

This guide will help you install and set up the Ai-Thinker RD-03D radar library for your Arduino or PlatformIO project.

## 📋 Prerequisites

Before installing the library, ensure you have:

- **Arduino IDE 1.8+** or **PlatformIO**
- **ESP32 development board** (tested with ESP32 DevKit)
- **Ai-Thinker RD-03D radar sensor**
- **USB cable** for programming
- **Jumper wires** for connections

## 🔧 Installation Methods

### Method 1: Arduino IDE (Recommended for Beginners)

#### Step 1: Download the Library
1. Download the library as a ZIP file from GitHub:
   - Go to [https://github.com/omd0/Ai-Thinker-RD-03](https://github.com/omd0/Ai-Thinker-RD-03)
   - Click the green "Code" button
   - Select "Download ZIP"

#### Step 2: Install in Arduino IDE
1. Open Arduino IDE
2. Go to **Sketch** → **Include Library** → **Add .ZIP Library**
3. Select the downloaded ZIP file
4. Click "Open"

#### Step 3: Verify Installation
1. Go to **Sketch** → **Include Library**
2. Look for "Ai-Thinker-RD-03" in the list
3. If present, installation was successful

### Method 2: PlatformIO (Recommended for Advanced Users)

#### Step 1: Create PlatformIO Project
```bash
# Create new project
pio project init --board esp32dev

# Navigate to project directory
cd your-project-name
```

#### Step 2: Add Library Dependency
Edit your `platformio.ini` file:
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

# Add the radar library
lib_deps = 
    https://github.com/omd0/Ai-Thinker-RD-03.git
```

#### Step 3: Install Dependencies
```bash
pio lib install
```

### Method 3: Manual Installation

#### Step 1: Clone Repository
```bash
git clone https://github.com/omd0/Ai-Thinker-RD-03.git
```

#### Step 2: Copy to Libraries Folder
**Arduino IDE:**
```bash
# On Windows
copy Ai-Thinker-RD-03 C:\Users\YourName\Documents\Arduino\libraries\

# On macOS
cp -r Ai-Thinker-RD-03 ~/Documents/Arduino/libraries/

# On Linux
cp -r Ai-Thinker-RD-03 ~/Arduino/libraries/
```

**PlatformIO:**
```bash
# Copy to PlatformIO libraries folder
cp -r Ai-Thinker-RD-03 ~/.platformio/lib/
```

## 🔌 Hardware Setup

### Required Connections

| RD-03D Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VCC        | 3.3V      | Power supply |
| GND        | GND       | Ground |
| TX         | GPIO 16   | Radar data output |
| RX         | GPIO 17   | Radar command input |

### Wiring Diagram

```
ESP32 DevKit          RD-03D Radar
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

## 🚀 Quick Test

### Arduino IDE Test
1. Open Arduino IDE
2. Go to **File** → **Examples** → **Ai-Thinker-RD-03** → **RD-03D_Basic_Example**
3. Select your ESP32 board and port
4. Click "Upload"
5. Open Serial Monitor (115200 baud)
6. You should see radar data output

### PlatformIO Test
1. Copy the example from `examples/PlatformIO/src/main.cpp`
2. Build and upload:
```bash
pio run --target upload
pio device monitor
```

## ⚙️ Configuration

### Basic Configuration
```cpp
#include "Ai-Thinker-RD-03.h"

AiThinker_RD_03D radar;

void setup() {
    Serial.begin(115200);
    
    // Initialize radar
    if (radar.begin(Serial2, 16, 17)) {
        Serial.println("Radar initialized successfully");
    }
    
    // Configure tracking (optional)
    radar.setMaxTrackDistance(500);  // 5 meters max
    radar.setMaxTrackVelocity(25);   // 25 cm/s max
    radar.setTrackingEnabled(true);  // Enable tracking
}
```

### Advanced Configuration
```cpp
// Set detection range
radar.setDetectionRange(20, 800);  // 20cm to 800cm

// Set sensitivity (0-255)
radar.setSensitivity(128);

// Set output format
radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY);

// Set detection mode
radar.setMultiTargetMode();  // or setSingleTargetMode()
```

## 🔍 Verification

### Check Library Installation
```cpp
#include "Ai-Thinker-RD-03.h"

void setup() {
    Serial.begin(115200);
    Serial.print("Library version: ");
    Serial.println(AI_THINKER_RD_03D_LIB_VERSION);
}
```

### Test Hardware Connection
```cpp
void setup() {
    Serial.begin(115200);
    
    if (radar.begin(Serial2, 16, 17)) {
        Serial.println("✓ Hardware connection OK");
    } else {
        Serial.println("✗ Hardware connection failed");
    }
}
```

## 🐛 Troubleshooting

### Common Issues

**Library not found:**
- Ensure the library is in the correct Arduino libraries folder
- Restart Arduino IDE after installation

**Compilation errors:**
- Make sure you're using an ESP32 board
- Check that all required libraries are installed

**No radar data:**
- Verify wiring connections
- Check baud rate (should be 256000)
- Ensure radar sensor is powered

**Ghost targets:**
- Enable tracking algorithm: `radar.setTrackingEnabled(true)`
- Adjust tracking parameters for your environment

## 📚 Next Steps

After successful installation:

1. **Read the [Quick Start Guide](quick-start.md)** for basic usage
2. **Check [Hardware Setup](hardware-setup.md)** for detailed wiring
3. **Explore [Examples](examples/basic-examples.md)** for sample code
4. **Learn about [Target Tracking](tracking-algorithm.md)** for advanced features

## 🆘 Need Help?

- Check the [FAQ](faq.md) for common questions
- Review [Troubleshooting Guide](troubleshooting.md) for solutions
- Open an issue on [GitHub](https://github.com/omd0/Ai-Thinker-RD-03/issues) 