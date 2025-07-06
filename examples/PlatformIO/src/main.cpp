/*
    Name:       RD-03 PlatformIO Example
    Created:    2024
    Author:     PlatformIO Example
    Description: Example for Ai-Thinker RD-03 24Ghz FMCW radar sensor
*/

#include <Arduino.h>
#include <Ai-Thinker-RD-03.h>

// Pin definitions
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17
#define BUILTIN_LED_PIN 2

// Serial definitions
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1

// Radar instance
AiThinker_RD_03 radar;

// Timing variables
unsigned int last_frame_ts = 0;
unsigned long loopStart;

// Target information
char target[10] = "?";
int range = -1;

// Mode control
AiThinker_RD_03::RadarMode modes[] = { 
    AiThinker_RD_03::DEBUGGING_MODE, 
    AiThinker_RD_03::REPORTING_MODE, 
    AiThinker_RD_03::OPERATING_MODE 
};
int currentMode = 2; // Start with OPERATING_MODE

void setup() {
    // Initialize serial communication
    MONITOR_SERIAL.begin(115200);
    RADAR_SERIAL.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    
    // Initialize built-in LED
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    
    MONITOR_SERIAL.println("RD-03 PlatformIO Example Starting...");
    delay(2000);

    // Initialize radar
    if (radar.begin(RADAR_SERIAL, RADAR_RX_PIN, RADAR_TX_PIN)) {
        MONITOR_SERIAL.println("Radar initialized successfully");
    } else {
        MONITOR_SERIAL.println("Failed to initialize radar");
        while(1) {
            digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
            delay(500);
        }
    }

    // Configure radar settings
    radar.setFrameTimeOut(200);
    radar.setInterCommandDelay(100);

    // Get firmware information
    const char* firmwareVersion = radar.getFirmwareVersion();
    uint16_t bufferSize, protocolVersion = radar.getProtocolVersion(bufferSize);
    
    MONITOR_SERIAL.println("\n=== RD-03 Radar Sensor Information ===");
    MONITOR_SERIAL.print("Firmware: ");
    MONITOR_SERIAL.println(firmwareVersion);
    
    if (protocolVersion) {
        MONITOR_SERIAL.print("Protocol Version: ");
        MONITOR_SERIAL.print(protocolVersion);
        MONITOR_SERIAL.print(", Buffer Size: ");
        MONITOR_SERIAL.println(bufferSize);
    }

    // Set operating mode
    MONITOR_SERIAL.print("Setting mode to OPERATING_MODE... ");
    if (radar.setSystemMode(AiThinker_RD_03::OPERATING_MODE)) {
        MONITOR_SERIAL.println("SUCCESS");
    } else {
        MONITOR_SERIAL.println("FAILED");
    }
    delay(500);

    // Exit config mode if needed
    if (radar.isInConfigMode()) {
        MONITOR_SERIAL.println("Exiting config mode...");
        radar.disableConfigMode();
    }

    // Display current mode
    switch (radar.getSystemMode()) {
        case AiThinker_RD_03::OPERATING_MODE:
            MONITOR_SERIAL.println("Current Mode: OPERATING_MODE");
            break;
        case AiThinker_RD_03::REPORTING_MODE:
            MONITOR_SERIAL.println("Current Mode: REPORTING_MODE");
            break;
        case AiThinker_RD_03::DEBUGGING_MODE:
            MONITOR_SERIAL.println("Current Mode: DEBUGGING_MODE");
            break;
        default:
            MONITOR_SERIAL.print("Current Mode: UNKNOWN (0x");
            MONITOR_SERIAL.print(radar.getSystemMode(), HEX);
            MONITOR_SERIAL.println(")");
            break;
    }

    MONITOR_SERIAL.println("\n=== Starting Main Loop ===");
    loopStart = last_frame_ts = millis();
}

void loop() {
    // Read radar data
    radar.read();

    // Process based on current mode
    if (radar.getSystemMode() == AiThinker_RD_03::OPERATING_MODE) {
        // Operating mode - get target information
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME) &&
            last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME)) {
            
            last_frame_ts = max(radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME), 
                               radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME));

            // Parse target information
            if (!sscanf(radar.lastFrame(AiThinker_RD_03::CURRENT_FRAME), "Range%d", &range)) {
                sscanf(radar.lastFrame(AiThinker_RD_03::CURRENT_FRAME), "%4s", target);
            }
            if (!sscanf(radar.lastFrame(AiThinker_RD_03::LAST_FRAME), "Range%d", &range)) {
                sscanf(radar.lastFrame(AiThinker_RD_03::LAST_FRAME), "%4s", target);
            }

            // Display target information
            MONITOR_SERIAL.printf("Target: %s, Range: %d cm", target, range);
            
            // Control LED based on target presence
            if (strcmp(target, "None") != 0 && range > 0) {
                digitalWrite(BUILTIN_LED_PIN, HIGH);
                MONITOR_SERIAL.println(" - LED ON");
            } else {
                digitalWrite(BUILTIN_LED_PIN, LOW);
                MONITOR_SERIAL.println(" - LED OFF");
            }
        }
    }
    else if (radar.getSystemMode() == AiThinker_RD_03::REPORTING_MODE) {
        // Reporting mode - energy data
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME)) {
            last_frame_ts = radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME);
            
            MONITOR_SERIAL.printf("Target: %d, Distance: %d cm", 
                                 radar.target(), radar.targetDistance());
            
            // Display energy data for first few distance gates
            for (int i = 0; i < 8; i++) {
                MONITOR_SERIAL.printf(", E%d:%d", i, radar.energyAtDistance(i));
            }
            MONITOR_SERIAL.println();
        }
    }
    else if (radar.getSystemMode() == AiThinker_RD_03::DEBUGGING_MODE) {
        // Debugging mode - doppler data visualization
        if (last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME) &&
            last_frame_ts < radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME)) {
            
            last_frame_ts = max(radar.frameStartMillis(AiThinker_RD_03::LAST_FRAME), 
                               radar.frameStartMillis(AiThinker_RD_03::CURRENT_FRAME));

            if (radar.frameLength() == 1288) {
                MONITOR_SERIAL.println("\n=== Doppler Data Visualization ===");
                for (int i = 0; i < 10; i++) {  // Show first 10 time slots
                    for (int j = 0; j < 16; j++) {  // Show all 16 distance gates
                        if (radar.doppler(i, j) > 100000UL)
                            MONITOR_SERIAL.print('#');
                        else if (radar.doppler(i, j) > 10000UL)
                            MONITOR_SERIAL.print('+');
                        else if (radar.doppler(i, j) > 1000UL)
                            MONITOR_SERIAL.print('.');
                        else
                            MONITOR_SERIAL.print(' ');
                    }
                    MONITOR_SERIAL.println();
                }
                MONITOR_SERIAL.println();
            } else {
                MONITOR_SERIAL.printf("Debug frame length: %d\n", radar.frameLength());
            }
        }
    }

    // Mode switching every 10 seconds (for demonstration)
    static unsigned long lastModeChange = 0;
    if (millis() - lastModeChange > 10000) {
        currentMode = (currentMode + 1) % 3;
        MONITOR_SERIAL.printf("\n=== Switching to Mode %d ===\n", currentMode);
        
        if (radar.setSystemMode(modes[currentMode])) {
            MONITOR_SERIAL.println("Mode change successful");
        } else {
            MONITOR_SERIAL.println("Mode change failed");
        }
        lastModeChange = millis();
    }

    delay(100); // Small delay to prevent overwhelming output
} 