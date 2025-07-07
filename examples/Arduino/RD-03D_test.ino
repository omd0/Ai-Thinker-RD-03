/*
    Name:       RD-03D_test.ino
    Created:    2024
    Author:     Refactored for RD-03D
    Purpose:    Example for Ai-Thinker RD-03D 24Ghz FMCW radar sensor
*/

#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1

#define RADAR_SERIAL_SPEED 256000  // RD-03D uses 256000 baud
#define CONSOLE_SERIAL_SPEED 115200

#define RADAR_RX_PIN 26
#define RADAR_TX_PIN 27

#define PUSH_BUTTON1 0

#include <Ai-Thinker-RD-03.h>

// Mode selection
#define SINGLE_TARGET_MODE
//#define MULTI_TARGET_MODE
//#define DEBUG_MODE

AiThinker_RD_03D radar;

unsigned long lastFrameTime = 0;
unsigned long frameCount = 0;

// Button for mode switching
bool buttonPressed = false;
unsigned long lastButtonPress = 0;

void setup()
{
    Serial.begin(CONSOLE_SERIAL_SPEED);
    Serial.println("RD-03D Radar Sensor Test");
    Serial.println("=========================");
    
    // Initialize radar
    if (!radar.begin(RADAR_SERIAL, RADAR_RX_PIN, RADAR_TX_PIN))
    {
        Serial.println("Failed to initialize radar!");
        return;
    }
    
    Serial.println("Radar initialized successfully");
    
    // Set timeouts
    radar.setFrameTimeout(500);
    radar.setInterCommandDelay(100);
    
    // Get firmware information
    Serial.print("Firmware Version: ");
    Serial.println(radar.getFirmwareVersion());
    
    Serial.print("Serial Number: ");
    Serial.println(radar.getSerialNumber());
    
    // Configure radar parameters
    Serial.println("Configuring radar...");
    
    // Set detection range (20cm to 800cm)
    if (radar.setDetectionRange(20, 800))
    {
        Serial.println("Detection range set: 20-800 cm");
    }
    else
    {
        Serial.println("Failed to set detection range");
    }
    
    // Set sensitivity (0-255, higher = more sensitive)
    if (radar.setSensitivity(128))
    {
        Serial.println("Sensitivity set to 128");
    }
    else
    {
        Serial.println("Failed to set sensitivity");
    }
    
    // Set output format
    if (radar.setOutputFormat(AiThinker_RD_03D::FORMAT_BINARY))
    {
        Serial.println("Output format set to binary");
    }
    else
    {
        Serial.println("Failed to set output format");
    }
    
#ifdef SINGLE_TARGET_MODE
    if (radar.setSingleTargetMode())
    {
        Serial.println("Set to single target mode");
    }
    else
    {
        Serial.println("Failed to set single target mode");
    }
#endif

#ifdef MULTI_TARGET_MODE
    if (radar.setMultiTargetMode())
    {
        Serial.println("Set to multi-target mode");
    }
    else
    {
        Serial.println("Failed to set multi-target mode");
    }
#endif

#ifdef DEBUG_MODE
    if (radar.setDebugMode())
    {
        Serial.println("Set to debug mode");
    }
    else
    {
        Serial.println("Failed to set debug mode");
    }
#endif
    
    // Exit configuration mode
    if (radar.exitConfigMode())
    {
        Serial.println("Exited configuration mode");
    }
    else
    {
        Serial.println("Failed to exit configuration mode");
    }
    
    Serial.println("Setup complete. Starting detection...");
    Serial.println("Press button to switch modes");
    Serial.println();
}

void loop()
{
    // Check for button press (mode switching)
    if (digitalRead(PUSH_BUTTON1) == LOW && !buttonPressed && (millis() - lastButtonPress > 1000))
    {
        buttonPressed = true;
        lastButtonPress = millis();
        switchMode();
    }
    else if (digitalRead(PUSH_BUTTON1) == HIGH)
    {
        buttonPressed = false;
    }
    
    // Read radar data
    int bytesRead = radar.read();
    
    if (bytesRead > 0 && radar.frameReady())
    {
        frameCount++;
        lastFrameTime = millis();
        
        // Process the frame based on type
        switch (radar.getFrameType())
        {
            case AiThinker_RD_03D::TARGET_DATA:
                processSingleTarget();
                break;
                
            case AiThinker_RD_03D::MULTI_TARGET_DATA:
                processMultiTarget();
                break;
                
            case AiThinker_RD_03D::DEBUG_DATA:
                processDebugData();
                break;
                
            case AiThinker_RD_03D::ACK_FRAME:
                Serial.println("Received ACK frame");
                break;
                
            default:
                Serial.println("Unknown frame type");
                break;
        }
    }
    
    // Print status every 5 seconds
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000)
    {
        printStatus();
        lastStatusTime = millis();
    }
}

void processSingleTarget()
{
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0)
    {
        AiThinker_RD_03D::TargetInfo target;
        if (radar.getTargetInfo(0, target))
        {
            Serial.printf("Target detected - ID: %d, Distance: %d cm, Angle: %d°, Velocity: %d cm/s, Energy: %d\n",
                         target.targetId,
                         target.distance,
                         target.angle,
                         target.velocity,
                         target.energy);
        }
    }
    else
    {
        Serial.println("No target detected");
    }
}

void processMultiTarget()
{
    uint8_t targetCount = radar.getTargetCount();
    
    if (targetCount > 0)
    {
        Serial.printf("Multi-target detected: %d targets\n", targetCount);
        
        for (uint8_t i = 0; i < targetCount; i++)
        {
            AiThinker_RD_03D::TargetInfo target;
            if (radar.getTargetInfo(i, target))
            {
                Serial.printf("  Target %d - ID: %d, Distance: %d cm, Angle: %d°, Velocity: %d cm/s, Energy: %d\n",
                             i,
                             target.targetId,
                             target.distance,
                             target.angle,
                             target.velocity,
                             target.energy);
            }
        }
    }
    else
    {
        Serial.println("No targets detected");
    }
}

void processDebugData()
{
    Serial.println("Debug data received");
    // In a real implementation, you would process the raw debug data here
    // For now, just acknowledge receipt
}

void switchMode()
{
    Serial.println("Switching mode...");
    
    if (radar.enterConfigMode())
    {
        // Cycle through modes
        static int modeIndex = 0;
        modeIndex = (modeIndex + 1) % 3;
        
        switch (modeIndex)
        {
            case 0:
                if (radar.setSingleTargetMode())
                {
                    Serial.println("Switched to single target mode");
                }
                break;
                
            case 1:
                if (radar.setMultiTargetMode())
                {
                    Serial.println("Switched to multi-target mode");
                }
                break;
                
            case 2:
                if (radar.setDebugMode())
                {
                    Serial.println("Switched to debug mode");
                }
                break;
        }
        
        radar.exitConfigMode();
    }
    else
    {
        Serial.println("Failed to enter config mode for switching");
    }
}

void printStatus()
{
    Serial.printf("Status - Frames: %lu, Last frame: %lu ms ago, Mode: %d\n",
                 frameCount,
                 millis() - lastFrameTime,
                 radar.getCurrentMode());
    
    // Print current configuration
    uint16_t minDist, maxDist;
    if (radar.getDetectionRange(minDist, maxDist))
    {
        Serial.printf("Detection range: %d-%d cm, Sensitivity: %d\n",
                     minDist, maxDist, radar.getSensitivity());
    }
} 