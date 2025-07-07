/**
  Ai-Thinker-RD-03D library v 1.0
  Name: Ai-Thinker-RD-03D
  Purpose: Arduino library for the Ai-Thinker RD-03D 24Ghz FMCW radar sensor 

  @author Dejan Gjorgjevikj (original), Refactored for RD-03D
  @version 1.0, 2024

  This sensor is a Frequency Modulated Continuous Wave radar with multi-target detection,
  capable of tracking targets with distance, angle, and velocity measurements.
 
  Based on RD-03D documentation from Ai-Thinker:
  https://aithinker.blog.csdn.net/article/details/133338984

Known limitations:
- Requires little-endian systems
- Limited to ESP32 architecture for now

Resources:
  The code in this library was developed from scratch based on the manufacturer datasheet(s) 
  and refactored for RD-03D specifications.
  
History of changes:
    01.02.2024 - v0.1.0 initial RD-03 version
    09.02.2024 - v0.7.0 RD-03 version
    2024 - v1.0.0 Refactored for RD-03D

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Released under LGPL-2.1 see https://github.com/Gjorgjevikj/... for full license

https://github.com/Gjorgjevikj/...todo
*/


#pragma once

#ifndef _AI_THINKER_RD_03D_H
#define _AI_THINKER_RD_03D_H
#include <Arduino.h>

// Platform detection and serial support
#if defined(ARDUINO_ARCH_ESP32)
    // ESP32 has multiple hardware serial ports
    #define PLATFORM_HAS_HARDWARE_SERIAL
    #define PLATFORM_NAME "ESP32"
#elif defined(ARDUINO_ARCH_ESP8266)
    // ESP8266 has limited hardware serial
    #define PLATFORM_HAS_HARDWARE_SERIAL
    #define PLATFORM_NAME "ESP8266"
#elif defined(ARDUINO_ARCH_AVR)
    // AVR boards like Arduino Uno, Nano, etc.
    #define PLATFORM_NAME "AVR"
    #if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        // Arduino Mega has multiple hardware serial ports
        #define PLATFORM_HAS_HARDWARE_SERIAL
    #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        // Arduino Uno/Nano have limited hardware serial
        #define PLATFORM_HAS_HARDWARE_SERIAL
        #define PLATFORM_LIMITED_HARDWARE_SERIAL
    #endif
#elif defined(ARDUINO_ARCH_STM32)
    // STM32 boards
    #define PLATFORM_HAS_HARDWARE_SERIAL
    #define PLATFORM_NAME "STM32"
#elif defined(ARDUINO_ARCH_SAMD)
    // SAMD boards like Arduino Zero
    #define PLATFORM_HAS_HARDWARE_SERIAL
    #define PLATFORM_NAME "SAMD"
#else
    // Generic Arduino platform
    #define PLATFORM_NAME "Generic"
    #define PLATFORM_HAS_HARDWARE_SERIAL
#endif

// Include SoftwareSerial for platforms that support it
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266)
    #include <SoftwareSerial.h>
    #define PLATFORM_HAS_SOFTWARE_SERIAL
#endif

#define AI_THINKER_RD_03D_LIB_VERSION 1.0

#ifndef _LOG_LEVEL 
#define _LOG_LEVEL 2
#endif

//#define LOG_FATAL(...) Serial.printf(__VA_ARGS__)
#if _LOG_LEVEL > 0
#define R_LOG_ERROR(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_ERROR(...) { }
#endif
#if _LOG_LEVEL >= 2
#define R_LOG_WARN(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_WARN(...) { }
#endif
#if _LOG_LEVEL >= 4
#define R_LOG_INFO(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_INFO(...) { }
#endif
#if _LOG_LEVEL >= 6
#define R_LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_DEBUG(...) { }
#endif
#if _LOG_LEVEL >= 7
#define DEBUG_DUMP_FRAME(...) _dumpFrame(__VA_ARGS__)
#else
#define DEBUG_DUMP_FRAME(...) { }
#endif
#if _LOG_LEVEL > 7
#define R_LOG_VERBOSE(...) Serial.printf(__VA_ARGS__)
#else
#define R_LOG_VERBOSE(...) { }
#endif

// This library is made for little endian systems only!
#define TEST_LITTLE_ENDIAN (((union { unsigned x; unsigned char c; }){1}).c)
#ifdef TEST_LITTLE_ENDIAN

// Buffer size for RD-03D data frames
#define RECEIVE_BUFFER_SIZE 2048

// RD-03D specific buffer and command definitions based on STM32 implementation
#define RX_BUFFER_SIZE 64

class AiThinker_RD_03D
{
public:
    enum FrameType : uint8_t 
    { 
        UNIDENTIFIED_FRAME = 0x00,
        TARGET_DATA = 0x01,
        MULTI_TARGET_DATA = 0x02,
        DEBUG_DATA = 0x03,
        UNKNOWN_DATA_FORMAT = 0x0f,
        ACK_FRAME = 0xff,
        UNKNOWN_FRAME = 0xfe
    };

    enum RadarCommand : uint8_t 
    {
        // RD-03D specific commands
        SET_SINGLE_TARGET_MODE = 0x01,    // Single target detection mode
        SET_MULTI_TARGET_MODE = 0x02,     // Multi-target detection mode
        READ_FIRMWARE_VERSION = 0x03,     // Read firmware version
        READ_SERIAL_NUMBER = 0x04,        // Read serial number
        SET_DETECTION_RANGE = 0x05,       // Set detection range
        READ_DETECTION_RANGE = 0x06,      // Read detection range
        SET_SENSITIVITY = 0x07,           // Set sensitivity
        READ_SENSITIVITY = 0x08,          // Read sensitivity
        SET_OUTPUT_FORMAT = 0x09,         // Set output format
        READ_OUTPUT_FORMAT = 0x0A,        // Read output format
        ENTER_CONFIG_MODE = 0xFE,         // Enter configuration mode
        EXIT_CONFIG_MODE = 0xFF           // Exit configuration mode
    };

    enum RadarMode : uint8_t
    {
        SINGLE_TARGET_MODE = 0x01,        // Single target detection
        MULTI_TARGET_MODE = 0x02,         // Multi-target detection
        DEBUG_MODE = 0x03                 // Debug mode with raw data
    };

    enum OutputFormat : uint8_t
    {
        FORMAT_ASCII = 0x01,              // ASCII format output
        FORMAT_BINARY = 0x02              // Binary format output
    };

    // Frame headers for RD-03D
    enum FrameHeader : uint8_t
    {
        FRAME_HEADER = 0xAA,              // Frame header byte
        FRAME_TRAILER = 0x55              // Frame trailer byte
    };

#pragma pack (1)
    // RD-03D Radar Data Frame Structure (from STM32 blog implementation)
    struct RadarDataFrame
    {
        uint8_t RX_BUF[64];               // 缓存数组 - Buffer array
        uint8_t RX_count;                 // 计数位 - Count position
        uint8_t RX_temp;                  // 缓存字符 - Buffer character
        
        uint8_t Radar_1;                  // 目标1标志位 - Target 1 flag
        uint8_t Radar_2;                  // 目标2标志位 - Target 2 flag  
        uint8_t Radar_3;                  // 目标3标志位 - Target 3 flag
    };

    // RD-03D Command Frame Structure
    struct RadarCommandFrame
    {
        // Single Target Detection Command
        uint8_t Single_Target_Detection_CMD[15];  // = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01}
        
        // Multi Target Detection Command  
        uint8_t Multi_Target_Detection_CMD[15];   // = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01}
    };

    // Static command arrays from STM32 blog implementation
    static const uint8_t Single_Target_Detection_CMD[15];
    static const uint8_t Multi_Target_Detection_CMD[15];

    // Target information structure for RD-03D
    struct TargetInfo
    {
        uint8_t targetId;                 // Target ID (0-255)
        int16_t x;                        // X-coordinate in cm
        int16_t y;                        // Y-coordinate in cm
        uint16_t distance;                // Distance in cm
        int16_t angle;                    // Angle in degrees
        int16_t velocity;                 // Velocity in cm/s
        uint8_t energy;                   // Signal energy (0-255)
        uint8_t status;                   // Target status
    };

    // Command frame structure
    struct CommandFrame
    {
        uint8_t header;                   // 0xAA
        uint8_t command;                  // Command byte
        uint8_t dataLength;               // Length of data
        uint8_t data[16];                 // Command data
        uint8_t checksum;                 // Checksum
        uint8_t trailer;                  // 0x55
    };

    // Acknowledgment frame structure
    struct AckFrame
    {
        uint8_t header;                   // 0xAA
        uint8_t ackType;                  // Acknowledgment type
        uint8_t dataLength;               // Length of data
        uint8_t status;                   // Status (0=OK, 1=Error)
        uint8_t data[16];                 // Response data
        uint8_t checksum;                 // Checksum
        uint8_t trailer;                  // 0x55
    };

    // Firmware version structure
    struct FirmwareVersion
    {
        uint8_t major;                    // Major version
        uint8_t minor;                    // Minor version
        uint8_t patch;                    // Patch version
        char buildDate[8];                // Build date (YYYYMMDD)
    };

    // Configuration parameters
    struct ConfigParams
    {
        uint16_t minDistance;             // Minimum detection distance (cm)
        uint16_t maxDistance;             // Maximum detection distance (cm)
        uint8_t sensitivity;              // Sensitivity level (0-255)
        uint8_t outputFormat;             // Output format
    };

    // Target tracking structure for filtering ghost targets
    struct TrackedTarget
    {
        uint8_t id;                       // Track ID
        float x, y;                       // Position coordinates (cm)
        float velocity;                   // Velocity (cm/s)
        uint16_t distance;                // Distance (cm)
        int16_t angle;                    // Angle (degrees)
        uint8_t age;                      // How many frames this target has been tracked
        uint8_t frames_since_seen;        // Frames since last detection
        bool confirmed;                   // Whether this is a confirmed track
        unsigned long last_update_time;   // Last update timestamp
    };

    // Tracking algorithm constants
    static const uint16_t MAX_TRACK_DISTANCE = 500;     // Maximum tracking distance (cm)
    static const uint16_t MAX_TRACK_VELOCITY = 25;      // Maximum velocity for stationary scenario (cm/s)
    static const uint8_t MIN_TRACK_AGE = 3;             // Minimum age for track confirmation
    static const uint8_t MAX_FRAMES_MISSING = 5;        // Maximum frames before track deletion
    static const uint8_t MAX_TRACKED_TARGETS = 8;       // Maximum number of tracked targets
    static const float ASSOCIATION_THRESHOLD = 50.0;    // Distance threshold for data association (cm)

#pragma pack()

    // Constructor
    AiThinker_RD_03D();

    // Initialize the radar module with Hardware Serial
    bool begin(HardwareSerial& rSerial, int rxPin, int txPin, int rxBufferSize = 2048);
    
#ifdef PLATFORM_HAS_SOFTWARE_SERIAL
    // Initialize the radar module with Software Serial
    bool begin(SoftwareSerial& rSerial, int rxBufferSize = 2048);
#endif
    
    // Initialize the radar module with any Stream (advanced users)
    bool begin(Stream& rSerial, int rxBufferSize = 2048);
    
    // Platform information
    static const char* getPlatformName() { return PLATFORM_NAME; }
    static bool hasHardwareSerial() { 
#ifdef PLATFORM_HAS_HARDWARE_SERIAL
        return true;
#else
        return false;
#endif
    }
    static bool hasSoftwareSerial() { 
#ifdef PLATFORM_HAS_SOFTWARE_SERIAL
        return true;
#else
        return false;
#endif
    }

    // Configuration methods
    bool enterConfigMode();
    bool exitConfigMode();
    bool isInConfigMode() const { return inConfigMode; }

    // Mode control
    bool setSingleTargetMode();
    bool setMultiTargetMode();
    bool setDebugMode();
    RadarMode getCurrentMode() const { return currentMode; }

    // Parameter configuration
    bool setDetectionRange(uint16_t minDist, uint16_t maxDist);
    bool getDetectionRange(uint16_t& minDist, uint16_t& maxDist);
    bool setSensitivity(uint8_t sensitivity);
    uint8_t getSensitivity();
    bool setOutputFormat(OutputFormat format);
    OutputFormat getOutputFormat();

    // Information queries
    const char* getFirmwareVersion();
    uint32_t getSerialNumber();

    // Data reading
    int read();
    bool frameReady() const { return frameAvailable; }
    FrameType getFrameType() const { return lastFrameType; }

    // Target data access
    uint8_t getTargetCount() const;
    bool getTargetInfo(uint8_t index, TargetInfo& target) const;
    uint16_t getTargetDistance(uint8_t index = 0) const;
    int16_t getTargetAngle(uint8_t index = 0) const;
    int16_t getTargetVelocity(uint8_t index = 0) const;
    uint8_t getTargetEnergy(uint8_t index = 0) const;

    // Target tracking configuration
    void setMaxTrackDistance(uint16_t distance) { maxTrackDistance = distance; }
    uint16_t getMaxTrackDistance() const { return maxTrackDistance; }
    void setMaxTrackVelocity(uint16_t velocity) { maxTrackVelocity = velocity; }
    uint16_t getMaxTrackVelocity() const { return maxTrackVelocity; }
    void setTrackingEnabled(bool enabled) { trackingEnabled = enabled; }
    bool isTrackingEnabled() const { return trackingEnabled; }

    // Utility methods
    void setFrameTimeout(unsigned long timeout) { frameTimeout = timeout; }
    unsigned long getFrameTimeout() const { return frameTimeout; }
    void setInterCommandDelay(unsigned long delay) { interCommandDelay = delay; }
    unsigned long getInterCommandDelay() const { return interCommandDelay; }
    void clearRxBuffer();
    
    int getFrameLength() const { return receivedFrameLen; }
    
    // Command array access methods
    static const uint8_t* getSingleTargetCommand() { return Single_Target_Detection_CMD; }
    static const uint8_t* getMultiTargetCommand() { return Multi_Target_Detection_CMD; }
    static constexpr size_t getCommandSize() { return 15; }
    
    // Debug methods
    void dumpLastFrame(String label = "", Stream& dumpStream = Serial) const;
    static void dumpFrame(const uint8_t* buff, int len, String pre = "", String post = "", Stream& dumpStream = Serial);

private:
    int16_t getSignedValue(uint8_t low, uint8_t high) const;
    void parseDataFrame();
    void init();
    bool sendCommand(RadarCommand command, const uint8_t* data = nullptr, uint8_t dataLen = 0);
    bool readAckFrame();
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length);
    bool validateFrame(const uint8_t* frame, uint8_t length);
    int readUntilHeader();
    void swapBuffers();

    // Member variables
    Stream* radarUART;
    uint8_t receiveBuffer[2][RECEIVE_BUFFER_SIZE];
    uint8_t* bufferCurrentFrame;
    uint8_t* bufferLastFrame;
    
    bool inConfigMode;
    bool inFrame;
    bool frameAvailable;
    RadarMode currentMode;
    OutputFormat outputFormat;
    
    unsigned long frameTimeout;
    unsigned long interCommandDelay;
    unsigned long lastFrameStartTS;
    unsigned long currentFrameStartTS;
    
    int currentFrameIndex;
    int receivedFrameLen;
    FrameType lastFrameType;
    
    // Parsed data
    uint8_t targetCount;
    TargetInfo targets[8];
    
    // Configuration data
    FirmwareVersion firmwareVersion;
    uint32_t serialNumber;
    ConfigParams configParams;

    // RD-03D specific data frame instances
    RadarDataFrame radarDataFrame;
    RadarCommandFrame radarCommandFrame;
    
    void initRadarDataFrame();  // Initialize RD-03D specific data frame

    // Target tracking variables
    uint16_t maxTrackDistance;
    uint16_t maxTrackVelocity;
    bool trackingEnabled;
    
    // Tracked targets array and management
    TrackedTarget trackedTargets[MAX_TRACKED_TARGETS];
    uint8_t trackedTargetCount;
    uint8_t nextTrackId;
    
    // Tracking algorithm methods
    void initTracking();
    bool isValidDetection(const TargetInfo& target) const;
    float calculateDistance(float x1, float y1, float x2, float y2) const;
    int findClosestTrack(const TargetInfo& detection);
    void updateTrack(int trackIndex, const TargetInfo& detection);
    void createNewTrack(const TargetInfo& detection);
    void updateTrackStates();
    void removeStaleTrack(int trackIndex);
    void processRawDetections(const TargetInfo* rawTargets, uint8_t rawCount);
    void buildConfirmedTargets();
};

#endif // _AI_THINKER_RD_03D_H
#endif // TEST_LITTLE_ENDIAN
