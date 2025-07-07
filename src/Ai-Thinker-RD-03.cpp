#include "Ai-Thinker-RD-03.h"
#include <Arduino.h>
#include <string.h>  // For memset and memcpy
#include <math.h>    // For sqrt and atan2

// Static command arrays from STM32 blog implementation
const uint8_t AiThinker_RD_03D::Single_Target_Detection_CMD[15] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t AiThinker_RD_03D::Multi_Target_Detection_CMD[15] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};

// This library is made for little endian systems only!
#ifdef TEST_LITTLE_ENDIAN

AiThinker_RD_03D::AiThinker_RD_03D() : radarUART(nullptr), frameTimeout(200), interCommandDelay(100), currentMode(SINGLE_TARGET_MODE), outputFormat(FORMAT_BINARY)
{
    init();
}

void AiThinker_RD_03D::init()
{
    inConfigMode = inFrame = frameAvailable = false;
    receivedFrameLen = currentFrameIndex = 0;
    lastFrameType = UNIDENTIFIED_FRAME;
    targetCount = 0;
    bufferLastFrame = receiveBuffer[1];
    bufferCurrentFrame = receiveBuffer[0];
    
    // Initialize configuration parameters
    configParams.minDistance = 20;    // 20cm minimum
    configParams.maxDistance = 800;   // 800cm maximum
    configParams.sensitivity = 128;   // Medium sensitivity
    configParams.outputFormat = FORMAT_BINARY;
    
    // Initialize tracking system
    initTracking();
    
    // Initialize RD-03D specific data frame from blog implementation
    initRadarDataFrame();
}

// Initialize the target tracking system
void AiThinker_RD_03D::initTracking()
{
    maxTrackDistance = MAX_TRACK_DISTANCE;
    maxTrackVelocity = MAX_TRACK_VELOCITY;
    trackingEnabled = true;
    trackedTargetCount = 0;
    nextTrackId = 1;
    
    // Clear all tracked targets
    for (uint8_t i = 0; i < MAX_TRACKED_TARGETS; i++)
    {
        trackedTargets[i].id = 0;
        trackedTargets[i].x = 0;
        trackedTargets[i].y = 0;
        trackedTargets[i].velocity = 0;
        trackedTargets[i].distance = 0;
        trackedTargets[i].angle = 0;
        trackedTargets[i].age = 0;
        trackedTargets[i].frames_since_seen = 0;
        trackedTargets[i].confirmed = false;
        trackedTargets[i].last_update_time = 0;
    }
}

// Check if a detection is valid (within range and velocity limits)
bool AiThinker_RD_03D::isValidDetection(const TargetInfo& target) const
{
    if (!trackingEnabled) return true;
    
    // Range gating: filter out targets beyond maximum distance
    if (target.distance > maxTrackDistance)
    {
        return false;
    }
    
    // Velocity gating: filter out targets moving too fast for stationary scenario
    if (abs(target.velocity) > maxTrackVelocity)
    {
        return false;
    }
    
    return true;
}

// Calculate Euclidean distance between two points
float AiThinker_RD_03D::calculateDistance(float x1, float y1, float x2, float y2) const
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Find the closest tracked target to a new detection
int AiThinker_RD_03D::findClosestTrack(const TargetInfo& detection)
{
    int bestTrackIndex = -1;
    float bestDistance = ASSOCIATION_THRESHOLD;
    
    for (uint8_t i = 0; i < trackedTargetCount; i++)
    {
        float distance = calculateDistance(
            trackedTargets[i].x, trackedTargets[i].y,
            detection.x, detection.y
        );
        
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestTrackIndex = i;
        }
    }
    
    return bestTrackIndex;
}

// Update an existing track with new detection data
void AiThinker_RD_03D::updateTrack(int trackIndex, const TargetInfo& detection)
{
    TrackedTarget& track = trackedTargets[trackIndex];
    
    // Simple moving average to smooth the data
    float alpha = 0.3f; // Smoothing factor
    
    track.x = alpha * detection.x + (1 - alpha) * track.x;
    track.y = alpha * detection.y + (1 - alpha) * track.y;
    track.velocity = alpha * detection.velocity + (1 - alpha) * track.velocity;
    track.distance = sqrt(track.x * track.x + track.y * track.y);
    track.angle = atan2(track.y, track.x) * 180.0 / PI;
    
    track.age++;
    track.frames_since_seen = 0;
    track.last_update_time = millis();
    
    // Confirm track if it has been seen enough times
    if (track.age >= MIN_TRACK_AGE)
    {
        track.confirmed = true;
    }
}

// Create a new track from a detection
void AiThinker_RD_03D::createNewTrack(const TargetInfo& detection)
{
    if (trackedTargetCount >= MAX_TRACKED_TARGETS) return;
    
    TrackedTarget& track = trackedTargets[trackedTargetCount];
    
    track.id = nextTrackId++;
    track.x = detection.x;
    track.y = detection.y;
    track.velocity = detection.velocity;
    track.distance = detection.distance;
    track.angle = detection.angle;
    track.age = 1;
    track.frames_since_seen = 0;
    track.confirmed = false;
    track.last_update_time = millis();
    
    trackedTargetCount++;
}

// Update track states and remove stale tracks
void AiThinker_RD_03D::updateTrackStates()
{
    for (int i = trackedTargetCount - 1; i >= 0; i--)
    {
        TrackedTarget& track = trackedTargets[i];
        track.frames_since_seen++;
        
        // Remove tracks that haven't been seen for too long
        if (track.frames_since_seen > MAX_FRAMES_MISSING)
        {
            removeStaleTrack(i);
        }
    }
}

// Remove a stale track from the array
void AiThinker_RD_03D::removeStaleTrack(int trackIndex)
{
    // Shift all tracks after the removed one forward
    for (uint8_t i = trackIndex; i < trackedTargetCount - 1; i++)
    {
        trackedTargets[i] = trackedTargets[i + 1];
    }
    
    trackedTargetCount--;
}

// Process raw detections through the tracking algorithm
void AiThinker_RD_03D::processRawDetections(const TargetInfo* rawTargets, uint8_t rawCount)
{
    if (!trackingEnabled)
    {
        // If tracking is disabled, pass through raw detections
        for (uint8_t i = 0; i < rawCount && i < 8; i++)
        {
            targets[i] = rawTargets[i];
        }
        targetCount = rawCount;
        return;
    }
    
    // First, update track states (increment frames_since_seen)
    updateTrackStates();
    
    // Process each raw detection
    for (uint8_t i = 0; i < rawCount; i++)
    {
        const TargetInfo& detection = rawTargets[i];
        
        // Apply gating filters
        if (!isValidDetection(detection))
        {
            continue; // Skip invalid detections
        }
        
        // Find the closest existing track
        int closestTrackIndex = findClosestTrack(detection);
        
        if (closestTrackIndex >= 0)
        {
            // Update existing track
            updateTrack(closestTrackIndex, detection);
        }
        else
        {
            // Create new track
            createNewTrack(detection);
        }
    }
    
    // Build the final target list from confirmed tracks
    buildConfirmedTargets();
}

// Build the target list from confirmed tracks only
void AiThinker_RD_03D::buildConfirmedTargets()
{
    targetCount = 0;
    
    for (uint8_t i = 0; i < trackedTargetCount && targetCount < 8; i++)
    {
        const TrackedTarget& track = trackedTargets[i];
        
        // Only include confirmed tracks
        if (track.confirmed)
        {
            targets[targetCount].targetId = track.id;
            targets[targetCount].x = (int16_t)track.x;
            targets[targetCount].y = (int16_t)track.y;
            targets[targetCount].velocity = (int16_t)track.velocity;
            targets[targetCount].distance = track.distance;
            targets[targetCount].angle = track.angle;
            targets[targetCount].energy = 0; // Not used in tracking
            targets[targetCount].status = 1; // Active
            
            targetCount++;
        }
    }
}

// Initialize RD-03D radar data frame with specific command arrays
void AiThinker_RD_03D::initRadarDataFrame()
{
    // Initialize radar data frame structure
    memset(&radarDataFrame, 0, sizeof(radarDataFrame));
    
    // Initialize command frame with specific command arrays from blog
    memcpy(radarCommandFrame.Single_Target_Detection_CMD, Single_Target_Detection_CMD, 15);
    memcpy(radarCommandFrame.Multi_Target_Detection_CMD, Multi_Target_Detection_CMD, 15);
}

bool AiThinker_RD_03D::begin(HardwareSerial& rSerial, int rxPin, int txPin, int rxBufferSize)
{
    radarUART = &rSerial;
    
    // Set buffer size if the platform supports it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    rSerial.setRxBufferSize(rxBufferSize);
#endif
    
    // Initialize hardware serial with platform-specific parameters
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    // ESP32/ESP8266 can specify pins
    rSerial.begin(256000, SERIAL_8N1, rxPin, txPin);
#else
    // Other platforms use default pins
    rSerial.begin(256000);
#endif
    
    init();
    R_LOG_INFO("Radar initialized with Hardware Serial on %s platform\n", PLATFORM_NAME);
    return true;
}

#ifdef PLATFORM_HAS_SOFTWARE_SERIAL
bool AiThinker_RD_03D::begin(SoftwareSerial& rSerial, int rxBufferSize)
{
    radarUART = &rSerial;
    
    // Software serial initialization
    rSerial.begin(256000);
    
    init();
    R_LOG_INFO("Radar initialized with Software Serial on %s platform\n", PLATFORM_NAME);
    return true;
}
#endif

bool AiThinker_RD_03D::begin(Stream& rSerial, int rxBufferSize)
{
    radarUART = &rSerial;
    
    init();
    R_LOG_INFO("Radar initialized with Stream interface on %s platform\n", PLATFORM_NAME);
    return true;
}

void AiThinker_RD_03D::swapBuffers()
{
    uint8_t* tmp = bufferLastFrame;
    bufferLastFrame = bufferCurrentFrame;
    receivedFrameLen = currentFrameIndex;
    lastFrameStartTS = currentFrameStartTS;
    bufferCurrentFrame = tmp;
    currentFrameIndex = 0;
    frameAvailable = true;
}

void AiThinker_RD_03D::clearRxBuffer()
{
    unsigned long int t = millis();
    while (radarUART->available() && millis() - t < 1500)
        radarUART->read();
}

int AiThinker_RD_03D::read()
{
    int rc = 0;
    
    while (radarUART->available())
    {
        uint8_t c = radarUART->read();
        rc++;
        
        radarDataFrame.RX_BUF[radarDataFrame.RX_count] = c;
        radarDataFrame.RX_count++;
        
        if (radarDataFrame.RX_count >= 64)
        {
            radarDataFrame.RX_count = 0;
        }
        
        if (radarDataFrame.RX_count >= 22)
        {
            uint8_t* RX_BUF = radarDataFrame.RX_BUF;
    
            // Parse raw target data into temporary array
            TargetInfo rawTargets[3];
            uint8_t rawTargetCount = 0;
            
            if (radarDataFrame.RX_count > 9)
            {
                int16_t target1_x = getSignedValue(RX_BUF[4], RX_BUF[5]);
                int16_t target1_y = getSignedValue(RX_BUF[6], RX_BUF[7]);
                int16_t target1_speed = getSignedValue(RX_BUF[8], RX_BUF[9]);
                
                if (target1_x != 0 || target1_y != 0)
                {
                    rawTargets[rawTargetCount].targetId = 1;
                    rawTargets[rawTargetCount].x = target1_x;
                    rawTargets[rawTargetCount].y = target1_y;
                    rawTargets[rawTargetCount].velocity = target1_speed;
                    rawTargets[rawTargetCount].distance = sqrt(pow(target1_x, 2) + pow(target1_y, 2));
                    rawTargets[rawTargetCount].angle = atan2(target1_y, target1_x) * 180.0 / PI;
                    rawTargets[rawTargetCount].energy = 0;
                    rawTargets[rawTargetCount].status = 1;
                    rawTargetCount++;
                    radarDataFrame.Radar_1 = 1;
                }
                else
                {
                    radarDataFrame.Radar_1 = 0;
                }
            }
            
            if (radarDataFrame.RX_count > 15)
            {
                int16_t target2_x = getSignedValue(RX_BUF[10], RX_BUF[11]);
                int16_t target2_y = getSignedValue(RX_BUF[12], RX_BUF[13]);
                int16_t target2_speed = getSignedValue(RX_BUF[14], RX_BUF[15]);
                
                if (target2_x != 0 || target2_y != 0)
                {
                    rawTargets[rawTargetCount].targetId = 2;
                    rawTargets[rawTargetCount].x = target2_x;
                    rawTargets[rawTargetCount].y = target2_y;
                    rawTargets[rawTargetCount].velocity = target2_speed;
                    rawTargets[rawTargetCount].distance = sqrt(pow(target2_x, 2) + pow(target2_y, 2));
                    rawTargets[rawTargetCount].angle = atan2(target2_y, target2_x) * 180.0 / PI;
                    rawTargets[rawTargetCount].energy = 0;
                    rawTargets[rawTargetCount].status = 1;
                    rawTargetCount++;
                    radarDataFrame.Radar_2 = 1;
                }
                else
                {
                    radarDataFrame.Radar_2 = 0;
                }
            }
            
            if (radarDataFrame.RX_count > 21)
            {
                int16_t target3_x = getSignedValue(RX_BUF[16], RX_BUF[17]);
                int16_t target3_y = getSignedValue(RX_BUF[18], RX_BUF[19]);
                int16_t target3_speed = getSignedValue(RX_BUF[20], RX_BUF[21]);
                
                if (target3_x != 0 || target3_y != 0)
                {
                    rawTargets[rawTargetCount].targetId = 3;
                    rawTargets[rawTargetCount].x = target3_x;
                    rawTargets[rawTargetCount].y = target3_y;
                    rawTargets[rawTargetCount].velocity = target3_speed;
                    rawTargets[rawTargetCount].distance = sqrt(pow(target3_x, 2) + pow(target3_y, 2));
                    rawTargets[rawTargetCount].angle = atan2(target3_y, target3_x) * 180.0 / PI;
                    rawTargets[rawTargetCount].energy = 0;
                    rawTargets[rawTargetCount].status = 1;
                    rawTargetCount++;
                    radarDataFrame.Radar_3 = 1;
                }
                else
                {
                    radarDataFrame.Radar_3 = 0;
                }
            }
            
            // Process raw detections through the tracking algorithm
            processRawDetections(rawTargets, rawTargetCount);
            
            // Set frame type based on confirmed target count
            if (targetCount == 1)
            {
                lastFrameType = TARGET_DATA;
            }
            else if (targetCount > 1)
            {
                lastFrameType = MULTI_TARGET_DATA;
            }
            else
            {
                lastFrameType = UNIDENTIFIED_FRAME;
            }

            frameAvailable = true;
            radarDataFrame.RX_count = 0;
        }
    }
    
    return rc;
}

void AiThinker_RD_03D::parseDataFrame()
{
    // This function is now empty as its logic has been moved to the read() method.
    // It is kept for now to avoid breaking other parts of the library that might call it.
}

int16_t AiThinker_RD_03D::getSignedValue(uint8_t low, uint8_t high) const
{
    uint16_t raw_val = (high << 8) | low;
    
    // Convert to signed 16-bit integer (two's complement)
    return (int16_t)raw_val;
}

int AiThinker_RD_03D::readUntilHeader()
{
    int rc = 0;
    while (radarUART->available())
    {
        uint8_t c = radarUART->read();
        rc++;
        if (currentFrameIndex == 0)
            currentFrameStartTS = millis();
        bufferCurrentFrame[currentFrameIndex++] = c;
        
        if (currentFrameIndex == 1)
        {
            if (c == FRAME_HEADER)
            {
                inFrame = true;
                break;
            }
            else
            {
                currentFrameIndex = 0; // Reset if not header
            }
        }
    }
    return rc;
}

bool AiThinker_RD_03D::enterConfigMode()
{
    // Blog implementation doesn't use config mode - commands are sent directly
    inConfigMode = false;
    R_LOG_INFO("Direct command mode (no config mode needed)\n");
    return true;
}

bool AiThinker_RD_03D::exitConfigMode()
{
    // Blog implementation doesn't use config mode
    inConfigMode = false;
    R_LOG_INFO("Direct command mode (no config mode needed)\n");
    return true;
}

bool AiThinker_RD_03D::setSingleTargetMode()
{
    clearRxBuffer();
    
    // Send the exact command sequence from the blog
    radarUART->write(Single_Target_Detection_CMD, 15);
    delay(interCommandDelay);
    
    currentMode = SINGLE_TARGET_MODE;
    R_LOG_INFO("Set to single target mode\n");
    return true;
}

bool AiThinker_RD_03D::setMultiTargetMode()
{
    clearRxBuffer();
    
    // Send the exact command sequence from the blog
    radarUART->write(Multi_Target_Detection_CMD, 15);
    delay(interCommandDelay);
    
    currentMode = MULTI_TARGET_MODE;
    R_LOG_INFO("Set to multi-target mode\n");
    return true;
}

bool AiThinker_RD_03D::setDebugMode()
{
    if (!inConfigMode && !enterConfigMode()) return false;
    
    currentMode = DEBUG_MODE;
    R_LOG_INFO("Set to debug mode\n");
    return true;
}

bool AiThinker_RD_03D::setDetectionRange(uint16_t minDist, uint16_t maxDist)
{
    if (!inConfigMode && !enterConfigMode()) return false;
    
    uint8_t data[4];
    data[0] = (minDist >> 8) & 0xFF;
    data[1] = minDist & 0xFF;
    data[2] = (maxDist >> 8) & 0xFF;
    data[3] = maxDist & 0xFF;
    
    bool success = sendCommand(SET_DETECTION_RANGE, data, 4);
    if (success)
    {
        configParams.minDistance = minDist;
        configParams.maxDistance = maxDist;
        R_LOG_INFO("Set detection range: %d-%d cm\n", minDist, maxDist);
    }
    return success;
}

bool AiThinker_RD_03D::getDetectionRange(uint16_t& minDist, uint16_t& maxDist)
{
    if (!inConfigMode && !enterConfigMode()) return false;
    
    // This would need to be implemented based on actual RD-03D protocol
    // For now, return stored values
    minDist = configParams.minDistance;
    maxDist = configParams.maxDistance;
    return true;
}

bool AiThinker_RD_03D::setSensitivity(uint8_t sensitivity)
{
    if (!inConfigMode && !enterConfigMode()) return false;
    
    bool success = sendCommand(SET_SENSITIVITY, &sensitivity, 1);
    if (success)
    {
        configParams.sensitivity = sensitivity;
        R_LOG_INFO("Set sensitivity: %d\n", sensitivity);
    }
    return success;
}

uint8_t AiThinker_RD_03D::getSensitivity()
{
    return configParams.sensitivity;
}

bool AiThinker_RD_03D::setOutputFormat(OutputFormat format)
{
    if (!inConfigMode && !enterConfigMode()) return false;
    
    uint8_t formatByte = static_cast<uint8_t>(format);
    bool success = sendCommand(SET_OUTPUT_FORMAT, &formatByte, 1);
    if (success)
    {
        outputFormat = format;
        R_LOG_INFO("Set output format: %d\n", format);
    }
    return success;
}

AiThinker_RD_03D::OutputFormat AiThinker_RD_03D::getOutputFormat()
{
    return outputFormat;
}

const char* AiThinker_RD_03D::getFirmwareVersion()
{
    if (!inConfigMode && !enterConfigMode()) return "Unknown";
    
    // This would need to be implemented based on actual RD-03D protocol
    return "RD-03D v1.0";
}

uint32_t AiThinker_RD_03D::getSerialNumber()
{
    if (!inConfigMode && !enterConfigMode()) return 0;
    
    // This would need to be implemented based on actual RD-03D protocol
    return serialNumber;
}

uint8_t AiThinker_RD_03D::getTargetCount() const
{
    return targetCount;
}

bool AiThinker_RD_03D::getTargetInfo(uint8_t index, TargetInfo& target) const
{
    if (index >= targetCount) return false;
    target = targets[index];
    return true;
}

uint16_t AiThinker_RD_03D::getTargetDistance(uint8_t index) const
{
    if (index >= targetCount) return 0;
    return targets[index].distance;
}

int16_t AiThinker_RD_03D::getTargetAngle(uint8_t index) const
{
    if (index >= targetCount) return 0;
    return targets[index].angle;
}

int16_t AiThinker_RD_03D::getTargetVelocity(uint8_t index) const
{
    if (index >= targetCount) return 0;
    return targets[index].velocity;
}

uint8_t AiThinker_RD_03D::getTargetEnergy(uint8_t index) const
{
    if (index >= targetCount) return 0;
    return targets[index].energy;
}

bool AiThinker_RD_03D::sendCommand(RadarCommand command, const uint8_t* data, uint8_t dataLen)
{
    if (!radarUART) return false;
    
    CommandFrame frame;
    frame.header = FRAME_HEADER;
    frame.command = static_cast<uint8_t>(command);
    frame.dataLength = dataLen;
    
    if (data && dataLen > 0)
    {
        if (dataLen > 16) dataLen = 16; // Safety limit
        memcpy(frame.data, data, dataLen);
    }
    
    // Calculate checksum
    uint8_t checksum = calculateChecksum(reinterpret_cast<uint8_t*>(&frame) + 1, 2 + dataLen);
    frame.checksum = checksum;
    frame.trailer = FRAME_TRAILER;
    
    // Send the command
    radarUART->write(reinterpret_cast<uint8_t*>(&frame), 4 + dataLen);
    
    // Wait for acknowledgment
    if (command != EXIT_CONFIG_MODE)
    {
        return readAckFrame();
    }
    
    return true;
}

bool AiThinker_RD_03D::readAckFrame()
{
    unsigned long startTime = millis();
    frameAvailable = false;
    
    while (millis() - startTime < frameTimeout)
    {
        if (read() > 0 && frameAvailable && lastFrameType == ACK_FRAME)
        {
            const AckFrame* ack = reinterpret_cast<const AckFrame*>(bufferLastFrame);
            return (ack->status == 0); // 0 = OK, 1 = Error
        }
        delay(1);
    }
    
    R_LOG_ERROR("ACK frame timeout\n");
    return false;
}

uint8_t AiThinker_RD_03D::calculateChecksum(const uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

bool AiThinker_RD_03D::validateFrame(const uint8_t* frame, uint8_t length)
{
    if (length < 6) return false; // Minimum frame size
    
    if (frame[0] != FRAME_HEADER || frame[length - 1] != FRAME_TRAILER)
        return false;
    
    // Calculate and verify checksum
    uint8_t expectedChecksum = calculateChecksum(frame + 1, length - 3);
    uint8_t actualChecksum = frame[length - 2];
    
    return (expectedChecksum == actualChecksum);
}

void AiThinker_RD_03D::dumpLastFrame(String label, Stream& dumpStream) const
{
    dumpFrame(bufferLastFrame, receivedFrameLen, label + "[", "]", dumpStream);
}

void AiThinker_RD_03D::dumpFrame(const uint8_t* buff, int len, String pre, String post, Stream& dumpStream)
{
    dumpStream.print(pre);
    for (int i = 0; i < len; i++)
    {
        if (i > 0) dumpStream.print(" ");
        if (buff[i] < 0x10) dumpStream.print("0");
        dumpStream.print(buff[i], HEX);
    }
    dumpStream.println(post);
}

#else
#error "This library is made for little endian systems only, and it seems it is being compiled for big endian system!"
#endif // TEST_LITTLE_ENDIAN
