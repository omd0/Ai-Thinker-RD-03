# Radar Tracking Algorithm Implementation

## Overview

This document describes the implementation of an advanced target tracking algorithm for the Ai-Thinker RD-03D radar sensor library. The algorithm addresses the common issues of ghost targets and erratic measurements that occur with raw radar data.

## Problem Description

The original radar implementation suffered from several issues:
- **Ghost Targets**: Multiple false targets reported at inconsistent locations
- **Erratic Measurements**: Distance, angle, and velocity readings fluctuating wildly
- **Detection Inconsistency**: Alternating between detecting multiple targets and no targets

## Solution: Target Tracking Algorithm

### Core Components

1. **Range and Velocity Gating**
   - Filters out targets beyond a configurable maximum distance (default: 500cm)
   - Filters out targets moving faster than a configurable velocity threshold (default: 25cm/s)
   - Ideal for stationary user scenarios

2. **Target Tracking System**
   - Maintains a history of tracked targets over time
   - Associates new detections with existing tracks
   - Smooths measurements using exponential moving average

3. **Track Confirmation**
   - New targets must be consistently detected for multiple frames before being confirmed
   - Only confirmed tracks are reported to the application
   - Prevents brief noise spikes from appearing as valid targets

4. **Stale Track Removal**
   - Tracks that haven't been updated for several frames are automatically removed
   - Prevents ghost targets from persisting indefinitely

### Algorithm Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `MAX_TRACK_DISTANCE` | 500 cm | Maximum detection range |
| `MAX_TRACK_VELOCITY` | 25 cm/s | Maximum velocity threshold |
| `MIN_TRACK_AGE` | 3 frames | Minimum age for track confirmation |
| `MAX_FRAMES_MISSING` | 5 frames | Maximum frames before track deletion |
| `ASSOCIATION_THRESHOLD` | 50 cm | Distance threshold for associating detections with tracks |

### New API Methods

#### Configuration Methods
```cpp
void setMaxTrackDistance(uint16_t distance);
uint16_t getMaxTrackDistance() const;
void setMaxTrackVelocity(uint16_t velocity);
uint16_t getMaxTrackVelocity() const;
void setTrackingEnabled(bool enabled);
bool isTrackingEnabled() const;
```

#### Usage Example
```cpp
// Configure tracking parameters
radar.setMaxTrackDistance(500);  // Filter targets beyond 5 meters
radar.setMaxTrackVelocity(25);   // Filter fast-moving targets
radar.setTrackingEnabled(true);  // Enable tracking algorithm

// The existing API remains unchanged
uint8_t targetCount = radar.getTargetCount();
for (int i = 0; i < targetCount; i++) {
    AiThinker_RD_03D::TargetInfo target;
    if (radar.getTargetInfo(i, target)) {
        // Process stable, confirmed target data
    }
}
```

## Benefits

1. **Stable Target Detection**: Eliminates erratic jumping between multiple ghost targets
2. **Improved Accuracy**: Smoothed measurements provide more reliable position and velocity data
3. **Reduced False Positives**: Only consistently detected targets are reported
4. **Configurable Filtering**: Parameters can be adjusted for different use cases
5. **Backward Compatibility**: Existing code continues to work unchanged

## Implementation Details

### Data Flow

1. **Raw Data Parsing**: Radar frames are parsed into raw target detections
2. **Gating Filters**: Invalid detections are filtered out based on range and velocity
3. **Data Association**: Valid detections are associated with existing tracks
4. **Track Updates**: Associated detections update track state with smoothing
5. **New Track Creation**: Unassociated detections create new tracks
6. **Track Management**: Stale tracks are removed, confirmed tracks are promoted
7. **Output Generation**: Only confirmed tracks are returned to the application

### Track State Management

Each tracked target maintains:
- Position coordinates (x, y) with exponential smoothing
- Velocity with smoothing
- Age counter (number of frames tracked)
- Frames-since-seen counter
- Confirmation status
- Unique track ID

### Smoothing Algorithm

The implementation uses exponential moving average for smoothing:
```cpp
float alpha = 0.3f; // Smoothing factor
track.x = alpha * detection.x + (1 - alpha) * track.x;
```

This provides a good balance between responsiveness and stability.

## Testing Recommendations

1. **Stationary User Test**: Place a person in a chair and verify single, stable target detection
2. **Moving User Test**: Have a person move slowly and verify smooth tracking
3. **Empty Room Test**: Verify no false targets are reported in an empty room
4. **Parameter Tuning**: Adjust thresholds based on your specific environment and use case

## Future Enhancements

Potential improvements for future versions:
- Kalman filter implementation for more sophisticated tracking
- Multi-hypothesis tracking for handling closely spaced targets
- Environmental noise learning and adaptation
- Target classification based on movement patterns 