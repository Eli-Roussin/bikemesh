#pragma once

// ============================================
// AntiTheft.h -- TRACKER-like beacons (30s idle / 10s on motion/tamper)
// Tamper alerts rate-limited to max once per 30 seconds per type (BUMP/TILT/DRIVE_AWAY/GRINDER).
// MQTT alert publishing for home broker (Node-RED/Home Assistant connectivity).
// ============================================

#include <stdint.h>
#include "../config.h"

// TAMPER types: each has its own independent rate limiter (max once per 30s)
typedef enum {
    TAMPER_NONE = 0,                // No tamper detected
    TAMPER_BUMP = 1,                 // Sudden jostle/impact (>2.0g on any axis)
    TAMPER_TILT = 2,               // Significant tilt angle change (>30 degrees -- bike being moved/dismantled)
    TAMPER_DRIVE_AWAY = 3,         // Combined motion + tilt -- bike started moving with angle change
    TAMPER_GRINDER = 4              // Angle grinder vibration signature (200-500 Hz high-frequency pattern)
} TamperType;

// Beacon states for AntiTheft state machine (same rates as MeshBroadcast):
typedef enum {
    STATIONARY_IDLE = 0,           // 30s idle (standard Meshtastic TRACKER default -- conserves battery when parked)
    MOTION_BURST = 1,             // 10s motion burst (accelerated for theft tracking -- bike is being moved)
    TAMPER_ALERT = 2               // 10s same as MOTION_BURST (additional MQTT alert sent per-type at 30s max rate)
} BeaconState;

class AntiTheft {
public:
    AntiTheft() : m_beaconState(STATIONARY_IDLE), 
                  m_stationaryTimerMs(0),
                  m_lastBroadcastMs(0),
                  m_initialized(false) {
        initTamperRateLimits();       // Initialize rate counters (all types start at 0 so first alert can fire immediately)
    }

      // Initialize anti-theft module (called from BikeMesh::setup())
    void setup(void);                              // Start in STATIONARY_IDLE beacon state
    
     // Main loop -- called at 1Hz from BikeMesh::runOnce()
    int32_t runOnce(void);                         // Handle beacon interval switching + tamper rate limiting

      // Called when motion/tamper events are detected by MotionDetector
    void onMotionDetected(void);                   // Switch to MOTION_BURST state (10s beacon rate)
     void onTamperDetected(TamperType tamperType);   // Switch to TAMPER_ALERT state + publish MQTT alert

       // Query methods for external modules (like MeshBroadcast or BikeMeshModule)
    bool isTamperActive(void) const;             // True if currently in TAMPER_ALERT state
    BeaconState getBeaconState(void) const;          // Get current beacon state
    int32_t getCurrentIntervalMs(void) const;     // Get current beacon interval (30s/10s based on state)

private:
      BeaconState m_beaconState;                   // Current beacon state (STATIONARY_IDLE, MOTION_BURST, TAMPER_ALERT)
    uint32_t m_stationaryTimerMs;                     // Track time since last motion (transition to idle after 5s stationary)
     uint32_t m_lastBroadcastMs;                      // Last position beacon timestamp
    
         bool m_initialized;                          // True after setup() completes

       // Tamper alert rate limiter -- max once per 30 seconds per type (independently for each tamper type)
    static const uint8_t TAMPER_TYPE_COUNT = 5;        // NONE + BUMP + TILT + DRIVE_AWAY + GRINDER
    uint32_t m_lastTamperBroadcastMs[TAMPER_TYPE_COUNT];   // Timestamp of last MQTT/mesh broadcast per type

      // Initialize rate limits (all types start at time 0 so first alert can fire immediately)
    void initTamperRateLimits(void);

        // Check if we can broadcast a tamper alert of the given type (rate-limited to max once per 30s)
    bool canBroadcastTamperAlert(TamperType type);      // Returns true if >=30s since last alert of this type

       // Broadcast control methods
     void broadcastPositionBeacon(void);               // Like Meshtastic TRACKER -- send position to mesh
    bool shouldBroadcast(void);                     // Returns true based on current state's interval (30s idle / 10s motion)

        // MQTT alert publishing to home broker (Node-RED/Home Assistant connectivity)
     void publishTamperAlertMQTT(TamperType tamperType);    // Send JSON alert payload to MQTT broker
    
         // State transition helpers
      void setBeaconState(BeaconState newState);       // Transition to new beacon state with proper handling

     bool mqttClientConnected(void);                // Check if the configured MQTT transport is connected
    bool mqttClientPublish(const char *topic, const char *payload);  // Publish JSON payload to MQTT topic

         // Internal state tracking for beacon interval control
      uint32_t m_motionSinceLastCheckMs;             // Track how long motion has been active since last check
};
