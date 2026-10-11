#pragma once

// ============================================
// MeshBroadcast.h -- Custom telemetry packet for route reconstruction + dashboard display
// 38-byte packed struct with elevation gains/drops, cumulative distance, speed profiles.
// Broadcast at 30s idle / 10s on motion for BOTH Mode A and Mode B.
// ============================================

#include <stdint.h>
#include "../config.h"

// === Telemetry Packet Schema (for both mesh packet AND CSV summary log) ===
// Captures enough data to reconstruct the ride with downstream software:
//   - latitude/longitude --> Route path / GPS track
//   - altitude_m_x100 + altitude_m_delta --> Elevation gains/drops per segment
//   - cumulative_distance_m --> Total meters tracked (zeroed per ride session)
//   - elevation_gain_m_x100, elevation_loss_m_x100 --> Pre-computed cumulative stats for dashboard
//   - timestamp_ms + speed_kph --> Speed profile / rest stops / pauses
//   - power_mode + battery_mv --> Battery state during ride
//   - event_flag --> Mark significant events (motion start/stop, tamper)

#pragma pack(push, 1)
// 38-byte packed telemetry struct for mesh packet and CSV summary log.
// Fields enable downstream route reconstruction, elevation gain/loss calculation,
// and dashboard display of speed profiles and battery state.
typedef struct {
    uint32_t    timestamp_ms;              // Unix epoch milliseconds for time-series analysis
    float       latitude_deg;              // WGS84 -- route path reconstruction
    float       longitude_deg;             // WGS84 -- route path reconstruction
    int16_t     altitude_m_x100;           // MSL altitude * 100 for elevation gain/loss calc
    float       altitude_m_delta;          // Delta from last broadcast in meters (+gain, -drop)
    uint8_t     speed_kph;                // Ground speed km/h for speed profile reconstruction
    uint8_t     heading_deg;              // 0-359 degrees for bearing/path direction
    uint8_t     satellites;               // GNSS fix quality as data validity indicator
    uint8_t     power_mode;               // 0=charging/ModeA, 1=battery/ModeB
    uint16_t    battery_mv;               // Battery voltage in mV for range estimation
    uint32_t    cumulative_distance_m;    // Total meters logged since ride start
    int16_t     elevation_gain_m_x100;    // Cumulative positive elevation gain * 100
    int16_t     elevation_loss_m_x100;    // Cumulative negative elevation loss * 100
    uint8_t     event_flag;               // 0=normal, 1=motion_start, 2=motion_stop, 3=tamper_detected
} RideTelemetryPkt_Packed_t;              // 38 bytes packed for mesh serialization
#pragma pack(pop)

// Broadcast interval control states (same for both Mode A and Mode B):
typedef enum {
    BEACON_IDLE = 0,            // 30s normal TRACKER rate (stationary/idle)
    BEACON_MOTION = 1           // 10s accelerated on motion/tamper (both modes share this rate)
} BeaconState;

class MeshBroadcast {
public:
    MeshBroadcast() : m_initialized(false), m_beaconState(BEACON_IDLE), 
                      m_lastBroadcastMs(0), m_lastLatitude(0), m_lastLongitude(0),
                      m_cumulativeDistanceM(0), m_cumulativeGainMx100(0), 
                      m_cumulativeLossMx100(0), m_lastEventFlag(0) {}

        // Initialize mesh broadcast (called from BikeMesh::setup())
    void setup(void);                              // Set up first broadcast timer and Meshtastic portnum
    
      // Called each time new GPS data is available (from GPSLogger or MotionDetector)
    void updatePosition(NMEA_GPGGA_t *gga, bool motion);     // Update cached position + state
    
         // Main loop -- called from BikeMesh::runOnce() at 1Hz
     int32_t runOnce(void);                        // Check if should broadcast (30s idle / 10s on motion)

       // Public API: access to the latest telemetry packet for AntiTheft or other modules
    RideTelemetryPkt_Packed_t getLastPacket(void) const;   // Return last built packet for relay

private:
       bool m_initialized;                         // True after setup() completes
     BeaconState m_beaconState;                    // Current beacon state (IDLE=30s, MOTION=10s)
    uint32_t m_lastBroadcastMs;                       // Last position beacon timestamp
    
        // Cached GPS data for packet building
     float m_lastLatitude;                             // Last known latitude (for distance calc)
    float m_lastLongitude;                            // Last known longitude (for distance calc)

         // Cumulative tracking state (zeroed per ride session)
      uint32_t m_cumulativeDistanceM;                // Total meters logged since start of ride segment
       int16_t  m_cumulativeGainMx100;               // Cumulative positive elevation gain * 100
    int16_t  m_cumulativeLossMx100;                // Cumulative negative elevation loss * 100

         uint8_t  m_lastEventFlag;                   // Last event flag sent (for delta detection)
     RideTelemetryPkt_Packed_t m_lastPacket;         // Cache of last built packet

       // Broadcast control
    bool shouldBroadcast(void);                     // Returns true based on current state's interval (30s/10s)
     void broadcastPacket(void);                    // Serialize and send the 38-byte packet over mesh
    
        // Packet building helpers
      RideTelemetryPkt_Packed_t buildPacket(NMEA_GPGGA_t *gga, bool motion);   // Assemble from GPS + state
    float calculateDistanceM(float lat1, float lon1, float lat2, float lon2);   // Haversine distance in meters
    
       // Cumulative tracking helpers (for elevation gains/drops)
    void updateCumulativeStats(int16_t currentAltMx100, int16_t prevAltMx100);     // Update gain/loss counters
    
       // Meshtastic-specific serialization helpers
    void serializeToMeshPacket(const RideTelemetryPkt_Packed_t *pkt);   // Pack into Meshtastic payload
};
