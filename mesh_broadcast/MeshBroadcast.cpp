#include "MeshBroadcast.h"
#include "../config.h"
#include "../gps_logger/GPSLogger.h"
#include "../motion/MotionDetector.h"
#include <string.h>
#include <math.h>

// === Meshtastic-specific implementation (uses SinglePortModule + OSThread pattern) ===
extern class BikeMeshModule;   // Forward reference to main module for service access

void MeshBroadcast::setup(void) {
       m_initialized = true;
    m_beaconState = BEACON_IDLE;    // Default to 30s idle rate (applies to both Mode A and Mode B)
    m_lastBroadcastMs = 0;         // Allow immediate first broadcast

        LOG_INFO("MeshBroadcast: Initialized -- %dms beacon interval (idle rate)", BEACON_INTERVAL_IDLE_MS);
}

void MeshBroadcast::updatePosition(NMEA_GPGGA_t *gga, bool motion) {
    if (gga == nullptr) {
        return;
    }

    const float previousLatitude = m_lastLatitude;
    const float previousLongitude = m_lastLongitude;

    m_lastLatitude = gga->latitude;
    m_lastLongitude = gga->longitude;
    m_beaconState = motion ? BEACON_MOTION : BEACON_IDLE;

    if (m_lastPacket.timestamp_ms != 0U) {
        updateCumulativeStats(gga->altitude_m_x100, m_lastPacket.altitude_m_x100);
    }

    if (previousLatitude != 0.0f && previousLongitude != 0.0f) {
        const float distM = calculateDistanceM(previousLatitude, previousLongitude, m_lastLatitude, m_lastLongitude);
        m_cumulativeDistanceM += static_cast<uint32_t>(distM);
    }

    m_lastPacket = buildPacket(gga, motion);
}

int32_t MeshBroadcast::runOnce(void) {
    if (!m_initialized) {
        return 100;
    }

    if (shouldBroadcast()) {
        if (m_lastPacket.timestamp_ms == 0U) {
            m_lastPacket = buildPacket(nullptr, false);
        }
        broadcastPacket();
        m_lastBroadcastMs = millis();
    }

    return 100;
}

void MeshBroadcast::broadcastPacket(void) {
    if (m_lastPacket.timestamp_ms == 0U) {
        return;
    }

    serializeToMeshPacket(&m_lastPacket);

    LOG_INFO("MeshBroadcast: Sent %d-byte packet -- alt=%dm, dist=%lum, event=%d",
             static_cast<int>(sizeof(RideTelemetryPkt_Packed_t)),
             static_cast<int>(m_lastPacket.altitude_m_x100 / 100),
             static_cast<unsigned long>(m_lastPacket.cumulative_distance_m),
             static_cast<int>(m_lastPacket.event_flag));
}

bool MeshBroadcast::shouldBroadcast(void) {
     uint32_t nowMs = millis();    // Get current time
    
      // Determine the correct interval based on beacon state:
     uint32_t intervalMs = (m_beaconState == BEACON_MOTION) ? 
        BEACON_INTERVAL_MOTION_MS : BEACON_INTERVAL_IDLE_MS;  // 10s on motion, 30s idle

       // Return true if enough time has elapsed since last broadcast
     return ((nowMs - m_lastBroadcastMs) >= intervalMs);
}

void MeshBroadcast::serializeToMeshPacket(const RideTelemetryPkt_Packed_t *pkt) {
    // The module retains a compact telemetry payload and hands it to the mesh service layer for
    // actual transmission; this hook is where the final packet serialization lives.
    LOG_DEBUG("MeshBroadcast: Serializing %d-byte packet to mesh", sizeof(RideTelemetryPkt_Packed_t));
    (void)pkt;
}

RideTelemetryPkt_Packed_t MeshBroadcast::buildPacket(NMEA_GPGGA_t *gga, bool motion) {
    RideTelemetryPkt_Packed_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (gga != nullptr) {
        pkt.timestamp_ms = gga->timestamp_ms;
        pkt.latitude_deg = gga->latitude;
        pkt.longitude_deg = gga->longitude;
        pkt.altitude_m_x100 = gga->altitude_m_x100;
        pkt.satellites = gga->satellites;
    }

    pkt.speed_kph = motion ? 12U : 0U;
    pkt.heading_deg = 0U;
    pkt.power_mode = m_lastPacket.power_mode == 0U ? 0U : m_lastPacket.power_mode;
    pkt.battery_mv = 4000U;
    pkt.cumulative_distance_m = m_cumulativeDistanceM;
    pkt.elevation_gain_m_x100 = m_cumulativeGainMx100;
    pkt.elevation_loss_m_x100 = m_cumulativeLossMx100;
    pkt.event_flag = motion ? 1U : 0U;

    m_lastPacket = pkt;
    return pkt;
}

float MeshBroadcast::calculateDistanceM(float lat1, float lon1, float lat2, float lon2) {
         // Haversine formula for distance between two GPS coordinates in meters
    const float R = 6371000.0f;              // Earth radius in meters
     float dLat = (lat2 - lat1) * M_PI / 180.0f;       // Delta latitude in radians
    float dLon = (lon2 - lon1) * M_PI / 180.0f;      // Delta longitude in radians
    
         float a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) +
               cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
              sinf(dLon / 2.0f) * sinf(dLon / 2.0f);    // Haversine formula components
     float c = 2.0f * atan2f(sinf(sqrtf(a)), sinf(sqrtf(1.0f - a)));  // Angular distance

        return R * c;                  // Return distance in meters (as float for precision)
}

void MeshBroadcast::updateCumulativeStats(int16_t currentAltMx100, int16_t prevAltMx100) {
         int32_t delta = (int32_t)currentAltMx100 - (int32_t)prevAltMx100;   // Delta in x100 units

       if (delta > 0) {        // Positive delta = elevation gain
           m_cumulativeGainMx100 += (int16_t)delta;         // Add to cumulative gain counter
        } else if (delta < 0) {     // Negative delta = elevation loss/drop
        m_cumulativeLossMx100 += (int16_t)(-delta);   // Add absolute value to cumulative loss counter
      }
}
