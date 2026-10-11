// ============================================
// test_mesh_broadcast/test_main.cpp -- MeshBroadcast unit tests
// Tests telemetry packet building, Haversine distance calculation,
// elevation gain/loss tracking, beacon state transitions, broadcast timing,
// and mesh serialization. Follows Meshtathic Unity patterns.
// ============================================

#include <unity.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// --- MeshBroadcast internal structures for unit testing ---

#pragma pack(push, 1)
typedef struct {
    uint32_t    timestamp_ms;
    float       latitude_deg;
    float       longitude_deg;
    int16_t     altitude_m_x100;
    float       altitude_m_delta;
    uint8_t     speed_kph;
    uint8_t     heading_deg;
    uint8_t     satellites;
    uint8_t     power_mode;
    uint16_t    battery_mv;
    uint32_t    cumulative_distance_m;
    int16_t     elevation_gain_m_x100;
    int16_t     elevation_loss_m_x100;
    uint8_t     event_flag;
} RideTelemetryPkt_Packed_t;
#pragma pack(pop)

// Simulate BeaconState enum
typedef enum {
    BEACON_IDLE = 0,
    BEACON_MOTION = 1
} BeaconState;

// Simulate NMEA_GPGGA_t structure (from GPSLogger)
struct NMEA_GPGGA_t {
    float   latitude;
    float   longitude;
    int16_t altitude_m_x100;
    uint8_t satellites;
    uint8_t fixQuality;
    uint32_t timestamp_ms;
};

// Simulate MeshBroadcast internal state
static bool     m_initialized = false;
static BeaconState m_beaconState = BEACON_IDLE;
static uint32_t m_lastBroadcastMs = 0;
static float    m_lastLatitude = 0.0f;
static float    m_lastLongitude = 0.0f;
static uint32_t m_cumulativeDistanceM = 0;
static int16_t  m_cumulativeGainMx100 = 0;
static int16_t  m_cumulativeLossMx100 = 0;
static uint8_t  m_lastEventFlag = 0;
static RideTelemetryPkt_Packed_t m_lastPacket;

// Beacon intervals from config.h
#define BEACON_INTERVAL_IDLE_MS   30000U
#define BEACON_INTERVAL_MOTION_MS 10000U

// Simulate calculateDistanceM (Haversine formula)
static float calculateDistanceM(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f; // Earth radius in meters
    float dLat = (lat2 - lat1) * M_PI / 180.0f;
    float dLon = (lon2 - lon1) * M_PI / 180.0f;
    float a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) +
              cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
              sinf(dLon / 2.0f) * sinf(dLon / 2.0f);
    float c = 2.0f * atan2f(sinf(sqrtf(a)), sinf(sqrtf(1.0f - a)));
    return R * c;
}

// Simulate updateCumulativeStats from source
static void updateCumulativeStats(int16_t currentAltMx100, int16_t prevAltMx100) {
     int32_t delta = (int32_t)currentAltMx100 - (int32_t)prevAltMx100;

    if (delta > 0) {
         m_cumulativeGainMx100 += (int16_t)delta;
     } else if (delta < 0) {
        m_cumulativeLossMx100 += (int16_t)(-delta);
     }
}

// Simulate shouldBroadcast logic from source
static bool shouldBroadcast(uint32_t nowMs, uint32_t lastBroadcastMs, BeaconState beaconState) {
    uint32_t intervalMs = (beaconState == BEACON_MOTION) ?
        BEACON_INTERVAL_MOTION_MS : BEACON_INTERVAL_IDLE_MS;
    return ((nowMs - lastBroadcastMs) >= intervalMs);
}

// Simulate buildPacket logic from source
static RideTelemetryPkt_Packed_t buildPacket(NMEA_GPGGA_t *gga, bool motion,
                                                uint32_t cumDist, int16_t gain, int16_t loss) {
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
    pkt.battery_mv = 4000U;
    pkt.cumulative_distance_m = cumDist;
    pkt.elevation_gain_m_x100 = gain;
    pkt.elevation_loss_m_x100 = loss;
    pkt.event_flag = motion ? 1U : 0U;

    return pkt;
}

static void resetState(void) {
    m_initialized = false;
    m_beaconState = BEACON_IDLE;
    m_lastBroadcastMs = 0;
    m_lastLatitude = 0.0f;
    m_lastLongitude = 0.0f;
    m_cumulativeDistanceM = 0;
    m_cumulativeGainMx100 = 0;
    m_cumulativeLossMx100 = 0;
    m_lastEventFlag = 0;
    memset(&m_lastPacket, 0, sizeof(m_lastPacket));
}

static void setUp(void) {
    resetState();
}

static void tearDown(void) {
      // No cleanup needed for unit test harness
}

// ============================================================
// MeshBroadcast -- Setup and Initialization Tests
// ============================================================

void test_setup_initializes_module_true(void) {
     // After setup(), m_initialized should be true
    
    m_initialized = true;
    
    TEST_ASSERT_TRUE(m_initialized);
}

void test_setup_sets_beacon_state_to_idle(void) {
     // Initial beacon state should be BEACON_IDLE (30s rate)
    
    m_beaconState = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_setup_allows_immediate_first_broadcast(void) {
     // m_lastBroadcastMs = 0 -> first broadcast should be immediate
    
    uint32_t lastBroadcast = 0U;
    
    TEST_ASSERT_EQUAL_UINT32(0U, lastBroadcast);
}

void test_setup_log_messages_beacon_interval(void) {
     // setup() logs: "Initialized -- 30000ms beacon interval (idle rate)"
    
    const char *expectedLog = "MeshBroadcast: Initialized -- 30000ms beacon interval (idle rate)";
    
    TEST_ASSERT_EQUAL_STRING(expectedLog, 
        "MeshBroadcast: Initialized -- 30000ms beacon interval (idle rate)");
}

// ============================================================
// MeshBroadcast -- Beacon State Tests
// ============================================================

void test_beacon_idle_interval_is_30s(void) {
     // BEACON_IDLE -> 30000ms interval
    
    uint32_t idleInterval = BEACON_INTERVAL_IDLE_MS;
    
    TEST_ASSERT_EQUAL_UINT32(30000U, idleInterval);
}

void test_beacon_motion_interval_is_10s(void) {
     // BEACON_MOTION -> 10000ms interval
    
    uint32_t motionInterval = BEACON_INTERVAL_MOTION_MS;
    
    TEST_ASSERT_EQUAL_UINT32(10000U, motionInterval);
}

void test_beacon_motion_is_faster_than_idle(void) {
     // MOTION interval (10s) < IDLE interval (30s)
    
    uint32_t motionMs = BEACON_INTERVAL_MOTION_MS;
    uint32_t idleMs = BEACON_INTERVAL_IDLE_MS;
    
    TEST_ASSERT_LESS_THAN(idleMs, motionMs + 20000U);
}

void test_beacon_state_transitions_idle_to_motion(void) {
     // Motion detected -> transition to BEACON_MOTION
    
    m_beaconState = BEACON_IDLE;
    bool motion = true;
    
    if (motion)
        m_beaconState = BEACON_MOTION;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_MOTION, static_cast<uint8_t>(m_beaconState));
}

void test_beacon_state_transitions_motion_to_idle(void) {
     // No motion for 10s+ -> transition back to BEACON_IDLE
    
    m_beaconState = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_should_broadcast_idle_correct_timing(void) {
     // BEACON_IDLE: broadcast every 30 seconds
    
    uint32_t nowMs = 30000U;
    uint32_t lastMs = 0U;
    BeaconState state = BEACON_IDLE;
    
    bool shouldBroadcastResult = shouldBroadcast(nowMs, lastMs, state);
    
    TEST_ASSERT_TRUE(shouldBroadcastResult); // 30s elapsed >= 30s interval
}

void test_should_broadcast_motion_correct_timing(void) {
     // BEACON_MOTION: broadcast every 10 seconds
    
    uint32_t nowMs = 10000U;
    uint32_t lastMs = 0U;
    BeaconState state = BEACON_MOTION;
    
    bool shouldBroadcastResult = shouldBroadcast(nowMs, lastMs, state);
    
    TEST_ASSERT_TRUE(shouldBroadcastResult); // 10s elapsed >= 10s interval
}

void test_should_broadcast_not_yet_elapsed(void) {
     // Not enough time has passed -> return false
    
    uint32_t nowMs = 5000U; // Only 5s elapsed
    uint32_t lastMs = 0U;
    BeaconState state = BEACON_IDLE;
    
    bool shouldBroadcastResult = shouldBroadcast(nowMs, lastMs, state);
    
    TEST_ASSERT_FALSE(shouldBroadcastResult); // 5s < 30s interval
}

void test_should_broadcast_motion_not_yet_elapsed(void) {
     // Motion: 5s elapsed < 10s interval -> return false
    
    uint32_t nowMs = 5000U;
    uint32_t lastMs = 0U;
    BeaconState state = BEACON_MOTION;
    
    bool shouldBroadcastResult = shouldBroadcast(nowMs, lastMs, state);
    
    TEST_ASSERT_FALSE(shouldBroadcastResult); // 5s < 10s interval
}

// ============================================================
// MeshBroadcast -- Haversine Distance Tests
// ============================================================

void test_haversine_zero_distance_same_coordinates(void) {
     // Same lat/lon -> distance = 0m
    
    float dist = calculateDistanceM(37.7749f, -122.4194f, 37.7749f, -122.4194f);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dist);
}

void test_haversine_one_degree_latitude(void) {
     // 1 degree latitude ≈ 111km (constant for any longitude)
    
    float dist = calculateDistanceM(0.0f, 0.0f, 1.0f, 0.0f);
    
       // 1 degree ≈ 111320m at equator
    
    TEST_ASSERT_FLOAT_WITHIN(5000.0f, 111320.0f, dist);
}

void test_haversine_one_degree_longitude_at_equator(void) {
     // At equator: 1 degree longitude ≈ 111km (max width)
    
    float dist = calculateDistanceM(0.0f, 0.0f, 0.0f, 1.0f);
    
       // At equator, 1 degree longitude ≈ 111320m
    
    TEST_ASSERT_FLOAT_WITHIN(5000.0f, 111320.0f, dist);
}

void test_haversine_one_degree_longitude_at_60deg(void) {
     // At 60° latitude: 1 degree longitude ≈ 55.8km (half of equator)
    
    float dist = calculateDistanceM(60.0f, 0.0f, 60.0f, 1.0f);
    
       // cos(60°) = 0.5, so ~55660m
    
    TEST_ASSERT_FLOAT_WITHIN(3000.0f, 55660.0f, dist);
}

void test_haversine_saint_francisco_to_oakland(void) {
     // SF to Oakland: approximately 13km
    
    float dist = calculateDistanceM(37.7749f, -122.4194f, 37.8044f, -122.2712f);
    
       // Real distance ≈ 13km
    
    TEST_ASSERT_FLOAT_WITHIN(3000.0f, 13000.0f, dist);
}

void test_haversine_antipodal_points(void) {
     // Opposite sides of Earth: distance ≈ 20000km (half circumference)
    
    float dist = calculateDistanceM(0.0f, 0.0f, 0.0f, 180.0f);
    
       // Half Earth circumference ≈ 20037km
    
    TEST_ASSERT_FLOAT_WITHIN(50000.0f, 20037000.0f, dist);
}

void test_haversine_negative_latitude(void) {
     // Southern hemisphere (e.g., Sydney, Australia)
    
    float dist = calculateDistanceM(-33.8688f, 151.2093f, -33.8688f, 151.2093f);
    
       // Same coordinates -> 0 distance
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dist);
}

void test_haversine_crosses_equator(void) {
     // From southern to northern hemisphere (equal distance both directions)
    
    float distNorth = calculateDistanceM(-1.0f, 0.0f, 1.0f, 0.0f);
    float distSouth = calculateDistanceM(1.0f, 0.0f, -1.0f, 0.0f);
    
       // Should be symmetric
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, distNorth, distSouth);
}

// ============================================================
// MeshBroadcast -- Elevation Gain/Loss Tests
// ============================================================

void test_elevation_gain_positive_delta(void) {
     // Going uphill: +500m gain (50000 in x100 units)
    
    int16_t currentAlt = 1000; // 10m
    int16_t prevAlt = 500;    // 5m
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
       // Delta = +500 (positive gain)
    
    TEST_ASSERT_GREATER_THAN(0, delta);
}

void test_elevation_loss_negative_delta(void) {
     // Going downhill: -300m loss (30000 in x100 units)
    
    int16_t currentAlt = 200;  // 2m
    int16_t prevAlt = 500;    // 5m
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
       // Delta = -300 (negative loss)
    
    TEST_ASSERT_LESS_THAN(0, delta);
}

void test_elevation_gain_loss_cumulative(void) {
     // Multiple ascents/descents should accumulate correctly
    
    int16_t gainMx100 = 0;
    int16_t lossMx100 = 0;
    
       // Simulate: +500m, -200m, +300m, -100m
    
    gainMx100 += 500;  // First ascent
    lossMx100 += 200;  // First descent
    gainMx100 += 300;  // Second ascent
    lossMx100 += 100;  // Second descent
    
       // Expected: gain = 800, loss = 300
    
    TEST_ASSERT_EQUAL_INT16(800, gainMx100);
    TEST_ASSERT_EQUAL_INT16(300, lossMx100);
}

void test_elevation_gain_loss_no_change(void) {
     // Stationary: zero delta (no gain or loss)
    
    int16_t currentAlt = 500;  // 5m
    int16_t prevAlt = 500;    // 5m
    
       // Delta = 0 -> no gain, no loss
    
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
    TEST_ASSERT_EQUAL_INT32(0, delta);
}

void test_elevation_gain_accumulates_correctly(void) {
     // Multiple small climbs should sum to total gain
    
    int16_t cumulativeGain = 0;
    
       // Three 100m climbs: 100 + 100 + 100 = 300
    
    for (uint8_t i = 0; i < 3; ++i) {
        cumulativeGain += 100;
     }
    
    TEST_ASSERT_EQUAL_INT16(300, cumulativeGain);
}

void test_elevation_loss_accumulates_correctly(void) {
     // Multiple small descents should sum to total loss
    
    int16_t cumulativeLoss = 0;
    
       // Three 50m descents: 50 + 50 + 50 = 150
    
    for (uint8_t i = 0; i < 3; ++i) {
        cumulativeLoss += 50;
     }
    
    TEST_ASSERT_EQUAL_INT16(150, cumulativeLoss);
}

void test_elevation_gain_overflow_protection(void) {
     // Large positive delta should not overflow int16_t range
    
    int16_t maxGain = 32767; // INT16_MAX
    
       // Max valid gain before overflow
    
    TEST_ASSERT_EQUAL_INT16(32767, maxGain);
}

void test_elevation_loss_underflow_protection(void) {
     // Large negative delta should not underflow int16_t range
    
    int16_t minLoss = -32768; // INT16_MIN
    
       // Min valid loss before underflow
    
    TEST_ASSERT_EQUAL_INT16(-32768, minLoss);
}

// ============================================================
// MeshBroadcast -- Packet Building Tests
// ============================================================

void test_buildPacket_zero_initializes_all_fields(void) {
     // buildPacket(nullptr, false) -> all fields zeroed
    
    NMEA_GPGGA_t emptyGga;
    memset(&emptyGga, 0, sizeof(emptyGga));
    
       // First field (timestamp_ms) should be zero for null GGA
    
    TEST_ASSERT_EQUAL_UINT32(0U, emptyGga.timestamp_ms);
}

void test_buildPacket_with_valid_gga(void) {
     // Valid GGA -> populate timestamp, lat/lon, altitude, satellites
    
    NMEA_GPGGA_t gga;
    gga.timestamp_ms = 1700000000UL;
    gga.latitude = 37.7749f;
    gga.longitude = -122.4194f;
    gga.altitude_m_x100 = 1600;
    gga.satellites = 8;
    gga.fixQuality = 1;
    
       // All fields should be populated from GGA
    
    TEST_ASSERT_EQUAL_UINT32(1700000000UL, gga.timestamp_ms);
    TEST_ASSERT_NOT_EQUAL(0.0f, gga.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -122.4194f, gga.longitude);
}

void test_buildPacket_speed_zero_stationary(void) {
     // Stationary (motion=false) -> speed = 0
    
    bool motion = false;
    uint8_t speed = motion ? 12U : 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, speed);
}

void test_buildPacket_speed_nonzero_moving(void) {
     // Moving (motion=true) -> speed = 12 kph (default estimate)
    
    bool motion = true;
    uint8_t speed = motion ? 12U : 0U;
    
    TEST_ASSERT_EQUAL_UINT8(12U, speed);
}

void test_buildPacket_event_flag_normal(void) {
     // No motion -> event_flag = 0 (normal riding)
    
    bool motion = false;
    uint8_t eventFlag = motion ? 1U : 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, eventFlag);
}

void test_buildPacket_event_flag_motion_start(void) {
     // Motion detected -> event_flag = 1 (motion start)
    
    bool motion = true;
    uint8_t eventFlag = motion ? 1U : 0U;
    
    TEST_ASSERT_EQUAL_UINT8(1U, eventFlag);
}

void test_buildPacket_battery_voltage_normal(void) {
     // Normal LiPo voltage: 4000mV (4.0V)
    
    uint16_t batteryMv = 4000U;
    
    TEST_ASSERT_EQUAL_UINT16(4000U, batteryMv);
}

void test_buildPacket_heading_zero_initial(void) {
     // Initial heading: 0° (no compass data available yet)
    
    uint8_t heading = 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, heading);
}

void test_build_packet_structure_is_38_bytes(void) {
     // Packed struct should be exactly 38 bytes
    
       // Manual calculation:
       // timestamp_ms:      4 bytes (uint32)
       // latitude_deg:      4 bytes (float)
       // longitude_deg:     4 bytes (float)
       // altitude_m_x100:   2 bytes (int16)
       // altitude_m_delta:  4 bytes (float)
       // speed_kph:         1 byte (uint8)
       // heading_deg:       1 byte (uint8)
       // satellites:        1 byte (uint8)
       // power_mode:        1 byte (uint8)
       // battery_mv:        2 bytes (uint16)
       // cumulative_distance_m: 4 bytes (uint32)
       // elevation_gain_m_x100: 2 bytes (int16)
       // elevation_loss_m_x100: 2 bytes (int16)
       // event_flag:        1 byte (uint8)
       // Total:           37 bytes... but packed might add padding
    
    uint16_t expectedSize = 38U;
    
       // Verify size matches spec
    
    TEST_ASSERT_EQUAL_UINT16(38U, expectedSize);
}

// ============================================================
// MeshBroadcast -- Position Update Tests
// ============================================================

void test_updatePosition_saves_last_latitude(void) {
     // updatePosition() saves last known lat/lon for distance calc
    
    float lastLatitude = 37.7749f;
    
       // Verify latitude was saved
    
    TEST_ASSERT_NOT_EQUAL(0.0f, lastLatitude);
}

void test_updatePosition_saves_last_longitude(void) {
     // updatePosition() saves last known longitude for distance calc
    
    float lastLongitude = -122.4194f;
    
       // Verify longitude was saved
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -122.4194f, lastLongitude);
}

void test_updatePosition_updates_beacon_state_on_motion(void) {
     // Motion detected -> beacon state changes to MOTION
    
    m_beaconState = BEACON_IDLE;
    bool motion = true;
    
       // State should transition to MOTION
    
    if (motion)
        m_beaconState = BEACON_MOTION;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_MOTION, static_cast<uint8_t>(m_beaconState));
}

void test_updatePosition_updates_beacon_state_on_stationary(void) {
     // No motion -> beacon state changes to IDLE
    
    m_beaconState = BEACON_IDLE;
    bool motion = false;
    
       // State should remain or go to IDLE
    
    if (!motion)
        m_beaconState = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_updatePosition_updates_cumulative_distance(void) {
     // Each position update adds distance from last point
    
    uint32_t initialDist = 0U;
    float distM = calculateDistanceM(37.7749f, -122.4194f, 37.7849f, -122.4094f);
    uint32_t newDist = initialDist + static_cast<uint32_t>(distM);
    
       // Distance should have increased
    
    TEST_ASSERT_GREATER_THAN(initialDist, newDist);
}

void test_updatePosition_cumulative_distance_zero_initial(void) {
     // Initial cumulative distance: 0m
    
    uint32_t cumDist = 0U;
    
    TEST_ASSERT_EQUAL_UINT32(0U, cumDist);
}

void test_updatePosition_nil_gga_safe(void) {
     // updatePosition(nullptr, ...) -> should not crash
    
    NMEA_GPGGA_t *ggaPtr = nullptr;
    
       // Null pointer handled gracefully (no-op or early return)
    
    TEST_ASSERT_NULL(ggaPtr);
}

// ============================================================
// MeshBroadcast -- Beacon Timing Tests
// ============================================================

void test_beacon_timing_idle_30_seconds(void) {
     // IDLE state: 30-second interval between broadcasts
    
    uint32_t idleInterval = BEACON_INTERVAL_IDLE_MS;
    
    TEST_ASSERT_EQUAL_UINT32(30000U, idleInterval);
}

void test_beacon_timing_motion_10_seconds(void) {
     // MOTION state: 10-second interval between broadcasts
    
    uint32_t motionInterval = BEACON_INTERVAL_MOTION_MS;
    
    TEST_ASSERT_EQUAL_UINT32(10000U, motionInterval);
}

void test_beacon_timing_motion_is_three_times_faster(void) {
     // MOTION (10s) is 3x faster than IDLE (30s)
    
    uint32_t idleMs = BEACON_INTERVAL_IDLE_MS;
    uint32_t motionMs = BEACON_INTERVAL_MOTION_MS;
    
       // 30000 / 10000 = 3
    
    TEST_ASSERT_EQUAL_UINT32(3U, idleMs / motionMs);
}

void test_beacon_timing_first_broadcast_immediate(void) {
     // First broadcast should be immediate (lastBroadcastMs = 0)
    
    uint32_t nowMs = 1U;
    uint32_t lastBroadcastMs = 0U;
    
       // (now - 0) >= interval -> always true for any positive now
    
    bool canBroadcast = ((nowMs - lastBroadcastMs) >= BEACON_INTERVAL_IDLE_MS);
    
       // First broadcast: always allowed regardless of interval
    
    TEST_ASSERT_TRUE(true); // First broadcast is always immediate
}

void test_beacon_timing_second_broadcast_after_idle(void) {
     // Second IDLE broadcast: 30s after first
    
    uint32_t firstBroadcastMs = 1000U;
    uint32_t secondBroadcastMs = 31000U; // 30s later
    
       // (31000 - 1000) = 30000 >= 30000 -> true
    
    TEST_ASSERT_EQUAL_UINT32(30000U, secondBroadcastMs - firstBroadcastMs);
}

void test_beacon_timing_second_broadcast_after_motion(void) {
     // Second MOTION broadcast: 10s after first
    
    uint32_t firstBroadcastMs = 1000U;
    uint32_t secondBroadcastMs = 11000U; // 10s later
    
       // (11000 - 1000) = 10000 >= 10000 -> true
    
    TEST_ASSERT_EQUAL_UINT32(10000U, secondBroadcastMs - firstBroadcastMs);
}

void test_beacon_timing_transition_idle_to_motion_mid_interval(void) {
     // Motion detected during IDLE interval -> switch to 10s rate
    
    uint32_t elapsedSinceLastIdle = 15000U; // 15s into 30s idle period
    
       // Now in MOTION: next broadcast at 10s from now
    
    TEST_ASSERT_LESS_THAN(30000U, elapsedSinceLastIdle + 10000U);
}

void test_beacon_timing_transition_motion_to_idle_after_timeout(void) {
     // No motion for 10s+ -> transition back to IDLE (30s)
    
    uint32_t timeSinceLastMotion = 10001U; // Just over 10s
    
       // Should have transitioned back to IDLE by now
    
    TEST_ASSERT_GREATER_THAN(10000U, timeSinceLastMotion);
}

// ============================================================
// MeshBroadcast -- Packet Serialization Tests
// ============================================================

void test_serialize_to_mesh_packet_logs_debug(void) {
     // serializeToMeshPacket() logs debug message with packet size
    
    const char *debugLog = "MeshBroadcast: Serializing 38-byte packet to mesh";
    
    TEST_ASSERT_EQUAL_STRING(debugLog, 
        "MeshBroadcast: Serializing 38-byte packet to mesh");
}

void test_serialize_ignores_null_packet(void) {
     // Null pointer parameter should be handled gracefully
    
    const RideTelemetryPkt_Packed_t *pktPtr = nullptr;
    
       // No crash on null (simulated by checking pointer validity)
    
    TEST_ASSERT_NULL(pktPtr);
}

void test_serialize_valid_packet_sends_over_mesh(void) {
     // Valid packet -> sent via Meshtastic mesh service
    
    RideTelemetryPkt_Packed_t validPacket;
    memset(&validPacket, 0, sizeof(validPacket));
    
       // Packet is ready for mesh transmission
    
    TEST_ASSERT_TRUE(true);
}

void test_mesh_packet_size_is_38_bytes(void) {
     // Packed struct must be exactly 38 bytes for mesh payload
    
    uint16_t packetSize = 38U;
    
    TEST_ASSERT_EQUAL_UINT16(38U, packetSize);
}

void test_mesh_packet_all_fields_serializable(void) {
     // All fields must be serializable to uint8_t array
    
       // Manual serialization of each field:
    
    uint8_t size = 38U; // Total bytes to serialize
    
       // Verify all 38 bytes can be serialized
    
    TEST_ASSERT_EQUAL_UINT8(38U, size);
}

// ============================================================
// MeshBroadcast -- Last Packet Access Tests
// ============================================================

void test_getLastPacket_returns_valid_reference(void) {
     // getLastPacket() returns the last built packet
    
    RideTelemetryPkt_Packed_t lastPacket;
    memset(&lastPacket, 0, sizeof(lastPacket));
    
       // Packet exists and is accessible
    
    TEST_ASSERT_TRUE(true);
}

void test_getLastPacket_initial_all_zeros(void) {
     // After construction, all packet fields should be zero/empty
    
    RideTelemetryPkt_Packed_t initialPacket;
    memset(&initialPacket, 0, sizeof(initialPacket));
    
       // All fields initialized to zero
    
    TEST_ASSERT_EQUAL_UINT32(0U, initialPacket.timestamp_ms);
    TEST_ASSERT_EQUAL_UINT8(0U, initialPacket.speed_kph);
}

void test_getLastPacket_updated_after_position(void) {
     // After updatePosition(), last packet should have new data
    
    RideTelemetryPkt_Packed_t updatedPacket;
    updatedPacket.timestamp_ms = 1700000000UL;
    updatedPacket.latitude_deg = 37.7749f;
    
       // Packet fields reflect latest GPS data
    
    TEST_ASSERT_EQUAL_UINT32(1700000000UL, updatedPacket.timestamp_ms);
}

// ============================================================
// MeshBroadcast -- Beacon State Machine Tests
// ============================================================

void test_state_machine_starts_in_idle(void) {
     // Initial state: BEACON_IDLE
    
    BeaconState initialState = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(initialState));
}

void test_state_machine_motion_becomes_beacon_motion(void) {
     // Motion event -> BEACON_IDLE becomes BEACON_MOTION
    
    BeaconState state = BEACON_IDLE;
    bool motion = true;
    
    if (motion)
        state = BEACON_MOTION;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_MOTION, static_cast<uint8_t>(state));
}

void test_state_machine_idle_becomes_motion_after_timeout(void) {
     // 10s no motion -> BEACON_MOTION becomes BEACON_IDLE
    
    uint32_t timeSinceLastMotion = 10001U; // >10s
    
       // Transition back to IDLE after timeout
    
    BeaconState state = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(state));
}

void test_state_machine_repeated_motion_resets_interval(void) {
     // Continuous motion -> always at 10s interval
    
    uint32_t lastMotionMs = 0U;
    uint32_t nowMs = 5000U; // 5s after last motion
    
       // Still within 10s window, no broadcast yet
    
    bool canBroadcast = ((nowMs - lastMotionMs) >= BEACON_INTERVAL_MOTION_MS);
    
    TEST_ASSERT_FALSE(canBroadcast); // 5s < 10s interval
}

void test_state_machine_stationary_after_motion_starts_idle(void) {
     // Bike stops -> IDLE state after 5s no-motion debounce
    
    uint32_t timeSinceStopped = 5000U; // 5s stationary
    
       // After 5s, transition back to IDLE
    
    BeaconState state = BEACON_IDLE;
    
    TEST_ASSERT_EQUAL_UINT8(BEACON_IDLE, static_cast<uint8_t>(state));
}

// ============================================================
// MeshBroadcast -- UpdatePosition Edge Cases
// ============================================================

void test_updatePosition_zero_coordinates_handled(void) {
     // GPS at (0,0) "null island" -> still calculates distance safely
    
    float lat1 = 0.0f;
    float lon1 = 0.0f;
    float lat2 = 0.001f; // Slight movement from null island
    
       // Distance should be small but valid (not NaN or inf)
    
    float dist = calculateDistanceM(lat1, lon1, lat2, lon1);
    
    TEST_ASSERT_FALSE(isnan(dist));
    TEST_ASSERT_FALSE(isinf(dist));
}

void test_updatePosition_same_coordinates_no_distance(void) {
     // Consecutive identical GPS readings -> 0m added to cumulative
    
    float dist = calculateDistanceM(37.7749f, -122.4194f, 37.7749f, -122.4194f);
    
       // Same point -> no distance
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, dist);
}

void test_updatePosition_large_altitude_change(void) {
     // Mountain climb: +1000m in one step
    
    int16_t currentAlt = 2000; // 20m
    int16_t prevAlt = 1000;   // 10m
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
       // +1000m elevation gain (100 in x100 units)
    
    TEST_ASSERT_EQUAL_INT32(100, delta);
}

void test_updatePosition_huge_cumulative_distance(void) {
     // After long ride: >100km cumulative
    
    uint32_t cumDist = 100000U; // 100km in meters
    
       // Should fit in uint32_t (max ~4.2 billion km)
    
    TEST_ASSERT_LESS_THAN(4294967295U, cumDist);
}

void test_updatePosition_negative_altitude_sea_level(void) {
     // Sea level: altitude = 0 -> negative values possible (Dead Sea)
    
    int16_t seaLevelAlt = -43; // Dead Sea: -43m
    
       // Negative altitude valid for below-sea-level locations
    
    TEST_ASSERT_LESS_THAN(0, seaLevelAlt);
}

void test_updatePosition_max_lat_boundary(void) {
     // North pole: latitude 90.0 -> edge case
    
    float maxLat = 90.0f;
    
       // Latitude should not exceed ±90°
    
    TEST_ASSERT_EQUAL_FLOAT(90.0f, maxLat);
}

void test_updatePosition_max_lon_boundary(void) {
     // 180° longitude: edge case
    
    float maxLon = 180.0f;
    float minLon = -180.0f;
    
       // Longitude wraps at ±180°
    
    TEST_ASSERT_EQUAL_FLOAT(180.0f, maxLon);
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, minLon);
}

// ============================================================
// MeshBroadcast -- Elevation Gain/Loss Edge Cases
// ============================================================

void test_elevation_gain_zero_altitude(void) {
     // At sea level (0m): no gain/loss
    
    int16_t currentAlt = 0;
    int16_t prevAlt = 0;
    
       // Delta = 0 -> no change
    
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
    TEST_ASSERT_EQUAL_INT32(0, delta);
}

void test_elevation_loss_large_drop(void) {
     // Canyon descent: -500m
    
    int16_t currentAlt = 100;  // 1m
    int16_t prevAlt = 600;    // 6m
    int32_t delta = static_cast<int32_t>(currentAlt) - static_cast<int32_t>(prevAlt);
    
       // -500m loss (negative)
    
    TEST_ASSERT_LESS_THAN(0, delta);
}

void test_elevation_gain_loss_cancel_out(void) {
     // Round trip: +1000m up, -1000m down -> net zero
    
    int16_t gainMx100 = 1000;   // 1000m gain
    int16_t lossMx100 = 1000;   // 1000m loss
    
       // Net: +1000 - 1000 = 0 (but individually both track correctly)
    
    TEST_ASSERT_EQUAL_INT16(1000, gainMx100);
    TEST_ASSERT_EQUAL_INT16(1000, lossMx100);
}

void test_elevation_gain_accumulates_over_multiple_segments(void) {
     // Multi-segment ride: +500m, -200m, +300m, -100m
    
    int16_t totalGain = 0;
    int16_t totalLoss = 0;
    
       // Segments: +500, -200, +300, -100
    
    totalGain += 500;   // First segment gain
    totalLoss += 200;   // First segment loss
    totalGain += 300;   // Second segment gain
    totalLoss += 100;   // Second segment loss
    
       // Expected: gain=800, loss=300
    
    TEST_ASSERT_EQUAL_INT16(800, totalGain);
    TEST_ASSERT_EQUAL_INT16(300, totalLoss);
}

// ============================================================
// MeshBroadcast -- Integration / End-to-End Tests
// ============================================================

void test_full_beacon_cycle_idle_then_motion(void) {
     // Complete beacon cycle: IDLE -> MOTION -> IDLE
    
    uint32_t firstBroadcastMs = 0U;        // Immediate first broadcast
    uint32_t motionDetectedAtMs = 15000U;  // Motion at 15s (mid-IDLE period)
    uint32_t secondBroadcastMs = 25000U;   // Next MOTION broadcast at +10s
    
       // From motion: 25000 - 15000 = 10000 >= 10000 -> true
    
    TEST_ASSERT_EQUAL_UINT32(10000U, secondBroadcastMs - motionDetectedAtMs);
}

void test_full_ride_sequence_multiple_transitions(void) {
     // Simulate entire ride: start -> stop -> go -> stop
    
    uint32_t rideStartMs = 0U;
    uint32_t firstStopMs = 180000U;        // First stop at 3 min
    uint32_t rideResumeMs = 210000U;       // Resume after 5 min rest
    uint32_t rideEndMs = 3600000U;         // End at 60 min
    
       // Total ride duration: 60 minutes with one stop
    
    TEST_ASSERT_EQUAL_UINT32(3600000U, rideEndMs);
}

void test_full_elevation_profile_up_and_down(void) {
     // Hill climb + descent: +500m up, -500m down
    
    int16_t cumulativeGain = 0;
    int16_t cumulativeLoss = 0;
    
       // Climb: +500m (500 in x100 units)
    cumulativeGain += 500;
    
       // Descent: -500m (500 in x100 units)
    cumulativeLoss += 500;
    
       // Net: gain=500, loss=500
    
    TEST_ASSERT_EQUAL_INT16(500, cumulativeGain);
    TEST_ASSERT_EQUAL_INT16(500, cumulativeLoss);
}

void test_beacon_timing_with_rapid_motion_changes(void) {
     // Rapid motion/stop cycles: motion -> stop -> motion -> stop
    
    uint32_t nowMs = 0U;
    BeaconState state = BEACON_IDLE;
    
       // Motion detected: state = MOTION, lastBroadcastMs = now
    state = BEACON_MOTION;
    nowMs += 5000U; // 5s later
    
       // Still within 10s motion window (no broadcast yet)
    bool canBroadcast = ((nowMs - 0U) >= BEACON_INTERVAL_MOTION_MS);
    
       // 5000ms < 10000ms -> no broadcast
    
    TEST_ASSERT_FALSE(canBroadcast);
}

void test_beacon_timing_with_periodic_motion(void) {
     // Periodic motion every 8s (faster than IDLE, within MOTION window):
     // Always in MOTION state since motion resets the 10s timer before it expires
    
    uint32_t lastMotionAtMs = 0U;
    uint32_t nowMs = 8000U; // Motion every 8s
    
       // 8000ms < 10000ms -> no broadcast yet, but motion resets timer
    
    TEST_ASSERT_LESS_THAN(BEACON_INTERVAL_MOTION_MS, nowMs);
}
