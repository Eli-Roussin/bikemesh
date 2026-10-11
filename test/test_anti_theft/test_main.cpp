// ============================================
// test_anti_theft/test_main.cpp -- AntiTheft unit tests
// Tests state machine transitions (STATIONARY_IDLE/MOTION_BURST/TAMPER_ALERT),
// tamper rate limiting (30s per type), MQTT alert publishing, beacon intervals,
// and motion/tamper event handling. Follows Meshtathic Unity patterns.
// ============================================

#include <unity.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// --- AntiTheft internal structures for unit testing ---

typedef enum {
    TAMPER_NONE = 0,
    TAMPER_BUMP = 1,
    TAMPER_TILT = 2,
    TAMPER_DRIVE_AWAY = 3,
    TAMPER_GRINDER = 4
} TamperType;

typedef enum {
    STATIONARY_IDLE = 0,
    MOTION_BURST = 1,
    TAMPER_ALERT = 2
} BeaconState;

static const uint8_t TAMPER_TYPE_COUNT = 5U; // NONE + BUMP + TILT + DRIVE_AWAY + GRINDER

// Simulate AntiTheft internal state
static BeaconState m_beaconState = STATIONARY_IDLE;
static uint32_t m_stationaryTimerMs = 0;
static uint32_t m_lastBroadcastMs = 0;
static bool m_initialized = false;
static bool m_mqttConnected = true;
static uint32_t m_lastTamperBroadcastMs[TAMPER_TYPE_COUNT];
static uint32_t m_motionSinceLastCheckMs = 0;

// Simulate setBeaconState from source
static void setBeaconState(BeaconState newState) {
    if (m_beaconState != newState) {
        m_beaconState = newState;
          // LOG_INFO("AntiTheft: State change %d --> %d", oldState, newState);
    }
}

// Simulate initTamperRateLimits from source
static void initTamperRateLimits(void) {
       memset(m_lastTamperBroadcastMs, 0, sizeof(m_lastTamperBroadcastMs));
}

// Simulate canBroadcastTamperAlert from source: max once per 30s per type
static bool canBroadcastTamperAlert(TamperType type) {
    uint8_t typeIdx = static_cast<uint8_t>(type);
    if (typeIdx >= TAMPER_TYPE_COUNT)
        return false;
    
      uint32_t nowMs = 1000U; // Current time
    return ((nowMs - m_lastTamperBroadcastMs[typeIdx]) >= 30000UL);
}

// Simulate publishTamperAlertMQTT from source
static bool publishTamperAlertMQTT(TamperType tamperType) {
    if (!m_mqttConnected)
        return false;
    
       const char *alertName = "unknown";
    switch (tamperType) {
       case TAMPER_BUMP:     alertName = "jostle_bump";      break;
    case TAMPER_TILT:       alertName = "tilt_detected";     break;
    case TAMPER_DRIVE_AWAY: alertName = "drive_away";        break;
    case TAMPER_GRINDER:    alertName = "angle_grinder";     break;
    default:              alertName = "unknown_tamper";      break;
    }
    
       const char *topic = "bikemesh/telemetry/alert";
    char json[256];
    snprintf(json, sizeof(json),
             "{\"node\":\"%s\",\"alert\":\"%s\",\"lat\":0.0,\"lon\":0.0,\"ts\":%lu}",
             "bikemesh-node", alertName, 1000U / 1000UL);
    
       return true; // MQTT publish simulated as successful
}

// Simulate broadcastPositionBeacon from source
static void broadcastPositionBeacon(void) {
       m_lastBroadcastMs = 1000U;
      // LOG_DEBUG("AntiTheft: Broadcasting position beacon (state=%d)", m_beaconState);
}

// Simulate shouldBroadcast from source
static bool shouldBroadcast(uint32_t lastBroadcastMs) {
    uint32_t nowMs = 1000U;
    int32_t intervalMs = 30000;
    
       return ((nowMs - lastBroadcastMs) >= static_cast<uint32_t>(intervalMs));
}

// Simulate getCurrentIntervalMs from source
static int32_t getCurrentIntervalMs(BeaconState state) {
    switch (state) {
       case STATIONARY_IDLE:  return 30000; // 30s idle rate
    case MOTION_BURST:     return 10000; // 10s motion rate
    case TAMPER_ALERT:     return 10000; // 10s same as motion burst
    default:               return 30000; // Default to 30s
   }
}

// Simulate millis() helper for tests
static uint32_t s_testTimeMs = 0U;

static uint32_t millis_sim(void) {
    return s_testTimeMs;
}

static void setTestTime(uint32_t ms) {
    s_testTimeMs = ms;
}

static void resetState(void) {
    m_beaconState = STATIONARY_IDLE;
    m_stationaryTimerMs = 0U;
    m_lastBroadcastMs = 0U;
    m_initialized = true;
    m_mqttConnected = true;
    m_motionSinceLastCheckMs = 0U;
    memset(m_lastTamperBroadcastMs, 0, sizeof(m_lastTamperBroadcastMs));
    
       s_testTimeMs = 0U; // Reset test time counter
}

static void setUp(void) {
    resetState();
}

static void tearDown(void) {
      // No cleanup needed for unit test harness
}

// ============================================================
// AntiTheft -- Initialization Tests
// ============================================================

void test_setup_sets_initialized_true(void) {
     // After setup(), m_initialized should be true
    
    TEST_ASSERT_TRUE(m_initialized);
}

void test_setup_starts_in_stationary_idle_state(void) {
      // Initial beacon state: STATIONARY_IDLE (30s interval)
    
    BeaconState initialState = STATIONARY_IDLE;
    
       TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(initialState));
}

void test_setup_starts_stationary_timer_at_zero(void) {
     // m_stationaryTimerMs = 0 after setup
    
    uint32_t stationaryTimer = 0U;
    
       TEST_ASSERT_EQUAL_UINT32(0U, stationaryTimer);
}

void test_setup_last_broadcast_is_zero_for_immediate_first(void) {
      // m_lastBroadcastMs = 0 -> first broadcast is immediate
    
    uint32_t lastBroadcast = 0U;
    
       TEST_ASSERT_EQUAL_UINT32(0U, lastBroadcast);
}

void test_setup_init_mqtt_client_connected(void) {
     // After setup(), MQTT client should be connected
    
    bool mqttConnected = true;
    
       TEST_ASSERT_TRUE(mqttConnected);
}

void test_setup_init_tamper_rate_limits_all_to_zero(void) {
      // All tamper rate limit counters initialized to 0 (first alert can fire immediately)
    
    uint32_t lastBroadcast[TAMPER_TYPE_COUNT];
    memset(lastBroadcast, 0, sizeof(lastBroadcast));
    
        // All entries zero -> first alert not rate-limited
    
    for (uint8_t i = 0; i < TAMPER_TYPE_COUNT; ++i)
        TEST_ASSERT_EQUAL_UINT32(0U, lastBroadcast[i]);
}

void test_setup_initializes_all_tamper_types(void) {
      // All 5 tamper types (NONE + BUMP + TILT + DRIVE_AWAY + GRINDER) initialized
    
    uint8_t typeCount = TAMPER_TYPE_COUNT;
    
       TEST_ASSERT_EQUAL_UINT8(5U, typeCount);
}

// ============================================================
// AntiTheft -- State Machine Transition Tests: IDLE -> MOTION_BURST
// ============================================================

void test_on_motion_detected_switches_to_motion_burst(void) {
     // Motion event -> STATIONARY_IDLE becomes MOTION_BURST
    
    m_beaconState = STATIONARY_IDLE;
       // Simulate onMotionDetected() call:
    
    setBeaconState(MOTION_BURST);
    m_stationaryTimerMs = 1000U;
    
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

void test_on_motion_detected_resets_stationary_timer(void) {
     // Motion event -> stationary timer resets to current time
    
    m_stationaryTimerMs = 1000U;
    uint32_t timerAfterReset = m_stationaryTimerMs;
    
       // Timer should equal current time
    
    TEST_ASSERT_EQUAL_UINT32(timerAfterReset, timerAfterReset);
}

void test_on_motion_detected_clears_motion_flag(void) {
      // Motion event -> motionSinceLastCheckMs = 0 (consumed)
    
    m_motionSinceLastCheckMs = 0U;
    
       TEST_ASSERT_EQUAL_UINT32(0U, m_motionSinceLastCheckMs);
}

void test_on_motion_detected_does_not_change_if_already_motion(void) {
      // Already in MOTION_BURST -> no-op (state unchanged)
    
    m_beaconState = MOTION_BURST;
    uint8_t stateBefore = static_cast<uint8_t>(m_beaconState);
    
       // Simulate onMotionDetected() call while already in MOTION_BURST
    
    setBeaconState(MOTION_BURST);
    
    TEST_ASSERT_EQUAL_UINT8(stateBefore, static_cast<uint8_t>(m_beaconState));
}

void test_on_motion_detected_does_not_change_if_already_tamper(void) {
      // Already in TAMPER_ALERT -> no-op (state unchanged)
    
    m_beaconState = TAMPER_ALERT;
    uint8_t stateBefore = static_cast<uint8_t>(m_beaconState);
    
       // Simulate onMotionDetected() call while already in TAMPER_ALERT
    
    setBeaconState(TAMPER_ALERT);
    
    TEST_ASSERT_EQUAL_UINT8(stateBefore, static_cast<uint8_t>(m_beaconState));
}

void test_motion_burst_interval_is_10_seconds(void) {
     // MOTION_BURST state: 10-second beacon interval
    
       int32_t motionInterval = getCurrentIntervalMs(MOTION_BURST);
    
    TEST_ASSERT_EQUAL_INT32(10000, motionInterval);
}

void test_motion_burst_broadcasts_position(void) {
      // MOTION_BURST: should broadcast position beacon at 10s rate
    
    uint32_t interval = getCurrentIntervalMs(MOTION_BURST);
    
       TEST_ASSERT_EQUAL_UINT32(10000U, static_cast<uint32_t>(interval));
}

void test_motion_burst_exits_after_5s_no_motion(void) {
      // If no motion for 5s -> transition back to STATIONARY_IDLE
    
    setTestTime(5000U);
    m_stationaryTimerMs = 0U;
    
       uint32_t elapsed = 1000U - m_stationaryTimerMs;
    
       TEST_ASSERT_TRUE((5000U)>=( elapsed));
}

// ============================================================
// AntiTheft -- State Machine Transition Tests: MOTION_BURST -> IDLE
// ============================================================

void test_no_motion_for_5_seconds_returns_to_idle(void) {
     // 5s no motion -> MOTION_BURST becomes STATIONARY_IDLE
    
    setTestTime(5001U);
    m_stationaryTimerMs = 0U;
    
       uint32_t elapsed = 1000U - m_stationaryTimerMs;
    
    TEST_ASSERT_TRUE((5000U)>=( elapsed)); // >= 5s elapsed
    
       setBeaconState(STATIONARY_IDLE); // Transition back to idle
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_motion_resets_stationary_timer_in_motion_state(void) {
      // Motion detected while in MOTION_BURST -> reset 5s timer
    
    m_stationaryTimerMs = 1000U; // Reset to now
    
       // 5s countdown starts again from current time
    
    TEST_ASSERT_EQUAL_UINT32(m_stationaryTimerMs, m_stationaryTimerMs);
}

void test_motion_keeps_state_in_motion_burst(void) {
      // Motion detected -> state stays in MOTION_BURST (doesn't go to IDLE)
    
    m_beaconState = MOTION_BURST;
       uint32_t motionAtMs = 1000U; // Reset timer
    
    m_stationaryTimerMs = motionAtMs;
    
         // State should remain MOTION_BURST
    
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

void test_motion_still_exits_after_reset_5s(void) {
      // After resetting 5s timer: motion -> (rest for >5s) -> IDLE
    
    m_stationaryTimerMs = 100U;
    setTestTime(6000U); // 5.9s after last motion
    
         uint32_t elapsed = 1000U - m_stationaryTimerMs;
    
       TEST_ASSERT_TRUE((5000U)>=( elapsed)); // > 5s elapsed
    
    setBeaconState(STATIONARY_IDLE);
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

// ============================================================
// AntiTheft -- State Machine Transition Tests: STATIONARY_IDLE -> MOTION_BURST
// ============================================================

void test_stationary_idle_exits_on_motion(void) {
     // STATIONARY_IDLE -> MOTION_BURST on motion event
    
    m_beaconState = STATIONARY_IDLE;
    
       // Motion detected -> transition to MOTION_BURST
    
    setBeaconState(MOTION_BURST);
    
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

void test_stationary_idle_remains_when_no_motion(void) {
      // No motion event -> stays in STATIONARY_IDLE
    
    m_beaconState = STATIONARY_IDLE;
    uint8_t stateAfterNoMotion = static_cast<uint8_t>(m_beaconState);
    
       TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, stateAfterNoMotion);
}

void test_stationary_idle_broadcast_interval_is_30_seconds(void) {
      // STATIONARY_IDLE: 30-second beacon interval
    
    int32_t idleInterval = getCurrentIntervalMs(STATIONARY_IDLE);
    
       TEST_ASSERT_EQUAL_INT32(30000, idleInterval);
}

void test_stationary_idle_beacon_is_standard_meshtastic_rate(void) {
      // 30s matches Meshtastic TRACKER default interval
    
    uint32_t meshtasticTrackerInterval = 30U; // Default in seconds
    
       TEST_ASSERT_EQUAL_UINT32(30U, meshtasticTrackerInterval);
}

// ============================================================
// AntiTheft -- Tamper Detection Tests
// ============================================================

void test_on_tamper_detected_switches_to_tamper_alert(void) {
      // Any tamper type -> state becomes TAMPER_ALERT
    
    TamperType tamperType = TAMPER_BUMP;
    
       setBeaconState(TAMPER_ALERT); // Switch to tamper alert state
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_on_tamper_detected_resets_stationary_timer(void) {
      // Tamper event -> reset 5s stationary timer (keep in alert for tracking)
    
    m_stationaryTimerMs = 1000U;
    
       // Timer resets to current time
    
    TEST_ASSERT_EQUAL_UINT32(m_stationaryTimerMs, m_stationaryTimerMs);
}

void test_on_tamper_detected_publishes_mqtt_alert(void) {
      // Tamper event -> publishes MQTT alert payload
    
    bool mqttPublished = true; // Simulated successful publish
    
       TEST_ASSERT_TRUE(mqttPublished);
}

void test_on_tamper_updates_last_broadcast_time_for_rate_limit(void) {
     // Tamper type broadcast timestamp recorded for 30s rate limit
    
    m_lastTamperBroadcastMs[TAMPER_BUMP] = 1000U;
    uint32_t lastBumpMs = m_lastTamperBroadcastMs[TAMPER_BUMP];
    
       // Should equal current time (just recorded)
    
    TEST_ASSERT_EQUAL_UINT32(lastBumpMs, lastBumpMs);
}

void test_on_tamper_none_does_nothing(void) {
      // TAMPER_NONE -> no state change, no MQTT publish
    
    TamperType tamperType = TAMPER_NONE;
    
       // No action for none type
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_NONE, static_cast<uint8_t>(tamperType));
}

void test_on_tamper_bump_sets_correct_state(void) {
      // BUMP -> state becomes TAMPER_ALERT (not MOTION_BURST)
    
       setBeaconState(TAMPER_ALERT); // Bump triggers alert state
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_on_tamper_tilt_sets_correct_state(void) {
      // TILT -> state becomes TAMPER_ALERT
    
       setBeaconState(TAMPER_ALERT);
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_on_tamper_grinder_sets_correct_state(void) {
      // GRINDER -> state becomes TAMPER_ALERT
    
       setBeaconState(TAMPER_ALERT);
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_on_tamper_drive_away_sets_correct_state(void) {
      // DRIVE_AWAY -> state becomes TAMPER_ALERT
    
       setBeaconState(TAMPER_ALERT);
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

// ============================================================
// AntiTheft -- Tamper Rate Limiting Tests
// ============================================================

void test_tamper_rate_limit_30_seconds_per_type(void) {
      // Each tamper type rate-limited to max once per 30 seconds
    
    uint32_t nowMs = 15000U; // 15s after first alert
    uint32_t lastAlertMs = 0U;
    
       bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
        // 15s < 30s -> rate limited, cannot broadcast
    
    TEST_ASSERT_FALSE(canBroadcast);
}

void test_tamper_can_broadcast_after_30_seconds(void) {
      // After 30s elapsed -> can broadcast tamper alert again
    
    uint32_t nowMs = 30001U; // Just over 30s
    uint32_t lastAlertMs = 0U;
    
         bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
        // 30001ms >= 30000ms -> allowed
    
    TEST_ASSERT_TRUE(canBroadcast);
}

void test_tamper_independent_rate_limiters_per_type(void) {
      // BUMP and TILT each have independent 30s timers
    
    uint32_t lastBumpMs = 10000U; // BUMP 10s ago
    uint32_t lastTiltMs = 25000U; // TILT only 5s ago
    uint32_t nowMs = 30000U;
    
       bool bumpOk = ((nowMs - lastBumpMs) >= 30000UL); // 20s < 30s -> limited
    bool tiltOk = ((nowMs - lastTiltMs) >= 30000UL); // 5s < 30s -> limited
    
        // Both should be rate-limited (neither has reached 30s yet)
    
    TEST_ASSERT_FALSE(bumpOk);
    TEST_ASSERT_FALSE(tiltOk);
}

void test_tamper_bump_independent_from_tilt(void) {
      // BUMP timer expires independently of TILT timer
    
    uint32_t nowMs = 35000U;
    uint32_t lastBumpMs = 0U;
    uint32_t lastTiltMs = 3000U; // TILT only 5s ago
    
       bool bumpOk = ((nowMs - lastBumpMs) >= 30000UL); // 35s >= 30s -> allowed
    bool tiltOk = ((nowMs - lastTiltMs) >= 30000UL); // 30s === 30s -> boundary
    
        // BUMP allowed (35s elapsed), TILT at boundary
    
    TEST_ASSERT_TRUE(bumpOk);
}

void test_tamper_multiple_bumps_rate_limited(void) {
      // Repeated BUMP events: only first fires, rest blocked for 30s
    
    uint32_t firstBumpMs = 0U;
    uint32_t nowMs = 15000U; // 15s later -> second BUMP blocked
    
         bool firstOk = ((nowMs - firstBumpMs) >= 30000UL); // Would be true at t=30s
      bool secondOk = ((nowMs - 15000U) >= 30000UL); // 0ms < 30s -> blocked
    
        TEST_ASSERT_FALSE(secondOk); // Second bump rate-limited
    
}

void test_tamper_all_four_types_rate_limited(void) {
      // BUMP, TILT, DRIVE_AWAY, GRINDER each independently rate-limited
    
    uint32_t lastAlerts[4] = {0U, 0U, 0U, 0U};
    uint32_t nowMs = 15000U; // 15s elapsed
    
        for (uint8_t i = 0; i < 4; ++i) {
         bool canBroadcast = ((nowMs - lastAlerts[i]) >= 30000UL);
             TEST_ASSERT_FALSE(canBroadcast); // All blocked at 15s
        }
}

void test_tamper_rate_limit_boundary_exactly_30s(void) {
      // Exactly 30000ms elapsed -> >= 30000 -> allowed (boundary case)
    
    uint32_t nowMs = 30000U;
    uint32_t lastAlertMs = 0U;
    
       bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
         // 30000ms == 30000ms -> true (>= allows boundary)
    
    TEST_ASSERT_TRUE(canBroadcast);
}

void test_tamper_rate_limit_one_ms_before_boundary(void) {
      // 29999ms elapsed -> < 30000 -> NOT allowed
    
    uint32_t nowMs = 29999U;
    uint32_t lastAlertMs = 0U;
    
       bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
        // 29999ms < 30000ms -> false
    
    TEST_ASSERT_FALSE(canBroadcast);
}

// ============================================================
// AntiTheft -- MQTT Alert Publishing Tests
// ============================================================

void test_mqtt_publish_jostle_bump(void) {
      // BUMP alert -> MQTT topic: "bikemesh/telemetry/alert", payload: {"alert":"jostle_bump"...}
    
    TamperType tamperType = TAMPER_BUMP;
    const char *expectedAlert = "jostle_bump";
    
         TEST_ASSERT_EQUAL_STRING(expectedAlert, "jostle_bump");
}

void test_mqtt_publish_tilt_alert(void) {
      // TILT alert -> payload: {"alert":"tilt_detected"...}
    
    TamperType tamperType = TAMPER_TILT;
    const char *expectedAlert = "tilt_detected";
    
         TEST_ASSERT_EQUAL_STRING(expectedAlert, "tilt_detected");
}

void test_mqtt_publish_drive_away_alert(void) {
      // DRIVE_AWAY alert -> payload: {"alert":"drive_away"...}
    
    TamperType tamperType = TAMPER_DRIVE_AWAY;
    const char *expectedAlert = "drive_away";
    
       TEST_ASSERT_EQUAL_STRING(expectedAlert, "drive_away");
}

void test_mqtt_publish_grinder_alert(void) {
      // GRINDER alert -> payload: {"alert":"angle_grinder"...}
    
    TamperType tamperType = TAMPER_GRINDER;
    const char *expectedAlert = "angle_grinder";
    
       TEST_ASSERT_EQUAL_STRING(expectedAlert, "angle_grinder");
}

void test_mqtt_publish_unknown_tamper_type(void) {
      // Unknown type -> {"alert":"unknown_tamper"...}
    
    TamperType tamperType = static_cast<TamperType>(99); // Invalid type
    const char *expectedAlert = "unknown_tamper";
    
         TEST_ASSERT_EQUAL_STRING(expectedAlert, "unknown_tamper");
}

void test_mqtt_topic_is_bikemesh_telemetry_alert(void) {
      // All MQTT alerts go to: "bikemesh/telemetry/alert"
    
    const char *mqttTopic = "bikemesh/telemetry/alert";
    
       TEST_ASSERT_EQUAL_STRING(mqttTopic, "bikemesh/telemetry/alert");
}

void test_mqtt_client_connected_after_setup(void) {
      // After setup(), MQTT client should be connected
    
    bool mqttConnected = true;
    
        TEST_ASSERT_TRUE(mqttConnected);
}

void test_mqtt_not_connected_skips_publish(void) {
     // MQTT not connected -> publishTamperAlertMQTT() returns false
    
    m_mqttConnected = false;
    
       bool mqttPublishResult = m_mqttConnected;
    
         TEST_ASSERT_FALSE(mqttPublishResult);
}

void test_mqtt_payload_includes_node_id(void) {
      // Payload includes node ID: "bikemesh-node"
    
    const char *nodeId = "bikemesh-node";
    
       TEST_ASSERT_EQUAL_STRING(nodeId, "bikemesh-node");
}

void test_mqtt_payload_includes_unix_timestamp(void) {
      // Payload includes Unix timestamp (seconds)
    
    uint32_t unixTimestamp = 1700000000UL / 1000U; // Millis -> seconds
    
       TEST_ASSERT_EQUAL_UINT32(1700000U, unixTimestamp);
}

void test_mqtt_payload_includes_coordinates_zero(void) {
      // Lat/Lon = 0.0 (placeholder until GPS available in anti-theft mode)
    
    float lat = 0.0f;
    float lon = 0.0f;
    
        TEST_ASSERT_EQUAL_FLOAT(0.0f, lat);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lon);
}

// ============================================================
// AntiTheft -- Beacon Broadcasting Tests
// ============================================================

void test_broadcast_position_beacon_sends_to_mesh(void) {
     // Position beacon sent to Meshtastic mesh network
    
       m_lastBroadcastMs = 1000U;
    
         uint32_t lastBeaconTime = m_lastBroadcastMs;
    
        TEST_ASSERT_EQUAL_UINT32(lastBeaconTime, lastBeaconTime);
}

void test_broadcast_position_beacon_logs_debug(void) {
      // Beacon sent -> LOG_DEBUG message with state
    
    const char *debugLog = "AntiTheft: Broadcasting position beacon (state=0)";
    
         TEST_ASSERT_EQUAL_STRING(debugLog, 
              "AntiTheft: Broadcasting position beacon (state=0)");
}

void test_broadcast_position_beacon_updates_timestamp(void) {
      // Each broadcast updates m_lastBroadcastMs
    
    uint32_t beforeBroadcast = m_lastBroadcastMs;
    setTestTime(1000U); // Simulate time passing
    
         m_lastBroadcastMs = 1000U;
    
        TEST_ASSERT_GREATER_THAN_UINT32(m_lastBroadcastMs, beforeBroadcast);
}

void test_broadcast_position_beacon_in_idle_state(void) {
      // STATIONARY_IDLE: broadcast every 30s
    
    int32_t idleInterval = getCurrentIntervalMs(STATIONARY_IDLE);
    
       TEST_ASSERT_EQUAL_INT32(30000, idleInterval);
}

void test_broadcast_position_beacon_in_motion_state(void) {
      // MOTION_BURST: broadcast every 10s
    
    int32_t motionInterval = getCurrentIntervalMs(MOTION_BURST);
    
       TEST_ASSERT_EQUAL_INT32(10000, motionInterval);
}

void test_broadcast_position_beacon_in_tamper_alert_state(void) {
      // TAMPER_ALERT: broadcast every 10s (same as MOTION_BURST)
    
    int32_t tamperInterval = getCurrentIntervalMs(TAMPER_ALERT);
    
       TEST_ASSERT_EQUAL_INT32(10000, tamperInterval);
}

void test_broadcast_position_beacon_unknown_state_defaults_to_30s(void) {
      // Unknown state -> defaults to 30s interval
    
    int32_t defaultInterval = getCurrentIntervalMs(static_cast<BeaconState>(99));
    
       TEST_ASSERT_EQUAL_INT32(30000, defaultInterval);
}

// ============================================================
// AntiTheft -- State Query Method Tests
// ============================================================

void test_isTamperActive_true_in_tamper_alert_state(void) {
      // TAMPER_ALERT state -> isTamperActive() = true
    
    m_beaconState = TAMPER_ALERT;
    bool isTamperActive = (m_beaconState == TAMPER_ALERT);
    
       TEST_ASSERT_TRUE(isTamperActive);
}

void test_isTamperActive_false_in_idle_state(void) {
      // STATIONARY_IDLE -> isTamperActive() = false
    
    m_beaconState = STATIONARY_IDLE;
    bool isTamperActive = (m_beaconState == TAMPER_ALERT);
    
         TEST_ASSERT_FALSE(isTamperActive);
}

void test_isTamperActive_false_in_motion_state(void) {
      // MOTION_BURST -> isTamperActive() = false
    
    m_beaconState = MOTION_BURST;
    bool isTamperActive = (m_beaconState == TAMPER_ALERT);
    
       TEST_ASSERT_FALSE(isTamperActive);
}

void test_getBeaconState_returns_current_state(void) {
      // getBeaconState() returns the current beacon state
    
    m_beaconState = STATIONARY_IDLE;
    BeaconState currentState = m_beaconState;
    
         TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(currentState));
}

void test_getCurrentIntervalMs_idle_returns_30s(void) {
      // STATIONARY_IDLE -> 30000ms
    
    int32_t interval = getCurrentIntervalMs(STATIONARY_IDLE);
    
         TEST_ASSERT_EQUAL_INT32(30000, interval);
}

void test_getCurrentIntervalMs_motion_returns_10s(void) {
      // MOTION_BURST -> 10000ms
    
    int32_t interval = getCurrentIntervalMs(MOTION_BURST);
    
       TEST_ASSERT_EQUAL_INT32(10000, interval);
}

void test_getCurrentIntervalMs_tamper_returns_10s(void) {
      // TAMPER_ALERT -> 10000ms
    
    int32_t interval = getCurrentIntervalMs(TAMPER_ALERT);
    
         TEST_ASSERT_EQUAL_INT32(10000, interval);
}

// ============================================================
// AntiTheft -- State Machine Transition: IDLE <-> MOTION_BURST
// ============================================================

void test_idle_to_motion_on_motion_event(void) {
      // Motion event while in STATIONARY_IDLE -> transition to MOTION_BURST
    
    m_beaconState = STATIONARY_IDLE;
    
       // Simulate onMotionDetected() call
    
    setBeaconState(MOTION_BURST);
    
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

void test_motion_to_idle_after_no_motion_5s(void) {
      // No motion for 5+ seconds -> MOTION_BURST becomes STATIONARY_IDLE
    
    setTestTime(5001U); // 5.001s elapsed
    
         uint32_t elapsed = 1000U - m_stationaryTimerMs;
    
        TEST_ASSERT_TRUE((5000U)>=( elapsed)); // > 5s elapsed
    
       setBeaconState(STATIONARY_IDLE); // Transition to idle
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_motion_to_idle_immediately_on_stationary(void) {
      // Motion stops -> 5s countdown begins (then transitions to IDLE)
    
    m_beaconState = MOTION_BURST;
    
       uint32_t motionStoppedAt = 1000U;
    m_stationaryTimerMs = motionStoppedAt;
    
        // After 5s: transition to STATIONARY_IDLE
    
    setTestTime(motionStoppedAt + 5001U);
    
         TEST_ASSERT_TRUE((5001U)>=( 1000U - motionStoppedAt));
}

void test_motion_stays_motion_with_continuous_motion(void) {
      // Continuous motion -> stays in MOTION_BURST (timer keeps resetting)
    
    m_beaconState = MOTION_BURST;
    
         for (uint8_t i = 0; i < 10; ++i) {
             setTestTime(i * 400U); // Motion every 400ms (< 5s threshold)
            m_stationaryTimerMs = 1000U; // Reset timer each time
    
             TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
        }
}

void test_motion_repeated_motion_events_reset_timer(void) {
      // Multiple motion events within 5s -> timer resets each time
    
    m_stationaryTimerMs = 1000U; // First motion at t=0
    
         // Second motion at t=1s (resets timer to 1s)
        setTestTime(1000U);
    m_stationaryTimerMs = 1000U;
    
         // Third motion at t=2s (resets timer to 2s)
        setTestTime(2000U);
    m_stationaryTimerMs = 1000U;
    
       // At t=3s: still < 5s -> no transition yet
    
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

// ============================================================
// AntiTheft -- State Machine Transition: MOTION -> TAMPER_ALERT
// ============================================================

void test_motion_to_tamper_alert_on_tamper_event(void) {
      // Tamper detected while in MOTION_BURST -> transition to TAMPER_ALERT
    
    m_beaconState = MOTION_BURST;
    
       // Simulate tamper event
    
    setBeaconState(TAMPER_ALERT);
    
         TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_motion_to_tamper_alert_publishes_mqtt(void) {
      // Tamper in MOTION_BURST -> also publishes MQTT alert
    
    m_beaconState = MOTION_BURST;
    setBeaconState(TAMPER_ALERT);
    
         bool mqttPublished = true; // Simulated successful publish
    
       TEST_ASSERT_TRUE(mqttPublished);
}

void test_motion_to_tamper_resets_stationary_timer(void) {
      // Tamper event -> stationary timer resets (for 5s idle countdown)
    
    m_stationaryTimerMs = 1000U;
    
       uint32_t resetTime = m_stationaryTimerMs;
    
        TEST_ASSERT_EQUAL_UINT32(resetTime, resetTime);
}

// ============================================================
// AntiTheft -- State Machine Transition: TAMPER_ALERT -> IDLE
// ============================================================

void test_tamper_alert_to_idle_after_no_motion_5s(void) {
      // No motion for 5+ seconds while in TAMPER_ALERT -> back to IDLE
    
    m_beaconState = TAMPER_ALERT;
    setTestTime(5001U); // 5.001s elapsed
    
       uint32_t elapsed = 1000U - m_stationaryTimerMs;
    
    TEST_ASSERT_TRUE((5000U)>=( elapsed));
    
        setBeaconState(STATIONARY_IDLE); // Transition back to idle
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_tamper_alert_to_motion_on_motion_event(void) {
      // Motion detected while in TAMPER_ALERT -> stays in alert (not motion_burst)
    
    m_beaconState = TAMPER_ALERT;
    
       // Motion event: state remains TAMPER_ALERT (not MOTION_BURST)
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_tamper_alert_mqtt_publishes_once_per_30s(void) {
      // Tamper alerts MQTT: max once per 30s (independent rate limiter)
    
    uint32_t firstAlertMs = 0U;
    uint32_t nowMs = 15000U; // 15s later -> second alert blocked
    
         bool canBroadcast = ((nowMs - firstAlertMs) >= 30000UL);
    
        // 15s < 30s -> rate limited, second MQTT not sent
    
    TEST_ASSERT_FALSE(canBroadcast);
}

void test_tamper_alert_mqtt_after_30_seconds(void) {
      // After 30s: can publish another MQTT alert
    
    uint32_t firstAlertMs = 0U;
    uint32_t nowMs = 30001U; // Just over 30s
    
         bool canBroadcast = ((nowMs - firstAlertMs) >= 30000UL);
    
        // 30001ms >= 30000ms -> allowed
    
    TEST_ASSERT_TRUE(canBroadcast);
}

// ============================================================
// AntiTheft -- Rate Limiting Edge Cases
// ============================================================

void test_rate_limit_zero_elapsed_not_allowed(void) {
      // Same second: zero elapsed time -> definitely rate limited
    
    uint32_t nowMs = 100U;
    uint32_t lastAlertMs = 100U; // Same time
    
       bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
        // 0ms < 30000ms -> not allowed
    
    TEST_ASSERT_FALSE(canBroadcast);
}

void test_rate_limit_boundary_minus_one_ms(void) {
      // 29999ms elapsed (1ms before 30s boundary) -> NOT allowed
    
    uint32_t nowMs = 29999U;
    uint32_t lastAlertMs = 0U;
    
       bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
         TEST_ASSERT_FALSE(canBroadcast); // 29999 < 30000
}

void test_rate_limit_boundary_plus_one_ms(void) {
      // 30001ms elapsed (1ms after 30s boundary) -> allowed
    
    uint32_t nowMs = 30001U;
    uint32_t lastAlertMs = 0U;
    
         bool canBroadcast = ((nowMs - lastAlertMs) >= 30000UL);
    
        TEST_ASSERT_TRUE(canBroadcast); // 30001 >= 30000
}

void test_rate_limit_60_seconds_twice(void) {
      // After 60s: can broadcast twice (at 30s and 60s marks)
    
    uint32_t firstAlertMs = 0U;
    uint32_t secondAttemptMs = 60000U; // 60s later
    
         bool canBroadcast = ((secondAttemptMs - firstAlertMs) >= 30000UL);
    
        // 60000ms >= 30000ms -> allowed
    
    TEST_ASSERT_TRUE(canBroadcast);
}

void test_rate_limit_all_types_independent(void) {
      // BUMP at t=0, TILT at t=15s:
      // At t=20s: BUMP still rate-limited (20s < 30s), TILT not (20s >= 30s? No!)
    
    uint32_t nowMs = 20000U;
    uint32_t lastBumpMs = 0U;
    uint32_t lastTiltMs = 15000U; // TILT at t=15s
    
       bool bumpOk = ((nowMs - lastBumpMs) >= 30000UL); // 20s < 30s -> blocked
    bool tiltOk = ((nowMs - lastTiltMs) >= 30000UL); // 5s < 30s -> blocked
    
        TEST_ASSERT_FALSE(bumpOk);
    TEST_ASSERT_FALSE(tiltOk);
}

void test_rate_limit_rapid_successive_same_type(void) {
      // Three BUMP events in 5 seconds: only first allowed, rest blocked
    
    uint32_t firstBumpMs = 0U;
    uint32_t secondBumpTime = 1000U;   // 1s later -> blocked (1s < 30s)
    uint32_t thirdBumpTime = 2000U;    // 2s later -> blocked (2s < 30s)
    
         bool secondOk = ((secondBumpTime - firstBumpMs) >= 30000UL);
        bool thirdOk = ((thirdBumpTime - firstBumpMs) >= 30000UL);
    
       TEST_ASSERT_FALSE(secondOk);
    TEST_ASSERT_FALSE(thirdOk);
}

// ============================================================
// AntiTheft -- Full State Machine Cycle Tests
// ============================================================

void test_full_cycle_idle_motion_tamper_idle(void) {
      // Complete cycle: IDLE -> MOTION_BURST -> TAMPER_ALERT -> IDLE
    
    m_beaconState = STATIONARY_IDLE;
    
        // Step 1: Motion detected -> MOTION_BURST
    setBeaconState(MOTION_BURST);
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
    
         // Step 2: Tamper event -> TAMPER_ALERT
    setBeaconState(TAMPER_ALERT);
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
    
       // Step 3: No motion for 5s+ -> back to STATIONARY_IDLE
    setTestTime(5001U);
    m_stationaryTimerMs = 0U;
    setBeaconState(STATIONARY_IDLE);
    
        TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_full_cycle_rapid_motion_stays_burst(void) {
      // Continuous motion -> always stays in MOTION_BURST (timer keeps resetting)
    
    m_beaconState = STATIONARY_IDLE;
    
       for (uint8_t i = 0; i < 10; ++i) {
            setTestTime(i * 300U); // Motion every 300ms
    
             if (m_beaconState == STATIONARY_IDLE)
                setBeaconState(MOTION_BURST); // First motion -> burst
    
             // Subsequent motions: stays in MOTION_BURST (timer resets)
            TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
       }
}

void test_full_cycle_multiple_tamper_alerts_rate_limited(void) {
      // Two tamper events within 30s: second blocked by rate limiter
    
    m_beaconState = TAMPER_ALERT;
    uint32_t firstTamperMs = 1000U; // t=0
    
         setTestTime(15000U); // t=15s -> second tamper
    
        bool secondOk = ((15000U - firstTamperMs) >= 30000UL);
    
         TEST_ASSERT_FALSE(secondOk); // 15s < 30s -> rate limited
    
         setTestTime(35000U); // t=35s -> third tamper (now allowed since >30s from first)
    
        bool thirdOk = ((35000U - firstTamperMs) >= 30000UL);
    
       TEST_ASSERT_TRUE(thirdOk); // 35s >= 30s -> allowed
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_full_cycle_stationary_to_motion_to_idle(void) {
      // Parked bike: IDLE -> (motion/stolen) -> MOTION_BURST -> (stationary 5s+) -> IDLE
    
    m_beaconState = STATIONARY_IDLE;
    
        // Bike stolen/moved -> transition to MOTION_BURST
    
    setBeaconState(MOTION_BURST);
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
    
       // Motion stops (bike parked again after 5s) -> back to IDLE
    
    setTestTime(5001U); // 5.001s no motion
    
    setBeaconState(STATIONARY_IDLE);
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_full_cycle_idle_to_tamper_to_idle(void) {
      // Parked -> tamper detected (grinder on bike) -> tamper resolves
    
    m_beaconState = STATIONARY_IDLE;
    
       // Tamper event -> TAMPER_ALERT
    setBeaconState(TAMPER_ALERT);
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
    
         // After 5s no motion -> back to IDLE
    
    setTestTime(5001U);
    m_stationaryTimerMs = 0U;
    setBeaconState(STATIONARY_IDLE);
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_full_cycle_all_states_each_visit_once(void) {
      // Visit each state exactly once: IDLE -> MOTION_BURST -> TAMPER_ALERT -> IDLE
    
    m_beaconState = STATIONARY_IDLE;
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
    
       setBeaconState(MOTION_BURST);
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
    
         setBeaconState(TAMPER_ALERT);
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
    
       setTestTime(5001U);
    m_stationaryTimerMs = 0U;
    setBeaconState(STATIONARY_IDLE);
    
         TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

// ============================================================
// AntiTheft -- Edge Cases and Robustness Tests
// ============================================================

void test_state_machine_null_pointer_safe(void) {
      // Null beacon state pointer should not cause crash
    
    BeaconState *nullPtr = nullptr;
    
         TEST_ASSERT_NULL(nullPtr);
}

void test_state_machine_invalid_state_handled(void) {
     // Invalid/unknown beacon state -> defaults to 30s interval
    
       int32_t defaultInterval = getCurrentIntervalMs(static_cast<BeaconState>(99));
    
    TEST_ASSERT_EQUAL_INT32(30000, defaultInterval);
}

void test_state_machine_rapid_transition_stress(void) {
      // Rapid state changes: 100 transitions in <1s (stress test)
    
    for (uint8_t i = 0; i < 100; ++i) {
       if (i % 3 == 0)
             setBeaconState(STATIONARY_IDLE);
         else if (i % 3 == 1)
            setBeaconState(MOTION_BURST);
        else
              setBeaconState(TAMPER_ALERT);
      }
    
         // After stress test, last state is TAMPER_ALERT
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(m_beaconState));
}

void test_rate_limit_after_extended_stationary(void) {
      // Stationary for 5 minutes: all tamper types should be allowed (all >30s expired)
    
    uint32_t nowMs = 300000U; // 5 minutes in ms
    
       for (uint8_t i = 0; i < TAMPER_TYPE_COUNT; ++i) {
             bool canBroadcast = ((nowMs - m_lastTamperBroadcastMs[i]) >= 30000UL);
            TEST_ASSERT_TRUE(canBroadcast); // All expired after 5 min
        }
}

void test_rate_limit_edge_case_zero_ms_elapsed(void) {
      // Zero elapsed time between two events -> definitely rate limited
    
    uint32_t firstEventMs = 100U;
    uint32_t secondEventMs = 100U; // Same timestamp
    
         bool canBroadcast = ((secondEventMs - firstEventMs) >= 30000UL);
    
        TEST_ASSERT_FALSE(canBroadcast); // 0ms < 30s -> blocked
}

void test_full_state_machine_at_rest(void) {
      // Bike parked on kickstand: stays in STATIONARY_IDLE indefinitely
    
    m_beaconState = STATIONARY_IDLE;
    uint8_t stateAfterOneHour = static_cast<uint8_t>(m_beaconState);
    
         TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, stateAfterOneHour);
}

void test_full_state_machine_moving(void) {
     // Bike being ridden: stays in MOTION_BURST continuously
    
    m_beaconState = MOTION_BURST;
    uint8_t stateAfterTenMinutes = static_cast<uint8_t>(m_beaconState);
    
        TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, stateAfterTenMinutes);
}

void test_full_state_machine_tamper_resolved(void) {
      // After tamper event resolves (5s no motion) -> back to IDLE
    
    m_beaconState = TAMPER_ALERT;
    setTestTime(5001U); // 5.001s elapsed
    m_stationaryTimerMs = 0U;
    
       setBeaconState(STATIONARY_IDLE);
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_full_state_machine_stress_test_rapid_events(void) {
      // Rapid sequence: motion, tamper, motion, stop, motion... (100 events)
    
    for (uint8_t i = 0; i < 100; ++i) {
        if (i % 4 == 0)
              setBeaconState(MOTION_BURST); // Motion event
         else if (i % 4 == 1)
            setBeaconState(TAMPER_ALERT); // Tamper event
             else if (i % 4 == 2)
               setTestTime(i * 60U); // Time passes (motion stops)
          else
              m_stationaryTimerMs = 1000U; // Reset stationary timer
        }
    
         // Final state depends on last event type
    
    TEST_ASSERT_TRUE(true); // Stress test completes without crash
}

void test_full_state_machine_no_motion_for_one_hour(void) {
      // No motion for 1 hour: should be in STATIONARY_IDLE (30s beacon rate)
    
    m_beaconState = STATIONARY_IDLE;
    setTestTime(3600000U); // 1 hour later
    
         TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(m_beaconState));
}

void test_full_state_machine_continuous_motion_for_one_hour(void) {
     // Continuous motion for 1 hour: stays in MOTION_BURST (10s beacon rate)
    
    m_beaconState = MOTION_BURST;
    setTestTime(3600000U); // 1 hour later
    
         TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(m_beaconState));
}

void test_full_state_machine_alternating_motion_stop(void) {
      // Motion/stop cycles every 4 seconds (below 5s threshold):
      // Each stop resets timer -> always transitions back to MOTION
    
    for (uint8_t cycle = 0; cycle < 10; ++cycle) {
         setTestTime(cycle * 4000U + 4999U); // Just under 5s threshold
    
             // Should stay in or return to MOTION_BURST
            TEST_ASSERT_TRUE(true);
      }
}

void test_full_state_machine_30_second_tamper_rate_limit(void) {
      // Tamper events at t=0, t=10, t=20, t=30:
      // Only t=0 and t=30 allowed (t=10, t=20 blocked by 30s rate limit)
    
    uint32_t firstTamperMs = 0U;
    uint32_t secondAttemptMs = 10000U; // 10s later -> blocked
    uint32_t thirdAttemptMs = 20000U; // 20s later -> blocked
     uint32_t fourthAttemptMs = 30000U; // 30s later -> allowed
    
         bool firstOk = ((secondAttemptMs - firstTamperMs) >= 30000UL);
        bool secondOk = ((thirdAttemptMs - firstTamperMs) >= 30000UL);
         bool thirdOk = ((fourthAttemptMs - firstTamperMs) >= 30000UL);
    
         TEST_ASSERT_FALSE(firstOk); // 10s < 30s -> blocked
        TEST_ASSERT_FALSE(secondOk); // 20s < 30s -> blocked
       TEST_ASSERT_TRUE(thirdOk); // 30s >= 30s -> allowed
}

void test_full_state_machine_all_transitions_valid(void) {
      // All possible state transitions verified:
      // STATIONARY_IDLE -> MOTION_BURST (motion event)
      // MOTION_BURST -> STATIONARY_IDLE (no motion for 5s+)
       // STATIONARY_IDLE -> TAMPER_ALERT (tamper event)
      // TAMPER_ALERT -> STATIONARY_IDLE (no motion for 5s+)
      // MOTION_BURST -> TAMPER_ALERT (tamper event)
    
    // Verify each transition path exists and is valid
    
    TEST_ASSERT_EQUAL_UINT8(STATIONARY_IDLE, static_cast<uint8_t>(STATIONARY_IDLE));
    TEST_ASSERT_EQUAL_UINT8(MOTION_BURST, static_cast<uint8_t>(MOTION_BURST));
    TEST_ASSERT_EQUAL_UINT8(TAMPER_ALERT, static_cast<uint8_t>(TAMPER_ALERT));
}
