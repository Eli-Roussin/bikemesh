#include "AntiTheft.h"
#include "../config.h"
#include <string.h>
#include <stdio.h>

static bool mock_mqtt_connected = false;

static void mock_mqtt_init(void) {
    mock_mqtt_connected = true;
}

static bool mock_mqtt_publish(const char *topic, const char *payload) {
    LOG_INFO("MQTT: Published to %s -- %s", topic, payload);
    return true;
}

void AntiTheft::setup(void) {
      m_initialized = true;
    m_beaconState = STATIONARY_IDLE;   // Start in idle state (30s beacon interval)
    m_stationaryTimerMs = millis();        // Initialize stationary timer
    m_lastBroadcastMs = 0;                    // Allow immediate first broadcast

       mock_mqtt_init();                 // Initialize MQTT client for home broker

     LOG_INFO("AntiTheft: Initialized -- TRACKER beacons at %dms idle, %dms motion", BEACON_IDLE_MS, BEACON_MOTION_MS);
}

int32_t AntiTheft::runOnce(void) {
    if (!m_initialized) {
        return 100;
    }

    switch (m_beaconState) {
    case STATIONARY_IDLE:
        if (m_motionSinceLastCheckMs > 0U) {
            setBeaconState(MOTION_BURST);
            m_stationaryTimerMs = millis();
            m_motionSinceLastCheckMs = 0U;
        }
        break;

    case MOTION_BURST:
    case TAMPER_ALERT:
        if (m_motionSinceLastCheckMs == 0U) {
            if ((millis() - m_stationaryTimerMs) >= 5000UL) {
                setBeaconState(STATIONARY_IDLE);
            }
        } else {
            m_stationaryTimerMs = millis();
            m_motionSinceLastCheckMs = 0U;
        }
        break;

    default:
        break;
    }

    if (shouldBroadcast()) {
        broadcastPositionBeacon();
        m_lastBroadcastMs = millis();
    }

    return 100;
}

void AntiTheft::onMotionDetected(void) {
    m_motionSinceLastCheckMs = millis();
    if (m_beaconState != MOTION_BURST && m_beaconState != TAMPER_ALERT) {
        setBeaconState(MOTION_BURST);
        m_stationaryTimerMs = millis();
    }
}

void AntiTheft::onTamperDetected(TamperType tamperType) {
    if (tamperType == TAMPER_NONE) {
        return;
    }

    if (canBroadcastTamperAlert(tamperType)) {
        setBeaconState(TAMPER_ALERT);
        m_stationaryTimerMs = millis();
        publishTamperAlertMQTT(tamperType);
        m_lastTamperBroadcastMs[static_cast<uint8_t>(tamperType)] = millis();
    }
}

bool AntiTheft::isTamperActive(void) const {
    return (m_beaconState == TAMPER_ALERT);              // True when in tamper alert state
}

BeaconState AntiTheft::getBeaconState(void) const {
       return m_beaconState;                            // Return current beacon state
}

int32_t AntiTheft::getCurrentIntervalMs(void) const {
    switch (m_beaconState) {
         case STATIONARY_IDLE:      return 30000UL;            // 30s idle rate (standard Meshtastic TRACKER default)
        case MOTION_BURST:     return 10000UL;                // 10s motion rate (accelerated for theft tracking)
       case TAMPER_ALERT:    return 10000UL;                   // 10s same as motion burst (MQTT alert sent separately at 30s max per type)
        default:                  return 30000UL;                // Default to 30s if unknown state
      }
}

void AntiTheft::setBeaconState(BeaconState newState) {
    if (m_beaconState != newState) {    // Only log if state actually changed
           LOG_INFO("AntiTheft: State change %d --> %d", m_beaconState, newState);
            m_beaconState = newState;
       }
}

void AntiTheft::broadcastPositionBeacon(void) {
    LOG_DEBUG("AntiTheft: Broadcasting position beacon (state=%d)", m_beaconState);
    LOG_INFO("AntiTheft: Beacon sent -- lat=0.0000, lon=0.0000, alt=0m, state=%d", m_beaconState);
}

bool AntiTheft::shouldBroadcast(void) {
    uint32_t nowMs = millis();     // Get current time in milliseconds
    
      int32_t intervalMs = getCurrentIntervalMs();       // Get current beacon interval (30s/10s based on state)
    return ((nowMs - m_lastBroadcastMs) >= (uint32_t)intervalMs);   // Return true if enough time elapsed
}

void AntiTheft::initTamperRateLimits(void) {
         // Initialize rate counters for each tamper type to 0 so first alert can fire immediately
    memset(m_lastTamperBroadcastMs, 0, sizeof(m_lastTamperBroadcastMs));
}

bool AntiTheft::canBroadcastTamperAlert(TamperType type) {
     uint32_t nowMs = millis();      // Get current time
    
       uint8_t typeIdx = (uint8_t)type;     // Convert enum to array index
    if (typeIdx >= TAMPER_TYPE_COUNT) return false;   // Invalid type -- don't allow broadcast

         // Check if 30 seconds have passed since last alert of this type
    return ((nowMs - m_lastTamperBroadcastMs[typeIdx]) >= 30000UL);   // Rate-limit: max once per 30s per type
}

void AntiTheft::publishTamperAlertMQTT(TamperType tamperType) {
       // Send JSON alert payload to home MQTT broker (Node-RED/Home Assistant connectivity)
     if (!mock_mqtt_connected) return;   // Skip if MQTT client not connected
    
    const char *alertName = "unknown";      // Default name for unknown types

       switch (tamperType) {
       case TAMPER_BUMP:         alertName = "jostle_bump";       break;     // Jostle/impact event
    case TAMPER_TILT:           alertName = "tilt_detected";     break;  // Tilt/dismantle event
    case TAMPER_DRIVE_AWAY:     alertName = "drive_away";        break;  // Bike started moving with angle change
         case TAMPER_GRINDER:    alertName = "angle_grinder";     break;  // Angle grinder vibration detected
       default:                alertName = "unknown_tamper";    break;      // Unknown tamper type
      }

         // Build JSON alert payload for Home Assistant / Node-RED MQTT broker
    char json[256];
     snprintf(json, sizeof(json),
          "{\"node\":\"%s\",\"alert\":\"%s\",\"lat\":0.0,\"lon\":0.0,\"ts\":%lu}",
        MQTT_CLIENT_ID,                              // Client ID for MQTT identification
          alertName,                                   // Alert type name (human-readable)
         millis() / 1000UL);                       // Unix timestamp in seconds
    
       // Publish to MQTT topic: bikemesh/telemetry/alert
     const char *topic = "bikemesh/telemetry/alert";   // Standard MQTT topic for tamper alerts

        mock_mqtt_publish(topic, json);
}

bool AntiTheft::mqttClientConnected(void) {
       return mock_mqtt_connected;                  // Return connection status to MQTT broker
}

bool AntiTheft::mqttClientPublish(const char *topic, const char *payload) {
    if (!mock_mqtt_connected) return false;    // Skip publish if not connected
    
      LOG_INFO("MQTT: Published to %s -- %s", topic, payload);
     return mock_mqtt_publish(topic, payload);   // Delegate to mock MQTT client
}
