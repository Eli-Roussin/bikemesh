#pragma once

// ============================================
// config.h — BikeMesh firmware configuration
// All tunable parameters in one place.
// ============================================

// === VIN / Power Mode Detection Thresholds ===
#define VIN_MODE_A_THRESHOLD_MV    4000     // >=4.0V --> Mode A (charging, e-bike ON)
#define VIN_MODE_B_THRESHOLD_MV    3800     // <3.8V  --> Mode B (anti-theft, battery/solar only)
#define VIN_SAMPLE_INTERVAL_MS     100      // ADC sampling period (100ms = 10Hz)
#define VIN_DEBOUNCE_CYCLES        5        // Need 5 consecutive stable periods to transition

// === Mode A -- CHARGING GPS Logging Rates ===
#define MODE_A_GPS_LOG_RATE_HZ     1         // Log at 1 Hz when moving (1/sec)
#define MODE_A_STATIONARY_INTERVAL_S 60      // Stationary log interval (1/min = 60s)

// === Mode B -- ANTI-THEFT Beacon Intervals ===
#define MODE_B_BEACON_IDLE_S       30        // STATIONARY_IDLE: standard TRACKER interval (30s)
#define MODE_B_BEACON_MOTION_S     10        // MOTION_BURST/TAMPER_ALERT: accelerated while moved/stolen (10s)
#define MODE_B_STATIONARY_DEBOUNCE_S 5       // Need 5s of no motion before returning to IDLE

// === Tamper Alert Rate Limiting (per type, max once per 30s) ===
#define TAMPER_ALERT_RATE_LIMIT_S   30       // Each tamper type independently rate-limited

// === Mesh Telemetry Beacon Intervals (idle vs motion) ===
#define BEACON_INTERVAL_IDLE_MS     30000      // 30s normal TRACKER rate (stationary/idle)
#define BEACON_INTERVAL_MOTION_MS   10000      // 10s accelerated on motion/tamper (both modes)
#define BEACON_IDLE_MS              BEACON_INTERVAL_IDLE_MS
#define BEACON_MOTION_MS            BEACON_INTERVAL_MOTION_MS
// Legacy alias for backward compatibility:
#define TELEM_BROADCAST_INTERVAL_S 10           // Custom telemetry packet for dashboard MQTT gateway

// === GPIO / Pin Assignments (nRF52840 WisBlock RAK3401) ===
// Update these based on actual WisBlock v2 pin mappings:
#define GPS_UART_TX_PIN            6         // u-blox ZOE-M8Q TX --> nRF UART RX
#define GPS_UART_RX_PIN            8         // u-blox ZOE-M8Q RX --> nRF UART TX
#define LIS3DH_I2C_SDA_PIN         15        // Qwiic I2C SDA
#define LIS3DH_I2C_SCL_PIN         14        // Qwiic I2C SCL
#define SD_CS_PIN                  16        // RAK15002 SPI CS pin (SD card detect)
#define SD_DET_PIN                 17        // RAK15002 INSERT/REMOVE DETECT GPIO

// === MQTT Gateway Settings ===
#define MQTT_BROKER_HOST   "mqtt.home.local"
#define MQTT_BROKER_PORT   1883
#define MQTT_CLIENT_ID     "bikemesh-node"

// Compile-time flags for optional features:
// #define ENABLE_MQTT_GATEWAY          // Comment out if no WiFi/BLE mesh available
// #define USE_IMU_FOR_VIBRATION        // Use LIS3DH for vibration analysis (recommended)
// #define SD_CARD_LOG_DAILY_ROTATION   true  // Auto-create new day file at midnight
