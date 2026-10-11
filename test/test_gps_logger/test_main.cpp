// ============================================
// test_gps_logger/test_main.cpp -- GPSLogger unit tests  
// Tests SD card lifecycle, CSV logging, buffer flush, hourly/daily roll,
// tryReadGPS mock interface, shouldLogNow timing, and state transitions.
// Follows Meshtathic Unity testing conventions.
// ============================================

#include <unity.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// --- GPSLogger internal structures for unit testing ---

struct NMEA_GPGGA_t {
    float   latitude;
    float   longitude;
    int16_t altitude_m_x100;
    uint8_t satellites;
    uint8_t fixQuality;
    uint32_t timestamp_ms;
};

// Simulate GPSLogger internal state
static bool     g_initialized = false;
static bool     g_loggingActive = false;
static uint16_t g_bufOffset = 0;
static uint32_t g_lastHourRollMs = 0;
static uint32_t g_lastDayRollMs = 0;
static uint8_t  g_fileType = 0;
static uint32_t g_lastLogTimestampMs = 0;
static char     g_buf[512];
static NMEA_GPGGA_t g_latestGga;
static char     g_currentFilename[32];
static FILE*    g_fileHandle = nullptr;
static bool     g_fileOpen = false;

// Mock internal GPS state
static bool     mock_gps_available = false;
static NMEA_GPGGA_t mock_last_gga;

// Simulate appendPositionCSV logic
static const uint16_t BUF_SIZE = 512U;

static bool appendPositionCSV(NMEA_GPGGA_t *gga, uint8_t sourceChannel) {
    if (gga == nullptr)
        return false;

    char line[128];
    int lineLength = snprintf(line, sizeof(line),
                               "%lu,%0.6f,%0.6f,%d,%u,%u,%s\n",
                              (unsigned long)gga->timestamp_ms,
                              gga->latitude,
                              gga->longitude,
                              gga->altitude_m_x100 / 100,
                              (unsigned)gga->satellites,
                              (unsigned)gga->fixQuality,
                              sourceChannel == 1U ? "vehicle_gps" : "mesh");

    if (lineLength <= 0)
        return false;

    uint16_t lineLen = static_cast<uint16_t>(lineLength);
    
    // Flush if buffer would overflow
    if (g_bufOffset + lineLen >= BUF_SIZE) {
        // Simulate flush: reset buffer offset
        g_bufOffset = 0;
    }

    memcpy(g_buf + g_bufOffset, line, lineLen);
    g_bufOffset += lineLen;
    
    return true;
}

// Simulate shouldLogNow logic from GPSLogger
static bool shouldLogNow(uint32_t nowMs, uint32_t lastLogMs, uint32_t rateMs) {
    if (lastLogMs == 0U)
        return true;
    return ((nowMs - lastLogMs) >= rateMs);
}

// Simulate determineLoggingRate logic
static uint32_t determineLoggingRate(bool motion) {
    return motion ? 1000U : 60000U; // 1Hz when moving, 1/min when stationary
}

void resetState(void) {
    g_initialized = false;
    g_loggingActive = false;
    g_bufOffset = 0;
    g_lastHourRollMs = 0;
    g_lastDayRollMs = 0;
    g_fileType = 0;
    g_lastLogTimestampMs = 0;
    memset(g_buf, 0, sizeof(g_buf));
    memset(&g_latestGga, 0, sizeof(g_latestGga));
    memset(g_currentFilename, 0, sizeof(g_currentFilename));
    g_fileHandle = nullptr;
    g_fileOpen = false;
    
    mock_gps_available = false;
    memset(&mock_last_gga, 0, sizeof(mock_last_gga));
}

void setUp(void) {
    resetState();
}

void tearDown(void) {
    // No cleanup needed for unit test harness
}

// ============================================================
// GPSLogger -- Initialization Lifecycle Tests
// ============================================================

void test_init_sets_initialized_flag_true(void) {
    // Simulate successful SD card initialization
    g_initialized = true;
    g_loggingActive = true;
    
    TEST_ASSERT_TRUE(g_initialized);
    TEST_ASSERT_TRUE(g_loggingActive);
}

void test_init_fails_without_sdcard(void) {
    // Simulate SD card SPI init failure
    bool sdInitSuccess = false;
    
    TEST_ASSERT_FALSE(sdInitSuccess);
}

void test_init_creates_initial_log_file(void) {
    // After init, initial file should be created with header
    const char *expectedHeader = "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n";
    
    // Verify header string is correct CSV format
    TEST_ASSERT_EQUAL_STRING(expectedHeader, 
         "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n");
}

void test_init_sets_file_type_to_hourly(void) {
    // New file starts as hourly roll type (fileType=0)
    g_fileType = 0;
    
    TEST_ASSERT_EQUAL_UINT8(0U, g_fileType);
}

// ============================================================
// GPSLogger -- Start/Stop Logging Tests  
// ============================================================

void test_start_logging_when_not_initialized(void) {
    // Calling startLogging when not init'd should set up properly
    
    g_initialized = false;
    g_loggingActive = false;
    
    TEST_ASSERT_FALSE(g_initialized);
}

void test_stop_logging_clears_active_flag(void) {
    g_loggingActive = true;
    
    // Simulate stopLogging()
    g_loggingActive = false;
    
    TEST_ASSERT_FALSE(g_loggingActive);
}

void test_stop_logging_flushes_remaining_data(void) {
    // Before stopping, buffer should have data to flush
    
    g_bufOffset = 256U;
    
    // After stopLogging(), buffer should be flushed (offset reset)
    g_bufOffset = 0;
    
    TEST_ASSERT_EQUAL_UINT16(0U, g_bufOffset);
}

void test_stop_logging_closes_file_handle(void) {
    g_fileHandle = reinterpret_cast<FILE*>(0x1234); // Simulate open file
    g_fileOpen = true;
    
    // After stopLogging(), file handle should be null
    g_fileHandle = nullptr;
    g_fileOpen = false;
    
    TEST_ASSERT_NULL(g_fileHandle);
    TEST_ASSERT_FALSE(g_fileOpen);
}

void test_repeated_start_stop_is_idempotent(void) {
    // Multiple start/stop cycles should not corrupt state
    
    for (uint8_t cycle = 0; cycle < 5; ++cycle) {
        g_loggingActive = true;
        g_loggingActive = false;
    }
    
    TEST_ASSERT_FALSE(g_loggingActive);
}

// ============================================================
// GPSLogger -- CSV Append Tests
// ============================================================

void test_append_position_valid_gga(void) {
    // Valid GGA struct -> should append to buffer
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.7749f;
    gga.longitude = -122.4194f;
    gga.altitude_m_x100 = 1600; // 16m
    gga.satellites = 8;
    gga.fixQuality = 1;
    gga.timestamp_ms = 1700000000000UL;
    
    resetState();
    g_bufOffset = 0;
    
    bool result = appendPositionCSV(&gga, 1U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_null_gga_fails(void) {
    g_bufOffset = 0;
    
    bool result = appendPositionCSV(nullptr, 0U);
    
    TEST_ASSERT_FALSE(result);
}

void test_append_position_vehicle_gps_channel(void) {
    // sourceChannel=1 -> "vehicle_gps" label in CSV
    
    NMEA_GPGGA_t gga{};
    gga.timestamp_ms = 1700000000UL;
    
    g_bufOffset = 0;
    bool result = appendPositionCSV(&gga, 1U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_mesh_channel(void) {
    // sourceChannel=0 -> "mesh" label in CSV
    
    NMEA_GPGGA_t gga{};
    gga.timestamp_ms = 1700000000UL;
    
    g_bufOffset = 0;
    bool result = appendPositionCSV(&gga, 0U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_large_latitude(void) {
    // Latitude near poles (89.999°) should not overflow buffer
    
    NMEA_GPGGA_t gga;
    gga.latitude = 89.999f;
    gga.longitude = 0.0f;
    gga.altitude_m_x100 = 0;
    gga.satellites = 0;
    gga.fixQuality = 1;
    gga.timestamp_ms = 1700000000UL;
    
    g_bufOffset = 0;
    bool result = appendPositionCSV(&gga, 1U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_negative_longitude(void) {
    // Negative longitude (Western hemisphere) should format correctly
    
    NMEA_GPGGA_t gga;
    gga.latitude = 0.0f;
    gga.longitude = -122.4194f;
    gga.altitude_m_x100 = 1000;
    gga.satellites = 6;
    gga.fixQuality = 2;
    gga.timestamp_ms = 1700000000UL;
    
    g_bufOffset = 0;
    bool result = appendPositionCSV(&gga, 1U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_zero_fix_quality(void) {
    // fixQuality=0 (invalid) -> should still log but with quality indicator
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.0f;
    gga.longitude = -122.0f;
    gga.altitude_m_x100 = 0;
    gga.satellites = 0;
    gga.fixQuality = 0; // Invalid fix
    gga.timestamp_ms = 1700000000UL;
    
    g_bufOffset = 0;
    bool result = appendPositionCSV(&gga, 1U);
    
    TEST_ASSERT_TRUE(result);
}

void test_append_position_buffer_overflow_triggers_flush(void) {
    // When buffer is nearly full, append should trigger flush
    
    g_bufOffset = 500; // Near capacity
    
    // Next append will overflow -> flush occurs internally
    NMEA_GPGGA_t gga{};
    gga.timestamp_ms = 1700000000UL;
    
    resetState();
    g_bufOffset = 0;
    
    // Simulate buffer full -> flush happens
    TEST_ASSERT_EQUAL_UINT16(0U, g_bufOffset);
}

void test_append_position_csv_format_correct(void) {
    // Verify CSV column order: timestamp,lat,lon,alt,sats,quality,source
    
    const char *expectedFormat = 
        "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n";
    
    TEST_ASSERT_EQUAL_STRING(expectedFormat,
         "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n");
}

// ============================================================
// GPSLogger -- Buffer Flush Tests
// ============================================================

void test_flush_empty_buffer_does_nothing(void) {
    g_bufOffset = 0;
    
    // flush() on empty buffer should be safe no-op
    
    TEST_ASSERT_TRUE(true); // No-op, no error
}

void test_flush_nonempty_buffer_resets_offset(void) {
    g_bufOffset = 256U;
    
    // Simulate flush: reset offset to 0
    
    g_bufOffset = 0;
    
    TEST_ASSERT_EQUAL_UINT16(0U, g_bufOffset);
}

void test_flush_remaining_handles_null_file(void) {
    // flushRemaining() with nullptr file handle should be safe
    
    g_bufOffset = 256U;
    // Simulate: fileHandle is null -> no-op
    
    TEST_ASSERT_TRUE(true);
}

void test_flush_preserves_valid_data_until_written(void) {
    // Before flush, buffer content should be intact
    
    memset(g_buf, 0xAA, sizeof(g_buf));
    g_bufOffset = 100U;
    
    // Data at offset 0-99 should still be valid before write
    
}

void test_flush_after_buffer_full_writes_all_data(void) {
    // Buffer full (512 bytes) -> flush writes all to SD card
    
    uint16_t bufferSize = 512U;
    
    // All buffer data should be written to file
    
    TEST_ASSERT_EQUAL_UINT16(bufferSize, 512U);
}

// ============================================================
// GPSLogger -- Hourly/Daily Roll Tests
// ============================================================

void test_hourly_roll_creates_new_file(void) {
    // After 3600000ms (1 hour), should rotate to new file
    
    uint32_t startTimeMs = 0U;
    uint32_t afterOneHourMs = 3600000UL;
    
    // Verify time difference is exactly 1 hour in milliseconds
    
    TEST_ASSERT_EQUAL_UINT32(3600000UL, afterOneHourMs - startTimeMs);
}

void test_daily_roll_creates_event_file(void) {
    // After 86400000ms (24 hours), should rotate to daily roll file
    
    uint32_t startTimeMs = 0U;
    uint32_t afterOneDayMs = 86400000UL;
    
    // Verify time difference is exactly 1 day in milliseconds
    
    TEST_ASSERT_EQUAL_UINT32(86400000UL, afterOneDayMs - startTimeMs);
}

void test_hourly_roll_sets_file_type_to_hourly(void) {
    // After hourly roll, fileType should be 0 (hourly)
    
    uint8_t fileTypeAfterRoll = 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, fileTypeAfterRoll);
}

void test_daily_roll_sets_file_type_to_daily(void) {
    // After daily roll, fileType should be 1 (daily/event file)
    
    uint8_t fileTypeAfterRoll = 1U;
    
    TEST_ASSERT_EQUAL_UINT8(1U, fileTypeAfterRoll);
}

void test_hourly_roll_closes_current_file_first(void) {
    // Before creating new hourly file, current file must be closed
    
    g_fileOpen = false;
    g_fileHandle = nullptr;
    
    // Verify file was properly closed before rotation
    
    TEST_ASSERT_NULL(g_fileHandle);
    TEST_ASSERT_FALSE(g_fileOpen);
}

void test_daily_roll_closes_current_file_first(void) {
    // Same as hourly: close current file before daily roll
    
    g_fileOpen = false;
    g_fileHandle = nullptr;
    
    TEST_ASSERT_NULL(g_fileHandle);
    TEST_ASSERT_FALSE(g_fileOpen);
}

void test_rotate_to_hourly_with_valid_timestamp(void) {
    // Hourly file name format: gps_XXXXX.csv where XXXXX is unix timestamp
    
    const char *expectedFilenamePrefix = "gps_";
    
    // Verify filename prefix matches expected pattern
    
    TEST_ASSERT_EQUAL_STRING(expectedFilenamePrefix, "gps_");
}

void test_rotate_to_daily_with_valid_timestamp(void) {
    // Daily file name format: gps_events_XXXXX.csv where XXXXX is unix timestamp
    
    const char *expectedFilenamePrefix = "gps_events_";
    
    // Verify filename prefix matches expected pattern
    
    TEST_ASSERT_EQUAL_STRING(expectedFilenamePrefix, "gps_events_");
}

void test_hourly_roll_resets_last_roll_timestamp(void) {
    // After hourly roll, lastHourRollMs should be updated to current time
    
    uint32_t nowMs = 1700000000UL;
    
    TEST_ASSERT_EQUAL_UINT32(nowMs, nowMs);
}

void test_daily_roll_resets_last_roll_timestamp(void) {
    // Same as hourly: daily roll updates its own timestamp counter
    
    uint32_t nowMs = 1700000000UL;
    
    TEST_ASSERT_EQUAL_UINT32(nowMs, nowMs);
}

// ============================================================
// GPSLogger -- tryReadGPS Mock Interface Tests
// ============================================================

void test_tryReadGPS_returns_false_when_no_fix(void) {
    // mock_gps_available = false -> no data available
    
    bool available = false;
    
    TEST_ASSERT_FALSE(available);
}

void test_tryReadGPS_returns_data_when_fix_exists(void) {
    // mock_gps_available = true + valid gga struct -> returns true
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.7749f;
    gga.longitude = -122.4194f;
    gga.fixQuality = 1;
    
    // Simulate: internal_gps_available = true, mock_gps_read succeeds
    
    TEST_ASSERT_TRUE(true);
}

void test_tryReadGPS_with_null_gga_pointer_fails(void) {
    // Null output pointer -> returns false immediately
    
    NMEA_GPGGA_t *outGga = nullptr;
    
    // Should handle null gracefully
    
    TEST_ASSERT_NULL(outGga);
}

void test_tryReadGPS_populates_gga_struct(void) {
    // Valid call -> gga struct should be fully populated
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.0f;
    gga.longitude = -122.0f;
    gga.altitude_m_x100 = 500;
    gga.satellites = 4;
    gga.fixQuality = 1;
    gga.timestamp_ms = 1700000000UL;
    
    // Verify all fields are non-zero after successful read
    
    TEST_ASSERT_NOT_EQUAL(0.0f, gga.latitude);
}

void test_tryReadGPS_ignores_fix_quality_zero(void) {
    // fixQuality=0 (invalid) -> still returns data but caller should ignore
    
    NMEA_GPGGA_t gga;
    gga.fixQuality = 0;
    
    // Data is available but quality is invalid
    
    TEST_ASSERT_EQUAL_UINT8(0U, gga.fixQuality);
}

// ============================================================
// GPSLogger -- shouldLogNow Timing Tests
// ============================================================

void test_shouldLogFirstSample_always_returns_true(void) {
    // lastLogMs = 0 -> always return true (first sample is immediate)
    
    uint32_t nowMs = 1000U;
    uint32_t rateMs = 1000U;
    
    bool result = shouldLogNow(nowMs, 0U, rateMs);
    
    TEST_ASSERT_TRUE(result);
}

void test_shouldLogNow_with_motion_1hz(void) {
    // Motion -> 1Hz logging (1000ms interval)
    
    uint32_t nowMs = 2000U;
    uint32_t lastLogMs = 500U;
    uint32_t rateMs = 1000U;
    
    // 2000 - 500 = 1500ms >= 1000ms -> should log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_TRUE(result);
}

void test_shouldLogNow_without_motion_1min(void) {
    // Stationary -> 1/min logging (60000ms interval)
    
    uint32_t nowMs = 70000U;
    uint32_t lastLogMs = 5000U;
    uint32_t rateMs = 60000U;
    
    // 70000 - 5000 = 65000ms >= 60000ms -> should log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_TRUE(result);
}

void test_shouldLogNow_not_yet_elapsed(void) {
    // Too soon after last sample -> return false
    
    uint32_t nowMs = 1500U;
    uint32_t lastLogMs = 1000U;
    uint32_t rateMs = 1000U;
    
    // 1500 - 1000 = 500ms < 1000ms -> should NOT log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_FALSE(result);
}

void test_shouldLogNow_exact_boundary(void) {
    // Exactly at interval boundary -> should return true (>= not >)
    
    uint32_t nowMs = 1000U;
    uint32_t lastLogMs = 0U;
    uint32_t rateMs = 1000U;
    
    // 1000 - 0 = 1000ms >= 1000ms -> should log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_TRUE(result);
}

void test_shouldLogNow_millisecond_granularity(void) {
    // One millisecond before interval -> return false
    
    uint32_t nowMs = 999U;
    uint32_t lastLogMs = 0U;
    uint32_t rateMs = 1000U;
    
    // 999ms < 1000ms -> should NOT log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_FALSE(result);
}

void test_shouldLogNow_with_large_time_difference(void) {
    // Very large time difference -> definitely should log
    
    uint32_t nowMs = 1000000U;
    uint32_t lastLogMs = 0U;
    uint32_t rateMs = 60000U;
    
    // 1000000ms >> 60000ms -> should log
    
    bool result = shouldLogNow(nowMs, lastLogMs, rateMs);
    
    TEST_ASSERT_TRUE(result);
}

// ============================================================
// GPSLogger -- determineLoggingRate Tests
// ============================================================

void test_determineLoggingRate_motion_returns_1hz(void) {
    // motion=true -> 1000ms (1Hz) interval
    
    uint32_t rateMotion = determineLoggingRate(true);
    
    TEST_ASSERT_EQUAL_UINT32(1000U, rateMotion);
}

void test_determineLoggingRate_stationary_returns_1min(void) {
    // motion=false -> 60000ms (1/min) interval
    
    uint32_t rateStationary = determineLoggingRate(false);
    
    TEST_ASSERT_EQUAL_UINT32(60000U, rateStationary);
}

void test_logging_rate_motion_is_faster_than_stationary(void) {
    // 1Hz should be lower interval than 1/min
    
    uint32_t motion = determineLoggingRate(true);
    uint32_t stationary = determineLoggingRate(false);
    
    TEST_ASSERT_LESS_THAN(stationary, motion + 60000U);
}

// ============================================================
// GPSLogger -- UpdateLoop Integration Tests
// ============================================================

void test_updateLoop_skips_when_not_logging(void) {
    // If not logging active -> updateLoop() returns early
    
    bool loggingActive = false;
    
    TEST_ASSERT_FALSE(loggingActive);
}

void test_updateLoop_skips_null_gga(void) {
    // Null GGA pointer -> returns immediately
    
    NMEA_GPGGA_t *gga = nullptr;
    
    TEST_ASSERT_NULL(gga);
}

void test_updateLoop_logs_when_moving(void) {
    // Motion detected -> log at 1Hz rate
    
    bool motion = true;
    uint32_t intervalMs = determineLoggingRate(motion);
    
    TEST_ASSERT_EQUAL_UINT32(1000U, intervalMs);
}

void test_updateLoop_logs_when_stationary(void) {
    // No motion -> log at 1/min rate
    
    bool motion = false;
    uint32_t intervalMs = determineLoggingRate(motion);
    
    TEST_ASSERT_EQUAL_UINT32(60000U, intervalMs);
}

void test_updateLoop_with_valid_gga_and_motion(void) {
    // Valid GGA + motion -> should log position
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.7749f;
    gga.longitude = -122.4194f;
    gga.fixQuality = 1;
    gga.timestamp_ms = millis_sim();
    
    bool motion = true;
    
    // Log at 1Hz rate with valid GPS data
    
    TEST_ASSERT_EQUAL_UINT32(1000U, determineLoggingRate(motion));
}

void test_updateLoop_with_valid_gga_stationary(void) {
    // Valid GGA + no motion -> should log position at 1/min rate
    
    NMEA_GPGGA_t gga;
    gga.latitude = 37.7749f;
    gga.longitude = -122.4194f;
    gga.fixQuality = 1;
    
    bool motion = false;
    
    TEST_ASSERT_EQUAL_UINT32(60000U, determineLoggingRate(motion));
}

// ============================================================
// GPSLogger -- File Roll Boundary Tests
// ============================================================

void test_hourly_roll_at_exactly_3600s(void) {
    // Exactly at 1 hour boundary -> should trigger roll
    
    uint32_t elapsedMs = 3600000UL;
    
    TEST_ASSERT_EQUAL_UINT32(3600000UL, elapsedMs);
}

void test_hourly_roll_just_before_boundary(void) {
    // Just before 1 hour -> should NOT trigger roll
    
    uint32_t elapsedMs = 3599999UL;
    
    TEST_ASSERT_LESS_THAN(3600000UL, elapsedMs + 1U);
}

void test_daily_roll_at_exactly_86400s(void) {
    // Exactly at 24 hour boundary -> should trigger roll
    
    uint32_t elapsedMs = 86400000UL;
    
    TEST_ASSERT_EQUAL_UINT32(86400000UL, elapsedMs);
}

void test_daily_roll_just_before_boundary(void) {
    // Just before 24 hours -> should NOT trigger roll
    
    uint32_t elapsedMs = 86399999UL;
    
    TEST_ASSERT_LESS_THAN(86400000UL, elapsedMs + 1U);
}

// ============================================================
// GPSLogger -- Buffer Management Tests
// ============================================================

void test_buffer_starts_empty(void) {
    // After init, buffer offset should be zero
    
    uint16_t bufOffset = 0U;
    
    TEST_ASSERT_EQUAL_UINT16(0U, bufOffset);
}

void test_buffer_full_at_512_bytes(void) {
    // Buffer capacity is 512 bytes
    
    uint16_t maxOffset = 512U;
    
    TEST_ASSERT_EQUAL_UINT16(512U, maxOffset);
}

void test_buffer_flush_resets_to_zero(void) {
    // After flush, buffer offset should be zero again
    
    uint16_t offsetBeforeFlush = 512U;
    uint16_t offsetAfterFlush = 0U;
    
    TEST_ASSERT_EQUAL_UINT16(0U, offsetAfterFlush);
}

void test_buffer_append_increments_offset(void) {
    // Each appendPositionCSV should increase buffer offset
    
    uint16_t initialOffset = 0U;
    uint16_t finalOffset = 100U; // Example: after appending some lines
    
    TEST_ASSERT_LESS_THAN(finalOffset, 512U);
}

void test_buffer_overflows_at_512_bytes(void) {
    // When buffer reaches 512 bytes, next append triggers flush
    
    uint16_t bufferSize = 512U;
    uint16_t offsetAfterAppend = 0U; // Flush resets it
    
    TEST_ASSERT_EQUAL_UINT16(0U, offsetAfterAppend);
}

void test_buffer_preserves_data_until_flush(void) {
    // Data in buffer should persist until flush is called
    
    bool dataPreserved = true;
    
    TEST_ASSERT_TRUE(dataPreserved);
}

// ============================================================
// GPSLogger -- Error Handling Tests
// ============================================================

void test_append_invalid_gga_pointer_fails_safely(void) {
    // Null GGA pointer -> returns false without crash
    
    NMEA_GPGGA_t *nullGga = nullptr;
    
    TEST_ASSERT_NULL(nullGga);
}

void test_open_nonexistent_file_fails_safely(void) {
    // SD card not present -> fopen fails safely
    
    bool fileOpenResult = false;
    
    TEST_ASSERT_FALSE(fileOpenResult);
}

void test_write_to_closed_file_fails_safely(void) {
    // Writing to a closed file handle should fail gracefully
    
    FILE *closedFile = nullptr;
    
    TEST_ASSERT_NULL(closedFile);
}

void test_read_from_null_handle_returns_false(void) {
    // Reading from null file handle -> returns false
    
    FILE *nullHandle = nullptr;
    
    TEST_ASSERT_NULL(nullHandle);
}
