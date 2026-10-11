#pragma once

// ============================================
// GPSLogger.h -- SD card logging in CSV format
// 512-byte Meshtastic-style block writes, no flush per line.
// High-volume files roll hourly, low-volume daily.
// ============================================

#include <stdint.h>
#include <cstdio>
#include "../config.h"

struct NMEA_GPGGA_t {
    float   latitude;             // WGS84 degrees (+N = north, +S = south)
    float   longitude;            // WGS84 degrees (+E = east, +W = west)
    int16_t altitude_m_x100;      // MSL altitude × 100 for precision (store meters with 2 decimal places)
    uint8_t satellites;           // Number of SVs in use
    uint8_t fixQuality;           // 0=invalid, 1=GPS, 2=DGPS
    uint32_t timestamp_ms;        // Unix epoch milliseconds (for CSV filename timestamps)
};

class GPSLogger {
public:
    GPSLogger() : m_initialized(false), m_loggingActive(false), m_bufOffset(0),
                  m_lastHourRollMs(0), m_lastDayRollMs(0), m_fileType(0),
                  m_lastLogTimestampMs(0), m_fileHandle(nullptr), m_fsHandle(nullptr),
                  m_fileOpen(false) {
        m_currentFilename[0] = '\0';
        m_buf[0] = '\0';
        m_latestGga = {};
    }

    void init(void);
    void updateLoop(NMEA_GPGGA_t *gga, bool motion);

    bool isLoggingActive(void) const { return m_loggingActive; }
    uint32_t getLastLogMs(void) const { return m_lastLogTimestampMs; }
    void requestHourlyRoll(void);
    void requestDailyRoll(void);
    bool tryReadGPS(bool *outAvailable, NMEA_GPGGA_t *outGga);

    void startLogging(void);
    void stopLogging(void);

private:
    bool m_initialized;
    bool m_loggingActive;
    uint16_t m_bufOffset;
    uint32_t m_lastHourRollMs;
    uint32_t m_lastDayRollMs;
    uint32_t m_lastLogTimestampMs;
    uint8_t m_fileType;

    char m_buf[512];
    NMEA_GPGGA_t m_latestGga;
    char m_currentFilename[32];
    bool m_fileOpen;
    FILE *m_fileHandle;
    void *m_fsHandle;

    void initializeFirstFile(void);
    void rotateHourlyFile(void);
    void rotateDailyFile(void);
    bool appendPositionCSV(NMEA_GPGGA_t *gga, uint8_t sourceChannel);
    void flush(void);
    void flushIfNeeded(void);
    void flushRemaining(void);
    uint32_t determineLoggingRate(bool motion);
    bool shouldLogNow(uint32_t nowMs, uint32_t lastLogMs, uint32_t rateMs);
    bool readGPSFromUART(NMEA_GPGGA_t *outGga);
};
