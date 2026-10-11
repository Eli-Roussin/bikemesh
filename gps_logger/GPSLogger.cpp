#include "GPSLogger.h"
#include "../config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static bool sdcard_spi_init(void) {
    return true;
}

static bool sdcard_open(const char *filename, FILE **fileHandle) {
    if (fileHandle == nullptr || filename == nullptr) {
        return false;
    }

    *fileHandle = fopen(filename, "a+");
    return *fileHandle != nullptr;
}

static bool sdcard_write(FILE *fileHandle, const uint8_t *data, uint16_t len, uint32_t *bytesWritten) {
    if (fileHandle == nullptr || data == nullptr || len == 0U) {
        if (bytesWritten != nullptr) {
            *bytesWritten = 0U;
        }
        return false;
    }

    size_t written = fwrite(data, sizeof(uint8_t), len, fileHandle);
    fflush(fileHandle);
    if (bytesWritten != nullptr) {
        *bytesWritten = static_cast<uint32_t>(written);
    }
    return written == len;
}

static bool sdcard_read(FILE *fileHandle, uint8_t *buffer, uint16_t len, uint32_t *bytesRead) {
    if (fileHandle == nullptr || buffer == nullptr || len == 0U) {
        if (bytesRead != nullptr) {
            *bytesRead = 0U;
        }
        return false;
    }

    size_t readCount = fread(buffer, sizeof(uint8_t), len, fileHandle);
    if (bytesRead != nullptr) {
        *bytesRead = static_cast<uint32_t>(readCount);
    }
    return readCount == len;
}

static bool sdcard_close(FILE *fileHandle) {
    if (fileHandle == nullptr) {
        return true;
    }
    return fclose(fileHandle) == 0;
}

static bool internal_gps_available = false;
static NMEA_GPGGA_t last_mock_gga;

static void mock_gps_init(void) {
    internal_gps_available = false;
    memset(&last_mock_gga, 0, sizeof(last_mock_gga));
}

static bool mock_gps_read(NMEA_GPGGA_t *outGga) {
    if (internal_gps_available && outGga != nullptr) {
        memcpy(outGga, &last_mock_gga, sizeof(last_mock_gga));
        return true;
    }
    return false;
}

void GPSLogger::init(void) {
    if (!sdcard_spi_init()) {
        LOG_ERROR("GPSLogger: SD card SPI init failed -- cannot log to SD");
        m_initialized = false;
        return;
    }

    m_fsHandle = nullptr;
    m_fileHandle = nullptr;
    m_loggingActive = true;
    m_fileType = 0;
    m_bufOffset = 0;
    memset(m_buf, 0, sizeof(m_buf));
    initializeFirstFile();
    m_initialized = true;
    mock_gps_init();

    LOG_INFO("GPSLogger: SD card initialized -- CSV logging active (hourly roll)");
}

void GPSLogger::startLogging(void) {
    m_loggingActive = true;

    if (!m_initialized) {
        init();
        return;
    }

    if (m_fileHandle == nullptr) {
        initializeFirstFile();
    }
}

void GPSLogger::stopLogging(void) {
    m_loggingActive = false;
    flushRemaining();

    if (m_fileHandle != nullptr) {
        sdcard_close(m_fileHandle);
        m_fileHandle = nullptr;
        m_fileOpen = false;
    }

    LOG_INFO("GPSLogger: Logging stopped (Mode B anti-theft active)");
}

void GPSLogger::updateLoop(NMEA_GPGGA_t *gga, bool motion) {
    const uint32_t nowMs = millis();

    if (!m_loggingActive || gga == nullptr) {
        return;
    }

    const uint32_t intervalMs = motion ? 1000U : 60000U;
    if (!shouldLogNow(nowMs, m_lastLogTimestampMs, intervalMs)) {
        return;
    }

    if (appendPositionCSV(gga, motion ? 1U : 0U)) {
        m_lastLogTimestampMs = nowMs;

        if (m_fileType == 0U) {
            if ((nowMs - m_lastHourRollMs) > 3600000UL) {
                flushRemaining();
                if (m_fileHandle != nullptr) {
                    sdcard_close(m_fileHandle);
                    m_fileHandle = nullptr;
                    m_fileOpen = false;
                }
                rotateHourlyFile();
                m_lastHourRollMs = nowMs;
            }
        } else if ((nowMs - m_lastDayRollMs) > 86400000UL) {
            flushRemaining();
            if (m_fileHandle != nullptr) {
                sdcard_close(m_fileHandle);
                m_fileHandle = nullptr;
                m_fileOpen = false;
            }
            rotateDailyFile();
            m_lastDayRollMs = nowMs;
        }
    }
}

bool GPSLogger::tryReadGPS(bool *outAvailable, NMEA_GPGGA_t *outGga) {
    if (!internal_gps_available || outGga == nullptr || outAvailable == nullptr) {
        return false;
    }

    const bool hasFix = mock_gps_read(outGga);
    *outAvailable = hasFix;
    return hasFix;
}

uint32_t GPSLogger::determineLoggingRate(bool motion) {
    return motion ? 1000U : 60000U;
}

bool GPSLogger::shouldLogNow(uint32_t nowMs, uint32_t lastLogMs, uint32_t rateMs) {
    if (lastLogMs == 0U) {
        return true;
    }
    return (nowMs - lastLogMs) >= rateMs;
}

void GPSLogger::initializeFirstFile(void) {
    uint32_t nowMs = millis();
    snprintf(m_currentFilename, sizeof(m_currentFilename), "gps_%lu.csv", static_cast<unsigned long>(nowMs / 1000UL));
    if (m_fileHandle != nullptr) {
        flushRemaining();
        sdcard_close(m_fileHandle);
        m_fileHandle = nullptr;
    }

    if (!sdcard_open(m_currentFilename, &m_fileHandle)) {
        LOG_ERROR("GPSLogger: Unable to open log file %s", m_currentFilename);
        m_fileOpen = false;
        m_loggingActive = false;
        return;
    }

    m_fileOpen = true;
    const char *header = "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n";
    fwrite(header, 1, strlen(header), m_fileHandle);
    fflush(m_fileHandle);
    m_lastHourRollMs = nowMs;
    m_lastDayRollMs = nowMs;
    m_fileType = 0U;
}

void GPSLogger::requestHourlyRoll(void) {
    flushRemaining();
    if (m_fileHandle != nullptr) {
        sdcard_close(m_fileHandle);
        m_fileHandle = nullptr;
        m_fileOpen = false;
    }
    rotateHourlyFile();
}

void GPSLogger::requestDailyRoll(void) {
    flushRemaining();
    if (m_fileHandle != nullptr) {
        sdcard_close(m_fileHandle);
        m_fileHandle = nullptr;
        m_fileOpen = false;
    }
    rotateDailyFile();
}

bool GPSLogger::readGPSFromUART(NMEA_GPGGA_t *outGga) {
    if (outGga == nullptr) {
        return false;
    }
    return mock_gps_read(outGga);
}

void GPSLogger::rotateHourlyFile(void) {
    const uint32_t nowMs = millis();
    snprintf(m_currentFilename, sizeof(m_currentFilename), "gps_%lu.csv", static_cast<unsigned long>(nowMs / 1000UL));

    if (!sdcard_open(m_currentFilename, &m_fileHandle)) {
        LOG_ERROR("GPSLogger: Unable to rotate to hourly file %s", m_currentFilename);
        m_fileOpen = false;
        return;
    }

    m_fileOpen = true;
    m_fileType = 0U;
    const char *header = "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n";
    fwrite(header, 1, strlen(header), m_fileHandle);
    fflush(m_fileHandle);
    LOG_INFO("GPSLogger: Hourly roll -- %s", m_currentFilename);
}

void GPSLogger::rotateDailyFile(void) {
    const uint32_t nowMs = millis();
    snprintf(m_currentFilename, sizeof(m_currentFilename), "gps_events_%lu.csv", static_cast<unsigned long>(nowMs / 1000UL));

    if (!sdcard_open(m_currentFilename, &m_fileHandle)) {
        LOG_ERROR("GPSLogger: Unable to rotate to daily file %s", m_currentFilename);
        m_fileOpen = false;
        return;
    }

    m_fileOpen = true;
    m_fileType = 1U;
    const char *header = "timestamp_ms,latitude_deg,longitude_deg,altitude_m_x100,satellites,fix_quality,source_channel\n";
    fwrite(header, 1, strlen(header), m_fileHandle);
    fflush(m_fileHandle);
    LOG_INFO("GPSLogger: Daily roll -- %s", m_currentFilename);
}

bool GPSLogger::appendPositionCSV(NMEA_GPGGA_t *gga, uint8_t sourceChannel) {
    if (gga == nullptr) {
        return true;
    }

    char line[128];
    const int lineLength = snprintf(line, sizeof(line),
                                   "%lu,%0.6f,%0.6f,%d,%u,%u,%s\n",
                                   static_cast<unsigned long>(gga->timestamp_ms),
                                   gga->latitude,
                                   gga->longitude,
                                   gga->altitude_m_x100 / 100,
                                   static_cast<unsigned>(gga->satellites),
                                   static_cast<unsigned>(gga->fixQuality),
                                   sourceChannel == 1U ? "vehicle_gps" : "mesh");

    if (lineLength <= 0) {
        return false;
    }

    const uint16_t lineLen = static_cast<uint16_t>(lineLength);
    if (m_bufOffset + lineLen >= sizeof(m_buf)) {
        flush();
    }

    memcpy(m_buf + m_bufOffset, line, lineLen);
    m_bufOffset += lineLen;
    flushIfNeeded();
    return true;
}

void GPSLogger::flushIfNeeded(void) {
    if (m_bufOffset >= sizeof(m_buf)) {
        flush();
    }
}

void GPSLogger::flush(void) {
    if (m_bufOffset == 0U || m_fileHandle == nullptr) {
        return;
    }

    uint32_t bytesWritten = 0U;
    sdcard_write(m_fileHandle, reinterpret_cast<const uint8_t *>(m_buf), m_bufOffset, &bytesWritten);
    memset(m_buf, 0, sizeof(m_buf));
    m_bufOffset = 0U;

    LOG_DEBUG("GPSLogger: Flush -- %lu bytes written to SD card", static_cast<unsigned long>(bytesWritten));
}

void GPSLogger::flushRemaining(void) {
    if (m_bufOffset > 0U && m_fileHandle != nullptr) {
        flush();
    }
}
