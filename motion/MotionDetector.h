#pragma once

// ============================================
// MotionDetector.h -- LIS3DH interrupt config and vibration analysis
// Free-fall/inactivity detection via I2C (RAK1904 on Qwiic bus)
// Vibration analysis for angle grinder/jostle/tilt/tamper detection.
// ============================================

#include <stdint.h>
#include "../config.h"

// LIS3DH register addresses (from STMicroelectronics LIS3DH datasheet):
#define LIS3DH_REG_CTRL1       0x20      // Control register 1 -- ODR, bandwidth
#define LIS3DH_REG_CTRL2       0x21      // Control register 2 -- FF_IA selection
#define LIS3DH_REG_CTRL3       0x22      // Control register 3 -- pin assignments (INT routing)
#define LIS3DH_REG_CTRL4       0x23      // Control register 4 -- full-scale, BDU, high-res
#define LIS3DH_REG_CTRL6       0x26      // Control register 6 -- thresholds for FF/IA
#define LIS3DH_REG_DURATION    0x2D      // Duration register for free-fall/inactivity detection
#define LIS3DH_REG_INT1_SRC    0x30      // Interrupt 1 source register
#define LIS3DH_REG_FREE_FALL   0x38      // Free-fall flag (bit 5 of INT1_SRC)
#define LIS3DH_REG_IA1         0x39      // Interrupt 1 register (inactivity flag bit 6)
#define LIS3DH_REG_X_L         0x28      // X-axis acceleration data LSB
#define LIS3DH_REG_Y_L         0x2A      // Y-axis acceleration data LSB
#define LIS3DH_REG_Z_L         0x2C      // Z-axis acceleration data LSB

// Configuration parameters (tunable via config.h):
#define LIS3DH_ODR_VALUE       0x50      // ODR = 1.25 Hz, normal mode (low power, reasonable for tamper)
#define LIS3DH_FF_THS_VALUE    0x40      // Free-fall threshold -- ~50% of g range (adjustable via config.h)
#define LIS3DH_IA_THS_VALUE    0x80      // Inactivity threshold -- slightly higher than FF
#define LIS3DH_DUR_FF_MAX      0x0A      // Max 10 * ODR cycles for free-fall detection
#define LIS3DH_DUR_IA_MIN      0x05      // Min 5 * ODR cycles before inactivity detected (low threshold)

// TAMPER types: each has its own rate limiter (max once per 30s independently)
typedef enum {
    TAMPER_NONE = 0,               // No tamper detected
    TAMPER_BUMP = 1,               // Sudden jostle/impact (>2.0g on any axis)
    TAMPER_TILT = 2,                // Significant tilt angle change (>30 degrees -- bike being moved/dismantled)
    TAMPER_DRIVE_AWAY = 3,        // Combined motion + tilt -- bike started moving with angle change
    TAMPER_GRINDER = 4             // Angle grinder vibration signature (200-500 Hz high-frequency pattern)
} TamperType;

// Vibration analysis window parameters.
// These values are tuned for LIS3DH at 1.25 Hz ODR on a stationary bike.
#define VIBRATION_WINDOW_MS    1000         // 1-second analysis window
#define WINDOW_SAMPLES          13          // Samples per window at 1.25 Hz ODR
#define TAMPER_G_THRESHOLD      2.0f        // Minimum g-force to flag as tamper

class MotionDetector {
public:
    MotionDetector() : m_initialized(false), m_motionActive(false), m_tamperActive(TAMPER_NONE),
                       m_windowIdx(0), m_aboveThresholdCount(0), m_lastSampleMs(0) {}

    void initLIS3DH(void);              // Configure LIS3DH via I2C for free-fall/inactivity interrupts on INT1/INT2

    bool isMotionDetected(void);        // True when free-fall interrupt is latched

    TamperType getTamperType(void);     // Latest classified tamper (clears after read by caller)

    bool isBumpDetected(void);          // True if a jostle/impact pattern was detected in the current window
    bool isTiltDetected(void);          // True if sustained tilt angle change was detected
    bool isGrinderDetected(void);       // True if angle-grinder vibration signature matched
    bool isDriveAwayDetected(void);     // True if motion + tilt together (bike being moved while upright)

    void updateLoop(void);              // Sample LIS3DH, push to analyzer, check for tamper patterns

private:
    bool m_initialized;                  // Set true after initLIS3DH() completes
    bool m_motionActive;                 // Free-fall interrupt is currently latched

    TamperType m_tamperActive;           // Most recent tamper classification from the analyzer
    uint8_t    m_windowIdx;              // Next write index into m_magnitudeHistory circular buffer
    float      m_magnitudeHistory[WINDOW_SAMPLES];  // Vector magnitude history for vibration analysis
    uint8_t    m_aboveThresholdCount;    // Samples exceeding TAMPER_G_THRESHOLD in current window
    uint32_t   m_lastSampleMs;           // Last time a sample was ingested for motion analysis

    // Low-level I2C -- replaced with Meshtastic TWIM abstraction at bringup time
    bool i2c_write(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t i2c_read(uint8_t addr, uint8_t reg);

    void configureLIS3DHRegisters(void); // Write register sequence from ST datasheet

    float calculateMagnitude(float ax, float ay, float az);  // Euclidean norm of acceleration vector
    void pushSample(float ax, float ay, float az);           // Append a new sample to the vibration window
    bool hasTamperingPattern(void);                            // True when >40% of samples exceed threshold

    TamperType classifyTamper(void);       // Classify tamper based on magnitude distribution and g-force profile
    void readLIS3DHData(float *outAx, float *outAy, float *outAz);  // Read raw X/Y/Z from data registers

    bool getFreeFallFlag(void);            // Free-fall interrupt latched in INT1_SRC register
    bool getInactivityFlag(void);          // Inactivity interrupt latched in INT2_SRC register
};
