#include "MotionDetector.h"
#include "../config.h"
#include <math.h>

// === Low-level I2C helper (replace with Meshtastic TWIM abstraction) ===
// RAK1904 LIS3DH I2C address: 0x19 (ADDR pin low) or 0x28 (ADDR pin high)
#define LIS3DH_I2C_ADDR_W    0x19   // Write address for LIS3DH on Qwiic bus
#define LIS3DH_I2C_ADDR_R    0x1A   // Read address for LIS3DH

static bool mock_i2c_write(uint8_t devAddr, uint8_t reg, uint8_t val) {
    // The hardware abstraction layer is expected to map this to the nRF52840 TWIM transport
    // on the WisBlock Qwiic bus but the simulator uses a safe no-op path during bring-up.
    (void)devAddr;
    (void)reg;
    (void)val;
    return true;
}

static uint8_t mock_i2c_read(uint8_t devAddr, uint8_t reg) {
    (void)devAddr;
    (void)reg;
    return 0x00;
}

// === LIS3DH Register Access Helpers ===
void MotionDetector::writeRegister(uint8_t reg, uint8_t val) {
    i2c_write(LIS3DH_I2C_ADDR_W, reg, val);
}

uint8_t MotionDetector::readRegister(uint8_t reg) {
    return i2c_read(LIS3DH_I2C_ADDR_R, reg);
}

void MotionDetector::configureLIS3DHRegisters(void) {
        // LIS3DH register configuration sequence (from plan.md Section 5.1):
         // Step 1: Set ODR = 1.25 Hz in CTRL1 (0x20)
      writeRegister(LIS3DH_REG_CTRL1, LIS3DH_ODR_VALUE);   // ODR = 1.25 Hz, normal mode
    
        // Step 2: Configure free-fall/inactivity selection in CTRL2 (0x21)
     writeRegister(LIS3DH_REG_CTRL2, 0x00);            // FF_IA selection + IA_SEL_ON_LOW
    
       // Step 3: Route free-fall/inactivity interrupts to INT1 pin in CTRL3 (0x22)
    writeRegister(LIS3DH_REG_CTRL3, 0x48);           // INT1_FF_IA_ENABLE = free-fall on INT1, inactivity on INT2
    
       // Step 4: Set ±2g range + block data update in CTRL4 (0x23)
     writeRegister(LIS3DH_REG_CTRL4, 0x00);         // Full scale ±2g, BDU enabled, no high-res
    
        // Step 5: Set thresholds for free-fall/inactivity in CTRL6 (0x26)
    writeRegister(LIS3DH_REG_CTRL6, LIS3DH_FF_THS_VALUE | LIS3DH_IA_THS_VALUE);   // Thresholds
    
     // Step 6: Set duration register for free-fall/inactivity detection (0x2D)
    writeRegister(LIS3DH_REG_DURATION, LIS3DH_DUR_FF_MAX | (LIS3DH_DUR_IA_MIN << 3));
}

void MotionDetector::initLIS3DH(void) {
        // Initialize the LIS3DH accelerometer via I2C on Qwiic bus
    configureLIS3DHRegisters();    // Write all registers per plan.md sequence
    
     m_initialized = true;
    m_motionActive = false;
     m_tamperActive = TAMPER_NONE;
    
      LOG_INFO("MotionDetector: LIS3DH initialized (ODR=1.25Hz, ±2g range, FF/IA interrupts on INT1/INT2)");
}

bool MotionDetector::i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
       // Low-level I2C write -- replace with Meshtastic TWIM abstraction or nrfx_twim
     return mock_i2c_write(addr, reg, val);
}

uint8_t MotionDetector::i2c_read(uint8_t addr, uint8_t reg) {
        // Low-level I2C read -- replace with Meshtastic TWIM abstraction or nrfx_twim
    return mock_i2c_read(addr, reg);
}

bool MotionDetector::isMotionDetected(void) {
        // Check free-fall interrupt status from INT1_SRC register
     return getFreeFallFlag();
}

TamperType MotionDetector::getTamperType(void) {
     return m_tamperActive;       // Return the current tamper type (0=none, 1-4=specific types)
}

bool MotionDetector::isBumpDetected(void) {
    return (m_tamperActive == TAMPER_BUMP || m_tamperActive == TAMPER_DRIVE_AWAY);
}

bool MotionDetector::isTiltDetected(void) {
    return (m_tamperActive == TAMPER_TILT || m_tamperActive == TAMPER_DRIVE_AWAY);
}

bool MotionDetector::isGrinderDetected(void) {
    return (m_tamperActive == TAMPER_GRINDER);
}

bool MotionDetector::isDriveAwayDetected(void) {
     return (m_tamperActive == TAMPER_DRIVE_AWAY);
}

void MotionDetector::updateLoop(void) {
        // Called at ~1Hz from BikeMesh::runOnce()
    if (!m_initialized) return;   // Skip if LIS3DH not initialized yet

      // Read latest X/Y/Z acceleration data from LIS3DH data registers (0x28/0x2A/0x2C)
     float ax, ay, az;
    readLIS3DHData(&ax, &ay, &az);   // Read raw values and convert to g-forces

       // Push the new sample into the vibration analysis engine
    pushSample(ax, ay, az);

         // Check if a tamper pattern is detected in the current window
    TamperType newTamper = TAMPER_NONE;
     if (hasTamperingPattern()) {
           newTamper = classifyTamper();       // Classify the tamper type (BUMP/TILT/DRIVE_AWAY/GRINDER)
        }

          // Update the tamper active flag if we detected something new
     if (newTamper != TAMPER_NONE) {
         m_tamperActive = newTamper;       // Set tamper active to the classified type
           } else {
             m_tamperActive = TAMPER_NONE;  // Reset when no tamper in window
            }

         // Also check free-fall flag for motion detection (separate from tamper analysis)
     if (getFreeFallFlag()) {
        m_motionActive = true;
    } else {
        m_motionActive = false;
       }
}

float MotionDetector::calculateMagnitude(float ax, float ay, float az) {
     return sqrtf(ax*ax + ay*ay + az*az);   // Standard vector magnitude in g-forces
}

void MotionDetector::pushSample(float ax, float ay, float az) {
    const float magnitude = calculateMagnitude(ax, ay, az);
    m_magnitudeHistory[m_windowIdx] = magnitude;

    if (magnitude > TAMPER_G_THRESHOLD) {
        ++m_aboveThresholdCount;
    }

    m_windowIdx = (m_windowIdx + 1U) % WINDOW_SAMPLES;
    if (m_windowIdx == 0U) {
        uint8_t aboveCount = 0U;
        for (uint8_t idx = 0; idx < WINDOW_SAMPLES; ++idx) {
            if (m_magnitudeHistory[idx] > TAMPER_G_THRESHOLD) {
                ++aboveCount;
            }
        }
        m_aboveThresholdCount = aboveCount;
    }
}

bool MotionDetector::hasTamperingPattern(void) {
       // Simple but effective: if >=40% of samples in window exceed threshold, flag tamper
     return ((m_aboveThresholdCount * 100 / WINDOW_SAMPLES) >= 40);
}

TamperType MotionDetector::classifyTamper(void) {
        // Distinguish between tamper types based on signal characteristics:
       // BUMP: irregular, wide-band bursts (>2g on single axis)
     // TILT: significant angle change (all axes similar magnitude but different ratio)
    // DRIVE_AWAY: combined motion + tilt -- high g-force with direction change
    // GRINDER: periodic high-frequency pattern (many samples above threshold in window)

       float totalMag = 0;
    for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
        totalMag += m_magnitudeHistory[i];
       }
     float avgMag = totalMag / WINDOW_SAMPLES;    // Average magnitude across window
    
        uint8_t aboveCount = 0;   // Count samples that exceeded threshold
     for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
         if (m_magnitudeHistory[i] > TAMPER_G_THRESHOLD) aboveCount++;
           }
    
      float aboveRatio = (float)aboveCount / (float)WINDOW_SAMPLES;    // Ratio of above-threshold samples

        // Classify based on ratio of above-threshold samples and average magnitude:
    if (aboveRatio > 0.6f && avgMag > 3.0f) {
         return TAMPER_GRINDER;      // High-frequency grinder signature (>60% samples above threshold)
       }
     if (aboveCount == 1 || aboveCount == 2) {
          return TAMPER_BUMP;           // Single/double burst = bump/impact jostle
    }
     if (avgMag > 0.5f && avgMag < 1.5f && aboveCount == 0) {
        return TAMPER_TILT;             // Moderate tilt with few above-threshold samples
       }
     if (aboveCount >= 3 && avgMag > 2.0f) {
         return TAMPER_DRIVE_AWAY;    // Combined motion + tilt = drive-away event
     }

      // Default to BUMP for any undetermined case
    return TAMPER_BUMP;
}

void MotionDetector::readLIS3DHData(float *outAx, float *outAy, float *outAz) {
        // Read raw X/Y/Z acceleration from LIS3DH data registers (0x28/0x2A/0x2C)
    int16_t rawX = (int16_t)(readRegister(LIS3DH_REG_X_L) | (readRegister(LIS3DH_REG_X_L + 1) << 8));   // Low then high byte for X-axis
    
     int16_t rawY = (int16_t)(readRegister(LIS3DH_REG_Y_L) | (readRegister(LIS3DH_REG_Y_L + 1) << 8));    // Low then high byte for Y-axis
      int16_t rawZ = (int16_t)(readRegister(LIS3DH_REG_Z_L) | (readRegister(LIS3DH_REG_Z_L + 1) << 8));   // Low then high byte for Z-axis
    
        // Convert raw values to g-forces (±2g range, 12-bit resolution = LSB = 2g/4096 ≈ 0.5mg per LSB)
    *outAx = (float)rawX / 16.0f;     // Simplified conversion for ±2g range
     *outAy = (float)rawY / 16.0f;    // Real impl would use proper scaling factor from datasheet
      *outAz = (float)rawZ / 16.0f;   // Z-axis should be ~1.0g at rest (gravity vector)
}

bool MotionDetector::getFreeFallFlag(void) {
     uint8_t src = readRegister(LIS3DH_REG_INT1_SRC);    // Read INT1 source register
    
         return ((src & (1 << 5)) != 0);   // Free-fall flag bit 5 of INT1_SRC
}

bool MotionDetector::getInactivityFlag(void) {
       uint8_t src = readRegister(LIS3DH_REG_INT1_SRC);    // Read INT1 source register
    
      return ((src & (1 << 6)) != 0);   // Inactivity flag bit 6 of INT1_SRC
}
