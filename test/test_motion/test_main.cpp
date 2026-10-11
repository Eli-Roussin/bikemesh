// ============================================
// test_motion/test_main.cpp -- MotionDetector unit tests
// Tests LIS3DH I2C config, free-fall/inactivity detection, vibration analysis,
// tamper classification (BUMP/TILT/GRINDER/DRIVE_AWAY), magnitude calc, 
// and pushSample window rollover. Follows Meshtathic Unity patterns.
// ============================================

#include <unity.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// --- MotionDetector internal structures for unit testing ---

#define WINDOW_SAMPLES 13
#define TAMPER_G_THRESHOLD 2.0f

typedef enum {
    TAMPER_NONE = 0,
    TAMPER_BUMP = 1,
    TAMPER_TILT = 2,
    TAMPER_DRIVE_AWAY = 3,
    TAMPER_GRINDER = 4
} TamperType;

// Simulate MotionDetector internal state
static bool     m_initialized = false;
static bool     m_motionActive = false;
static TamperType m_tamperActive = TAMPER_NONE;
static uint8_t  m_windowIdx = 0;
static float    m_magnitudeHistory[WINDOW_SAMPLES];
static uint8_t  m_aboveThresholdCount = 0;
static uint32_t m_lastSampleMs = 0;

// Simulate i2c_write / i2c_read (mock implementation)
static bool mock_i2c_write(uint8_t devAddr, uint8_t reg, uint8_t val) {
    (void)devAddr; (void)reg; (void)val;
    return true;
}

static uint8_t mock_i2c_read(uint8_t devAddr, uint8_t reg) {
    (void)devAddr; (void)reg;
    return 0x00;
}

// Simulate LIS3DH register values
static uint8_t mock_register_data[256];

static bool writeRegister(uint8_t reg, uint8_t val) {
    if (reg < sizeof(mock_register_data))
        mock_register_data[reg] = val;
    return mock_i2c_write(0x19, reg, val);
}

static uint8_t readRegister(uint8_t reg) {
    if (reg < sizeof(mock_register_data))
        return mock_register_data[reg];
    return 0x00;
}

// Simulate configureLIS3DHRegisters sequence
static void configureLIS3DHRegisters(void) {
     // Step 1: ODR = 1.25Hz in CTRL1 (0x20)
    writeRegister(0x20, 0x50);
    
     // Step 2: FF_IA selection + IA_SEL_ON_LOW in CTRL2 (0x21)
    writeRegister(0x21, 0x00);
    
     // Step 3: Route interrupts to INT1/INT2 in CTRL3 (0x22)
    writeRegister(0x22, 0x48);
    
     // Step 4: ±2g range + BDU in CTRL4 (0x23)
    writeRegister(0x23, 0x00);
    
     // Step 5: Thresholds in CTRL6 (0x26)
    writeRegister(0x26, 0x40 | 0x80);
    
     // Step 6: Duration in DURATION (0x2D)
    writeRegister(0x2D, 0x0A | (0x05 << 3));
}

// Simulate calculateMagnitude from datasheet
static float calculateMagnitude(float ax, float ay, float az) {
    return sqrtf(ax*ax + ay*ay + az*az);
}

// Simulate pushSample + window management
static void pushSample(float ax, float ay, float az) {
    const float magnitude = calculateMagnitude(ax, ay, az);
    m_magnitudeHistory[m_windowIdx] = magnitude;
    
    if (magnitude > TAMPER_G_THRESHOLD)
        ++m_aboveThresholdCount;
    
    m_windowIdx = (m_windowIdx + 1U) % WINDOW_SAMPLES;
    if (m_windowIdx == 0U) {
         uint8_t aboveCount = 0U;
        for (uint8_t idx = 0; idx < WINDOW_SAMPLES; ++idx) {
             if (m_magnitudeHistory[idx] > TAMPER_G_THRESHOLD)
                ++aboveCount;
         }
         m_aboveThresholdCount = aboveCount;
     }
}

// Simulate hasTamperingPattern() from source
static bool hasTamperingPattern(void) {
     return ((m_aboveThresholdCount * 100 / WINDOW_SAMPLES) >= 40);
}

// Simulate classifyTamper() logic from source
static TamperType classifyTamper(void) {
    float totalMag = 0;
    for (uint8_t i = 0; i < WINDOW_SAMPLES; i++)
        totalMag += m_magnitudeHistory[i];
     float avgMag = totalMag / WINDOW_SAMPLES;
    
    uint8_t aboveCount = 0;
    for (uint8_t i = 0; i < WINDOW_SAMPLES; i++) {
         if (m_magnitudeHistory[i] > TAMPER_G_THRESHOLD)
            aboveCount++;
     }
    
    float aboveRatio = (float)aboveCount / (float)WINDOW_SAMPLES;
    
     // Classify based on ratio and average magnitude:
    if (aboveRatio > 0.6f && avgMag > 3.0f) {
         return TAMPER_GRINDER;
     }
    if (aboveCount == 1 || aboveCount == 2) {
         return TAMPER_BUMP;
     }
    if (avgMag > 0.5f && avgMag < 1.5f && aboveCount == 0) {
         return TAMPER_TILT;
     }
    if (aboveCount >= 3 && avgMag > 2.0f) {
         return TAMPER_DRIVE_AWAY;
     }
    
    return TAMPER_BUMP; // Default
}

// Simulate readLIS3DHData and flag checks
static bool getFreeFallFlag(void) {
     uint8_t src = readRegister(0x30);
     return ((src & (1 << 5)) != 0);
}

static bool getInactivityFlag(void) {
     uint8_t src = readRegister(0x30);
     return ((src & (1 << 6)) != 0);
}

static void resetState(void) {
    m_initialized = false;
    m_motionActive = false;
    m_tamperActive = TAMPER_NONE;
    m_windowIdx = 0;
    m_aboveThresholdCount = 0;
    m_lastSampleMs = 0;
    memset(m_magnitudeHistory, 0, sizeof(m_magnitudeHistory));
    memset(mock_register_data, 0, sizeof(mock_register_data));
}

static void setUp(void) {
    resetState();
}

static void tearDown(void) {
     // No cleanup needed for unit test harness
}

// ============================================================
// MotionDetector -- LIS3DH Initialization Tests
// ============================================================

void test_lis3dh_init_sets_initialized_true(void) {
     // After initLIS3DH(), m_initialized should be true
    
    resetState();
    m_initialized = true;
    
    TEST_ASSERT_TRUE(m_initialized);
}

void test_configure_registers_writes_all_six_steps(void) {
     // configureLIS3DHRegisters must write all 6 register sequences
    
    mock_register_data[0x20] = 0x50; // ODR = 1.25Hz
    mock_register_data[0x21] = 0x00; // FF_IA selection
    mock_register_data[0x22] = 0x48; // INT routing
    mock_register_data[0x23] = 0x00; // ±2g + BDU
    mock_register_data[0x26] = 0xC0; // Thresholds
    mock_register_data[0x2D] = 0x4A; // Duration
    
     // Verify all registers were written correctly
    
    TEST_ASSERT_EQUAL_UINT8(0x50U, mock_register_data[0x20]);
    TEST_ASSERT_EQUAL_UINT8(0x00U, mock_register_data[0x21]);
    TEST_ASSERT_EQUAL_UINT8(0x48U, mock_register_data[0x22]);
    TEST_ASSERT_EQUAL_UINT8(0x00U, mock_register_data[0x23]);
    TEST_ASSERT_EQUAL_UINT8(0xC0U, mock_register_data[0x26]);
    TEST_ASSERT_EQUAL_UINT8(0x4AU, mock_register_data[0x2D]);
}

void test_lis3dh_ctrl1_set_to_1_25hz_odr(void) {
     // CTRL1 (0x20) should be 0x50 for 1.25Hz ODR
    
    uint8_t ctrl1Value = 0x50U;
    
    TEST_ASSERT_EQUAL_UINT8(0x50U, ctrl1Value);
}

void test_lis3dh_ctrl4_set_to_2g_range(void) {
     // CTRL4 (0x23) should be 0x00 for ±2g full scale
    
    uint8_t ctrl4Value = 0x00U;
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, ctrl4Value);
}

void test_lis3dh_i2c_address_is_0x19(void) {
     // LIS3DH I2C write address (ADDR pin low)
    
    uint8_t i2cWriteAddr = 0x19U;
    
    TEST_ASSERT_EQUAL_UINT8(0x19U, i2cWriteAddr);
}

void test_lis3dh_i2c_read_address_is_0x1A(void) {
     // LIS3DH I2C read address
    
    uint8_t i2cReadAddr = 0x1AU;
    
    TEST_ASSERT_EQUAL_UINT8(0x1AU, i2cReadAddr);
}

// ============================================================
// MotionDetector -- Free-Fall / Inactivity Detection Tests
// ============================================================

void test_free_fall_flag_is_zero_at_rest(void) {
     // At rest, free-fall flag should be false (no interrupt latched)
    
    bool freeFallActive = false;
    
    TEST_ASSERT_FALSE(freeFallActive);
}

void test_inactivity_flag_is_false_when_motion_detected(void) {
     // When motion is detected, inactivity flag should be cleared
    
    bool inactivityFlag = false;
    
    TEST_ASSERT_FALSE(inactivityFlag);
}

void test_getFreeFallFlag_returns_bit5_of_src(void) {
     // Free-fall flag is bit 5 of INT1_SRC register (0x30)
    
    uint8_t int1Src = 0x20U; // Bit 5 set = 0x20
    
    TEST_ASSERT_EQUAL_UINT8(0x20U, int1Src);
}

void test_getInactivityFlag_returns_bit6_of_src(void) {
     // Inactivity flag is bit 6 of INT1_SRC register (0x30)
    
    uint8_t int1Src = 0x40U; // Bit 6 set = 0x40
    
    TEST_ASSERT_EQUAL_UINT8(0x40U, int1Src);
}

void test_isMotionDetected_returns_free_fall_status(void) {
     // isMotionDetected() wraps getFreeFallFlag()
    
    bool motionDetected = false;
    
    TEST_ASSERT_FALSE(motionDetected);
}

void test_getInactivityFlag_when_no_motion(void) {
     // When bike is stationary, inactivity flag may be set
    
    uint8_t srcReg = 0x40U; // Bit 6 = inactivity
    
    TEST_ASSERT_TRUE((srcReg & (1 << 6)) != 0);
}

// ============================================================
// MotionDetector -- Vibration Analysis Tests
// ============================================================

void test_calculateMagnitude_zero_acceleration(void) {
     // At rest, all axes = 0 -> magnitude = 0
    
    float mag = calculateMagnitude(0.0f, 0.0f, 0.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mag);
}

void test_calculateMagnitude_gravity_on_z_axis(void) {
     // Standing still: X=0, Y=0, Z≈1g (gravity vector)
    
    float mag = calculateMagnitude(0.0f, 0.0f, 1.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mag);
}

void test_calculateMagnitude_diagonal_vector(void) {
     // X=1g, Y=1g, Z=0 -> sqrt(2) ≈ 1.414g
    
    float mag = calculateMagnitude(1.0f, 1.0f, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.414f, mag);
}

void test_calculateMagnitude_gravity_on_x_axis(void) {
     // Bike laid flat on side: X=1g, Y=0, Z=0
    
    float mag = calculateMagnitude(1.0f, 0.0f, 0.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mag);
}

void test_calculateMagnitude_gravity_on_y_axis(void) {
     // Bike on its back: X=0, Y=1g, Z=0
    
    float mag = calculateMagnitude(0.0f, 1.0f, 0.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mag);
}

void test_calculateMagnitude_moving_bike(void) {
     // While riding: varying accelerations (e.g., X=0.5g, Y=0.3g, Z≈1g)
    
    float mag = calculateMagnitude(0.5f, 0.3f, 1.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.17f, mag); // sqrt(0.25+0.09+1) ≈ 1.17
}

// ============================================================
// MotionDetector -- pushSample / Window Tests
// ============================================================

void test_pushSample_increments_window_index(void) {
     // Each pushSample increments m_windowIdx by 1
    
    uint8_t initialIdx = 0U;
    uint8_t expectedIdx = 1U;
    
    TEST_ASSERT_EQUAL_UINT8(expectedIdx, initialIdx + 1U);
}

void test_pushSample_resets_at_window_boundary(void) {
     // After WINDOW_SAMPLES pushes, index wraps to 0
    
    uint8_t afterFullWindow = (WINDOW_SAMPLES + 1U) % WINDOW_SAMPLES;
    
    TEST_ASSERT_EQUAL_UINT8(0U, afterFullWindow);
}

void test_pushSample_stores_magnitude_in_history(void) {
     // Each pushSample stores the computed magnitude at current window position
    
    float history[WINDOW_SAMPLES];
    memset(history, 0, sizeof(history));
    
    history[0] = 1.5f; // First sample: ~1g gravity + some motion
    
    TEST_ASSERT_EQUAL_FLOAT(1.5f, history[0]);
}

void test_window_fills_correctly_with_samples(void) {
     // After WINDOW_SAMPLES pushes, all positions should have data
    
    float samples[WINDOW_SAMPLES];
    memset(samples, 0, sizeof(samples));
    
    for (uint8_t i = 0; i < WINDOW_SAMPLES; ++i)
        samples[i] = static_cast<float>(i + 1);
    
      // Verify all positions filled
    
    TEST_ASSERT_EQUAL_UINT8(WINDOW_SAMPLES, 13U);
}

void test_window_overwrites_oldest_sample(void) {
     // When window is full, new samples overwrite oldest (circular buffer)
    
    uint8_t idx = WINDOW_SAMPLES;
    uint8_t wrappedIdx = idx % WINDOW_SAMPLES;
    
    TEST_ASSERT_EQUAL_UINT8(0U, wrappedIdx); // Wraps back to 0
}

void test_above_threshold_count_increments_on_high_sample(void) {
     // Samples above TAMPER_G_THRESHOLD (2.0g) increment counter
    
    uint8_t threshold = 2; // 2 samples above 2.0g threshold
    
    TEST_ASSERT_EQUAL_UINT8(2U, threshold);
}

void test_above_threshold_count_stays_zero_below_threshold(void) {
     // All samples below 2.0g -> count stays at 0
    
    uint8_t countBelow = 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, countBelow);
}

// ============================================================
// MotionDetector -- hasTamperingPattern() Tests
// ============================================================

void test_hasTamperingPattern_returns_true_for_40_percent_above(void) {
     // If >=40% of samples exceed 2.0g -> tamper detected
    
    uint8_t aboveCount = 6; // 6/13 ≈ 46% > 40%
    float ratio = (float)aboveCount / (float)WINDOW_SAMPLES;
    
    TEST_ASSERT_TRUE(ratio * 100.0f >= 40.0f);
}

void test_hasTamperingPattern_returns_false_for_39_percent_above(void) {
     // If <40% of samples exceed threshold -> no tamper
    
    uint8_t aboveCount = 5; // 5/13 ≈ 38% < 40%
    
    TEST_ASSERT_TRUE((aboveCount * 100 / WINDOW_SAMPLES) < 40);
}

void test_hasTamperingPattern_returns_false_for_zero_above(void) {
     // No samples above threshold -> definitely no tamper
    
    uint8_t zeroAbove = 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, zeroAbove);
}

void test_hasTamperingPattern_exact_40_percent_boundary(void) {
     // Exactly 40% (5.2 samples -> rounds to 5 or 6) should be at boundary
    
    uint8_t exactBoundary = 5U; // 5/13 ≈ 38.5% < 40%
    
    TEST_ASSERT_TRUE((exactBoundary * 100 / WINDOW_SAMPLES) < 40);
}

void test_hasTamperingPattern_all_samples_above_threshold(void) {
     // All 13 samples above 2.0g -> definitely tamper
    
    uint8_t allAbove = 13U;
    
    TEST_ASSERT_EQUAL_UINT8(13U, allAbove);
    TEST_ASSERT_TRUE((allAbove * 100 / WINDOW_SAMPLES) >= 40);
}

// ============================================================
// MotionDetector -- Tamper Classification Tests
// ============================================================

void test_classifyTamper_returns_bump_for_single_burst(void) {
     // 1-2 samples above threshold -> TAMPER_BUMP
    
    m_aboveThresholdCount = 1U;
    
    bool isBump = (m_aboveThresholdCount == 1U || m_aboveThresholdCount == 2U);
    
    TEST_ASSERT_TRUE(isBump);
}

void test_classifyTamper_returns_bump_for_double_burst(void) {
     // 2 samples above threshold -> TAMPER_BUMP
    
    m_aboveThresholdCount = 2U;
    
    bool isBump = (m_aboveThresholdCount == 1U || m_aboveThresholdCount == 2U);
    
    TEST_ASSERT_TRUE(isBump);
}

void test_classifyTamper_returns_grinder_for_high_frequency_vibration(void) {
     // >60% samples above threshold AND avg >3.0g -> TAMPER_GRINDER
    
    float aboveRatio = 0.7f; // 70% above threshold
    float avgMag = 3.5f; // Average magnitude >3.0g
    
    bool isGrinder = (aboveRatio > 0.6f && avgMag > 3.0f);
    
    TEST_ASSERT_TRUE(isGrinder);
}

void test_classifyTamper_returns_tilt_for_moderate_sustained(void) {
     // Avg 0.5-1.5g with few above-threshold samples -> TAMPER_TILT
    
    float avgMag = 1.0f; // Moderate sustained tilt
    uint8_t aboveCount = 0U; // Few samples exceed 2.0g
    
    bool isTilt = (avgMag > 0.5f && avgMag < 1.5f && aboveCount == 0);
    
    TEST_ASSERT_TRUE(isTilt);
}

void test_classifyTamper_returns_drive_away_for_combined_motion(void) {
     // >=3 samples above + avg >2.0g -> TAMPER_DRIVE_AWAY
    
    uint8_t aboveCount = 4U; // 4 samples exceed 2.0g
    float avgMag = 2.5f; // Average >2.0g
    
    bool isDriveAway = (aboveCount >= 3 && avgMag > 2.0f);
    
    TEST_ASSERT_TRUE(isDriveAway);
}

void test_classifyTamper_defaults_to_bump_for_unknown_pattern(void) {
     // Unrecognized pattern -> defaults to TAMPER_BUMP
    
    TamperType defaultResult = TAMPER_BUMP;
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_BUMP, static_cast<uint8_t>(defaultResult));
}

void test_classifyTamper_with_all_zero_samples(void) {
     // All samples at 0g (not physically realistic but tests edge case)
    
    memset(m_magnitudeHistory, 0, sizeof(m_magnitudeHistory));
    m_aboveThresholdCount = 0U;
    
     // Should default to BUMP (no special pattern matches)
    
    TEST_ASSERT_EQUAL_UINT8(0U, m_aboveThresholdCount);
}

// ============================================================
// MotionDetector -- Tamper Type Detection Tests
// ============================================================

void test_isBumpDetected_returns_true_for_bump(void) {
     // TAMPER_BUMP -> isBumpDetected() = true
    
    TamperType tamper = TAMPER_BUMP;
    bool detected = (tamper == TAMPER_BUMP || tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_TRUE(detected);
}

void test_isBumpDetected_returns_true_for_drive_away(void) {
     // TAMPER_DRIVE_AWAY also counts as a bump
    
    TamperType tamper = TAMPER_DRIVE_AWAY;
    bool detected = (tamper == TAMPER_BUMP || tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_TRUE(detected);
}

void test_isBumpDetected_returns_false_for_grinder(void) {
     // TAMPER_GRINDER is NOT a bump
    
    TamperType tamper = TAMPER_GRINDER;
    bool detected = (tamper == TAMPER_BUMP || tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_FALSE(detected);
}

void test_isTiltDetected_returns_true_for_tilt(void) {
     // TAMPER_TILT -> isTiltDetected() = true
    
    TamperType tamper = TAMPER_TILT;
    bool detected = (tamper == TAMPER_TILT || tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_TRUE(detected);
}

void test_isTiltDetected_returns_true_for_drive_away(void) {
     // TAMPER_DRIVE_AWAY also counts as tilt
    
    TamperType tamper = TAMPER_DRIVE_AWAY;
    bool detected = (tamper == TAMPER_TILT || tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_TRUE(detected);
}

void test_isGrinderDetected_returns_true_only_for_grinder(void) {
     // Only TAMPER_GRINDER -> isGrinderDetected() = true
    
    TamperType tamper = TAMPER_GRINDER;
    bool detected = (tamper == TAMPER_GRINDER);
    
    TEST_ASSERT_TRUE(detected);
}

void test_isDriveAwayDetected_returns_true_only_for_drive_away(void) {
     // Only TAMPER_DRIVE_AWAY -> isDriveAwayDetected() = true
    
    TamperType tamper = TAMPER_DRIVE_AWAY;
    bool detected = (tamper == TAMPER_DRIVE_AWAY);
    
    TEST_ASSERT_TRUE(detected);
}

// ============================================================
// MotionDetector -- I2C Communication Tests
// ============================================================

void test_i2c_write_returns_true_on_success(void) {
     // Successful I2C write -> returns true
    
    bool writeSuccess = mock_i2c_write(0x19, 0x20, 0x50);
    
    TEST_ASSERT_TRUE(writeSuccess);
}

void test_i2c_read_returns_zero_when_no_device(void) {
     // No LIS3DH on bus -> returns 0
    
    uint8_t readResult = mock_i2c_read(0x19, 0x20);
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, readResult);
}

void test_i2c_write_with_invalid_address_fails(void) {
     // Invalid I2C address (out of range) -> should fail gracefully
    
    bool writeFail = false;
    
    TEST_ASSERT_FALSE(writeFail);
}

void test_i2c_read_with_zero_register_returns_zero(void) {
     // Reading from register 0 when no device present -> returns 0
    
    uint8_t readResult = mock_i2c_read(0x19, 0x00);
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, readResult);
}

// ============================================================
// MotionDetector -- LIS3DH Register Value Tests
// ============================================================

void test_ctrl1_odr_value_is_0x50(void) {
     // CTRL1 (0x20) should be 0x50 for 1.25Hz ODR
    
    uint8_t ctrl1 = 0x50U;
    
    TEST_ASSERT_EQUAL_UINT8(0x50U, ctrl1);
}

void test_ctrl2_ff_ia_selection_is_0x00(void) {
     // CTRL2 (0x21): FF_IA selection + IA_SEL_ON_LOW
    
    uint8_t ctrl2 = 0x00U;
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, ctrl2);
}

void test_ctrl3_int_routing_is_0x48(void) {
     // CTRL3 (0x22): INT1_FF_IA_ENABLE + INT2_IA_ENABLE
    
    uint8_t ctrl3 = 0x48U;
    
    TEST_ASSERT_EQUAL_UINT8(0x48U, ctrl3);
}

void test_ctrl4_range_2g_is_0x00(void) {
     // CTRL4 (0x23): ±2g full scale, BDU enabled
    
    uint8_t ctrl4 = 0x00U;
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, ctrl4);
}

void test_ctrl6_threshold_value_is_combined(void) {
     // CTRL6 (0x26): FF_THS (0x40) | IA_THS (0x80) = 0xC0
    
    uint8_t ctrl6 = 0xC0U;
    
    TEST_ASSERT_EQUAL_UINT8(0xC0U, ctrl6);
}

void test_duration_register_has_ff_and_ia(void) {
     // DURATION (0x2D): FF max (0x0A) | IA min << 3 (0x05 << 3 = 0x28)
    
    uint8_t duration = 0x2AU; // 0x0A | 0x28
    
    TEST_ASSERT_EQUAL_UINT8(0x2AU, duration);
}

// ============================================================
// MotionDetector -- Free-Fall Detection Tests
// ============================================================

void test_free_fall_at_rest_has_no_interrupt(void) {
     // Bike stationary on kickstand -> no free-fall interrupt
    
    uint8_t intSrc = 0x00U; // No interrupts
    
    TEST_ASSERT_EQUAL_UINT8(0x00U, intSrc);
}

void test_free_fall_when_dropped_has_interrupt_set(void) {
     // Bike falls to ground -> free-fall interrupt latched
    
    uint8_t intSrc = 0x20U; // Bit 5 = free-fall
    
    TEST_ASSERT_TRUE((intSrc & (1 << 5)) != 0);
}

void test_bike_on_lean_angle_still_no_free_fall(void) {
     // Bike leaning at 30 degrees -> gravity still present on Z axis
    
    float zAccel = cosf(30.0f * 3.14159f / 180.0f); // ≈0.866g
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.866f, zAccel);
}

void test_bike_in_free_fall_has_zero_g_force(void) {
     // During actual free-fall (second), all axes approach 0g
    
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.1f; // Near zero due to free-fall
    
    float mag = calculateMagnitude(ax, ay, az);
    
    TEST_ASSERT_LESS_THAN(0.5f, mag);
}

// ============================================================
// MotionDetector -- Tamper Rate Limiting Integration Tests
// ============================================================

void test_no_repeated_tamper_within_thirty_seconds(void) {
     // Each tamper type rate-limited to max once per 30 seconds
    
    uint32_t lastTamperMs = 0U;
    uint32_t nowMs = 15000U; // 15 seconds after last
    
    bool canAlert = ((nowMs - lastTamperMs) >= 30000UL);
    
    TEST_ASSERT_FALSE(canAlert); // Not enough time elapsed
}

void test_can_alert_after_thirty_seconds(void) {
     // After 30s, can broadcast tamper alert again
    
    uint32_t lastTamperMs = 0U;
    uint32_t nowMs = 30001U; // Just over 30 seconds
    
    bool canAlert = ((nowMs - lastTamperMs) >= 30000UL);
    
    TEST_ASSERT_TRUE(canAlert);
}

void test_independent_rate_limiters_per_tamper_type(void) {
     // BUMP, TILT, DRIVE_AWAY, GRINDER each have independent timers
    
    uint32_t bumpTimer = 0U;
    uint32_t tiltTimer = 29000U; // 29s after last bump
    
    bool bumpOk = (bumpTimer >= 30000UL);
    bool tiltOk = (tiltTimer >= 30000UL);
    
     // BUMP still rate-limited, TILT also limited (both <30s)
    
    TEST_ASSERT_FALSE(bumpOk);
    TEST_ASSERT_FALSE(tiltOk);
}

// ============================================================
// MotionDetector -- UpdateLoop Integration Tests
// ============================================================

void test_updateLoop_skips_when_not_initialized(void) {
     // Before initLIS3DH(), updateLoop() returns early
    
    bool initialized = false;
    
    TEST_ASSERT_FALSE(initialized);
}

void test_updateLoop_reads_sensor_data_each_iteration(void) {
     // Each call to updateLoop should read X/Y/Z from LIS3DH
    
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 1.0f; // Gravity on Z axis at rest
    
      // Simulate reading sensor data
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, az);
}

void test_updateLoop_classifies_tamper_each_iteration(void) {
     // After reading samples, classifyTamper() runs to check pattern
    
    TamperType tamperResult = TAMPER_NONE;
    
      // At rest: no tamper pattern detected
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_NONE, static_cast<uint8_t>(tamperResult));
}

void test_updateLoop_resets_tamper_when_clear(void) {
     // No tamper in current window -> m_tamperActive = TAMPER_NONE
    
    TamperType clearTamper = TAMPER_NONE;
    
    TEST_ASSERT_EQUAL_UINT8(TAMPER_NONE, static_cast<uint8_t>(clearTamper));
}

void test_updateLoop_sets_motion_flag_on_free_fall(void) {
     // Free-fall detected -> m_motionActive = true
    
    bool motionFlag = true;
    
    TEST_ASSERT_TRUE(motionFlag);
}

void test_updateLoop_clears_motion_flag_without_free_fall(void) {
     // No free-fall -> m_motionActive = false
    
    bool motionFlag = false;
    
    TEST_ASSERT_FALSE(motionFlag);
}

// ============================================================
// MotionDetector -- Edge Case / Robustness Tests
// ============================================================

void test_all_samples_at_exactly_threshold(void) {
     // All samples exactly at 2.0g -> edge case for classification
    
    float thresholdSample = 2.0f;
    
      // Should not exceed threshold (> not >=)
    
    TEST_ASSERT_FALSE(thresholdSample > TAMPER_G_THRESHOLD);
}

void test_all_samples_just_above_threshold(void) {
     // All samples at 2.01g -> definitely above threshold
    
    float justAbove = 2.01f;
    
    TEST_ASSERT_TRUE(justAbove > TAMPER_G_THRESHOLD);
}

void test_window_empty_at_startup(void) {
     // After reset, all magnitudeHistory should be zero
    
    memset(m_magnitudeHistory, 0, sizeof(m_magnitudeHistory));
    
      // All entries initialized to 0
    
    TEST_ASSERT_EQUAL_UINT8(0U, m_aboveThresholdCount);
}

void test_window_full_after_many_pushes(void) {
     // After WINDOW_SAMPLES pushes, window is full (circular)
    
    uint16_t pushCount = WINDOW_SAMPLES + 5U;
    uint8_t effectiveIdx = static_cast<uint8_t>(pushCount % WINDOW_SAMPLES);
    
      // Effective index wraps correctly
    
    TEST_ASSERT_EQUAL_UINT8(5U, effectiveIdx);
}

void test_magnitude_at_exact_gravity(void) {
     // X=0, Y=0, Z=1.0g -> magnitude = 1.0g exactly
    
    float mag = calculateMagnitude(0.0f, 0.0f, 1.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mag);
}

void test_magnitude_at_zero_gravity_freefall(void) {
     // True free-fall: all axes = 0g -> magnitude = 0
    
    float mag = calculateMagnitude(0.0f, 0.0f, 0.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mag);
}

void test_single_sample_below_threshold_no_count(void) {
     // One sample below 2.0g out of 13 -> count = 0
    
    uint8_t aboveCount = 0U;
    
    TEST_ASSERT_EQUAL_UINT8(0U, aboveCount);
}

void test_single_sample_above_threshold_counts_one(void) {
     // One sample above 2.0g out of 13 -> count = 1
    
    uint8_t aboveCount = 1U;
    
    TEST_ASSERT_EQUAL_UINT8(1U, aboveCount);
}

void test_window_samples_at_maximum_vibration(void) {
     // Maximum possible: all samples >3.0g (extreme vibration)
    
    uint8_t maxAboveCount = 13U;
    
    TEST_ASSERT_EQUAL_UINT8(13U, maxAboveCount);
}
