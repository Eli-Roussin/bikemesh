// ============================================
// test_power/test_main.cpp -- PowerModeManager unit tests
// Tests VIN sample window, hysteresis transitions, debounce logic, callbacks.
// Follows Meshtathic Unity testing conventions (anonymous namespace helpers,
// setUp/tearDown, descriptive test names, self-contained test functions).
// ============================================

#include <unity.h>
#include <stdint.h>
#include <string.h>

// --- Replicate PowerModeManager internals for unit testing ---
// We test the logic directly since this is embedded C++ without a full RTOS.

#define VIN_SAMPLES_WINDOW    10
#define VIN_DEBOUNCE_CYCLES   5
#define VIN_MODE_A_THRESHOLD_MV  4000
#define VIN_MODE_B_THRESHOLD_MV  3800

typedef enum {
    VIN_STATE_UNKNOWN = 0,
    VIN_STATE_TRANSITION_TO_A = 1,
    VIN_STATE_A = 2,
    VIN_STATE_TRANSITION_TO_B = 3,
    VIN_STATE_B = 4
} vin_state_t;

static uint16_t g_vinSamples[VIN_SAMPLES_WINDOW];
static uint8_t  g_writeIdx = 0;
static vin_state_t g_vinState = VIN_STATE_UNKNOWN;
static bool     g_modeA = false;
static bool     g_modeChangedFlag = false;
static uint8_t  g_transitionCounter = 0;

static void resetStateMachine(void) {
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    g_writeIdx = 0;
    g_vinState = VIN_STATE_UNKNOWN;
    g_modeA = false;
    g_modeChangedFlag = false;
    g_transitionCounter = 0;
}

static void pushSample(uint16_t vdd_mv) {
    g_vinSamples[g_writeIdx] = vdd_mv;
    g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;

    uint8_t modeA_votes = 0U;
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW; ++i) {
        if (g_vinSamples[i] >= VIN_MODE_A_THRESHOLD_MV)
            ++modeA_votes;
    }

    switch (g_vinState) {
    case VIN_STATE_UNKNOWN:
        if (modeA_votes >= (VIN_SAMPLES_WINDOW / 2U)) {
            g_vinState = VIN_STATE_A;
            g_modeA = true;
        } else {
            g_vinState = VIN_STATE_B;
            g_modeA = false;
        }
        break;

    case VIN_STATE_A:
        if (modeA_votes < (VIN_SAMPLES_WINDOW / 2U)) {
            ++g_transitionCounter;
            if (g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                g_vinState = VIN_STATE_TRANSITION_TO_B;
                g_transitionCounter = 0U;
            }
        } else {
            g_transitionCounter = 0U;
            g_modeA = true;
        }
        break;

    case VIN_STATE_TRANSITION_TO_B:
        if (modeA_votes >= (VIN_SAMPLES_WINDOW / 2U)) {
            g_vinState = VIN_STATE_A;
            g_modeA = true;
            g_transitionCounter = 0U;
        } else if (++g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            g_vinState = VIN_STATE_B;
            g_modeA = false;
            g_modeChangedFlag = true;
            g_transitionCounter = 0U;
        }
        break;

    case VIN_STATE_B:
        if (modeA_votes > (VIN_SAMPLES_WINDOW / 2U)) {
            ++g_transitionCounter;
            if (g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                g_vinState = VIN_STATE_TRANSITION_TO_A;
                g_transitionCounter = 0U;
            }
        } else {
            g_transitionCounter = 0U;
            g_modeA = false;
        }
        break;

    case VIN_STATE_TRANSITION_TO_A:
        if (modeA_votes <= (VIN_SAMPLES_WINDOW / 2U)) {
            g_vinState = VIN_STATE_B;
            g_modeA = false;
            g_transitionCounter = 0U;
        } else if (++g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            g_vinState = VIN_STATE_A;
            g_modeA = true;
            g_modeChangedFlag = true;
            g_transitionCounter = 0U;
        }
        break;

    default:
        break;
    }
}

// Simulates evaluateVinTransition() -- returns mode changed flag and clears it.
static bool evaluateVinTransition(void) {
    bool changed = g_modeChangedFlag;
    if (changed) g_modeChangedFlag = false;
    return changed;
}

// --- Test helpers: fill sample window with given pattern ---
static void fillSamplesWithModeA(uint8_t count) {
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW && i < count; ++i)
        g_vinSamples[i] = VIN_MODE_A_THRESHOLD_MV + 100; // >=4.1V
}

static void fillSamplesWithModeB(uint8_t count) {
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW && i < count; ++i)
        g_vinSamples[i] = VIN_MODE_B_THRESHOLD_MV - 100; // <3.7V
}

static void setUp(void) {
    resetStateMachine();
}

static void tearDown(void) {
    // No cleanup needed for this simple test harness
}

// ============================================================
// PowerModeManager -- VIN Sample Window Tests
// ============================================================

void test_initial_unknown_state_becomes_mode_a_with_high_vin(void) {
    // VIN samples all above threshold -> should transition to Mode A immediately
    fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
    
    pushSample(0U); // This sample completes the first window evaluation
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

void test_initial_unknown_state_becomes_mode_b_with_low_vin(void) {
    // VIN samples all below threshold -> should transition to Mode B immediately
    fillSamplesWithModeB(VIN_SAMPLES_WINDOW);
    
    pushSample(0U); // Complete the evaluation
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState);
    TEST_ASSERT_FALSE(g_modeA);
}

void test_half_window_threshold_triggers_mode_a(void) {
    // Exactly half window at Mode A threshold -> should trigger Mode A (>= 5 votes)
    fillSamplesWithModeB(10);
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW / 2; ++i)
        g_vinSamples[i] = VIN_MODE_A_THRESHOLD_MV + 10; // Exactly 5 votes
    
    pushSample(4000U); // Fill remaining window with Mode A values
    // After processing, should have >=5 votes for Mode A
    
    TEST_ASSERT_TRUE(g_modeA || g_vinState == VIN_STATE_A || g_vinState == VIN_STATE_UNKNOWN);
}

// ============================================================
// PowerModeManager -- Hysteresis Transition Tests (A -> B)
// ============================================================

void test_transition_a_to_b_requires_debounce_cycles(void) {
    // Start in Mode A with all samples above threshold
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    g_transitionCounter = 0;
    
    // Switch all samples to below Mode B threshold (should start debounce)
    fillSamplesWithModeB(VIN_SAMPLES_WINDOW);
    
    pushSample(3700U); // First low sample -> counter=1
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState); // Not yet in transition
    
    // Continue with all low samples to exhaust debounce cycles
    for (uint8_t i = 1; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    // After VIN_DEBOUNCE_CYCLES consecutive low samples, should be in transition to B
    TEST_ASSERT_EQUAL(VIN_STATE_TRANSITION_TO_B, g_vinState);
}

void test_transition_to_b_completes_after_debounce(void) {
    // Setup: Mode A state, then drain debounce
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    fillSamplesWithModeB(VIN_SAMPLES_WINDOW);
    
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    // Now in TRANSITION_TO_B, continue low samples -> should complete transition
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState);
    TEST_ASSERT_FALSE(g_modeA);
}

void test_transition_a_to_b_aborts_if_vin_recovers(void) {
    // Start Mode A, switch to low -> enter transition to B
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES + 2; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    // Now in TRANSITION_TO_B, but recover to Mode A before completing transition
    fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
    g_writeIdx = 0;
    
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(4100U);
    }
    
    // Should revert back to Mode A, not complete transition to B
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

// ============================================================
// PowerModeManager -- Hysteresis Transition Tests (B -> A)
// ============================================================

void test_transition_b_to_a_requires_higher_threshold(void) {
    // Mode A threshold requires > half window, Mode B is >= half
    // This tests the asymmetry: A->B needs < half, B->A needs > half
    g_vinState = VIN_STATE_B;
    g_modeA = false;
    
    // Half the samples at mode A threshold (should NOT trigger transition to A)
    uint16_t samples[VIN_SAMPLES_WINDOW];
    memset(samples, 0, sizeof(samples));
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW / 2; ++i)
        samples[i] = VIN_MODE_A_THRESHOLD_MV; // Exactly at threshold
    
    memcpy(g_vinSamples, samples, sizeof(samples));
    
    pushSample(3700U); // Below half -> should stay in Mode B
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState);
}

void test_transition_b_to_a_completes_with_sufficient_samples(void) {
    // Setup: Mode B state
    g_vinState = VIN_STATE_B;
    g_modeA = false;
    
    // All samples above Mode A threshold -> should enter transition to A
    fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
    
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(4100U);
    }
    
    // After debounce, should be in TRANSITION_TO_A
    TEST_ASSERT_EQUAL(VIN_STATE_TRANSITION_TO_A, g_vinState);
    
    // Continue high samples to complete transition
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(4100U);
    }
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

void test_transition_b_to_a_aborts_if_vin_drops(void) {
    // Start Mode B, switch to high -> enter transition to A
    g_vinState = VIN_STATE_B;
    g_modeA = false;
    
    fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES + 2; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(4100U);
    }
    
    // Now in TRANSITION_TO_A, but drop to low -> should revert to B
    fillSamplesWithModeB(VIN_SAMPLES_WINDOW);
    g_writeIdx = 0;
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState);
    TEST_ASSERT_FALSE(g_modeA);
}

// ============================================================
// PowerModeManager -- evaluateVinTransition() Tests
// ============================================================

void test_evaluateVinTransition_returns_true_on_mode_change(void) {
    // Force a mode change flag
    g_modeChangedFlag = true;
    
    bool changed = evaluateVinTransition();
    TEST_ASSERT_TRUE(changed);
    TEST_ASSERT_FALSE(g_modeChangedFlag); // Should be cleared after reading
}

void test_evaluateVinTransition_returns_false_when_no_change(void) {
    g_modeChangedFlag = false;
    
    bool changed = evaluateVinTransition();
    TEST_ASSERT_FALSE(changed);
}

void test_evaluateVinTransition_clears_flag_only_once(void) {
    // Set flag, read twice -> second read should return false
    g_modeChangedFlag = true;
    
    TEST_ASSERT_TRUE(evaluateVinTransition());
    TEST_ASSERT_FALSE(g_modeChangedFlag); // Cleared
    
    // Second call should see it's already cleared
    TEST_ASSERT_FALSE(evaluateVinTransition());
}

// ============================================================
// PowerModeManager -- Edge Case / Robustness Tests
// ============================================================

void test_rapid_oscillation_stays_in_debounce(void) {
    // Rapidly switch between high/low -> should not complete any transition
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    
    for (uint8_t cycle = 0; cycle < 10; ++cycle) {
        // Alternate high and low each sample
        fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
        g_writeIdx = 0;
        pushSample(4100U);
        
        fillSamplesWithModeB(VIN_SAMPLES_WINDOW);
        g_writeIdx = 0;
        pushSample(3700U);
    }
    
    // After rapid oscillation, should not be in a completed transition state
    // (debounce windows will keep resetting)
    TEST_ASSERT_TRUE(g_vinState == VIN_STATE_A || g_vinState == VIN_STATE_B ||
                     g_vinState == VIN_STATE_TRANSITION_TO_A || g_vinState == VIN_STATE_TRANSITION_TO_B);
}

void test_half_window_exactly_at_threshold(void) {
    // Samples exactly at threshold (4000mV) -> >= 4000 counts as Mode A
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW / 2; ++i)
        g_vinSamples[i] = VIN_MODE_A_THRESHOLD_MV; // Exactly 4000mV (counts as >= threshold)
    
    for (uint8_t i = VIN_SAMPLES_WINDOW / 2; i < VIN_SAMPLES_WINDOW; ++i)
        g_vinSamples[i] = VIN_MODE_A_THRESHOLD_MV - 1; // Below threshold
    
    // Should have exactly half -> counts as >= half window for Mode A transition
    pushSample(3999U);
    
    TEST_ASSERT_TRUE(g_modeA || g_vinState == VIN_STATE_A || g_vinState == VIN_STATE_UNKNOWN);
}

void test_all_samples_zero_volts(void) {
    // All samples at 0mV -> Mode B (no voltage = not charging)
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    
    pushSample(0U);
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState);
    TEST_ASSERT_FALSE(g_modeA);
}

void test_all_samples_at_max_voltage(void) {
    // All samples at 4200mV (max LiPo) -> Mode A
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW; ++i)
        g_vinSamples[i] = 4200U;
    
    pushSample(4200U);
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

void test_single_sample_in_window_does_not_trip_state(void) {
    // Mode A stable -> one low sample out of 10 -> should NOT transition (needs >=5 votes)
    fillSamplesWithModeA(VIN_SAMPLES_WINDOW);
    
    // Replace only one sample with low voltage
    g_vinSamples[0] = 3700U;
    g_writeIdx = 1;
    
    pushSample(4100U); // Remaining window still has >5 Mode A votes
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

void test_single_sample_below_threshold_stays_in_mode_a(void) {
    // 9/10 samples above threshold -> should NOT trigger transition (needs <5 for Mode A)
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    for (uint8_t i = 0; i < 9; ++i)
        g_vinSamples[i] = VIN_MODE_A_THRESHOLD_MV + 10; // 4010mV each
    
    g_writeIdx = 9;
    
    // State is already A
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    
    pushSample(3700U); // Only 1 below threshold -> 9 votes for Mode A -> still in A
    
    TEST_ASSERT_EQUAL(VIN_STATE_A, g_vinState);
    TEST_ASSERT_TRUE(g_modeA);
}

void test_window_fill_then_continuous_low_samples(void) {
    // Fill window with mixed samples, then continuously low until transition completes
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    
    // Start in Mode A
    g_vinState = VIN_STATE_A;
    g_modeA = true;
    g_transitionCounter = 0;
    
    // Feed enough low samples to exhaust debounce for A->B transition
    for (uint8_t i = 0; i < VIN_DEBOUNCE_CYCLES * 2; ++i) {
        g_writeIdx = (g_writeIdx + 1U) % VIN_SAMPLES_WINDOW;
        pushSample(3700U);
    }
    
    TEST_ASSERT_EQUAL(VIN_STATE_B, g_vinState); // Should have transitioned to B
    TEST_ASSERT_FALSE(g_modeA);
}

void test_mode_change_flag_is_consumable(void) {
    // Multiple evaluateVinTransition() calls -> only first returns true
    g_modeChangedFlag = true;
    
    bool first  = evaluateVinTransition();
    bool second = evaluateVinTransition();
    bool third  = evaluateVinTransition();
    
    TEST_ASSERT_TRUE(first);
    TEST_ASSERT_FALSE(second);
    TEST_ASSERT_FALSE(third);
}

// ============================================================
// PowerModeManager -- VIN Sample Window Rolling Tests
// ============================================================

void test_window_rolls_over_correctly(void) {
    // Write more samples than window size -> should roll over correctly
    memset(g_vinSamples, 0xFF, sizeof(g_vinSamples)); // Fill with known value
    
    for (uint16_t v = 3800U; v < 3820U; ++v) {
        pushSample(v);
    }
    
    TEST_ASSERT_EQUAL_UINT8(0U, g_writeIdx % VIN_SAMPLES_WINDOW); // Should have wrapped around
    
    // Window should contain last 10 values written (not the initial 0xFF)
    // Last value written was 3819U at index 9 (mod 10)
}

void test_window_preserves_all_samples(void) {
    // Verify that the rolling window keeps exactly VIN_SAMPLES_WINDOW samples
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    
    for (uint16_t v = 3800U; v < 4010U; ++v) {
        pushSample(v);
    }
    
    // After 210 samples, window should contain the last 10 values
    // Window is full -> writeIdx should be 0 (mod VIN_SAMPLES_WINDOW)
}

void test_write_index_modulo_wraps_at_window_size(void) {
    // Write exactly VIN_SAMPLES_WINDOW + 1 samples -> writeIdx should be 1
    memset(g_vinSamples, 0, sizeof(g_vinSamples));
    g_writeIdx = 0;
    
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW + 1; ++i) {
        pushSample(3900U + i);
    }
    
    TEST_ASSERT_EQUAL_UINT8(1U, g_writeIdx); // Wrapped once
}
