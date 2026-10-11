#pragma once

#include <stdint.h>
#include <vector>
#include <functional>

// Rolling window size and debounce cycles for VIN power-mode detection.
// nRF52840 internal VDD peripheral reads VIN on-chip; no discrete components needed.
#define VIN_SAMPLES_WINDOW     10         // Rolling sample count for debouncing
#define VIN_DEBOUNCE_CYCLES    5          // Consecutive stable periods before transition

// Power mode states with hysteresis to prevent flip-flop at threshold boundaries.
typedef enum {
    VIN_STATE_UNKNOWN = 0,
    VIN_STATE_TRANSITION_TO_A = 1,      // Sufficient high readings observed -- awaiting stabilization
    VIN_STATE_A = 2,                     // CHARGING mode confirmed (VIN >= 4.0V)
    VIN_STATE_TRANSITION_TO_B = 3,      // Low readings detected -- debouncing against false transition
    VIN_STATE_B = 4                      // ANTI-THEFT mode confirmed (VIN < 3.8V)
} vin_state_t;

// Monitors VIN supply voltage via nRF52840 VDD peripheral and manages
// power-mode state transitions (charging vs anti-theft).
class PowerModeManager {
public:
    PowerModeManager() : m_vinState(VIN_STATE_UNKNOWN), m_modeA(false), 
                        m_modeChangedFlag(false), m_writeIdx(0), m_transitionCounter(0) {}

    void init(void);                           // Begin VIN monitoring loop

    bool evaluateVinTransition(void);          // True if a mode change just occurred (consumed next call)

    bool isCharging(void) const { return m_modeA; }   // Current power mode
    bool wasModeChanged(void) const { return m_modeChangedFlag; }  // Clear after consuming
    void updateLoop(void);

    typedef std::function<void(bool modeA_new)> PowerModeChangeCallback;
    void registerCallback(PowerModeChangeCallback cb);
    void unregisterCallback(PowerModeChangeCallback cb);

private:
    vin_state_t m_vinState;                    // Current power-mode state machine state
    bool        m_modeA;                       // True = Mode A (CHARGING), false = Mode B (ANTI-THEFT)
    bool        m_modeChangedFlag;             // Set on transition, cleared by first consumer

    uint8_t     m_writeIdx;                    // Next write position in VIN sample circular buffer
    uint16_t    m_vinSampleBuf[VIN_SAMPLES_WINDOW];  // Last N VDD readings in millivolts
    uint8_t     m_transitionCounter;           // Debounce cycle counter for state transitions

    std::vector<PowerModeChangeCallback> m_callbacks;   // Listeners notified on mode changes

    uint16_t readVddMv(void);                   // Read VIN voltage via nRF52840 internal ADC/PMU
    void processVinSample(uint16_t vdd_mv);     // Feed one reading through the debounced state machine
};
