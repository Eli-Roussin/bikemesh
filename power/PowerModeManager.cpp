#include "PowerModeManager.h"
#include "../config.h"
#include "MeshService.h"
#include <string.h>

void PowerModeManager::init(void) {
    m_callbacks.clear();

    const uint16_t vdd = readVddMv();
    memset(m_vinSampleBuf, 0, sizeof(m_vinSampleBuf));
    m_vinSampleBuf[0] = vdd;
    m_writeIdx = 1U;
    processVinSample(vdd);

    if (m_vinState == VIN_STATE_A || m_vinState == VIN_STATE_TRANSITION_TO_A) {
        m_modeA = true;
    } else if (m_vinState == VIN_STATE_B || m_vinState == VIN_STATE_TRANSITION_TO_B) {
        m_modeA = false;
    } else {
        m_modeA = (vdd >= VIN_MODE_A_THRESHOLD_MV);
    }
}

bool PowerModeManager::evaluateVinTransition(void) {
      // Return true if mode just changed, then clear the flag
     bool changed = m_modeChangedFlag;
    if (changed) m_modeChangedFlag = false;
    return changed;
}

void PowerModeManager::registerCallback(PowerModeChangeCallback cb) {
    m_callbacks.push_back(cb);
}

void PowerModeManager::unregisterCallback(PowerModeChangeCallback cb) {
    for (auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it) {
        if (*it == cb) { m_callbacks.erase(it); break; }
    }
}

uint16_t PowerModeManager::readVddMv(void) {
      // Use nRF52840 internal ADC via nrfx_power_vdd() or equivalent
       // Returns millivolts directly on nRF52840 QFN packages
     // Return VIN measurement -- replace with actual SDK call:
     // return nrfx_power_vdd_get();   // Or equivalent from Meshtastic abstraction
    
    // Default to the charging threshold so the module stays in the safe state until the
    // board-specific VDD API is connected during hardware bring-up.
    uint32_t raw = 0U;
    if (raw > 0) {
        float mv = (raw / 1023.0f) * 3.3f * 6.0f;    // nRF52840 VDD divided by 6 internally, VREF = 3.3V
           return (uint16_t)(mv);
        }
       return 4000;   // Default to Mode A (CHARGING) for initial testing/firmware simulation
    }

void PowerModeManager::processVinSample(uint16_t vdd_mv) {
    m_vinSampleBuf[m_writeIdx] = vdd_mv;
    m_writeIdx = (m_writeIdx + 1U) % VIN_SAMPLES_WINDOW;

    uint8_t modeA_votes = 0U;
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW; ++i) {
        if (m_vinSampleBuf[i] >= VIN_MODE_A_THRESHOLD_MV) {
            ++modeA_votes;
        }
    }

    switch (m_vinState) {
    case VIN_STATE_UNKNOWN:
        if (modeA_votes >= (VIN_SAMPLES_WINDOW / 2U)) {
            m_vinState = VIN_STATE_A;
            m_modeA = true;
        } else {
            m_vinState = VIN_STATE_B;
            m_modeA = false;
        }
        break;

    case VIN_STATE_A:
        if (modeA_votes < (VIN_SAMPLES_WINDOW / 2U)) {
            ++m_transitionCounter;
            if (m_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                m_vinState = VIN_STATE_TRANSITION_TO_B;
                m_transitionCounter = 0U;
            }
        } else {
            m_transitionCounter = 0U;
            m_modeA = true;
        }
        break;

    case VIN_STATE_TRANSITION_TO_B:
        if (modeA_votes >= (VIN_SAMPLES_WINDOW / 2U)) {
            m_vinState = VIN_STATE_A;
            m_modeA = true;
            m_transitionCounter = 0U;
        } else if (++m_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            m_vinState = VIN_STATE_B;
            m_modeA = false;
            m_modeChangedFlag = true;
            for (auto &cb : m_callbacks) {
                cb(false);
            }
            m_transitionCounter = 0U;
        }
        break;

    case VIN_STATE_B:
        if (modeA_votes > (VIN_SAMPLES_WINDOW / 2U)) {
            ++m_transitionCounter;
            if (m_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                m_vinState = VIN_STATE_TRANSITION_TO_A;
                m_transitionCounter = 0U;
            }
        } else {
            m_transitionCounter = 0U;
            m_modeA = false;
        }
        break;

    case VIN_STATE_TRANSITION_TO_A:
        if (modeA_votes <= (VIN_SAMPLES_WINDOW / 2U)) {
            m_vinState = VIN_STATE_B;
            m_modeA = false;
            m_transitionCounter = 0U;
        } else if (++m_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            m_vinState = VIN_STATE_A;
            m_modeA = true;
            m_modeChangedFlag = true;
            for (auto &cb : m_callbacks) {
                cb(true);
            }
            m_transitionCounter = 0U;
        }
        break;

    default:
        break;
    }
}

void PowerModeManager::updateLoop(void) {
      uint16_t vdd = readVddMv();    // Read VDD at VIN_SAMPLE_INTERVAL_MS (100ms)
    processVinSample(vdd);
}
