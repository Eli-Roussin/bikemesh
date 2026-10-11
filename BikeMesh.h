#pragma once

// ============================================
// BikeMesh.h -- Main module class for nRF52840 WisBlock platform
// Combines all sub-modules into a single Meshtastic plugin.
// ============================================

#include "SinglePortModule.h"
#include <vector>

// Forward declarations
namespace power {    class PowerModeManager; }
namespace gps_log {  class GPSLogger;     }
namespace motion {   class MotionDetector;}
namespace mesh_brst{ class MeshBroadcast; }
namespace anti_thft{ class AntiTheft;   }

class BikeMeshModule : public SinglePortModule, private concurrency::OSThread
{
public:
    BikeMeshModule()
        : SinglePortModule("BikeMesh", meshtastic_PortNum_BIKE_MESHTASTIC_APP),
          OSThread("BikeMeshMain")
    {}

    void setup(void) override;     // Initialize all sub-modules in order
    int32_t runOnce(void) override; // Main loop: evaluate power mode, dispatch to sub-modules

private:
    // Sub-modules (owned)
    power::PowerModeManager  m_powerMgr;
    gps_log::GPSLogger       m_gpsLogger;
    motion::MotionDetector   m_motionDet;
    mesh_brst::MeshBroadcast m_meshBcast;
    anti_thft::AntiTheft     m_antiTheft;

    // Internal state
    bool m_modeAActive = false; // Current active mode: true = Mode A (CHARGING), false = Mode B (ANTI-THEFT)
};

extern BikeMeshModule *bikeMeshModule;
