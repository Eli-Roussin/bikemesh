#include "BikeMesh.h"
#include "power/PowerModeManager.h"
#include "gps_logger/GPSLogger.h"
#include "motion/MotionDetector.h"
#include "mesh_broadcast/MeshBroadcast.h"
#include "anti_theft/AntiTheft.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "configuration.h"

BikeMeshModule *bikeMeshModule;

void BikeMeshModule::setup(void) {
     // Initialize sub-modules in dependency order (PowerModeManager first)
    m_powerMgr.init();                    // VIN monitoring loop begins
    
    if (m_powerMgr.isCharging()) {
        // Mode A: SD card logging + mesh telemetry broadcast
        m_gpsLogger.init();             // Init SPI SD card, FATFS, create initial log file
        m_meshBcast.setup();            // Set up telemetry broadcast timer (30s idle / 10s on motion)
    }
    
    m_motionDet.initLIS3DH();           // Config LIS3DH for free-fall/inactivity interrupts
    m_antiTheft.setup();                // Start in STATIONARY_IDLE beacon state
    
    m_modeAActive = m_powerMgr.isCharging();  // Set initial mode flag

    LOG_INFO("BikeMesh module initialized -- Mode %s", m_modeAActive ? "A (CHARGING)" : "B (ANTI-THEFT)");
}

int32_t BikeMeshModule::runOnce(void) {
    m_motionDet.updateLoop();

    if (m_powerMgr.evaluateVinTransition()) {
        bool newModeA = m_powerMgr.isCharging();
        if (!newModeA && m_modeAActive) {
            LOG_INFO("Switched to Mode B -- ANTI-THEFT");
            m_gpsLogger.stopLogging();
        } else if (newModeA && !m_modeAActive) {
            LOG_INFO("Switched to Mode A -- CHARGING");
            m_gpsLogger.startLogging();
        }
        m_modeAActive = newModeA;
    }

    if (m_modeAActive) {
        // === Mode A: CHARGING -- High-rate GPS logging + mesh telemetry broadcast ===

           // Read latest GPS fix from u-blox UART (if available)
        bool gpsAvailable = false;
         NMEA_GPGGA_t gga{};  // Zero-initialize
        
             // Attempt to read a GPS frame from the UART buffer
            if (m_gpsLogger.tryReadGPS(&gpsAvailable, &gga)) {
                bool motion = m_motionDet.isMotionDetected();  // Check LIS3DH interrupt status
                if (gpsAvailable && gga.fixQuality > 0) {
                    m_gpsLogger.updateLoop(&gga, motion);       // Log position to SD card
                    
                       // Also update mesh broadcast with latest GPS data
                    m_meshBcast.updatePosition(&gga, motion);
                  }
             }

              // Mesh broadcast runs independently on its own interval (30s idle / 10s on motion)
           m_meshBcast.runOnce();

         } else {
            // === Mode B: ANTI-THEFT -- Beacon scheduling + tamper detection ===

               // Check for motion events
            bool motion = m_motionDet.isMotionDetected();
            if (motion) {
                m_antiTheft.onMotionDetected();  // Switch to MOTION_BURST state
             }

               // Check for tamper events:
            uint8_t tamperType = m_motionDet.getTamperType();   // 0=none, 1=jostle, 2=grinder
            if (tamperType != 0) {
                m_antiTheft.onTamperDetected((AntiTheft::TamperType)tamperType);
               }

              // Anti-theft runOnce handles beacon interval switching internally:
            m_antiTheft.runOnce();   // 30s idle / 10s motion / rate-limited tamper alerts
          }

         // Always update power mode manager (runs at VIN_SAMPLE_INTERVAL_MS internally):
    m_powerMgr.updateLoop();

       // Return next poll interval (100ms recommended for responsiveness)
    return 100;
}
