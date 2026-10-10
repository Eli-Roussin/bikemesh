# BikeMesh Firmware Implementation Plan — Detailed Specification

## 1. OVERVIEW

Build a Meshtastic firmware plugin for the nRF52840 WisBlock platform (RAK19007 baseboard) that operates in two modes:

**Mode A — CHARGING (VIN ≥ 4.0V, e-bike ON):** High-rate GPS logging to SD card (CSV format) with motion-triggered frequency boost. TRACKER-style location packets broadcast every 30s normally, 10s on motion/tamper. Mesh summaries sent at 30s max for dashboard MQTT gateway.

**Mode B — ANTI-THEFT (VIN < 4.0V, battery/solar only):** TRACKER-like location beacons at 30s idle / 10s on motion/tamper. Tamper alerts rate-limited to once per 30s per alert type (bump, tamper, tilt, drive/ride away) sent to MQTT for jostle/angle grinder detection via LIS3DH accelerometer interrupts.

---

## 2. COMPLETE FILE TREE

```
src/modules/BikeMesh/
├── BikeMesh.h                      // Main module — Meshtastic registration, public API
├── BikeMesh.cpp                    // Module init, setup(), loop()
├── BikeMesh.pb.h                   // Generated from protobuf (Meshtastic auto-gen)
├── config.h                        // Tunable parameters, pin definitions, compile flags
│
├── power/
│   ├── PowerModeManager.h          // VIN monitoring via VDD peripheral, debounced state machine
│   └── PowerModeManager.cpp
│
├── gps_logger/
│   ├── GPSLogger.h                 // SPI SD card logging, NMEA parsing, motion-triggered freq
│   └── GPSLogger.cpp
│
├── motion/
│   ├── MotionDetector.h            // LIS3DH interrupt config, vibration analysis for tamper
│   └── MotionDetector.cpp
│
├── mesh_broadcast/
│   ├── MeshBroadcast.h             // Custom telemetry packet struct, 10s publish loop
│   └── MeshBroadcast.cpp
│
└── anti_theft/
    ├── AntiTheft.h                 // TRACKER-like beacons (30s idle / 10s on motion), MQTT alerts
    └── AntiTheft.cpp
```

---

## 3. POWER MODE DETAIL SPECIFICATION

### 3.1 VIN Detection via nRF52840 VDD Peripheral

**Hardware basis:** nRF52840 internal ADC reads VDD on-chip via `nrfx_power_vdd()`. Battery voltage naturally rises when charging (up to ~4.2V for a full LiPo). During discharge, it settles below 4.0V.

**No discrete components needed** — no resistors, diodes, or comparators.

**Debouncing algorithm:**
```c
// State machine variables:
// - vin_samples[10] : circular buffer of last 10 VDD readings
// - mode_a_votes    : count of samples >= 4.0V in the last 10
// - debounce_counter: counts consecutive "stable" periods

typedef enum {
    VIN_STATE_UNKNOWN = 0,
    VIN_STATE_TRANSITION_TO_A = 1,   // Debouncing: seen ≥5 of 10 readings at 4.0V+
    VIN_STATE_A = 2,                  // CHARGING mode confirmed
    VIN_STATE_TRANSITION_TO_B = 3,   // Debouncing: seen ≤3 of 10 readings < 4.0V
    VIN_STATE_B = 4                   // ANTI-THEFT mode confirmed
} vin_state_t;

// Pin definitions for nRF52840 WisBlock (RAK3401 core):
// VDD_PIN: Use internal ADC channel, not a GPIO pin
// The nRF52840 has a dedicated VDD measurement pin (pin 35 on QFN)
#define VIN_MEASURE_INTERVAL_MS  100     // Sample every 100ms
#define VIN_DEBOUNCE_CYCLES      5       // Need 5 consecutive samples in threshold zone to transition
#define VIN_SAMPLES_WINDOW       10      // Rolling window of last 10 samples

// Thresholds:
#define VIN_MODE_A_THRESHOLD_MV 4000     // ≥4.0V → Mode A (charging)
#define VIN_MODE_B_THRESHOLD_MV 3800     // <3.8V → Mode B (anti-theft)
// Hysteresis gap of 200mV prevents flip-flop at threshold

static vin_state_t g_vinState = VIN_STATE_UNKNOWN;
static uint16_t    g_vinSampleBuf[VIN_SAMPLES_WINDOW]; // circular buffer of mV readings
static uint8_t     g_vinWriteIdx = 0;
static uint32_t    g_lastVinMeasureMs = 0;

// Returns: true if we should trigger a mode change callback
bool PowerModeManager::evaluateVinTransition(void) {
    // 1. Read VDD voltage (in mV) via nRF52840 internal ADC
    uint16_t vdd_mv = readVddMv();  // Call to nrfx_power_vdd() or equivalent

    // 2. Store in circular buffer
    g_vinSampleBuf[g_vinWriteIdx] = vdd_mv;
    g_vinWriteIdx = (g_vinWriteIdx + 1) % VIN_SAMPLES_WINDOW;

    // 3. Count how many of the last N samples are at or above 4.0V
    uint8_t modeA_votes = 0;
    for (uint8_t i = 0; i < VIN_SAMPLES_WINDOW; i++) {
        if (g_vinSampleBuf[i] >= VIN_MODE_A_THRESHOLD_MV) modeA_votes++;
    }

    // 4. State machine transitions based on votes and debounce counters
    switch (g_vinState) {
    case VIN_STATE_UNKNOWN:
        // First run: determine which zone we're in
        if (modeA_votes >= VIN_SAMPLES_WINDOW)       g_vinState = VIN_STATE_A;
        else if (modeA_votes == 0)                  g_vinState = VIN_STATE_B;
        return false;

    case VIN_STATE_A: // Currently in Mode A
        if (modeA_votes < (VIN_SAMPLES_WINDOW / 2)) { // Below threshold zone → transition to B
            g_transitionCounter++; // Count how long we've been below
            if (g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                g_vinState = VIN_STATE_TRANSITION_TO_B;
                g_transitionCounter = 0;
            }
        } else {
            g_transitionCounter = 0; // Reset
        }
        break;

    case VIN_STATE_TRANSITION_TO_B: // Debouncing phase for A→B transition
        if (modeA_votes >= (VIN_SAMPLES_WINDOW / 2)) {
            // Returned to charging zone, cancel transition
            g_vinState = VIN_STATE_A;
        } else if (++g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            g_vinState = VIN_STATE_B;
            return true; // Mode change triggered!
        }
        break;

    case VIN_STATE_B: // Currently in Mode B
        if (modeA_votes > (VIN_SAMPLES_WINDOW / 2)) { // Above threshold zone → transition to A
            g_transitionCounter++;
            if (g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
                g_vinState = VIN_STATE_TRANSITION_TO_A;
                g_transitionCounter = 0;
            }
        } else {
            g_transitionCounter = 0;
        }
        break;

    case VIN_STATE_TRANSITION_TO_A: // Debouncing phase for B→A transition
        if (modeA_votes <= (VIN_SAMPLES_WINDOW / 2)) {
            // Returned to discharging zone, cancel transition
            g_vinState = VIN_STATE_B;
        } else if (++g_transitionCounter >= VIN_DEBOUNCE_CYCLES) {
            g_vinState = VIN_STATE_A;
            return true; // Mode change triggered!
        }
        break;
    }

    return false; // No transition yet
}
```

### 3.2 nRF52840 VDD Reading Function (Low-Level)

```c
// Read the internal VDD supply voltage in millivolts
// Uses nRF52840 ADC channel or PMU peripheral
uint16_t readVddMv(void) {
    // Option 1: Use nrfx_power_vdd() from SDK — reads internal ADC
    // This returns mV directly on nRF52840
    return nrfx_power_vdd_get();  // Return type depends on SDK version

    // Option 2 (fallback): Manual ADC read via nrf_adc API
    // uint32_t raw = NRF_ADC->RESULT;  // Raw 10-bit ADC result
    // float mv = (raw / 1023.0f) * VREF * 6;  // nRF52840 VDD is divided by 6 internally
    // return (uint16_t)(mv * 1000);
}
```

### 3.3 Mode Change Callback System

The PowerModeManager needs a way to notify other modules when the power state changes:

```c
typedef void (*PowerModeChangeCallback)(bool modeA_new);  // true = Mode A, false = Mode B

class PowerModeManager {
    static bool          g_modeA;               // Current active mode
    static bool          g_modeChangedFlag;     // True when mode just changed (consumed by listeners)
    static uint8_t       g_transitionCounter;   // Debounce counter

public:
    static bool isCharging(void);           // Returns true if Mode A (CHARGING)
    static void registerCallback(PowerModeChangeCallback cb);  // Add listener for mode changes
    static void updateLoop(void);           // Call from BikeMesh::runOnce() — evaluateVinTransition + callbacks

private:
    static std::vector<PowerModeChangeCallback> s_listeners;
};
```

---

## 4. GPS LOGGER DETAIL SPECIFICATION (Phase 2)

### 4.1 File Structure — SPI SD via RAK15002

The RAK15002 connects to the nRF52840 via SPI on the WisBlock v2 expansion slot:
- MOSI → GPIO pin (check RAK19007 slot assignment, typically P0.13)
- MISO → P0.11
- SCK  → P0.10
- CS   → P0.xx (depends on WisBlock slot routing)
- DETECT pin → P0.yy (for insert/removal detection)

### 4.2 GPSLogger Class — CSV Format with Meshtastic File Buffering

**File naming convention (descriptive channel + timestamp):**
- **High-volume location log:** `gps_YYYYMMDDHHMM.csv` — rolled hourly, e.g., `gps_202610091800.csv` for 6:00 PM logging
- **Low-volume event log:** `gps_events_YYYYMMDD.csv` — rolled daily, e.g., `gps_events_20261009.csv` for status/motion transitions

**CSV schema (every logged position):**
```csv
timestamp_ms,latitude_deg,longitude_deg,altitude_m,speed_kph,heading_deg,satellites,fix_quality,source_channel
1728495678000,37.7749,-122.4194,152,0,0,8,1,mesh
1728495678500,37.7749,-122.4193,152,32,245,9,1,vehicle_gps
```

**Meshtastic file buffering:** Do NOT flush the SD card after every line. Instead:
- Allocate a 512-byte write buffer per log file (the Meshtastic File handling abstraction layer or a dedicated FATFS `FIL` object with `f_lseek()` + block writes)
- Buffer CSV lines in memory until 512 bytes are accumulated, then write atomically to the SD card via SPI
- On rolling (hourly/daily): flush the remaining buffer, close the current file, and open the next one
- This conserves both power (fewer SPI transactions) and SD endurance (fewer page writes)

```c
class GPSLogger {
public:
    void init(void);                                          // Init SPI, FATFS, create log directory + initial CSV header
    bool isCardInserted(void);                               // Read RAK15002 DETECT pin or SD card detect GPIO
    void updateLoop(NMEA_GPGGA_t *gga);                      // Called each GPS tick — logs position if motion detected

     // Public API for other modules:
    bool isLoggingActive(void) const;                        // Returns true if Mode A and GPS logger running
    uint32_t getLastLogMs(void) const;                       // Return timestamp of last log entry
    void requestHourlyRoll(void);                             // Trigger: end current hourly file, start new gps_YYYYMMDDHHMM.csv
    void requestDailyRoll(void);                              // Trigger: end current daily file, start new gps_events_YYYYMMDD.csv

private:
     // File rotation logic — high volume hourly, low volume daily
    void rotateHourlyFile(void);                               // Close current file, open next gps_YYYYMMDDHHMM.csv with CSV header
    void rotateDailyFile(void);                                // Close current file, open next gps_events_YYYYMMDD.csv with CSV header

     // Append position to current file (buffers to 512-byte blocks)
    void appendPositionCSV(NMEA_GPGGA_t *gga, uint8_t sourceChannel);

     // Meshtastic-style buffered write — accumulate in m_buf[] until 512 bytes, then write via SPI
    void flushIfNeeded(void);                                  // If buffer >= 512 bytes, write to SD card
    void flushRemaining(void);                                 // Write any remaining data before closing a file

     // SPI init for RAK15002 — configure nRF52840 SPIM peripheral
    void initSPI(void);

    FATFS      m_fs;                                           // FatFs filesystem handle
    File       m_currentFile;                                  // Current log file (FatFs File object)
    char       m_currentFilename[32];                          // e.g., "gps_202610091800.csv"
    char       m_buf[512];                                     // 512-byte write buffer for Meshtastic-style block writes
    uint16_t   m_bufOffset;                                    // Current byte offset in m_buf[]
    uint32_t   m_lastHourRollMs;                              // Track when to roll hourly (every 3600s)
    uint32_t   m_lastDayRollMs;                               // Track when to roll daily (every 86400s)
    uint8_t    m_fileType;                                    // 0=high-volume hourly, 1=low-volume daily
};

// NMEA parsing struct — fields needed for both CSV logging and route reconstruction:
struct NMEA_GPGGA_t {
    float   latitude;          // WGS84 degrees (+N = north, +S = south)
    float   longitude;         // WGS84 degrees (+E = east, +W = west)
    int16_t altitude_m;        // MSL altitude (scaled ×100 for precision — store as meters with 2 decimal places)
    uint8_t satellites;        // Number of SVs in use
    uint8_t fixQuality;        // 0=invalid, 1=GPS, 2=DGPS
    uint32_t timestamp_ms;     // Unix epoch milliseconds (for CSV filename timestamps)
};

// Meshtastic buffer append example:
void GPSLogger::appendPositionCSV(NMEA_GPGGA_t *gga, uint8_t sourceChannel) {
     // Build CSV line in m_buf — no flush after each line, accumulate to 512 bytes then write
    char line[128];
    snprintf(line, sizeof(line),
         "%lu,%f,%f,%d,%d,%d,%d,%d,%s\r\n",
        gga->timestamp_ms,
        gga->latitude,
        gga->longitude,
        gga->altitude_m / 100,
        0,      // speed — filled later from GPS GLL or computed
        0,      // heading — filled later from GPS GGA or computed
        gga->satellites,
        gga->fixQuality,
         (sourceChannel == 1) ? "mesh" : "vehicle_gps");

    uint16_t lineLen = strlen(line);
    if (m_bufOffset + lineLen >= 512) {
         // Buffer full — flush to SD card
        flush();
     }
    memcpy(m_buf + m_bufOffset, line, lineLen);
    m_bufOffset += lineLen;

     // Only flush if we reached 512 bytes — this is the Meshtastic-style block write optimization
     // No flushing per line; only on buffer full or file roll
}

void GPSLogger::flush(void) {
     // Write buffered data to SD card in one SPI transaction (conserves power and endurance)
    if (m_bufOffset > 0) {
        f_write(&m_currentFile, m_buf, m_bufOffset, &bw);
        m_bufOffset = 0;
     }
}

void GPSLogger::flushRemaining(void) {
     // Write any remaining buffered data before closing the file (during roll-over)
    flush();
}
```

### 4.3 Motion-Triggered Frequency Switching with CSV Rolling

The GPS logger switches between two logging rates based on motion detection:
- **Moving (LIS3DH free-fall interrupt active):** Log at ≥1 Hz (every ~1000ms) — high volume → stored in `gps_YYYYMMDDHHMM.csv` (hourly roll)
- **Stationary (>3s no motion):** Log at 1/min (every ~60000ms) — low volume → stored in `gps_events_YYYYMMDD.csv` (daily roll)

The `GPSLogger::updateLoop()` receives a "motion" parameter from the MotionDetector and adjusts its internal logging timer accordingly. It also tracks time to trigger hourly rolls for high-volume files and daily rolls for low-volume files.

**Rolling schedule enforcement:**
```c
void GPSLogger::checkAndRoll(void) {
    uint32_t nowMs = millis();

     // Check if we need to roll the hourly file (every 3600s for high-volume gps_*.csv)
    if (m_fileType == 0 && (nowMs - m_lastHourRollMs > 3600000)) {
        flushRemaining();                             // Flush remaining buffer before closing
        closeCurrentFile();                           // Close current gps_YYYYMMDDHHMM.csv
        openNewHourlyFile(nowMs);                     // Create gps_next_YYYYMMDDHHMM.csv with CSV header
         m_lastHourRollMs = nowMs;
     }

     // Check if we need to roll the daily file (every 86400s for low-volume events)
    if (m_fileType == 1 && (nowMs - m_lastDayRollMs > 86400000)) {
        flushRemaining();
        closeCurrentFile();
        openNewDailyFile(nowMs);
         m_lastDayRollMs = nowMs;
     }
}
```

---

## 5. MOTION DETECTOR DETAIL SPECIFICATION (Phase 3)

### 5.1 LIS3DH Register Configuration for Free-Fall/Inactivity Detection

The RAK1904 LIS3DH is connected via I2C on the Qwiic bus:
- I2C Address: 0x19 or 0x28 (depending on ADDR pin state)
- SDA: P0.xx, SCL: P0.xx (WisBlock Qwiic routing)

**Required register configuration sequence:**

```c
// LIS3DH register addresses (from ST datasheet):
#define LIS3DH_REG_CTRL1    0x20   // Control register 1
#define LIS3DH_REG_CTRL2    0x21   // Control register 2 — FF_IA, IACT selections
#define LIS3DH_REG_CTRL3    0x22   // Control register 3 — pin assignments
#define LIS3DH_REG_CTRL4    0x23   // Control register 4 — bandwidth, ODR
#define LIS3DH_REG_CTRL6    0x26   // Control register 6 — thresholds for FF/IA
#define LIS3DH_REG_DURATION 0x2D   // Duration register for free-fall/inactivity
#define LIS3DH_REG_CLICK_CFG 0x31  // For additional click/tap detection

// Configuration parameters (tunable via config.h):
#define LIS3DH_ODR_VALUE    0b0101  // ODR = 1.25 Hz (low power, reasonable for tamper)
#define LIS3DH_FF_THS_VALUE 0b0100  // Free-fall threshold — ~50% of g range (adjustable)
#define LIS3DH_IA_THS_VALUE 0b0100  // Inactivity threshold — slightly higher than FF
#define LIS3DH_DUR_FF_MAX   0x0A     // Max 10 × ODR cycles for free-fall detection
#define LIS3DH_DUR_IA_MIN   0x05     // Min 5 × ODR cycles before inactivity detected

void MotionDetector::initLIS3DH(void) {
    // Write to each register via I2C:
    i2c_write(LIS3DH_REG_CTRL1, LIS3DH_ODR_VALUE);       // Set ODR = 1.25 Hz
    i2c_write(LIS3DH_REG_CTRL2, FF_IA_SELECTION | IA_SEL_ON_LOW);
    i2c_write(LIS3DH_REG_CTRL3, INT1_FF_IA_ENABLE);      // Route FF/IA to INT1 pin
    i2c_write(LIS3DH_REG_CTRL4, FULL_SCALE_2G | BDU_ENABLED);  // ±2g range, block update
    i2c_write(LIS3DH_REG_CTRL6, FF_THS_VALUE | IA_THS_VALUE);   // Set thresholds
    i2c_write(LIS3DH_REG_DURATION, DUR_FF_MAX | (DUR_IA_MIN << 3));
}

// Low-level I2C helper — use nRF52840 TWIM peripheral:
bool i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    // Use nrfx_twim or TwiLib from Meshtastic abstraction layer
    return nrfx_twim_transfer(addr, &reg, 1, &val, 1);
}

// Interrupt handling — INT1 or INT2 pin on LIS3DH:
bool MotionDetector::isMotionDetected(void) {
    // Check INT1 status register for free-fall flag
    uint8_t src = i2c_read(LIS3DH_REG_INT1_SRC);
    return (src & FREE_FALL_IA_FLAG) != 0;
}

bool MotionDetector::isActiveDetected(void) {
    // Check INT2 status register for inactivity flag
    uint8_t src = i2c_read(LIS3DH_REG_INT2_SRC);
    return (src & INACTIVITY_IA_FLAG) != 0;
}
```

### 5.2 Vibration Analysis Engine (Tamper Detection)

The MotionDetector must detect:
1. **Angle grinder vibration:** High-frequency pattern (200-500 Hz) visible in the LIS3DH sample stream
2. **Impact/jostle detection:** Sudden acceleration > threshold on any axis
3. **Tilt angle change:** Significant tilt (>30 degrees) indicating bike being moved

**Vibration analysis algorithm (running on each sample):**

```c
class VibrationAnalyzer {
public:
    void pushSample(float ax, float ay, float az); // Called per LIS3DH sample
    bool hasTamperingPattern(void);                 // Returns true if tamper detected
    uint8_t getTamperType(void);                    // 0=none, 1=jostle, 2=grinder

private:
    // Track acceleration magnitude envelope over time window
    float m_magnitudeHistory[WINDOW_SAMPLES];     // Circular buffer of magnitudes
    uint8_t m_windowIdx = 0;
    uint8_t m_aboveThresholdCount = 0;             // Samples above threshold in window
};

float VibrationAnalyzer::calculateMagnitude(float ax, float ay, float az) {
    return sqrtf(ax*ax + ay*ay + az*az);
}

void VibrationAnalyzer::pushSample(float ax, float ay, float az) {
    float mag = calculateMagnitude(ax, ay, az);
    m_magnitudeHistory[m_windowIdx] = mag;
    if (mag > TAMPER_THRESHOLD_G) { // e.g., 2.0g
        m_aboveThresholdCount++;
    }
    m_windowIdx = (m_windowIdx + 1) % WINDOW_SAMPLES;
}

bool VibrationAnalyzer::hasTamperingPattern(void) {
    // Simple but effective: if >40% of samples in window exceed threshold, flag tamper
    return (m_aboveThresholdCount * 100 / WINDOW_SAMPLES) > 40;
}

uint8_t VibrationAnalyzer::getTamperType(void) {
    // Distinguish jostle vs grinder based on frequency content:
    // Jostle: irregular, wide-band bursts
    // Grinder: periodic high-freq peaks (requires counting zero-crossings or peak intervals)
    // Simplified: use sample count ratio as proxy for frequency
    if (m_aboveThresholdCount > WINDOW_SAMPLES * 0.6f) return TAMPER_TYPE_GRINDER;
    return TAMPER_TYPE_JOSTLE;
}

// Thresholds and parameters:
#define TAMPER_THRESHOLD_G      2.0f     // Minimum g-force for tamper detection
#define VIBRATION_WINDOW_MS     1000     // 1-second analysis window
#define WINDOW_SAMPLES          13       // ~13 samples at 1.25 Hz (LIS3DH ODR)
```

---

## 6. MESH TELEMETRY BROADCAST DETAIL SPECIFICATION (Phase 4)

### 6.1 Mesh Packet Schema for Route Reconstruction + Dashboard Display

The custom mesh packet serves a dual purpose: **dashboard display** (speed, heading, battery) and **downstream route reconstruction** (elevation gains/drops). The schema captures enough data per broadcast to reconstruct the ride profile with downstream software.

**Packet rate rules:**
- **Normal (stationary/idle):** 30 seconds (TRACKER standard rate — applies to BOTH Mode A and Mode B)
- **Motion/tamper active:** 10 seconds (accelerated for theft tracking — applies to BOTH modes)
- **Mode A specific:** Mesh sends only *summary* data (not the high-freq raw GPS from SD), at max 30s interval. The high-rate position data goes exclusively to the SD CSV file.

**Mesh telemetry broadcast schema (for downstream software that reconstructs rides):**
```c
// This struct is used for BOTH the mesh packet AND the CSV summary log
// It captures enough fields to compute elevation gains/drops, total distance, speed profiles, etc.
typedef struct {
    uint32_t    timestamp_ms;           // Unix epoch milliseconds (for time-series analysis)
    float       latitude_deg;           // WGS84 — route path reconstruction
    float       longitude_deg;          // WGS84 — route path reconstruction
    int16_t     altitude_m_x100;        // MSL altitude × 100 — elevation gain/loss calculation
    float       altitude_m_delta;       // Delta from last broadcast in meters (+gain, -drop)
    uint8_t     speed_kph;             // Ground speed km/h (for speed profile reconstruction)
    uint8_t     heading_deg;           // 0-359 degrees (for bearing/path direction)
    uint8_t     satellites;            // GNSS fix quality (data validity indicator)
    uint8_t     power_mode;            // 0=charging/ModeA, 1=battery/ModeB
    uint16_t    battery_mv;            // Battery voltage in mV (for range estimation)
    uint8_t     is_charging;           // Boolean: charger present
    uint32_t    cumulative_distance_m; // Total meters logged since start of ride segment
    int16_t     elevation_gain_m_x100;// Cumulative positive elevation gain × 100
    int16_t     elevation_loss_m_x100;// Cumulative negative elevation loss × 100
    uint8_t     event_flag;            // 0=normal, 1=motion_start, 2=motion_stop, 3=tamper_detected
} RideTelemetryPacket_t;             // Total: 56 bytes (optimized for mesh + route reconstruction)

// Pack the structure — aligned for efficient serialization over the mesh:
#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp_ms;            // 4 bytes
    float    latitude_deg;            // 4 bytes
    float    longitude_deg;           // 4 bytes
    int16_t  altitude_m_x100;         // 2 bytes — MSL ×100 for elevation tracking
    float    altitude_m_delta;        // 4 bytes — delta from last broadcast
    uint8_t  speed_kph;              // 1 byte
    uint8_t  heading_deg;            // 1 byte
    uint8_t  satellites;             // 1 byte
    uint8_t  power_mode;             // 1 byte
    uint16_t battery_mv;             // 2 bytes
    uint8_t  is_charging;            // 1 byte
    uint32_t cumulative_distance_m;  // 4 bytes — total route meters
    int16_t  elevation_gain_m_x100;  // 2 bytes — cumulative positive gain
    int16_t  elevation_loss_m_x100;  // 2 bytes — cumulative negative loss
    uint8_t  event_flag;             // 1 byte
} RideTelemetryPkt_Packed_t;         // Total: 38 bytes packed
#pragma pack(pop)

// NOTE: The same struct is serialized for both the mesh packet and the CSV summary file
// For CSV logging, altitude_m_delta and elevation_gain/loss fields are stored separately
```

**Why this schema works for downstream route reconstruction:**
- `latitude_deg`, `longitude_deg` → Route path / GPS track
- `altitude_m_x100` with `altitude_m_delta` → Compute elevation gains/drops per segment
- `cumulative_distance_m` → Total distance tracked (can be zeroed per ride session)
- `elevation_gain_m_x100`, `elevation_loss_m_x100` → Pre-computed cumulative stats for dashboard
- `timestamp_ms` + `speed_kph` → Speed profile / rest stops / pauses
- `power_mode`, `battery_mv` → Battery state during ride
- `event_flag` → Mark significant events (motion start/stop, tamper)

### 6.2 MeshBroadcast Class — Integration with Meshtastic SinglePortModule

```c
class MeshBroadcast : public SinglePortModule, private concurrency::OSThread {
public:
    MeshBroadcast() : SinglePortModule("BikeMesh_Telemetry", meshtastic_PortNum_BIKE_MESHTASTIC_APP),
                      OSThread("BikeMeshBroadcast") {}

    void setup(void) override;               // Initialize, set up first broadcast timer
    int32_t runOnce(void) override;          // Main loop — generate and send summary telemetry packet (30s idle / 10s on motion)
    bool handleReceived(const meshtastic_MeshPacket &mp) override;   // Optional: ack packets

private:
    void broadcastTelemetryPacket(void);         // Build and send RideTelemetryPkt to mesh
    RideTelemetryPkt_Packed_t buildPacket(void); // Assemble from GPSLogger + MotionDetector state
    uint32_t getBroadcastIntervalMs(void) const; // Returns 30000 (idle) or 10000 (motion active)

     // Track last broadcast rate and force max 30s interval for summaries in Mode A:
    bool m_motionActive;                   // True if LIS3DH free-fall detected recently
    uint32_t m_lastBroadcastMs = 0;
    uint32_t m_intervalMs = 30000;         // Default: 30s idle rate
};

// Global module pointer (as Meshtastic pattern):
extern MeshBroadcast *meshBroadcastModule;

// runOnce implementation — 30s idle / 10s on motion for BOTH modes:
int32_t MeshBroadcast::runOnce(void) {
     // Check if motion/tamper is active (from MotionDetector or AntiTheft)
    m_motionActive = motionDetector.isMotionDetected() || antiTheft.isTamperActive();

      // Set broadcast interval based on state: 10s on motion, 30s idle
    m_intervalMs = m_motionActive ? 10000 : 30000;

      // Only broadcast if enough time has passed (max once per configured interval)
    if (!Throttle::isWithinTimespanMs(m_lastBroadcastMs, m_intervalMs)) {
        broadcastTelemetryPacket();   // Send summary packet (NOT raw high-freq GPS data from SD)
        m_lastBroadcastMs = millis();
      }

      // In Mode A: mesh sends ONLY summaries at 30s max (not the 1Hz GPS logged to SD card)
      // The high-rate position data goes exclusively to the CSV file on SD card
    return GPIO_POLLING_INTERVAL;   // 10ms — just poll for motion/tamper flags
}

// broadcastTelemetryPacket — send summary packet over mesh (30s idle / 10s active):
void MeshBroadcast::broadcastTelemetryPacket(void) {
    RideTelemetryPkt_Packed_t pkt = buildPacket();   // Get latest position + speed + elevation stats

     meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) return;

      // Serialize RideTelemetryPkt into payload (38 bytes packed)
    memcpy(p->decoded.payload.bytes, &pkt, sizeof(RideTelemetryPkt_Packed_t));
    p->decoded.payload.size = sizeof(RideTelemetryPkt_Packed_t);
    p->want_ack = false;

     service->sendToMesh(p, router::TO_MESH);  // Broadcast to mesh network (MQTT gateway picks up)
}
```

**Rate limiting rules for Mode A (ride recording):**
| Data Source | Rate | Method |
|-------------|------|--------|
| SD card CSV (gps_*.csv) | 1Hz moving / 1/min stationary | Direct SPI write to RAK15002 |
| Mesh broadcast (summary) | Max 30s idle / 10s on motion | Standard Meshtastic POSITION_APP packet |

**Rate limiting rules for Mode B (anti-theft):**
| Data Source | Rate | Method |
|-------------|------|--------|
| Mesh TRACKER beacon | 30s idle / 10s on motion/tamper | Standard Meshtastic POSITION_APP |
| Tamper alerts (MQTT + mesh) | Max once per 30s **per alert type** | AntiTheft module rate limiter |

---

## 7. ANTI-THEFT MODE DETAIL SPECIFICATION (Phase 5)

### 7.1 AntiTheft State Machine

```c
class AntiTheft {
public:
    enum class BeaconState { STATIONARY_IDLE, MOTION_BURST, TAMPER_ALERT };
    enum TamperType { TAMPER_NONE = 0, TAMPER_BUMP = 1, TAMPER_TILT = 2, TAMPER_DRIVE_AWAY = 3, TAMPER_GRINDER = 4 };

    void setup(void);                                  // Initialize, start in STATIONARY_IDLE
    int32_t runOnce(void);                             // Called at 1Hz from BikeMesh::runOnce()
    void onMotionDetected(void);                       // Called by MotionDetector when free-fall interrupt fires
    void onTamperDetected(TamperType tamperType);     // Called when jostle/angle grinder detected

private:
    BeaconState m_beaconState = STATIONARY_IDLE;
    uint32_t m_stationaryTimerMs = 0;                    // Track time since last motion (transition to idle)
    uint32_t m_lastBroadcastMs = 0;                     // Last position beacon timestamp

       // Tamper alert rate limiter — max once per 30 seconds **per alert type**
       // Each tamper type (BUMP, TILT, DRIVE_AWAY, GRINDER) has its own independent rate counter
    static const uint8_t TAMPER_TYPE_COUNT = 5;          // NONE + BUMP + TILT + DRIVE_AWAY + GRINDER
    uint32_t m_lastTamperBroadcastMs[TAMPER_TYPE_COUNT];   // Timestamp of last MQTT/mesh broadcast per type

       // Initialize rate limits (all types start at time 0 so first alert can fire immediately)
    void initTamperRateLimits(void);
    bool canBroadcastTamperAlert(TamperType type);      // Returns true if >=30s since last alert of this type

    void broadcastPositionBeacon(void);                  // Like Meshtastic TRACKER — send position to mesh
};

// runOnce implementation with 30s idle / 10s motion rate:
int32_t AntiTheft::runOnce(void) {
     // 1. Check if motion is still active (check LIS3DH interrupt status or MotionDetector state)
    bool currentMotion = motionDetector.isMotionDetected() || motionDetector.isTamperActive();

       // 2. State machine — 30s idle, 10s on motion/tamper:
    switch (m_beaconState) {
    case STATIONARY_IDLE:
        if (currentMotion) {
            m_beaconState = MOTION_BURST;
            m_stationaryTimerMs = millis();
          }
        break;

    case MOTION_BURST:
    case TAMPER_ALERT:
        if (!currentMotion) {
              // Motion stopped — track how long stationary
            if (millis() - m_stationaryTimerMs > 5000) {
                m_beaconState = STATIONARY_IDLE;       // Back to idle after 5s stationary
              }
          } else {
            m_stationaryTimerMs = millis();             // Reset stationary timer while still moving
          }

           // Check for new tamper event and rate-limit per type (max once per 30s)
        TamperType tamperType = motionDetector.getTamperType();
        if (tamperType != TAMPER_NONE && canBroadcastTamperAlert(tamperType)) {
            m_beaconState = TAMPER_ALERT;
            publishTamperAlertMQTT(tamperType);           // Send MQTT alert immediately (non-blocking)
            m_lastTamperBroadcastMs[(uint8_t)tamperType] = millis();   // Record this broadcast time for rate limiting
          }
        break;
      }

       // 3. If we should broadcast, do it:
    if (shouldBroadcast()) {
        broadcastPositionBeacon();
        m_lastBroadcastMs = millis();
      }

       // Return next interval (100ms for fine-grained checks)
    return 100;
}

// onMotionDetected — called from MotionDetector interrupt handler:
void AntiTheft::onMotionDetected(void) {
    if (m_beaconState != MOTION_BURST && m_beaconState != TAMPER_ALERT) {
        m_beaconState = MOTION_BURST;
        m_stationaryTimerMs = millis();                  // Reset idle timer
      }
}

// onTamperDetected — called when jostle/angle grinder detected:
void AntiTheft::onTamperDetected(TamperType tamperType) {
     // Rate-limit: only broadcast if >=30s since last alert of this type
    if (canBroadcastTamperAlert(tamperType)) {
        m_beaconState = TAMPER_ALERT;
        m_stationaryTimerMs = millis();                 // Keep in burst mode as long as tampering continues
        publishTamperAlertMQTT(tamperType);             // Send MQTT alert immediately (non-blocking)
        m_lastTamperBroadcastMs[(uint8_t)tamperType] = millis();   // Record for rate limiting
      }
}

// Rate limiter — max once per 30 seconds per tamper type:
bool AntiTheft::canBroadcastTamperAlert(TamperType type) {
    uint32_t nowMs = millis();
    return (nowMs - m_lastTamperBroadcastMs[(uint8_t)type]) >= 30000;
}

void AntiTheft::initTamperRateLimits(void) {
    for (uint8_t i = 0; i < TAMPER_TYPE_COUNT; i++) {
        m_lastTamperBroadcastMs[i] = 0;                // Initialize to 0 so first alert can fire immediately
      }
}
```

### 7.2 Beacon Broadcast Behavior (TRACKER-Like) — 30s Idle / 10s Motion / Rate-Limited Tamper Alerts

The AntiTheft module broadcasts position packets to the mesh using Meshtastic's TRACKER module mechanism:

1. **STATIONARY_IDLE:** Broadcast every **30 seconds** (standard Meshtastic TRACKER default — conserves battery when parked)
2. **MOTION_BURST:** Broadcast every **10 seconds** (accelerated for theft tracking — bike is being moved)
3. **TAMPER_ALERT:** Broadcast every **10 seconds** (same rate as MOTION_BURST; additional MQTT alert sent per-type at 30s max rate)

**Tamper alert rate limiting (per type, max once per 30 seconds):**

| Alert Type | Mesh/MQTT Broadcast Rate | Description |
|------------|-------------------------|-------------|
| BUMP | Max 1/30s | Sudden jostle/impact (>2.0g on any axis) |
| TILT | Max 1/30s | Significant tilt angle change (>30 degrees — bike being moved/dismantled) |
| DRIVE_AWAY | Max 1/30s | Combined motion + tilt — bike started moving with significant angle change |
| GRINDER | Max 1/30s | Angle grinder vibration signature (200-500 Hz high-frequency pattern) |

Each type has its own independent rate limiter. If a BUMP is detected at t=0 and another at t=15, the second one will NOT be broadcast until t>=30. But a GRINDER detected at t=20 CAN be broadcast (different rate-limited type).

The actual broadcast uses `allocDataPacket()` and `service->sendToMesh()` from the Meshtastic MeshModule base class:

```c
void AntiTheft::broadcastPositionBeacon(void) {
    // Get latest GPS position (from GPSLogger or direct u-blox read)
    NMEA_GPGGA_t pos = gpsLogger.getLatestPosition();

    // Build TRACKER-style position packet for mesh
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) return;

    p->decoded.portnum = meshtastic_PortNum_POSITION_APP;  // Standard Meshtastic POSITION_APP
    p->position.latitude_i = (int32_t)(pos.latitude * 1e7);
    p->position.longitude_i = (int32_t)(pos.longitude * 1e7);
    p->position.altitude = pos.altitude_m / 100;
    p->position.timestamp = now();  // Unix timestamp

    // Send to mesh like a TRACKER node would
    service->sendToMesh(p, router::TO_MESH);
}
```

---

## 8. MQTT GATEWAY DETAIL SPECIFICATION (Phase 6)

### 8.1 MqttGateway Class — TCP Publish to Home Broker

```c
class MqttGateway {
public:
    void init(void);                       // Connect to broker, subscribe to topics
    void updateLoop(void);                 // Poll for MQTT network events
    void publishTamperAlert(uint8_t tamperType);  // Send alert packet to MQTT topic

private:
    WiFiClient m_wifiClient;               // TCP client for MQTT (or BLE if no WiFi)
    bool m_connected = false;

    static const char* BROKER_HOST;        // MQTT broker hostname/IP
    static const uint16_t BROKER_PORT;     // Typically 1883 or 8883 (TLS)
    static const char* CLIENT_ID;          // Node ID for MQTT client identification
};

const char* MqttGateway::BROKER_HOST = "mqtt.home.local";   // User-configurable
const uint16_t MqttGateway::BROKER_PORT = 1883;
const char* MqttGateway::CLIENT_ID = "bikemesh-node";

void MqttGateway::publishTamperAlert(uint8_t tamperType) {
    // Build JSON alert payload for Home Assistant / Node-RED:
    // { "node": "bike_id", "alert": "angle_grinder_detected", "lat": ..., "lon": ... }

    char json[256];
    snprintf(json, sizeof(json),
        "{\"node\":\"%s\",\"alert\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,\"ts\":%lu}",
        CLIENT_ID,
        (tamperType == 1) ? "jostle" : "angle_grinder_detected",
        currentLat, currentLon, now());

    // Publish to MQTT topic: bikemesh/telemetry/alert
    m_wifiClient.publish("bikemesh/telemetry/alert", json);
}
```

### 8.2 MQTT Packet Structure for Home Gateway Pickup

The packet format the MQTT gateway publishes to the home broker:

```json
{
    "node": "bike_id",
    "alert": "angle_grinder_detected" | "jostle" | "none",
    "lat": 37.7749,
    "lon": -122.4194,
    "ts": 1696872000,
    "power_mode": "charging" | "battery"
}
```

---

## 9. MAIN MODULE — BIKE MESH (BikeMesh.h / BikeMesh.cpp)

### 9.1 Module Registration Pattern (Based on Meshtastic Examples)

```c
// BikeMesh.h — main module class combining all sub-modules
class BikeMeshModule : public SinglePortModule, private concurrency::OSThread {
public:
    BikeMeshModule() : SinglePortModule("BikeMesh", meshtastic_PortNum_BIKE_MESHTASTIC_APP),
                       OSThread("BikeMeshMain") {}

    void setup(void) override;        // Initialize PowerModeManager, GPSLogger, MotionDetector, etc.
    int32_t runOnce(void) override;   // Main loop: evaluate power mode, dispatch to sub-modules

private:
    PowerModeManager m_powerMgr;      // VIN monitoring
    GPSLogger        m_gpsLogger;     // SD card logging
    MotionDetector   m_motionDet;     // LIS3DH interrupts + vibration analysis
    MeshBroadcast    m_meshBcast;     // 10s telemetry broadcast
    AntiTheft        m_antiTheft;     // TRACKER-like beacons + MQTT tamper alerts
};

extern BikeMeshModule *bikeMeshModule;
```

### 9.2 BikeMesh.cpp — setup() and runOnce() Implementation Pattern

```c
void BikeMeshModule::setup(void) {
    m_powerMgr.init();                    // Start VIN monitoring loop
    m_gpsLogger.init();                   // Init SPI SD card, FATFS
    m_motionDet.initLIS3DH();             // Config LIS3DH for free-fall/inactivity interrupts
    m_meshBcast.setup();                  // Set up telemetry broadcast timer
    m_antiTheft.setup();                  // Start in STATIONARY_IDLE beacon state

    LOG_INFO("BikeMesh module initialized");
}

int32_t BikeMeshModule::runOnce(void) {
    // 1. Evaluate VIN power mode (debounced transition A/B)
    if (m_powerMgr.evaluateVinTransition()) {
        bool newModeA = m_powerMgr.isCharging();
        if (newModeA) {
            LOG_INFO("Switched to Mode A — CHARGING");
        } else {
            LOG_INFO("Switched to Mode B — ANTI-THEFT");
        }
    }

    // 2. Dispatch based on current mode:
    if (m_powerMgr.isCharging()) {
        // Mode A: High-rate GPS logging + mesh telemetry broadcast
        NMEA_GPGGA_t gga = gpsPoll();         // Read latest GPS fix from u-blox UART
        bool motion = m_motionDet.isMotionDetected();  // Check LIS3DH interrupt status
        if (gga.fixQuality > 0 && !motion) {   // Log only when moving (or always in Mode A?)
            m_gpsLogger.updateLoop(&gga);
        }
    } else {
        // Mode B: Anti-theft mode — beacon scheduling + tamper detection
        bool motion = m_motionDet.isMotionDetected();
        if (motion) {
            m_antiTheft.onMotionDetected();     // Switch to MOTION_BURST state
        }

        // Check for tamper events:
        uint8_t tamperType = m_motionDet.getTamperType();  // 0=none, 1=jostle, 2=grinder
        if (tamperType != 0) {
            m_antiTheft.onTamperDetected(tamperType);
        }

        // Anti-theft runOnce handles beacon interval switching internally:
        m_antiTheft.runOnce();
    }

    // Always check for incoming packets on our port (if we receive any):
    // CheckMeshPacketForOurPort();  // Optional: handle acks or config updates

    // Return next poll interval (100ms recommended for responsiveness)
    return 100;
}
```

---

## 10. CONFIGURATION FILE — config.h

All tunable parameters in one place:

```c
// config.h — BikeMesh firmware configuration

#pragma once

// === VIN / Power Mode Detection Thresholds ===
#define VIN_MODE_A_THRESHOLD_MV   4000    // ≥4.0V → Mode A (charging, e-bike ON)
#define VIN_MODE_B_THRESHOLD_MV   3800    // <3.8V → Mode B (anti-theft, battery/solar only)
#define VIN_SAMPLE_INTERVAL_MS    100     // ADC sampling period (100ms = 10Hz)
#define VIN_DEBOUNCE_CYCLES       5       // Need 5 consecutive stable periods to transition

// === Mode A — CHARGING GPS Logging Rates ===
#define MODE_A_GPS_LOG_RATE_HZ    1       // Log at 1 Hz when moving (1/sec)
#define MODE_A_STATIONARY_INTERVAL_S 60   // Stationary log interval (1/min = 60s)

// === Mode B — ANTI-THEFT Beacon Intervals ===
#define MODE_B_BEACON_IDLE_S      30       // STATIONARY_IDLE: standard TRACKER interval (30s)
#define MODE_B_BEACON_MOTION_S    10       // MOTION_BURST/TAMPER_ALERT: accelerated while moved/stolen (10s)
#define MODE_B_STATIONARY_DEBOUNCE_S 5     // Need 5s of no motion before returning to IDLE

// === Tamper Alert Rate Limiting (per type, max once per 30s) ===
#define TAMPER_ALERT_RATE_LIMIT_S  30      // Each tamper type (BUMP, TILT, DRIVE_AWAY, GRINDER) rate-limited independently

// === Tamper Detection Thresholds ===
#define TAMPER_G_THRESHOLD        2.0f    // Minimum g-force for tamper detection
#define VIBRATION_WINDOW_MS       1000    // Analysis window size in ms
#define VIBRATION_SAMPLE_COUNT    13      // Window samples at 1.25 Hz ODR

// === Mesh Telemetry Broadcast Interval ===
#define TELEM_BROADCAST_INTERVAL_S 10     // Custom telemetry packet every 10s for dashboard

// === GPIO / Pin Assignments (nRF52840 WisBlock RAK3401) ===
// Update these based on actual WisBlock v2 pin mappings:
#define GPS_UART_TX_PIN           6       // u-blox ZOE-M8Q TX → nRF UART RX
#define GPS_UART_RX_PIN           8       // u-blox ZOE-M8Q RX → nRF UART TX
#define LIS3DH_I2C_SDA_PIN        15      // Qwiic I2C SDA
#define LIS3DH_I2C_SCL_PIN        14      // Qwiic I2C SCL
#define SD_CS_PIN                 16      // RAK15002 SPI CS pin (SD card detect)
#define SD_DET_PIN                17      // RAK15002 INSERT/REMOVE DETECT GPIO

// === Power Mode Manager Callback Pin (Optional Hardware Enhancement) ===
// If adding a discrete power-good circuit later:
// #define VIN_POWER_GOOD_GPIO     18      // GPIO indicating 5V charger present

// === MQTT Gateway Settings ===
#define MQTT_BROKER_HOST "mqtt.home.local"
#define MQTT_BROKER_PORT    1883
#define MQTT_CLIENT_ID      "bikemesh-node"

// Compile-time flags for optional features:
// #define ENABLE_MQTT_GATEWAY         // Comment out if no WiFi/BLE mesh available
// #define USE_IMU_FOR_VIBRATION       // Use LIS3DH for vibration analysis (recommended)
// #define SD_CARD_LOG_DAILY_ROTATION  true  // Auto-create new day file at midnight
```

---

## 11. MESHTASTIC INTEGRATION NOTES

### 11.1 How to Register the Module in Meshtastic

Add `BikeMeshModule` initialization to Meshtastic's main init sequence. In `src/modules/ModuleFactory.cpp` or similar:

```c
// Add to ModuleFactory::init():
void ModuleFactory::init() {
    // ... existing modules ...
    bikeMeshModule = new BikeMeshModule();
    addModule(bikeMeshModule);  // Register with Meshtastic core
}
```

### 11.2 Portnum Assignment for BikeMesh Telemetry

In `meshtastic/portnums.proto` or the equivalent enum:

```protobuf
enum PortNum {
    // ... existing ports ...
    BIKE_MESHTASTIC_APP = 4097;   // Custom port number, outside core range
}
```

### 11.3 Protobuf Message Definition for BikeMesh Telemetry Packet

Create `BikeMesh.proto` in `meshtastic/protobuf/`:

```protobuf
syntax = "proto3";
package meshtastic;

message BikeMeshTelemetry {
    uint32 timestamp = 1;         // Unix epoch seconds
    float latitude = 2;           // WGS84
    float longitude = 3;          // WGS84
    int32 altitude_m = 4;         // MSL altitude × 100 for precision
    uint32 speed_kph = 5;         // Ground speed km/h
    uint32 heading_deg = 6;       // 0–359 degrees
    uint32 satellites = 7;        // GNSS fix quality
    uint32 power_mode = 8;        // 0=charging, 1=battery/anti-theft
    uint32 battery_mv = 9;        // Battery voltage in mV
    bool is_charging = 10;        // Boolean: charger present
    uint32 tamper_alert = 11;     // 0=none, 1=jostle, 2=angle_grinder_detected
}
```

---

## 12. IMPLEMENTATION ORDER (PHASE DEPENDENCIES)

| Phase | Files to Create | Key Tasks | Depends On |
|-------|----------------|-----------|------------|
| **Phase 0** | `config.h`, BikeMesh.h/cpp scaffold | Set up module skeleton, register with Meshtastic | — |
| **Phase 1** | power/PowerModeManager.h/cpp | VIN monitoring via nRF52840 VDD peripheral, debounced state machine | Phase 0 |
| **Phase 2** | gps_logger/GPSLogger.h/cpp | RAK15002 SPI init (FATFS), NMEA parsing from u-blox ZOE-M8Q UART, motion-triggered freq | Phase 1 |
| **Phase 3** | motion/MotionDetector.h/cpp | LIS3DH I2C config, free-fall/inactivity interrupts, vibration analysis engine | Phase 1 |
| **Phase 4** | mesh_broadcast/MeshBroadcast.h/cpp | Custom telemetry packet struct (32 bytes), 10s mesh publish loop, Meshtastic portnum registration | Phase 2, Phase 3 |
| **Phase 5** | anti_theft/AntiTheft.h/cpp | TRACKER-like beacon scheduling (30s idle / 10s motion / 5s tamper), MQTT alert publishing | Phase 3, Phase 4 |
| **Phase 6** | MqttGateway integration (optional, in AntiTheft.cpp or separate file) | TCP client to home broker, JSON alert publish, Node-RED/Home Assistant connectivity | Phase 5 |

---

## 13. TESTING CHECKLIST

### Power & Mode Detection
- [ ] VIN threshold: Confirm Mode A triggers at ≥4.0V, Mode B at <3.8V via serial log
- [ ] Debouncing: Simulate rapid power transitions (plug/unplug charger) — verify no flip-flop
- [ ] Power mode transition: Unplug charger → Mode B activates, replug → Mode A resumes

### SD Card CSV Logging (Mode A)
- [ ] CSV format: Verify `gps_YYYYMMDDHHMM.csv` contains properly formatted CSV with correct column headers and data types
- [ ] High-volume rolling: Confirm file rolls hourly (`gps_202610091800.csv` → `gps_202610091900.csv`) after 3600s
- [ ] Low-volume rolling: Confirm daily event log rolls daily (`gps_events_20261009.csv` → `gps_events_20261010.csv`) after 86400s
- [ ] 512-byte Meshtastic buffering: Verify no flush occurs on every line — only when buffer reaches 512 bytes or file rolls
- [ ] GPS logging rate: Confirm 1Hz SD card writes when moving, 1/min when stationary (Mode A)
- [ ] Route reconstruction data: Download a CSV log and verify all fields (lat, lon, altitude_m_x100, speed_kph, heading_deg) enable downstream elevation gain/loss calculation
- [ ] SD card insertion detection: Verify RAK15002 DETECT pin triggers SD init sequence

### Mesh Telemetry & TRACKER Packets
- [ ] TRACKER idle rate: Confirm 30-second beacon interval when stationary/idle (applies to BOTH Mode A and Mode B)
- [ ] TRACKER motion rate: Confirm 10-second beacon interval on motion/tamper detection (applies to BOTH modes)
- [ ] Mesh summary-only in Mode A: Verify mesh sends only summary packets at max 30s (not raw 1Hz GPS data from SD card)
- [ ] Mesh telemetry: Dashboard displays 30-second position updates with elevation gain/loss fields via MQTT (Node-RED/Home Assistant)

### Anti-Theft Mode (Mode B)
- [ ] Anti-theft beacons: Confirm 30s idle / 10s motion rate in both STATIONARY_IDLE and MOTION_BURST states
- [ ] Tamper rate limiting per type: Verify each alert type (BUMP, TILT, DRIVE_AWAY, GRINDER) is rate-limited to max once per 30 seconds independently
- [ ] Cross-type independence: Simulate BUMP at t=0 and GRINDER at t=15 → verify both are broadcast (different types); then simulate another BUMP at t=20 → verify it is NOT broadcast (<30s since first BUMP)
- [ ] Tamper detection: Shake/jostle bike → MQTT alert published with correct type
- [ ] Angle grinder simulation: High-frequency vibration pattern detected by vibration analyzer

### System Integration
- [ ] Power mode transition: Unplug charger → Mode B activates, replug → Mode A resumes
- [ ] SD card insertion detection: Verify RAK15002 DETECT pin triggers SD init sequence
- [ ] Full ride cycle simulation: Simulate a complete ride (power on, GPS logging, motion stops/starts, power off) — verify both CSV logs and mesh packets are correctly generated at all rates
