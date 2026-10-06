<todos title="Todos" rule="Review steps frequently throughout the conversation and DO NOT stop between steps unless they explicitly require it.">
- No current todos
</todos>

# BikeMesh — Embedded Systems Copilot Instructions

## Hardware Context
**Primary target: nRF52840** (ARM Cortex-M4F, 64 MHz, 1 MB Flash / 256 KB RAM) on the **RAKwireless RAK3401 WisBlock Core Module**.
- LoRa PHY: **SX1262** via SPI + SKY66122 PA booster for 1W TX output on US915 band.
- GPS: **RAK12500** (u-blox ZOE-M8Q) over UART (NMEA 0183, default 9600 bps).
- Accelerometer: **RAK1904** (ST LIS3DH) over I2C.
- Base board: **RAK19007** WisBlock v2 — Qwiic I2C, UART, SPI, GPIO expansion.

**Future target: ESP32-S3** — preserve this repo for porting. All ESP32-S3 rules below apply when targeting that MCU.

---

## Agent 2: Firmware Architecture (nRF52840 & ESP32-S3)

### Meshtastic Module Rules (Apply to Both MCUs)
- Personal/educational/hobby use only — never contribute to core Meshtastic firmware.
- **Never edit core `.meshtastic.org` firmware files directly.** Create a self-contained plugin/plugin-api wrapper in the `BikeMesh` module only.
- `BikeMesh` is a git submodule of the main Meshtastic repo; keep all custom logic inside it.
- Core Meshtastic firmware edits allowed **only** for ESP-IDF API incompatibility patches, and only on branches prefixed `feature/bikemesh/`.

### nRF52840 (Primary Platform)
- Framework: **nRF Connect SDK / Zephyr RTOS** with CMake + FreeRTOS tasks.
- All WisBlock modules use the 4-pin Qwiic I2C bus on RAK19007 — default pull-ups per datasheet (4.7 kΩ).
- SX1262 over SPI: max 8 MHz, configure MOSI/MISO/SCK/CS/DIO per Semtech driver.
- TCXO built into RAK3401 for BLE timing. Sleep states: < 2 µA system-off, ~30–90 µA active.

### ESP32-S3 (Preserved)
- Framework: **ESP-IDF (C/C++)** with CMake + FreeRTOS tasks. Use `ESP_ERROR_CHECK()` on all API calls.
- **Strapping pins to avoid:** GPIO 0, 3, 45, 46 (disrupt boot/JTAG).
- **Reserved for Octal SPI:** GPIO 33–37.
- **USB native:** GPIO 19 (D-), GPIO 20 (D+).

---

## Agent 1: BOM & Component Sourcing
- Source only from tier-1 distributors: DigiKey, Mouser, Arrow, SparkFun, Rokland.
- Prefer WisBlock/Qwiic-compatible plug-and-play modules; no custom footprints or adapters unless required.
- Approved manufacturers: RAKwireless, SparkFun, Pololu. Avoid known low-quality brands and gray-market sellers (AliExpress, eBay, Amazon).
- Verify all passives have ≥ 20% voltage/power derating margin.

---

## Agent 3: Engineering Review & Validation
- Always double-check calculations with standard formulas (voltage dividers, RC corners, I2C pull-ups, UART baud divisors).
- Validate protocol timings: SPI Mode 0–3, I2C ≤ 400 kHz, UART baud rates.
- Decoupling caps near MCU VDD: **nRF52840** = 1 µF + 0.1 µF per pin; **ESP32-S3** = 10 µF + 0.1 µF per VDDPWR pin.
