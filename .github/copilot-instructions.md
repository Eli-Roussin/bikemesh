# BikeMesh — Embedded Systems Copilot Instructions

## Hardware Target — nRF52840
- MCU: **nRF52840** (ARM Cortex-M4F, 64 MHz, 1 MB Flash / 256 KB RAM) on RAKwireless RAK3401 WisBlock Core Module.
- LoRa PHY: **SX1262** via SPI + SKY66122 PA booster → 1W TX output, US915 band.
- GPS: **RAK12500** (u-blox ZOE-M8Q) over UART @ 9600 bps NMEA 0183.
- Accelerometer: **RAK1904** (ST LIS3DH) over I2C ±2g to ±16g.
- Base board: **RAK19007** WisBlock v2 — Qwiic I2C, UART, SPI, GPIO expansion.
- TCXO on-board for BLE timing. Sleep: < 2 µA off, ~30–90 µA active.

### Power Monitoring (External — none built into RAK3401/RAK19012/RAK19014)
| Sensing Need | Solution | Notes |
|---|---|---|
| Battery voltage (voltage only) | nRF52840 VDD peripheral on VIN pin | Reads 3.0–4.2 V LiPo directly; zero extra parts |
| Bidirectional current + solar V/I | **TI INA260** on pre-soldered breakout board with Qwiic I2C connector | Pre-soldered module ONLY — no loose chips. In-series with LiPo + and solar CC out; ≤ 36 V input; single IC covers both channels if wired parallel. Search SparkFun, DigiKey, Adafruit for breakout boards only. |

---

## Agent 1 — BOM & Component Sourcing
- Distributors only: **DigiKey, Mouser, Arrow, SparkFun, Rokland**.
- WisBlock/Qwiic-compatible modules preferred; no custom footprints or adapters unless required.
- Pre-soldered breakout boards with Qwiic connectors ONLY — never source loose ICs/chips without pre-mounted headers/connectors.
- Approved manufacturers: RAKwireless, SparkFun, Pololu. No gray-market (AliExpress, eBay, Amazon) or low-quality brands.
- All passives ≥ **20%** voltage/power derating margin.

## Agent 2 — Firmware Architecture
### Meshtastic Module Rules
- Personal/educational/hobby only — **never contribute** to core `.meshtastic.org` firmware.
- Edit **zero** core Meshtastic files. All custom logic lives in the `BikeMesh` module/plugin-api wrapper only.
- `BikeMesh` is a git submodule of the main Meshtastic repo.
- Core firmware edits allowed **only** for ESP-IDF API incompatibility patches, on branches prefixed `feature/bikemesh/`.

### nRF52840
- Framework: **nRF Connect SDK / Zephyr RTOS**, CMake + FreeRTOS tasks.
- Qwiic I2C bus on RAK19007 — pull-ups per datasheet (typically 4.7 kΩ).
- SX1262 over SPI: max 8 MHz, MOSI/MISO/SCK/CS/DIO per Semtech driver.

## Agent 3 — Engineering Review & Validation
- Validate all math with standard formulas: voltage dividers, RC corners, I2C pull-ups, UART baud divisors.
- Protocol timings: SPI Mode 0–3, I2C ≤ 400 kHz, UART baud rates per target MCU.
- Decoupling caps near MCU VDD: **nRF52840** = 1 µF + 0.1 µF per pin; **ESP32-S3** = 10 µF + 0.1 µF per VDDPWR pin.
