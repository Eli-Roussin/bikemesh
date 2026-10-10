# BikeMesh — Single Prototype BOM (Verified Pricing Only)

> **Target Build:** 1 × complete BikeMesh Meshtastic node  
> **Core MCU:** nRF52840 on RAKwireless RAK3401 WisBlock Core Module  

---

## Confirmed Components

| # | Component | Description | Price | Stock Status |
| --- | ----------- | ------------- | ------- | -------------- |
| 1 | **RAK10724** | WisMesh High Power Booster Kit, RAK3401 (u-blox ZOE-M8Q GNSS + SX1262 + SKY66122 PA), US915 variant, SKU: 116256 | $39.00 | ✅ In Stock |
| 2 | **RAK12500** | GNSS GPS Location Module u-blox ZOE-M8Q (External GPS extension) | $19.00 | ✅ In Stock |
| 3 | **RAK1904** | 3-Axis Acceleration Sensor STMicroelectronics LIS3DH, Qwiic/Two-Wire I2C, SKU: 114505 | $3.30 | ✅ In Stock |
| 4 | **RAK19007** | WisBlock Base Board 2nd Gen (Expansion Baseboard) | $9.99 | ✅ In Stock |
| 5 | **RAK15002** | SD Card Module — MicroSD via SPI (4-line interface), 3.3V I/O, insert/removal detection pin, fits WisBlock v2 expansion slot. Product ID: 6747999994054, SKU: 100031 | $4.00 | ✅ In Stock (RAKwireless Store) |

### Confirmed Subtotal: **$75.29 USD**

---

## Build Sequence Recommendations

1. **Base System:** RAK19007 → RAK3401 core module (RAK10724 handles) → USB/Serial interface
2. **GPS Extension:** Add RAK12500 module to I2C bus, connect antenna  
3. **Sensor Extension:** Add RAK1904 LIS3DH accelerometer to Qwiic I2C bus

---

## Additional Components Requiring Manual Sourcing

The following parts were NOT available from SparkFun or RAKwireless Store and require sourcing from DigiKey, Mouser, Arrow, or TI directly:

| Component | Description | Notes |
| ----------- | ------------- | ------- |
| **TI INA260** (e.g., `INA260AIPWR`, `INA260AIPWRG4`) | I2C Power Monitor IC — Bidirectional current sensing for LiPo battery, unidirectional V/I for solar panel input (≤36V). VQFN-16 package. | Requires manual sourcing from TI/DigiKey/Mouser. Not available on RAKwireless or SparkFun. |
| **LiPo Battery Connector (JST-PH 2.0)** | 2-pin connector for external LiPo pack attachment to RAK19007 power slot. | Generic electronics supplier. |
| **Solar Charge Controller** | Buck/boost charger for panel input to RAK19007 power slot (e.g., TP4056+ or similar). | Generic electronics supplier. |
| **u-blox ZOE-M8Q compatible GPS antenna** | Ceramic patch or active antenna for L1 band (1575 MHz), SMA/SMAF connector. | Generic electronics supplier. |
| **SMA Female Panel Mount Connector** | For external GPS antenna mounting on RAK12500. | Generic electronics supplier. |
