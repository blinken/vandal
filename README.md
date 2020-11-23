# vandal
### A lighting and fan control system for campervans (or anything else)

vandal is an ESP32-based 10-channel smart switch module for 12-24V DC systems, intended to control camper or RV lighting. It has

 * 7 switched-positive output channels, capable of each sourcing 5A at Vin (up to 27V) . All channels support fade, blink, breathe modes via an SX1509 driver.
 * 3 switched-negative channels, capable of each sinking 200mA at up to 50V.
 * 6 protected IO lines, 5V-tolerant and operating at a 3.3V logic level
 * CANBus interface (via a SN65HVD230 transceiver)
 * Protected 1Wire bus, operating at 5V, for temperature sensors (DS1820B and others)
 * LoRa transceiver (HopeRF RFM95) with SMA antenna connection
 * USB programming for the ESP32 (using a FT232R)
 * Footprints and headers to support 2x [Adafruit 0.54" LED backpacks](https://learn.adafruit.com/adafruit-led-backpack/0-54-alphanumeric) 
 * Footprints and headers to support 4x MAX7219-based 8x8 LED arrays [Amazon](https://amzn.to/3kYg7Mb) [AliExpress 1](https://www.aliexpress.com/item/32580532205.html) [AliExpress 2](https://www.aliexpress.com/item/33038259447.html)
 * Protected power supply supporting 12V and 24V lead-acid and LiFePO4 battery systems

Protected lines feature diode clamping to 5V and 0V, and high-current lines include TVS diodes on the output.

The PCB was created in KiCad. The software for the ESP32 doesn't exist yet.