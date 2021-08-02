# LoRaBLE

A low power Long-Range [LoRa](https://en.wikipedia.org/wiki/LoRa)
Bluetooth ([BLE](https://en.wikipedia.org/wiki/Bluetooth_Low_Energy)) relay for the ESP32 LoRa v2.

# Hardware

The ESP32 LoRa 1-CH Gateway combines an ESP32 -- a **programmable** microcontroller featuring both WiFi and Bluetooth
radios -- with an RFM95W LoRa transceiver to create a single-channel LoRa gateway. It's a perfect, low-cost tool for
monitoring a dozen-or-so LoRa devices, and relaying their messages up to the cloud.

[ESP32 LoRa 32 (V2), ESP32 Development Board](https://www.amazon.com/MakerFocus-Development-Bluetooth-0-96inch-Display/dp/B076MSLFC9)

![ESP32 LoRa V2](https://m.media-amazon.com/images/I/710x6RMB+YL._AC_SX679_.jpg)

# Credits

Derived from example code provided here:

- https://github.com/HelTecAutomation/Heltec_ESP32/blob/master/examples/Factory_Test/WiFi_LoRa_32FactoryTest/WiFi_LoRa_32FactoryTest.ino
- https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/BLE_uart/BLE_uart.ino
