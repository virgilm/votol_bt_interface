# Bluetooth interface for Votol Controllers

Arduino code for a CAN &lt;-> BT interface for Votol controllers.

This will work on a very specific hardware (based on ESP32), as described in this [project](https://github.com/virgilm/votol_tuner_hardware)

If you find this project helpful, you can support future development by buying me a coffee! ☕

<p align="center">
  <a href="https://www.buymeacoffee.com/metahack" target="_blank">
    <img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" width="200">
  </a>
</p>

Youtube Channel: https://www.youtube.com/@metacyclehack

If you would like to purchase a ready-to-use module or unlock premium app features, please visit:

👉 [Store](https://buymeacoffee.com/metahack/extras)

## License
This work is licensed under the [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0)](https://creativecommons.org/licenses/by-nc-sa/4.0/).

You are free to copy, modify, and share this project **for non-commercial purposes only**, as long as you give credit and license your new creations under the same terms.

# Tested good setup as of May 2025

## Install Arduino IDE version 2.3.6
Other versions might work

## Libraries
Under Tools->Manage Libraries, add the following:

### arduino-CAN install this specific library due to a bug
https://github.com/avlasic/arduino-CAN/tree/patch-1
(fork from https://github.com/sandeepmistry/arduino-CAN ver 0.3.1)

### EspSoftwareSerial (v8.1.0 - latest)

from https://github.com/plerup/espsoftwareserial/

## Board Selection
Under Tools->Boards Manager, add the following board

### ESP32 by Espressif Systems version 2.0.17

from https://github.com/espressif/arduino-esp32

Version 3.x has
 - a different way to deal with watchdogs - easy fix
 - CAN library is not working anymore, made an attempt to use TWAI CAN library - fail

## Load sketch (metahack.ino)

File -> Open

## Select board/port

E.g. ESP32 Dev Module, /dev/cu.usbserial-0001 Serial Port (USB)

## Succesful connection/init

Set USB serial baud rate to 250000 baud.

A normal init sequence should look like this:

RED LED will turn ON when power is applied

Serial Console should say

```
Serial init ok!
WiFi disabled!
CAN init ok!
BT init ok!
```

BLUE LED will turn ON and start blinking when BT connection with the app is established.

## Simulator Mode

This project now includes a **SIMULATOR mode** that allows testing without physical CAN hardware! 🎯

In simulator mode, the ESP32:
- Loads controller configuration from a file stored in SPIFFS
- Responds to BT read/write commands using simulated memory
- Saves configuration changes back to the file system
- Works completely standalone without CAN bus connection

### Quick Start

1. **Enable simulator mode**: Uncomment `#define SIMULATOR` in `metahack.ino`
2. **Upload config file**: Use Arduino IDE's "ESP32 Sketch Data Upload" to upload the `data/` folder
3. **Upload sketch**: Compile and upload to ESP32
4. **Connect and test**: Your BT app will work exactly as with real hardware!

📖 **See detailed documentation:**
- [Quick Start Guide](SIMULATOR_QUICKSTART.md) - Get running in 5 minutes
- [Complete Documentation](SIMULATOR_README.md) - Full simulator mode reference

Perfect for app development, testing, and demonstrations without needing the full hardware setup!

