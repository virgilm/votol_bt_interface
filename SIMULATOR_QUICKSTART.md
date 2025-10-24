# Simulator Mode - Quick Start Guide

## What is Simulator Mode?

Simulator mode lets your ESP32 pretend to be a Votol controller without needing a real CAN connection. Perfect for:
- Testing your BT app without hardware
- Developing new features safely
- Debugging protocol issues
- Demonstrating functionality

## 5-Minute Setup

### Step 1: Enable Simulator Mode

Edit `metahack/metahack.ino` line 41:

```cpp
// Change this:
// #define SIMULATOR

// To this:
#define SIMULATOR
```

### Step 2: Upload Config File to ESP32

1. Install arduino-littlefs-upload tool from https://github.com/earlephilhower/arduino-littlefs-upload/releases/tag/1.5.4
2. The `metahack/data/config.ini` file is already prepared for you
3. In Arduino IDE: run LittleFS Filesystem Uploader
4. Wait for upload to complete

```

### Step 3: Upload Sketch

1. Compile and upload `metahack/metahack.ino` to your ESP32
2. Open Serial Monitor at 250000 baud
3. You should see:
   ```
   SPIFFS initialized!
   Loaded 119 bytes from config file
   Config loaded from file!
   SIMULATOR MODE - CAN disabled
   BT init ok!
   ```

### Step 4: Connect and Test

1. Connect to "MetaHack" BT device from your phone/app
2. Blue LED should blink when connected
3. Send read command (0x07) to get configuration data
4. Send write command (0x0A) to modify configuration
5. Changes are automatically saved!

## What Works in Simulator Mode

✅ **Configuration Read/Write** - Full 119-byte memory simulation  
✅ **Display Messages** - Fake voltage, current, status data  
✅ **Speedometer** - Incrementing odometer values  
✅ **Persistence** - All writes saved to flash memory  
✅ **Bluetooth** - Same BT protocol as real hardware  

❌ **No CAN Bus** - CAN interface is disabled  
❌ **No Real Data** - Values are simulated/fake  

## Switching Back to Normal Mode

1. Comment out `#define SIMULATOR` in the code:
   ```cpp
   // #define SIMULATOR
   ```
2. Re-compile and upload
3. Connect your CAN bus wiring
4. Normal operation resumes

## Troubleshooting

**"SPIFFS initialization failed!"**
→ You forgot to upload the data folder. Do Step 2 again.

**"No config file found"**
→ File upload didn't work. Check the `data/config.ini` exists and retry upload.

**BT connects but no data**
→ Check serial monitor for errors. Make sure `#define SIMULATOR` is uncommented.

**Changes don't save**
→ Look for "Config saved to file" in serial monitor. Check SPIFFS has space.

## Advanced Tips

### Load Different Configurations

1. Edit `metahack/data/config.ini` with your values
2. Re-upload SPIFFS (don't need to re-upload sketch)
3. Reboot ESP32

### Debug Mode

Add `#define DEBUG` to see detailed message logging:
```cpp
#define SIMULATOR
#define DEBUG
```
⚠️ Warning: Debug mode can cause instability. Use only for troubleshooting.

### Multiple Configs

Create different config files and swap them:
```bash
# Backup current config
cp metahack/data/config.ini metahack/data/config_backup.ini

# Load a different one
cp my_custom_config.ini metahack/data/config.ini

# Upload to ESP32
# (Arduino IDE → Tools → ESP32 Sketch Data Upload)
```

## Need Help?

- 💬 Main README: `README.md`
- ☕ Support project: https://www.buymeacoffee.com/metahack

---

**Ready to go?** Follow the 5-minute setup above and you'll be testing in no time! 🚀

