# Votol BT Interface - Architecture Overview

## Normal Mode vs Simulator Mode

### Normal Mode (Production)
```
┌─────────────────┐
│   BT Client     │
│  (Phone/App)    │
└────────┬────────┘
         │ Bluetooth LE
         │ (CAFE/BABE)
         ▼
┌─────────────────┐
│     ESP32       │
│  ┌───────────┐  │
│  │  BLE      │  │
│  │  Server   │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │ BT Queue  │  │
│  │ (FreeRTOS)│  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │CAN Driver │  │
│  │(MCP2515)  │  │
│  └─────┬─────┘  │
└────────┼────────┘
         │ CAN Bus
         │ (500kbps)
         ▼
┌─────────────────┐
│     Votol       │
│   Controller    │
│  (Motor Driver) │
└─────────────────┘
```

**Data Flow:**
1. App sends command via BT → ESP32 BLE Server
2. Command queued in FreeRTOS queue
3. CAN task sends to CAN bus
4. Controller processes and responds
5. CAN callback receives response
6. Response sent back via BT to app

---

### Simulator Mode (Testing/Development)
```
┌─────────────────┐
│   BT Client     │
│  (Phone/App)    │
└────────┬────────┘
         │ Bluetooth LE
         │ (CAFE/BABE)
         ▼
┌─────────────────────────────────┐
│           ESP32                 │
│  ┌───────────┐                  │
│  │  BLE      │                  │
│  │  Server   │                  │
│  └─────┬─────┘                  │
│        │                        │
│  ┌─────▼──────────────────┐     │
│  │   BT Callback Handler  │     │
│  │  (Direct Processing)   │     │
│  └─────┬──────────────────┘     │
│        │                        │
│  ┌─────▼─────┐   ┌──────────┐   │
│  │ Simulator │◄─►│  SPIFFS  │   │
│  │  Memory   │   │ Storage  │   │
│  │ (119 b)   │   │config.ini│   │
│  └───────────┘   └──────────┘   │
│                                 │
│  ❌ CAN Driver (Disabled)       │
│  ❌ CAN Queue (Not Created)     │
└─────────────────────────────────┘
```

**Data Flow:**
1. App sends command via BT → ESP32 BLE Server
2. BT callback handles directly (no queue)
3. Simulator reads from in-memory config (119 bytes)
4. Response built from simulator memory
5. Response sent back via BT to app
6. Writes persist to SPIFFS `/config.ini`

---

## Code Structure

### Key Files

```
votol_bt_interface/
├── metahack/
│   ├── metahack.ino          # Main Arduino sketch
│   └── data/
│       └── config.ini        # SPIFFS config file (119 lines)
├── Config_ryvid_original.ini # Example configuration
├── README.md                 # Main documentation
├── SIMULATOR_README.md       # Simulator mode guide
└── SIMULATOR_QUICKSTART.md   # Quick start guide

```

---

## Compilation Modes

The sketch supports three mutually exclusive modes:

### 1. Normal Mode (Default)
```cpp
// #define FAKE
// #define SIMULATOR
```
- Full CAN functionality
- Production mode
- Requires CAN hardware

### 2. Fake Mode (Legacy)
```cpp
#define FAKE
// #define SIMULATOR
```
- Static fake data
- No SPIFFS
- For basic BT testing

### 3. Simulator Mode (New)
```cpp
// #define FAKE
#define SIMULATOR
```
- Dynamic config from SPIFFS
- Full read/write simulation
- Persistent storage
- No CAN required

**⚠️ Important:** Never enable multiple modes simultaneously!

---

## GPIO Pin Assignments

### Normal ESP32 Board
- GPIO 25: CAN RX
- GPIO 26: CAN TX
- GPIO 2: Blue LED (built-in)

### LOLIN32 Board (Not fully working)
- GPIO 22: CAN RX (experimental)
- GPIO 21: CAN TX (experimental)
- GPIO 5: LED (experimental)

---

## Debugging

### Serial Monitor Output (250000 baud)

**Normal Mode:**
```
Serial init ok!
WiFi disabled!
CAN init ok!
BT init ok!
```

**Simulator Mode:**
```
Serial init ok!
WiFi disabled!
SPIFFS initialized!
Loaded 119 bytes from config file
Config loaded from file!
SIMULATOR MODE - CAN disabled
BT init ok!
```

### Debug Mode
Uncomment `#define DEBUG` for detailed logging:
- BT message lengths
- CAN packet IDs
- Queue status
- Watchdog resets

⚠️ **Warning:** Debug mode causes instability due to serial overhead.

---

## License

Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)

- ✅ Non-commercial use
- ✅ Modification allowed
- ✅ Must credit author
- ✅ Share-alike required
- ❌ Commercial use prohibited

---

## Support

- 💬 Issues: GitHub repository
- 📖 Documentation: This repo
- ☕ Donations: https://www.buymeacoffee.com/metahack
- 🎥 Videos: https://www.youtube.com/@metacyclehack

