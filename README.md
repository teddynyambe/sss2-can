# SSS2 Firmware — PlatformIO Project (Teensy 3.6)

Converted from the original Arduino IDE project for use with PlatformIO.

## Project Structure

```
SSS2_PlatformIO/
├── platformio.ini              ← PlatformIO config (board, libs, flags)
├── src/
│   ├── main.cpp                ← Main firmware (was SSS2_Firmware.ino)
│   ├── SSS2_functions.h        ← Core functions header
│   ├── SSS2_board_defs_rev_5.h ← Board pin definitions (Rev 5 - default)
│   └── SSS2_board_defs_rev_3.h ← Board pin definitions (Rev 3)
└── lib/
    ├── FlexCAN_Library/        ← Teensy CAN bus library
    ├── MCP23017/               ← Adafruit MCP23017 I/O expander (i2c_t3 version)
    ├── MCP_CAN_lib/            ← MCP2515 SPI CAN library
    ├── OneButton/              ← Button debounce library
    ├── ArduinoThread/          ← Thread management library
    └── TeensyID/               ← Teensy unique ID library
```

## Setup & Build

1. Install [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) (VS Code extension recommended)
2. Open this folder in VS Code
3. PlatformIO will auto-install dependencies from `platformio.ini`
4. Click **Build** (✓) or run: `pio run`
5. To upload: `pio run --target upload`

## Dependencies Auto-Installed by PlatformIO

| Library | Source |
|---|---|
| Encoder | paulstoffregen/Encoder |
| Time | paulstoffregen/Time |
| i2c_t3 | github.com/nox771/i2c_t3 |

## Bundled Local Libraries (in `lib/`)

| Library | Notes |
|---|---|
| FlexCAN_Library | Teensy FlexCAN driver |
| MCP23017 | Modified Adafruit lib using i2c_t3 instead of Wire |
| MCP_CAN_lib | MCP2515 SPI CAN driver |
| OneButton | Single-button state machine |
| ArduinoThread | Cooperative threading |
| TeensyID | Reads Teensy chip UID |

## Board Revision

By default, **Rev 5** board definitions are used (`SSS2_board_defs_rev_5.h`).  
To target Rev 3 hardware, edit `src/main.cpp` and change:
```cpp
#include "SSS2_board_defs_rev_5.h"
// to:
#include "SSS2_board_defs_rev_3.h"
```

## Changes from Original Arduino Project

- `SSS2_Firmware.ino` → `src/main.cpp` with `#include <Arduino.h>` added at top
- `SSS2_functions.h` → `#include <Arduino.h>` added at top
- All libraries moved to `lib/` (no need to install manually into Arduino libraries folder)
- `ENCODER_OPTIMIZE_INTERRUPTS` and `USE_SPI1` moved to `build_flags` in `platformio.ini`
