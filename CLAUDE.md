# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Smart Sensor Simulator 2 (SSS2)** — PlatformIO firmware for a Teensy 3.6-based device that simulates automotive sensors and communicates over CAN, LIN, J1708, and J1939 protocols. Used for heavy-duty vehicle testing and development.

## Build Commands

```bash
pio run                        # Build firmware
pio run --target upload        # Build and upload to connected Teensy
pio run --target clean         # Clean build artifacts
```

There is no automated test framework. Testing is done via Serial commands and live hardware interaction.

## Architecture

### Key Source Files

- **`src/main.cpp`** — Entry point. `setup()` initializes hardware; `loop()` processes CAN messages from 3 buses, J1708 checksum handling, Serial commands, and encoder input.
- **`src/SSS2_functions.h`** — Core library (~2,400 lines). Defines the `CanThread` class, `ThreadController`, all 93 configurable settings, hardware control functions, and the Serial command processor.
- **`src/SSS2_board_defs_rev_5.h`** — Pin assignments, default values, and I2C addresses for Rev 5 hardware (default). Switch to `_rev_3.h` for older boards by changing the `#include` in `main.cpp`.

### Communication Buses

| Bus | Library | Default Baud |
|-----|---------|--------------|
| CAN0 | FlexCAN (native) | 250 kbps |
| CAN1 | FlexCAN (native) | 250 kbps |
| CAN2 | MCP_CAN_lib (SPI/MCP2515) | 250 kbps |
| LIN | Serial2 | 19,200 |
| J1708 | Serial3 | 9,600 |
| USB | RawHID + CDC | — |

### Cooperative Threading

A 1ms `IntervalTimer` drives a `ThreadController` that manages up to 256 `CanThread` instances. Each thread handles periodic transmission of a single CAN message independently. This is the mechanism for all timed CAN message output and decouples message timing from the main loop.

### Settings System (93 parameters)

`setSetting(id, value)` in `SSS2_functions.h` is the central dispatch for all hardware output changes. Settings map to:
- **1–16**: SPI digital potentiometer wiper values
- **17–24**: 12-bit DAC outputs
- **25–32**: Potentiometer terminal connection states
- **33–36, 81–85**: PWM duty cycles and frequencies
- **37–48**: Switch/connection configurations
- **49–50**: High-voltage output and ignition relay
- **51–66**: Potentiometer terminal connection modes
- **67–74**: PWM routing modes
- **75–80**: Additional potentiometer settings

### Serial Command Interface

Commands are comma-delimited strings sent over USB Serial. Key commands include `SM` (set/configure CAN message), `SP` (set parameter), `GO`/`STOPCAN`/`STARTCAN`, `STATS`/`CLEARSTATS`, `AI` (analog input stream), `DB` (CAN debug stream), `J1708`, `LIN`/`SENDLIN`, `CANSEND`, `RELOAD`, `TIME`.

### Libraries (in `lib/`)

All dependencies are vendored locally to avoid version drift:
- **FlexCAN_Library** — Native Teensy CAN driver (CAN0, CAN1)
- **MCP_CAN_lib** — SPI CAN for MCP2515 (CAN2)
- **MCP23017** — I2C I/O expander (modified for `i2c_t3`)
- **OneButton** — Button debounce/state machine
- **ArduinoThread** — Cooperative threading
- **TeensyID** — Reads Teensy 128-bit unique chip ID

### Build Flags

Defined in `platformio.ini`:
- `-DENCODER_OPTIMIZE_INTERRUPTS` — Required for Encoder library on Teensy
- `-DUSE_SPI1` — Selects SPI1 bus for the MCP2515 CAN module
- `lib_ldf_mode = deep+` — Enables deep library dependency resolution

### EEPROM

Component ID is stored in EEPROM starting at a high address (~1000+). Modified via the `CI` / `changeComponentID()` command.
