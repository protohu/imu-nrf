# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Firmware for **nice!nano** (nRF52840): reads IMU data from BNO085 over I2C and streams it via BLE to a Python server (`imu_server`). Stack: Zephyr RTOS / nRF Connect SDK v2.7.0, USB CDC ACM, BLE GATT Notify, I2C TWI, CEVA sh2 library.

## Build & Flash

**Always source the environment first (once per terminal session):**
```bash
source /home/devs/ncs/env.sh
```

**Full build + flash cycle (recommended):**
```bash
./dev.sh
```
This builds, sends `dfu` over serial, waits for the NICENANO UF2 disk, flashes, and opens the console.

**Manual build only:**
```bash
west build -b promicro_nrf52840/nrf52840 --build-dir build -- -DBOARD_ROOT=/home/devs/ncs/custom_boards
```
Output: `build/zephyr/zephyr.uf2`

**Pristine build (required after Kconfig changes):**
```bash
west build --pristine -b promicro_nrf52840/nrf52840 --build-dir build -- -DBOARD_ROOT=/home/devs/ncs/custom_boards
```

**Flash manually:**
```bash
# Enter bootloader via shell (if board is running):
screen /dev/ttyACM0 115200   # then type: dfu
# Or double-press Reset until LED pulses and NICENANO disk appears, then:
cp build/zephyr/zephyr.uf2 /media/devs/NICENANO/
```

**Open serial console:**
```bash
screen /dev/ttyACM0 115200   # Exit: Ctrl+A → K → Y
```
Note: `dev.sh` must not be running when you open `screen` — it holds the port.

## Architecture

### Source files

| File | Responsibility |
|------|----------------|
| `src/main.c` | Main loop: reads IMU data, encodes payload, calls `bt_gatt_notify` |
| `src/transport.c` | USB CDC ACM init, LED blink status, `dfu` shell command |
| `src/ble.c` | BLE init, advertising, connection callbacks, watchdog thread |
| `src/imu.c` | Driver abstraction — delegates to registered `ImuDriverOps` |
| `src/bno085.c` | BNO085 driver: sh2 thread, sensor callbacks, `SYS_INIT` registration |
| `src/bno085_hal.c` | sh2 HAL: I2C read/write/open/close wrappers |
| `src/sh2/` | CEVA sh2 library (vendored, do not modify) |
| `app.overlay` | Switches i2c0 to `nordic,nrf-twi` (TWI, not TWIM — required for clock stretching) |
| `prj.conf` | Kconfig flags |

### Driver abstraction

`imu.c`/`imu_driver.h` define an `ImuDriverOps` vtable (`init`, `get_data`). `bno085.c` registers itself via `SYS_INIT(bno085_register, APPLICATION, ...)` so `main.c` calls `imu_get_data()` without knowing the concrete driver.

### Threads

| Thread | Stack | Priority | Role |
|--------|-------|----------|------|
| `bno085_tid` | 4096 | 5 | Calls `sh2_service()` + `k_yield()` in a tight loop; updates `latest ImuData` via mutex |
| `watchdog_tid` | 1024 | 7 | Every 3 s: restarts BLE advertising if not connected |
| main | — | — | Polls `imu_get_data()`, encodes payload, calls `bt_gatt_notify` every 2 ms (notifies at `IMU_BLE_HZ`=50 Hz) |

`bno085_thread` delays 5 s on startup to let USB CDC initialize before any `printk` output.

### BLE payload encoding

ATT MTU is fixed at 23 by BlueZ (max payload 20 bytes). The payload is 9 × `int16` = 18 bytes, using GY-85 legacy scaling so the Python server (`imu_server`) doesn't need changes:

- Gyro: `rad/s × 823.4` → ITG3200 counts
- Accel: `m/s² × 26.1` → ADXL345 counts  
- Mag: `µT × 10.9` → HMC5883L counts

Constants are in `main.c` (`GY85_GYRO_SCALE`, `GY85_ACCEL_SCALE`, `GY85_MAG_SCALE`).

BLE UUIDs:
- Service: `ceb5483e-36e1-4688-b7f5-ea07361b26a8`
- Characteristic (NOTIFY): `ceb5483e-36e1-4688-b7f5-ea07361b26a9`
- Device name: `LS_IMU_BLE`

### I2C / BNO085

- Address: `0x4B` (PS0 pulled to VCC on GY-BNO08x module)
- Pins: SDA=P0.17, SCL=P0.20
- **Must use TWI, not TWIM** — BNO085 uses clock stretching during writes, which TWIM does not support
- `app.overlay` sets `compatible = "nordic,nrf-twi"` at `I2C_BITRATE_STANDARD`

### IMU rates (defined in `src/imu.h`)

```c
#define IMU_SAMPLE_HZ  100U   // sensor poll rate
#define IMU_LOG_HZ      10U   // printk rate
#define IMU_BLE_HZ      50U   // BLE notify rate
```

## Key Notes

- `west` commands fail silently without `source /home/devs/ncs/env.sh` — run it every new session.
- The `dfu` shell command writes magic byte `0x57` to `GPREGRET` then cold-reboots; the UF2 bootloader checks this on startup.
- `compile_commands.json` in the project root is a filtered copy of `build/compile_commands.json` (only project sources). `dev.sh` regenerates it after each build — CLion uses this file for indexing.
- `CONFIG_I2C_NRFX_TWI` is not set manually in `prj.conf`; it is selected automatically from the DTS `compatible` string.
- BlueZ fixes ATT MTU=23 regardless of `bt_gatt_exchange_mtu` calls from either side.

## Python server (companion project)

Path: `/home/devs/PycharmProjects/imu_server`

Parses the 18-byte BLE payload with `struct.unpack('<9h', data)` and divides by the same GY-85 scale factors. The sensor is registered as `LS_IMU_BLE` / `joint_name="knee_left"` in `src/core/app.py`.
