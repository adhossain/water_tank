# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP32 firmware for a rooftop water-tank level controller / pump relay, built as an app inside the TuyaOpen SDK (`E:\Projects\TuyaOpen`, SDK root is three levels up). This directory is its own git repo nested in the SDK checkout. The SDK-level `AGENTS.md` mostly applies, but **reply in English** (the user overrides its Chinese-language rule). Avoid interactive `tos.py config choice` / `tos.py config menu` — edit `app_default.config` instead.

## Build / flash / monitor (Windows, PowerShell)

```powershell
cd E:\Projects\TuyaOpen; .\export.ps1          # activate SDK env (venv, tos.py)
cd E:\Projects\TuyaOpen\apps\tuya_cloud\water_tank
tos.py build                                    # outputs in .build/bin and dist/water_tank_1.0.0/
tos.py flash -p COM7
tos.py monitor -p COM7                          # or: python -m serial.tools.miniterm COM7 115200
```

- If `tos.py flash` fails, flash with esptool from `.build/bin` (deactivate the venv first): `python -m esptool --chip esp32 --port COM7 --baud 460800 write_flash 0x1000 bootloader.bin 0x8000 partition-table.bin 0xd000 ota_data_initial.bin 0x10000 water_tank.bin`. Hold BOOT if it won't connect.
- Build error on `tuya_ai_encoder.h` → set `CONFIG_ENABLE_AI=n` in `app_default.config`.
- No tests exist; validation = a clean `tos.py build`. Format check from SDK root: `python tools/check_format.py --debug --files <file>`.
- `CMakeLists.txt` uses `aux_source_directory(src)`, so every `.c` in `src/` is compiled automatically (including `reset_netcfg.c`, which is currently not called from main).

## Architecture

Nearly all logic is in [src/tuya_main.c](src/tuya_main.c); [src/ili9341.c](src/ili9341.c) is a bit-banged (software SPI) display driver. [include/tuya_config.h](include/tuya_config.h) holds DP IDs, pin map, and default settings.

Threads (started in `user_main`, entry `tuya_app_main`):
- **main thread** — `tuya_iot_yield` loop; `user_event_handler_on` handles MQTT connect (forces full DP snapshot) and `handle_dp_receive` (app → device settings/pump commands, then echoes DPs back).
- **sensor_task** (every ~3 s) — polls the sensor, runs a 1-D Kalman filter, computes level %, runs `pump_auto_control`, handles sensor-fault pump cutoff and deferred power-recovery restore, then calls `report_dps_smart`.
- **ui_task** — button polling and the HOME / MENU / EDIT screen state machine with partial-redraw dirty flags (`g_ui_dirty`, `g_full_redraw`, `g_menu_full`, `g_edit_full`, `g_disp_*` caches).

Shared state lives in the global `g_app` struct; there is no locking, so keep cross-thread writes simple.

Key behaviors to preserve:
- **Sensor**: A0221A4 RS-485 **Modbus RTU** sensor (polled, not free-running) via a MAX3485 on UART1 with manual direction control on GPIO33 (`PIN_RS485_DE`, HIGH = TX). Query reads register `0x0100` (distance in mm). UART1 pins must be remapped with `__tkl_uart1_set_txd_pin/rxd_pin` **before** `tal_uart_init`; the SDK supports only UART0/1. The README's A02YYUW/buffer-drain notes describe older firmware.
- **Valid vs. filtered reading**: `sensor_filtered_distance` returns the Kalman estimate but sets `valid` only on a fresh good poll — pump logic must key off `valid`, never off the stale estimate.
- **Safety**: after `SENSOR_FAIL_FAULT` consecutive failures, fault bit 0x01 is set and the pump is cut (auto mode only unless `FAULT_STOPS_MANUAL`). Power-recovery never restarts the pump blind — it sets `g_pump_restore_pending` and waits for a valid reading.
- **Relay polarity** is selected by `RELAY_ACTIVE_LOW` in tuya_main.c (currently 0 = active HIGH, despite README saying active LOW).
- **Cloud reporting**: `report_dps_smart` only sends DPs that changed (level ≥5%, distance ≥5 cm, or any setting change) plus a 10-minute heartbeat; `request_report_now()` forces a full report. DP 109 (fault) reporting is disabled to avoid schema errors; DP 101 min must be 0 on the Tuya platform.
- **Persistence**: settings and last pump state are stored with `tal_kv` (keys `wt_*`, validity magic `KV_MAGIC_VALUE`).
- Firmware version string `"1.3.2"` appears in three places in tuya_main.c (header comment, boot splash, `software_ver`) — bump all together.

Water level: `water_height = tank_height + sensor_offset - distance`, clamped, as % of `tank_height`.

## Notes

- Tuya PID/UUID/AuthKey live only in git-ignored `.env` (template: `.env.example`); `CMakeLists.txt` parses it into compile definitions, and `tuya_config.h` has an `#error` if they are missing. Never hard-code credentials, print them, or commit `.env` / `uuid_auth_key.xlsx`. After editing `.env`, rebuild (CMake reconfigures automatically).
- `hardware/` holds KiCad projects and Gerber/drill exports; not part of the build.
