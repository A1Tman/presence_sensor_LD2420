# LD2420 ESP-IDF Driver (ESP32-C3)

Minimal HLK-LD2420 driver and demo for ESP-IDF. Supports:

- OT1 digital detection input with ISR + callbacks
- UART initialization and sending known hex commands (init/restart/factory reset)
- Optional UART RX task that dumps raw frames as hex for wiring/baud verification

This is a starting point; full binary protocol parsing can be added later.

## Wiring (per example)

- ESP32-C3 `GPIO20` -> LD2420 `OT1` (digital detect)
- ESP32-C3 `GPIO21` -> LD2420 `RX`  (ESP TX -> sensor RX)
- ESP32-C3 `GPIO4`  -> LD2420 `OT2` (optional second digital output)
- Optional: LD2420 `TX` -> ESP32-C3 `GPIOx` (set `LD2420_UART_RX` in `main/app_main.c`)
- Common GND between ESP32-C3 and LD2420
- Power the LD2420 per its datasheet (e.g., 5V/3.3V as appropriate)

Note: To receive and parse frames over UART you must connect LD2420 TX to an ESP32-C3 RX pin.

## Build

Ensure ESP-IDF environment is activated, then:

```
idf.py set-target esp32c3
idf.py build flash monitor
```

## Files

- `components/ld2420/ld2420.c/.h`: Driver implementation and API
- `main/app_main.c`: Demo app wiring and callbacks

## Quick behavior

- OT1 trigger changes print to the console via callbacks.
- `ld2420_send_init()` is sent once on startup to configure the sensor.
- If UART RX is wired and enabled, raw frames are dumped in hex for verification.

## Next steps

- Implement full LD2420 binary frame parsing (header `FD FC FB FA`, length, payload, checksum)
- Expose filtered distance / presence states from UART data
- Add Kconfig options for pin selection instead of macros in `app_main.c`
- Debounce or filter OT1 input if needed for your application

## Protocol helpers included

- `ld2420_open_command_mode()` => `FF 00 01 00`
- `ld2420_close_command_mode()` => `FE 00`
- `ld2420_read_version()` => `00 00`
- `ld2420_read_param(paramId)` => `08 00 <paramLE>`
- `ld2420_set_param_u32(paramId, valueLE)` => `07 00 <paramLE> <valueLE>`

The UART RX task parses frames with header `FD FC FB FA`, little-endian payload length, payload bytes, and tail `04 03 02 01`. It logs frames and best-effort extracts an ASCII version if present.
