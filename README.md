# LD2420 Presence Sensor (ESP32‑C3, ESP‑IDF)

Small ESP‑IDF firmware for the HLK‑LD2420 24 GHz radar. It publishes presence and distance to MQTT with Home Assistant auto‑discovery, exposes tuning sliders, and supports “Apply Config”, “Restart”, and “Resend Discovery” buttons. Optional MQTTS with broker CA validation is built‑in.

## Wiring

ESP32‑C3 OLED dev board pin map used here:

- UART1 TX `GPIO10` -> LD2420 RX (ESP TX -> radar RX)
- UART1 RX `GPIO7`  <- LD2420 TX (ESP RX <- radar TX)
- OT2 `GPIO4`       <- LD2420 OT2 (presence output, optional but used as a backup)
- 3.3 V power and common GND between ESP32‑C3 and LD2420

Notes
- The board’s OLED (SCL=GPIO6, SDA=GPIO5) isn’t used by this firmware.
- Use a clean 3.3 V supply and common ground.

## Build

Ensure ESP-IDF environment is activated, then:

```
idf.py set-target esp32c3
idf.py build flash monitor
```

## Home Assistant

This firmware publishes HA discovery. After boot you’ll see a device with:

- `binary_sensor`: Presence
- `sensor`: Distance (cm), Wi‑Fi Signal (dBm), Uptime (s), LD2420 Firmware (diagnostic)
- `binary_sensor`: Movement zones (Near/Mid/Far)
- `number` (config): Movement Threshold (cm), Presence Hold (s)
- `number` (LD2420): Min/Max Gate, Response Delay (ms), Trigger Level, Tracking Level
- `button`: Apply Config (config), Restart (diagnostic), Resend Discovery (diagnostic)

Buttons only act on press and are rate‑limited. “Apply Config” writes your staged LD2420 values over UART in command mode.

## MQTT Topics (overview)

Base topic: `presence/<device-id>`

- State: `/presence` (`ON`/`OFF`)
- Distance: `/movement_distance_cm` (float cm)
- Availability: `/status` (`online` retained, `offline` LWT)
- RSSI: `/rssi`, Uptime: `/uptime_s`
- Zones: `/movement/near_range`, `/mid_range`, `/far_range` (`ON`/`OFF`)
- Numbers publish retained states under `/cfg/...` and receive commands under `/cmd/...`

## LD2420 Protocol (short version)

Frames use little‑endian fields.

Command/ACK frames:
- Header: `FD FC FB FA`
- Length: 2 bytes (LE)
- Payload
- Footer: `04 03 02 01`

Commands used here:
- Enter command mode: `0x00FF` + `0x0002` (protocol)
- Exit command mode:  `0x00FE`
- Read version:       `0x0000`
- Restart module:     `0x0068`
- Read parameter:     `0x0008` + (param_id)
- Set parameter:      `0x0007` + (param_id + u32 value)

Parameter IDs:
- Min gate: `0x0000` (0–15)
- Max gate: `0x0001` (0–15)
- Response delay (ms): `0x0004` (0–65535)
- Trigger threshold:  `0x0010`–`0x001F`
- Maintain threshold: `0x0020`–`0x002F`

Upload (“energy”) data:
- Header: `F4 F3 F2 F1`
- Length: LE
- Presence: 1 byte (0/1)
- Distance: 2 bytes (cm, LE)
- Energy: 16×u16 gate energies (32 bytes)
- Footer: `F8 F7 F6 F5`

## Security

- TLS: define a CA PEM in `config/secrets.h`; the client switches to `mqtts://` with server validation.
- Credentials: per‑device user/password supported.
- Safety: Apply/Restart are press‑only and rate‑limited.

## Files

- `components/ld2420`: UART driver and protocol helpers (enter/exit config, read/write params, read version)
- `components/ha_mqtt`: MQTT + HA discovery and entities
- `main`: app wiring, logic, and MQTT integration
