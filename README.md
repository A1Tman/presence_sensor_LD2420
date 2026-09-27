# LD2420 Presence Sensor (ESP32‑C3, ESP‑IDF)

Small ESP‑IDF firmware for the HLK‑LD2420 24 GHz radar. It publishes presence and distance to MQTT with Home Assistant auto‑discovery, exposes tuning sliders, and supports “Apply Config”, “Restart”, and “Resend Discovery” buttons. Optional MQTTS with broker CA validation is built‑in.

## Wiring

ESP32‑C3 OLED dev board pin map used here:

- UART1 TX `GPIO10` -> LD2420 RX (ESP TX -> radar RX)
- UART1 RX `GPIO7`  <- LD2420 TX (ESP RX <- radar TX)
- OT2 `GPIO4`       <- LD2420 OT2 (presence output, optional but used as a backup)
- 3.3 V power and common GND between ESP32‑C3 and LD2420

Notes
- The board’s OLED (SCL=GPIO6, SDA=GPIO5) shows a local status view:
  boot screen, live presence/network page, LD2420 config page, and fault overrides.
- Use a clean 3.3 V supply and common ground.

## Build

Ensure ESP-IDF environment is activated, then:

```
idf.py set-target esp32c3
idf.py build flash monitor
```

## Credentials (provisioning)

Wi-Fi and MQTT credentials are **not compiled into the firmware**. They live in a separate `creds` NVS partition (`partitions.csv`) that is written over USB and never touched by OTA, so a firmware image can be shared or hosted without exposing them.

1. Copy `config/creds.csv.template` to `config/creds.csv` (git-ignored) and fill in `wifi_ssid`, `wifi_pass`, `mqtt_user`, `mqtt_pass`.
2. With the board on USB: `./tools/provision.ps1 -Port COM3`. It generates the NVS image, flashes it to the `creds` partition and deletes the temporary image. The device restarts and connects.

To rotate a password, edit `creds.csv` and run `provision.ps1` again; no rebuild is needed. Without provisioned credentials the device stays offline and the OLED shows `No creds / Provision`. `config/secrets.h` keeps only non-secret settings (broker host/port, CA certificate, device name).

Never attach built firmware to GitHub releases: OTA goes through `tools/ota_release.ps1`, which also refuses to publish an image that contains a password from `creds.csv`.

## Tests

Fast host-side harness tests cover MQTT discovery/command behavior and LD2420 command-frame parsing:

```
powershell -ExecutionPolicy Bypass -File tests\run_host_tests.ps1
```

## Home Assistant

This firmware publishes HA discovery. After boot you’ll see a device with:

- `binary_sensor`: Presence
- `sensor`: Distance (cm), Wi‑Fi Signal (dBm), Uptime (s), LD2420 Firmware (diagnostic)
- `binary_sensor`: Movement zones (Near/Mid/Far)
- `number` (config): Movement Threshold (cm), Presence Hold (s)
- `number` (LD2420): Min/Max Gate, Response Delay (ms), Trigger Level, Tracking Level
- `button`: Apply Config (config), Restart (diagnostic), Resend Discovery (diagnostic)
- `update` (config): Firmware - shows "Update available" and installs OTA releases

The onboard 72x40 OLED mirrors key local state without needing Home Assistant:

- Page 1: presence, distance, Wi‑Fi RSSI, MQTT state, IP suffix
- Page 2: LD2420 min/max gate, delay, trigger, tracking, LD2420 firmware
- Fault override: radar init/no-data, Wi‑Fi down, or MQTT waiting

Buttons only act on press and are rate‑limited. “Apply Config” writes your staged LD2420 values over UART in command mode.

## MQTT Topics (overview)

Base topic: `presence/<device-id>`

- State: `/presence` (`ON`/`OFF`)
- Distance: `/movement_distance_cm` (float cm)
- Availability: `/status` (`online` retained, `offline` LWT)
- RSSI: `/rssi`, Uptime: `/uptime_s`
- Zones: `/movement/near_range`, `/mid_range`, `/far_range` (`ON`/`OFF`)
- Numbers publish retained states under `/cfg/...` and receive commands under `/cmd/...`
- OTA: state `/ota/state` (JSON for the HA update entity); retained manifest in `/cmd/ota/manifest`; install command `/cmd/ota/install`

## OTA updates

The version lives in one place: `PROJECT_VER` in `CMakeLists.txt`. It is reported to HA as the device firmware version.

Flash layout: two 1.94 MB app slots (`ota_0`/`ota_1`, see `partitions.csv`). Moving from the old single-factory layout needs **one USB flash** (`idf.py flash`); NVS settings are kept.

Release flow:

1. Bump `PROJECT_VER`, then `idf.py build` (the image is signed automatically).
2. `./tools/ota_release.ps1 -Notes "what changed"` copies the image to HA (`/config/www/ota/<device>/<random>/`) and publishes a retained manifest (version, URL, SHA-256, size) over MQTT.
3. In HA, press **Install** on the device's *Firmware* entity. The device downloads the image, checks size + SHA-256 + signature + version, switches slots and reboots.
4. After it reports the new version: `./tools/ota_release.ps1 -Clean` (removes the hosted file, clears the manifest).

Safety nets:

- **Signed images only** (`CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT`, RSA-3072). OTA images must be signed with the private key at `CONFIG_SECURE_BOOT_SIGNING_KEY` (kept outside the repo; back it up). The public key is `tools/ota_signing_pubkey.pem`. This is not Secure Boot: no eFuses are burned and USB flashing accepts any image.
- **Automatic rollback**: a new image must connect to MQTT and receive valid radar frames within 5 minutes (`OTA_ROLLBACK_TIMEOUT_S`), or it reboots into the previous image. Crashes before that also roll back.
- The release script needs an MQTT login (default `ota_release`) that can publish `presence/+/cmd/ota/manifest`; the password comes from `LD2420_OTA_MQTT_PASSWORD`, `~/.esp-keys/ld2420_ota_mqtt_password.txt`, or a prompt. The device's own MQTT user should only be able to *read* `presence/<device-id>/cmd/#`, so a leaked device credential cannot push commands or releases.

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
- Credentials: provisioned into the `creds` partition (see *Credentials*), never compiled into firmware. Command entities are enabled only when `mqtt_user` is provisioned, unless `MQTT_ALLOW_ANONYMOUS_COMMANDS` is explicitly set to `1`. The partition is not encrypted, so anyone with physical access to the board can read it.
- Safety: Apply/Restart are press‑only and rate‑limited. Use broker ACLs so each device/user can publish only to the intended `presence/<device-id>/cmd/...` topics.

## Files

- `components/ld2420`: UART driver and protocol helpers (enter/exit config, read/write params, read version)
- `components/ha_mqtt`: MQTT + HA discovery and entities
- `components/ota_update`: OTA download/verify/flash and rollback guard
- `tools/ota_release.ps1`: publish a build as an OTA update via Home Assistant
- `tools/provision.ps1`: write Wi-Fi/MQTT credentials to the `creds` partition over USB
- `main/device_creds.c`: load credentials from the `creds` partition at boot
- `main`: app wiring, logic, and MQTT integration
