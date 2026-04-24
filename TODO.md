# Project TODO

## Critical / High

- [x] Fix boot-time presence false positives and avoid timeout clears while radar presence is still active.
- [x] Harden Home Assistant discovery JSON construction so truncation cannot advance append pointers out of bounds.
- [x] Review MQTT credential, TLS, and command authorization handling; fix code or documentation gaps found during review.

## Medium

- [x] Move long LD2420 apply-config work out of the MQTT event handler.
- [x] Make LD2420 command response parsing robust against streamed data before the response header.
- [x] Normalize locking for sensor snapshots and MQTT connection state reads.

## Low

- [x] Keep ESP application firmware and LD2420 module firmware explicitly separate in MQTT naming.
- [x] Wire `HA_DISCOVERY_PREFIX` from `config/secrets.h` into `ha_mqtt_cfg_t`.
- [ ] Add focused tests for presence state, MQTT payload assembly, zone normalization, and LD2420 frame parsing.

## Review Workflow

- [x] Re-run `idf.py build` after each fix batch.
- [x] Do a second focused security and logic review after the high-priority fixes land.
