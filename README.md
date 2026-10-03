[README.md](https://github.com/user-attachments/files/32997849/README.md)
# Rocket Telemetry Flight Computer

A two-board LoRa telemetry system for a model rocket: an ESP32-based **flight computer** (BMP280 barometer + SX1278 Ra-02 radio) streams live altitude at 10 Hz to a **ground station** (second ESP32 + Ra-02 broadcasting a WiFi dashboard viewable from any phone).

Flight computer: hand-soldered perfboard, single-cell 1S LiPo with a 100k/100k battery monitor divider, buck-boost regulator. Airframe: cardboard tube, MDF avionics plate, straight vertical whip antenna.

Based on a vendor-provided PlatformIO project (`rocket_altd_src/`). The flight firmware was reworked for the actual hardware and hardened with several fixes; the vendor ground-station code is kept unmodified (it defines the radio contract both sides must obey).

## Radio contract (frozen)
- 434.5 MHz, SF7, BW 125 kHz, CR 4/8, preamble 8, sync word 0x12, CRC on
- 6-byte big-endian packet: `rocket_id (u8), type (u8: 0x01 normal / 0x02 apogee), sequence (u16 BE), altitude (u16 BE, 0.1 m)`
- `ROCKET_ID = 37`, 10 Hz telemetry, 10-packet burst at apogee lock
## Firmware features (flight side)
- Launch / ascent / apogee / descent / landed state machine with median+EMA pressure filtering
- Apogee persistence counted at the 20 Hz log tick (immune to filter ripple)
- Landing requires ~2.5 s below 5 m
- Binary blackbox log on LittleFS (~20 Hz, batched writes) — `DUMP` over serial replays CSV incl. battery voltage
- Mid-flight reset survival: RTC RAM (magic-guarded) + LittleFS mirror of P0/max-altitude/state; recovers from soft reset *and* cold boot during ascent
- Non-blocking serial command handling, watchdog, 80 MHz CPU (battery endurance)
- Battery monitor: 100k/100k divider → GPIO34, logged in the blackbox
- 13 dBm TX (license-friendly; raise `LORA_TX_POWER` to 20 if your spec requires)

## Pin map (flight board)
| Function | ESP32 pin |
|---|---|
| BMP280 SDA / SCL | 21 / 22 |
| Ra-02 NSS / SCK / MOSI / MISO | 5 / 18 / 23 / 19 |
| Ra-02 RST / DIO0 | 14 / 26 |
| Battery divider node | 34 (input-only) |

## Arduino IDE sketches (`arduino_ide/`)
| Sketch | Purpose |
|---|---|
| `flight_sketch/flight_sketch.ino` | The flight firmware — final upload for flight |
| `altitude_beacon/altitude_beacon.ino` | Minimal transmitter for hardware bring-up / fault bisection |
| `ground_sketch/ground_sketch.ino` | Vendor ground-station code inlined — only for a homemade backup ground unit |

Board: **ESP32 Dev Module**, partition **Default 4MB with spiffs**, 115200 serial.
Libraries: LoRa (Sandeep Mistry), Adafruit BMP280 (+ Unified Sensor), ArduinoJson v7.

## Repo layout
```
arduino_ide/      ready-to-flash Arduino sketches
rocket_altd_src/  PlatformIO sources (flight modified, ground vendor-pristine)
tools/            sketch generator + wiring-diagram generators (Python + Pillow)
visualizations/   avionics bay cutaway, wiring map, power tree, launch checklist
```
    
## Safety notes
- Never power a SX1278 without its antenna attached.
- 1S LiPo without a protection board: fuse the wiring with the series switch, retire the cell below 3.4 V at rest, never charge unattended.
- Ejection charge compartment must be bulkhead-separated from the avionics bay; avionics bay needs static ports (not airtight).
