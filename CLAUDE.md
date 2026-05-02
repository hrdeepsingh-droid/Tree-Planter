# CLAUDE.md — Tree Marker ALR Project Context

This file is for Claude Code. Read this first before touching any file in this repo.

---

## What this project is

A GPS-triggered relay marker for precision orchard row planting.
The KinCony ALR board (ESP32-S3) sits on a tractor, listens for RTK position
from an existing AgOpenGPS setup over WiFi UDP, and fires its on-board relay
every time the tractor crosses a tree grid intersection.
The relay triggers an existing electric paint sprayer solenoid to mark the ground.

This is a **pilot deployment** at a friend's farm in the Mildura/Sunraysia region
of Victoria, Australia. Row spacing 6.7m, tree spacing 3.0m.

---

## Hardware

### KinCony ALR board
- Chip: ESP32-S3-WROOM-1U N16R8
- On-board: 1x relay (AC250V/10A), SSD1306 OLED, SX1278 LoRa (not used in this project)
- Power: DC 9–24V (tractor uses 12V)
- Programming: USB-C, Arduino IDE or arduino-cli
- Product page: https://www.kincony.com/esp32-lora-sx1278-gateway.html
- GPIO map (confirmed from KinCony forum thread #6164):

```
RELAY        GPIO 48
OLED SDA     GPIO 39
OLED SCL     GPIO 38
ANALOG A1    GPIO 4
ANALOG A2    GPIO 6
ANALOG A3    GPIO 5
ANALOG A4    GPIO 7
1-wire       GPIO 15
Digital IN   GPIO 16
DIP K1–K8    GPIO 47,21,14,13,12,11,10,9
SX1278 CS    GPIO 41
SX1278 MOSI  GPIO 44
SX1278 MISO  GPIO 43
SX1278 SCK   GPIO 42
SX1278 RST   GPIO 2
SX1278 DIO0  GPIO 40
DL button    GPIO 0
```

### Tractor / AgOpenGPS setup
- Tractor: 1994–1997 Case IH 3230, 12V, hydrostatic power steering
- AIO board: AIO v4.5 Standard with Teensy 4.1 + Ethernet
- GPS: Ardusimple SimpleRTK2B (u-blox F9P), RTK fix, ~2cm accuracy
- IMU: BNO085
- AgOpenGPS version: v6.8.2-beta.4 on Windows tablet in cab
- AgIO broadcasts $GNGGA NMEA sentences via UDP broadcast on port 9999
- Network: phone hotspot in tractor cab (all devices same subnet)

### Paint sprayer
- Existing electric solenoid, 12V circuit
- ALR relay wired in series with existing solenoid trigger wire (volt-free contact)

---

## How position data flows

```
F9P RTK GPS
    ↓ UART
AIO Teensy 4.1
    ↓ Ethernet UDP broadcast → 255.255.255.255:9999
AgIO (Windows tablet)  ←──────────────────────────┘
                                ↑ also received by:
KinCony ALR (ESP32-S3)  ← WiFi UDP port 9999
    ↓ parses $GNGGA
    ↓ checks against pre-calculated tree grid
    ↓ GPIO 48 HIGH for RELAY_PULSE_MS
Solenoid → paint marker fires
```

No separate GPS on the ALR. It piggybacks the F9P RTK position that is already
being used for autosteer. This gives ~2–5cm accuracy.

---

## Repo structure

```
tree-marker-alr/
├── CLAUDE.md                          ← you are here
├── README.md                          ← user-facing docs
├── .gitignore
├── .github/
│   └── workflows/
│       └── build.yml                  ← arduino-cli build + GitHub Release on tag
└── firmware/
    └── tree_marker/
        └── tree_marker.ino            ← main sketch (single file, 396 lines)
```

---

## Firmware overview — tree_marker.ino

### Arduino IDE settings
- Board: **ESP32S3 Dev Module** (esp32 by Espressif, 2.x)
- USB Mode: USB-CDC On Boot — Enabled
- Flash Size: 16MB (matches N16R8 module)
- PSRAM: OPI PSRAM
- Upload Speed: 921600

### Libraries required
Install via Sketch → Manage Libraries:
- `PubSubClient` by Nick O'Leary
- `Adafruit SSD1306`
- `Adafruit GFX Library`

LoRa library is NOT needed — LoRa hardware is present on the ALR but not used in
this firmware. Do not add LoRa.h unless explicitly re-enabling it.

### Config block (top of .ino — always edit here)

```cpp
const char* WIFI_SSID     = "YourHotspotName";    // phone hotspot SSID
const char* WIFI_PASSWORD = "YourPassword";

const char* MQTT_HOST     = "your-id.s1.eu.hivemq.cloud";  // HiveMQ Cloud
const int   MQTT_PORT     = 8883;                           // TLS
const char* MQTT_USER     = "your-mqtt-user";
const char* MQTT_PASS     = "your-mqtt-pass";
const char* MQTT_CLIENT   = "tree-marker-alr-01";

const double ORIGIN_LAT   = -34.200000;   // << FILL FROM FIELD SURVEY
const double ORIGIN_LON   =  142.150000;  // << FILL FROM FIELD SURVEY
const double ROW_BEARING  =   90.0;       // << FILL FROM AGOPENGPS AB LINE
const double ROW_SPACING  =    6.7;       // metres — CONFIRMED
const double TREE_SPACING =    3.0;       // metres — CONFIRMED
const int    NUM_ROWS     =   10;         // update after field survey
const int    NUM_TREES    =   50;         // update after field survey
```

### MQTT topics
| Topic | Direction | Payload |
|---|---|---|
| `treemarker/status` | board → broker | JSON heartbeat every 30s |
| `treemarker/hit` | board → broker | `{"row":3,"tree":12,"dist":0.18,"lat":...,"lon":...}` |
| `treemarker/ota/url` | broker → board | GitHub release .bin download URL |
| `treemarker/ota/status` | board → broker | `queued` / `starting` / `success` / `failed: ...` |
| `treemarker/restart` | broker → board | any payload triggers ESP.restart() |

### OTA update flow
1. Push a semver tag → GitHub Actions builds .bin with arduino-cli → creates Release
2. Copy the .bin asset URL from the GitHub Release page
3. Publish URL string to MQTT topic `treemarker/ota/url`
4. Board receives URL in mqttCallback(), sets otaPending=true, otaUrl=URL
5. In loop(), doOTA() is called — uses HTTPUpdate with WiFiClientSecure (setInsecure)
6. HTTPUpdate follows GitHub redirects, flashes, reboots
7. Monitor `treemarker/ota/status` for progress

### Grid maths
Origin (row=0, tree=0) is ORIGIN_LAT/LON.
Rows extend perpendicular to ROW_BEARING.
Trees within each row extend along ROW_BEARING.
All coordinates pre-calculated at startup into `grid[row][tree]` struct array.
Hit detection uses flat-earth distance formula — accurate to <1mm at these scales.

---

## MQTT broker — HiveMQ Cloud

Mandeep (the owner) uses HiveMQ Cloud for all IoT projects including:
- FarmControl irrigation dashboard (Supabase project: `farmcontrol`)
- Filter backwash controller (ACB project, Board 3)
- This tree marker project

Broker URL format: `xxxxx.s1.eu.hivemq.cloud` port 8883 (TLS).
Credentials are in the user's HiveMQ Cloud console — not stored in this repo.

---

## GitHub Actions — build.yml

Triggers:
- Push to `main` → compile only (confirms it builds, no release)
- Push tag `v*.*.*` → compile + create GitHub Release with .bin attached

FQBN: `esp32:esp32:esp32s3`
Output binary: `build/tree_marker.ino.bin`

To release a new version:
```bash
# bump FW_VERSION in tree_marker.ino first, then:
git add .
git commit -m "Bump to v1.0.1"
git tag v1.0.1
git push origin main
git push origin v1.0.1
```

---

## Current project status

- [x] Firmware written and reviewed
- [x] GitHub repo structure created
- [ ] WiFi / MQTT credentials filled in
- [ ] First flash and Serial Monitor test
- [ ] Simulation test with AgOpenGPS (Phase 3 in project plan)
- [ ] Field survey for ORIGIN_LAT, ORIGIN_LON, ROW_BEARING (Phase 2)
- [ ] Tractor install and wiring (Phase 4)
- [ ] First field run at friend's farm (Phase 5)

### Simulation test procedure (Phase 3 — do this at desk before going to farm)
1. Fill in WiFi + MQTT config, flash ALR
2. OLED should show IP address then "Waiting for AgIO"
3. Open AgOpenGPS on Windows tablet → enable simulation mode
4. Set simulated position to ORIGIN_LAT, ORIGIN_LON (the first tree)
5. OLED shows "MARK!" and relay clicks — Serial Monitor shows `HIT Row 0 Tree 0`
6. Drag sim position along the row — relay should fire every 3.0m
7. Check `treemarker/hit` in HiveMQ for JSON payloads

---

## Known issues / things to watch

- `tlsClient.setInsecure()` is used for both HiveMQ MQTT and OTA HTTP.
  Fine for a farm IoT device — if you want cert pinning add it later.
- PubSubClient buffer set to 512 bytes via `mqtt.setBufferSize(512)`.
  GitHub release URLs are ~100 chars so this is safe.
- HTTPUpdate follows redirects automatically — GitHub release assets redirect
  once to the CDN, which HTTPUpdate handles correctly.
- If OLED doesn't appear at startup, check Wire.begin(39, 38) and I2C address 0x3C.
  Some SSD1306 clones use 0x3D — try that if 0x3C fails.
- AgIO must be in broadcast mode (to 255.255.255.255) not unicast to tablet IP only.
  Confirm in AgIO → GPS tab → From GPS port shows broadcast address.

---

## Owner context

Mandeep operates:
- Sunraysia Acres Pty Ltd — farm in Mildura/Sunraysia, Victoria
- AquaControl Automation — irrigation automation business targeting
  horticultural farms in the region
- CDC Mildura bus driver (V/Line Sunlink services C011/C012)

Development machine: HP Spectre x360 13, i7-8550U, 16GB RAM, Windows
Primary mobile: iPhone
Git root: `C:\Users\...\GIT\`
