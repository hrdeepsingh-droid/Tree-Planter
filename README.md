# Tree Marker ALR

GPS-triggered relay marker for precision orchard row planting.  
Fires an on-board relay at each tree grid intersection using RTK accuracy from an AgOpenGPS setup.

**Hardware:** [KinCony ALR](https://www.kincony.com/esp32-lora-sx1278-gateway.html) (ESP32-S3 + SSD1306 OLED + relay)  
**Position source:** AgOpenGPS / AgIO via WiFi UDP — no separate GPS needed  
**Accuracy:** ~2–5 cm with F9P RTK  

---

## How it works

```
F9P RTK GPS → AIO Teensy → UDP broadcast (port 9999) → AgIO on tablet
                                   ↓
                         ESP32-S3 on ALR listens on same WiFi
                         → parses $GNGGA → checks grid → fires relay
```

The ALR, your Windows tablet (AgIO), and AIO Teensy Ethernet all connect to the same phone hotspot in the tractor cab. No extra GPS wiring required.

---

## Hardware wiring

| Connection | Details |
|---|---|
| Power | 12–24V DC to ALR terminals |
| Relay output | ALR relay COM + NO → existing solenoid trigger circuit |
| WiFi | Same hotspot as AgOpenGPS tablet |

That's it. OLED, relay, and WiFi are all on-board.

---

## Setup

### 1. Install Arduino IDE dependencies

- Board package: **ESP32 by Espressif** — add this URL in board manager:  
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- Board: **ESP32S3 Dev Module**
- Libraries (Sketch → Manage Libraries):
  - `PubSubClient`
  - `Adafruit SSD1306`
  - `Adafruit GFX Library`

### 2. Edit config in `tree_marker.ino`

```cpp
const char* WIFI_SSID     = "YourHotspotName";
const char* WIFI_PASSWORD = "YourPassword";
const char* MQTT_HOST     = "your-id.s1.eu.hivemq.cloud";
const char* MQTT_USER     = "your-mqtt-user";
const char* MQTT_PASS     = "your-mqtt-pass";

const double ORIGIN_LAT   = -34.xxxxxx;   // First tree lat from AgOpenGPS
const double ORIGIN_LON   =  142.xxxxxx;  // First tree lon from AgOpenGPS
const double ROW_BEARING  =  90.0;        // AB line bearing from AgOpenGPS
const double ROW_SPACING  =   6.7;        // Metres
const double TREE_SPACING =   3.0;        // Metres
const int    NUM_ROWS     =  10;
const int    NUM_TREES    =  50;
```

### 3. Flash and test

1. Connect ALR via USB-C
2. Select **ESP32S3 Dev Module**, correct COM port
3. Upload
4. Open Serial Monitor (115200 baud)
5. OLED should show WiFi IP then "Waiting for AgIO"
6. Run AgOpenGPS simulation — drag cursor over a grid point — relay clicks

---

## MQTT topics

| Topic | Direction | Payload |
|---|---|---|
| `treemarker/status` | board → you | JSON heartbeat every 30s |
| `treemarker/hit` | board → you | `{"row":3,"tree":12,"dist":0.18,"lat":...,"lon":...}` |
| `treemarker/ota/url` | you → board | GitHub release .bin URL |
| `treemarker/ota/status` | board → you | `queued` / `starting` / `success` / `failed: ...` |
| `treemarker/restart` | you → board | Any payload triggers reboot |

---

## OTA firmware update via MQTT

1. Push a version tag to trigger a build and release:
   ```
   git tag v1.0.1
   git push origin v1.0.1
   ```
2. Wait for GitHub Actions to finish (~3 min)
3. Go to **Releases** → copy the `.bin` asset URL
4. Publish the URL to `treemarker/ota/url` via your MQTT client  
   (HiveMQ web client, MQTT Explorer, or Node-RED)
5. Board downloads, flashes, and reboots — monitor `treemarker/ota/status`

---

## Grid coordinate system

```
Tree (0,0) ─── Tree (0,1) ─── Tree (0,2) ───  ← Row 0 (ROW_BEARING direction)
    │                                          spacing = TREE_SPACING (3.0m)
Tree (1,0) ─── Tree (1,1) ─── Tree (1,2) ───  ← Row 1
    │
    ↕ ROW_SPACING (6.7m)
```

Origin `(ORIGIN_LAT, ORIGIN_LON)` = position of Tree (row=0, tree=0).  
`ROW_BEARING` = compass heading your rows run (get from AgOpenGPS AB line).

---

## Tuning

| Setting | Default | Notes |
|---|---|---|
| `HIT_RADIUS` | 0.35m | Tighten to 0.15m with solid RTK fix |
| `RELAY_PULSE_MS` | 600ms | Adjust for your solenoid / spray nozzle |
| `COOLDOWN_MS` | 2000ms | Prevents double-trigger on same tree |

---

## Project context

Part of the [AquaControl / Sunraysia Acres](https://github.com/mandeepMildura) precision ag toolkit.  
Mildura / Sunraysia region, Victoria, Australia.
