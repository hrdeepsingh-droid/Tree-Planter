# TreeMarker

ESP32-based tree planting mark-out system. The ESP32 receives a grid of tree positions from a tablet via WiFi/UDP, tracks the GPS nozzle position in real time, and fires a relay (dye sprayer) the instant the nozzle passes each tree position.

---

## Contents

```
TreeMarker/
├── TreeMarker/
│   └── TreeMarker.ino   ESP32 Arduino sketch
├── sender.py            Python field data sender (run on tablet/laptop)
└── README.md            This file
```

---

## Wiring

```
ESP32 Pin   →   Device
──────────────────────────────────────────────────────────
GPIO 26     →   Relay module IN (active HIGH)
GPIO  2     →   On-board LED (built-in, no external wiring needed)
GPIO 16     →   GPS module TX  (ESP32 RX)
GPIO 17     →   GPS module RX  (ESP32 TX – only needed if sending RTCM)
3.3 V       →   GPS module VCC  (check module datasheet – some need 5 V)
GND         →   GPS module GND, Relay GND, common ground

Relay common/NO  →  Dye sprayer solenoid or pump
External 12 V   →  Sprayer supply (relay isolates from ESP32 logic)
```

> **Important:** use an optocoupler relay module (not a bare relay coil) to protect the ESP32 from inductive kickback.

GPS baud rate defaults to **9600**. If your GPS module uses a different baud rate change `Serial2.begin(9600, ...)` in `setup()`.

---

## Arduino IDE Setup

### 1. Install ESP32 board support

1. Open **File → Preferences** and add this URL to *Additional Board Manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. Open **Tools → Board → Boards Manager**, search **esp32**, install **esp32 by Espressif Systems** (version 2.x or later).

### 2. Install ArduinoJson

1. Open **Sketch → Include Library → Manage Libraries**.
2. Search **ArduinoJson**, select version **6.x** by Benoit Blanchon, click **Install**.

> All other libraries (Preferences, WebServer, WiFi, WiFiUDP) are included with the ESP32 board package.

### 3. Select board and port

- **Tools → Board → ESP32 Arduino → ESP32 Dev Module** (or your specific board variant)
- **Tools → Port** → select the ESP32 COM port
- **Tools → Upload Speed** → 921600 recommended

### 4. Upload

Open `TreeMarker/TreeMarker.ino` and click **Upload** (→).

---

## First Boot Procedure

1. Power the ESP32. The onboard LED will blink while connecting to WiFi.
2. The sketch attempts to join **AgOpenGPS** (password **treeplant**) — the compile-time defaults.
3. If connection fails within 20 seconds the ESP32 falls back to **AP mode** (`TreeMarker-Setup`, password `setup1234`). Connect your phone/laptop to that AP.
4. Open a browser and navigate to `http://192.168.4.1` (AP mode) or the IP shown in the Serial Monitor.
5. Go to **WiFi** and enter your field network credentials. The device reboots and connects.
6. Verify the status page shows the IP address — note it down. You'll need it in `sender.py` as `ESP32_IP`.

---

## Daily Field Workflow

### AgOpenGPS mode

1. In AgOpenGPS, create two AB lines:
   - One aligned with the **row direction** (default name *Row Direction*)
   - One aligned with the **tree in-row direction** (default name *Tree Direction*)
   - Names must match the **AB Line Names** settings on the ESP32 web UI.
2. On the tablet/laptop, edit `sender.py`:
   - Set `FIELD_FOLDER` to your AgOpenGPS field folder (containing `Field.txt` and `ABLines.txt`).
   - Set `ESP32_IP` to the ESP32's IP address.
3. Run `python sender.py`, choose **Mode 1**.
4. The ESP32 receives the AB lines, builds the intersection grid, and the status page shows the tree count.
5. Begin planting. The relay fires each time the nozzle passes a tree position.

### AutoCAD DXF mode

1. Export your tree layout as a DXF containing:
   - `CIRCLE` entities for individual tree centres, and/or
   - `LINE`, `LWPOLYLINE`, or `POLYLINE` entities forming a planting grid (intersections become tree positions).
2. Edit `sender.py`:
   - Set `DXF_FILE` to the DXF path.
   - Set `DXF_REAL_WORLD_COORDS = True` if the DXF is in UTM coordinates, `False` if already in AgOpenGPS local coordinates.
   - Set `FIELD_FOLDER` (still needed for the UTM offsets in `Field.txt`).
3. Run `python sender.py`, choose **Mode 2**.
4. Confirm the sample points look correct, then send.

---

## Settings Reference (Web UI)

Navigate to `http://<ESP32-IP>/` in a browser.

| Page | Setting | Default | Description |
|------|---------|---------|-------------|
| Spray | Relay ON (ms) | 800 | How long the relay stays energised per trigger |
| Spray | Cooldown (ms) | 3000 | Minimum time between consecutive fires |
| Nozzle | Fore/Aft offset (m) | 2.0 | Distance nozzle is behind GPS antenna. Positive = behind. |
| Nozzle | Lateral offset (m) | 0.0 | Distance nozzle is right of antenna centreline. Positive = right. |
| Grid | Row spacing (m) | 6.7 | Distance between planting rows |
| Grid | Tree spacing (m) | 3.0 | Distance between trees along a row |
| Grid | Lines each side | 40 | Grid extends ±N rows/columns from the AB origin |
| AB Names | Row AB line name | Row Direction | Must match AgOpenGPS AB line name exactly |
| AB Names | Tree AB line name | Tree Direction | Must match AgOpenGPS AB line name exactly |
| WiFi | SSID / Password | AgOpenGPS / treeplant | Field network credentials |

All settings survive power cycles (stored in NVS flash).

---

## How Minimum-Distance Triggering Works

Traditional radius triggers fire as soon as the nozzle enters a circle around each tree. This causes early firing at speed because the nozzle is still approaching the position.

TreeMarker uses **minimum-distance (closest-point) detection**:

1. Each GPS update computes the nozzle's local position (antenna position shifted by fore/aft and lateral offsets).
2. The nearest tree position is found.
3. When the nozzle comes within **0.5 m** of that tree, the system *arms*.
4. While armed, the running minimum distance is tracked.
5. The relay fires the moment the distance starts *increasing* (i.e., the nozzle has just passed the closest point).

This means the spray mark lands exactly at the calculated tree position regardless of speed, with no radius parameter to tune. The only accuracy-limiting factor is the fore/aft offset calibration.

---

## Calibration Procedure (Fore/Aft Offset)

The fore/aft offset compensates for the physical distance between the GPS antenna and the spray nozzle.

1. Set up a string line across the field at a known position.
2. Drive slowly along a row crossing the string, heading approximately **north**.
3. Note where the spray mark lands relative to the string. Call this error A.
4. Drive the same line heading **south**.
5. Note where the spray mark lands. Call this error B.
6. The true fore/aft error = **(A + B) / 2** (half the distance between the two marks).
7. Adjust *Fore/Aft offset* in the web UI by this amount. If the mark was consistently late (behind target in both directions), increase the offset.
8. Repeat until both-direction marks coincide.

The **Calibration Log** on the status page shows the last 20 nozzle positions at fire time in local coordinates — useful for comparing bidirectional runs.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| LED blinking, no web UI | Connecting to WiFi | Wait 10 s; if still blinking, use AP mode to change WiFi credentials |
| Status shows 0 tree positions | Field data not sent yet | Run `sender.py` with the ESP32 powered and on the same network |
| AB lines received but 0 trees | AB line names don't match | Check **AB Names** page; names are case-sensitive |
| Spray fires at wrong position | Fore/aft offset wrong | Run bidirectional calibration (see above) |
| Spray fires twice per tree | Cooldown too short | Increase **Cooldown** on the Spray page |
| Spray never fires | GPS not valid, or nozzle offset too large | Check status page for GPS valid status; verify offset values |
| Web UI unreachable | ESP32 not on same subnet | Check ESP32 IP in Serial Monitor; ensure tablet is on same WiFi |
| Large DXF sends slowly | Many pairwise intersections | Reduce segment count in DXF, or pre-explode to points in AutoCAD |
| `ezdxf` not found | Library not installed | Run `pip install ezdxf` |

---

## Field.txt Format (AgOpenGPS)

The Python sender reads these keys (case-insensitive):

```
UTMZone=54
Easting=500000.0
Northing=6200000.0
```

## ABLines.txt Format

Each row (comma-separated, no header):

```
Row Direction,90.0,1234.56,5678.90
Tree Direction,0.0,1234.56,5678.90
```

Columns: **Name, Heading (degrees true), Easting (local), Northing (local)**

## Boundary.txt Format

Each row (comma-separated, no header):

```
100.0,200.0
150.0,210.0
…
```

Columns: **Easting (local), Northing (local)**
The boundary is automatically closed (last point connects to first).
