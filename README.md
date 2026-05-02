# TreeMarker

GPS-triggered relay mark-out system for precision tree planting.
The ESP32 receives a field grid via WiFi/UDP, tracks the nozzle position in real time
using RTK GPS, and fires a relay (dye sprayer) the instant the nozzle crosses each tree position.

---

## Files

```
TreeMarker/
├── TreeMarker_ESP32/
│   └── TreeMarker_ESP32.ino   ESP32 Arduino sketch
├── send_field_data.py          Python field data sender (run on tablet/laptop)
└── README.md
```

---

## Wiring

```
ESP32 Dev Module
┌─────────────────────────────────────────────────────┐
│  GPIO 16 (RX2)  ←──[1kΩ]──[2kΩ to GND]──  GPS TX  │  voltage divider for 5V GPS modules
│  GPIO 17 (TX2)  ──────────────────────────  GPS RX  │  (only needed if sending config to GPS)
│                                                      │
│  GPIO 26        ──────────────────────────  Relay IN │  active HIGH
│  5V / Vin       ──────────────────────────  Relay VCC│
│  GND            ──────────────────────────  Relay GND│
└─────────────────────────────────────────────────────┘

Relay module wiring:
  COM  ──  12V +ve
  NO   ──  Solenoid +ve
           Solenoid –ve  ──  12V GND (common with ESP32 GND)

NOTE: Use an optocoupler relay module to isolate the ESP32 from
      inductive kickback from the solenoid.
```

GPS baud rate default: **9600**. Change `Serial2.begin(9600, ...)` in `setup()` if your GPS module uses a different rate. The system reads `$GNGGA` (position + fix quality) and `$GNRMC` (heading + speed).

---

## Arduino IDE Setup

### 1. Add ESP32 board support
1. **File → Preferences** → paste into *Additional Board Manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Tools → Board → Boards Manager** → search `esp32` → install **esp32 by Espressif Systems** (v2.x)

### 2. Install libraries
Open **Sketch → Include Library → Manage Libraries** and install:

| Library | Author | Version |
|---------|--------|---------|
| ArduinoJson | Benoit Blanchon | **6.x** (not v7) |
| WebSockets | Markus Sattler | latest |

### 3. Select board settings
- **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
- **Tools → Port** → your ESP32 COM port
- **Tools → Upload Speed** → 921600

---

## First Boot

1. Create a **2.4 GHz** hotspot on your tablet:
   - SSID: `AgOpenGPS`
   - Password: `treeplant`
2. Power the ESP32. The LED blinks while connecting.
3. Open **Serial Monitor** at **115200 baud**. You will see the IP address:
   ```
   WiFi OK  IP=192.168.43.100
   Open browser: http://192.168.43.100/
   ```
4. Open that URL in a browser on the tablet. The green settings page should load.
5. Navigate to **WiFi** to change the network if needed.

If the LED blinks 20 times and stops, WiFi failed. The ESP32 falls back to AP mode:
- SSID: `TreeMarker-Setup` / Password: `setup1234`
- Connect your device to that AP and open `http://192.168.4.1/`

---

## Daily Workflow — AgOpenGPS Mode (Option 1)

1. Open AgOpenGPS and create two AB lines for the field:
   - One along the **row direction** — name it exactly `Row Direction`
   - One along the **tree in-row direction** — name it exactly `Tree Direction`
   - (Names must match the **AB Line Names** settings on the ESP32 page)
2. Edit `send_field_data.py`: set `FIELD_FOLDER` to your AgOpenGPS field folder and `ESP32_IP` to the IP shown in Serial Monitor.
3. Run `python send_field_data.py` → choose **Option 1**.
4. The script reads `Field.txt`, `ABLines.txt`, and optionally `Boundary.txt`, then sends to the ESP32.
5. Check the ESP32 settings page — **Trees** count should now show the grid size.
6. Begin planting. The relay fires each time the nozzle crosses a tree position.
7. Watch the **Live Map** page (`http://<ESP32-IP>/map`) for real-time guidance.

---

## Daily Workflow — AutoCAD DXF Mode (Options 2 & 3)

### Option 2 — Tree positions only
1. Export your tree layout as a DXF with:
   - `CIRCLE` entities at tree centres, and/or
   - `LINE` / `LWPOLYLINE` / `POLYLINE` entities forming the planting grid
2. Set `DXF_FILE` in `send_field_data.py`.
3. Set `DXF_REAL_WORLD_COORDS = True` if the DXF is in UTM coordinates, `False` if already in local (AgOpenGPS-relative) coordinates.
4. Run the script → choose **Option 2**. Confirm the sample points, then send.

### Option 3 — Tree positions + auto-generate AB lines
1. Same DXF requirements as Option 2.
2. Run the script → choose **Option 3**.
3. The script automatically detects two dominant perpendicular directions by clustering line segment angles into 1-degree bins and finding the two highest-weighted peaks. The group with larger median spacing is assigned as rows, the other as trees.
4. It writes **`ABLines_generated.txt`** to the same folder as the DXF file.
5. **Copy `ABLines_generated.txt` to your AgOpenGPS field folder** (the path in `FIELD_FOLDER`).
6. In AgOpenGPS, set **Tool Width = row spacing** (printed by the script) so parallel guidance tracks generate at the correct row spacing.
7. Then run the script → Option 1 (or the ESP32 already has the points from Option 3).

---

## Map Page

Open **`http://<ESP32-IP>/map`** in a browser on the same WiFi network.

### Colour coding
| Element | Appearance |
|---------|-----------|
| Current row line | Bright green horizontal line through centre |
| Adjacent rows (±3) | Dark green horizontal lines |
| Planned tree positions | Small white circles |
| Already-fired marks | Filled yellow circles |
| Nozzle position | Filled green triangle pointing in direction of travel |
| GPS antenna position | Small grey dot |

### Guidance panels (below the canvas)

| Panel | Meaning |
|-------|---------|
| **Cross-Track** | Perpendicular distance from nozzle to the current row line. Green ≤ 0.05 m, Yellow ≤ 0.15 m, Red > 0.15 m. LEFT/RIGHT tells which side to steer. |
| **Next Mark** | Distance along the current row to the next un-fired tree position. |
| **Heading Error** | Deviation of actual travel heading from row direction. LEFT = steer left, RIGHT = steer right. |
| **Speed** | Current speed in km/h from GPS. |
| **Total Marks** | Running count of relay fires this session. |

Use the cross-track panel for manual row-following: keep it green (< 5 cm) for accurate placement.

---

## Calibration Procedure (Fore/Aft Offset)

The fore/aft offset corrects for the physical distance between the GPS antenna and the spray nozzle.

1. Drive slowly along one row heading approximately **north**, firing marks.
2. Return on the **same row** heading south, firing marks on the same tree positions.
3. Measure the distance between the north-pass mark and south-pass mark at the same tree.
4. **Offset error = that distance ÷ 2**.
5. Adjust **Fore/Aft (m)** on the Settings page by that amount.
   - If the mark fires *late* (behind target) in both directions → **increase** fore/aft.
   - Positive fore/aft = nozzle is physically behind the GPS antenna.
6. Repeat until north and south pass marks coincide.

Use the **Calibration Log** table (bottom of Settings page) for precise measurement — it shows the local easting/northing of the last 20 fire positions, so you can compare coordinates directly without a tape measure.

---

## Uploading to ESP32

1. Use a **USB data cable** (not a charge-only cable). Many phone cables are charge-only — if no COM port appears, try a different cable.
2. Install the correct USB driver for your ESP32 board:
   - **CP2102 chip** (most dev boards): [silabs.com/developers/usb-to-uart-bridge-vcp-drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
   - **CH340 chip** (some cheap boards): [wch-ic.com/downloads](http://www.wch-ic.com/downloads/CH341SER_EXE.html)
3. Select **Tools → Board → ESP32 Dev Module** and the correct COM port.
4. Click **Upload** (→).
5. **If upload fails with "Connecting…"**: hold the **BOOT button** on the ESP32 while clicking Upload, release after the upload starts.

---

## How Minimum-Distance Triggering Works

Traditional radius triggers fire as soon as the nozzle enters a circle around a tree. At speed this causes early firing because the nozzle is still approaching.

TreeMarker uses **closest-point detection**:

1. Every GPS fix updates the nozzle position.
2. The nearest un-fired tree is found.
3. When the nozzle comes within **0.5 m** of that tree, the system *arms*.
4. While armed, the running minimum distance is tracked each fix.
5. When the distance starts *increasing* (the nozzle has just passed the closest point), the relay fires.

The spray mark therefore lands precisely at the calculated tree position regardless of speed, with no radius to tune. The only accuracy-limiting factor is the fore/aft offset calibration.

---

## How UTM Conversion Works and Why It Matches AgOpenGPS

AgOpenGPS uses **UTM (Universal Transverse Mercator)** coordinates to represent field positions. It reads a `Field.txt` file that contains the UTM zone, and easting/northing offsets. All internal positions are stored as offsets from those values, so coordinates stay small (hundreds of metres rather than millions).

TreeMarker applies the same WGS84 → UTM formula using the same zone and offsets received from `send_field_data.py`. Because the math is identical, a tree position exported from AgOpenGPS and a GPS fix on the ESP32 are directly comparable in the same coordinate space.

The formula handles the southern hemisphere correctly by adding **10,000,000 m** to the northing when latitude is negative (the standard false northing for southern UTM zones).

---

## How Automatic Direction Detection Works (Option 3)

1. Every `LINE`, `LWPOLYLINE`, and `POLYLINE` entity in the DXF is extracted as a set of segments.
2. Each segment's bearing (0–180°) is calculated and added to a 1-degree histogram, weighted by segment length (longer lines have more influence).
3. The **strongest peak** in the histogram is the dominant planting direction.
4. The script finds the second peak that is **most perpendicular** (closest to 90° away) from the first — this is the other planting direction.
5. For each group, segment midpoints are projected onto the perpendicular axis and clustered. The **median gap between clusters** gives the line spacing.
6. The group with **larger spacing = rows**, smaller spacing = trees. If the two spacings are within 20% of each other the script asks the user to confirm which is which.
7. The **longest segment** in each group becomes the reference AB line, with its start point as the origin and its bearing as the heading.
8. The result is written to `ABLines_generated.txt` in AgOpenGPS format, ready to copy to the field folder.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No COM port in Arduino IDE | Wrong cable or missing driver | Try a different USB cable; install CP2102 or CH340 driver |
| Upload fails "Connecting…" | Boot mode not triggered | Hold BOOT button on ESP32 during upload start |
| Settings page won't load | Wrong IP or different network | Check Serial Monitor for IP; ensure device is on same WiFi |
| No marks placed | No GPS fix or no grid loaded | Check Settings page GPS quality field; run `send_field_data.py` |
| Marks at wrong position | Fore/aft offset wrong | Run bidirectional calibration; adjust Fore/Aft on Settings page |
| Marks firing twice at same tree | Cooldown too short | Increase Cooldown on Settings page |
| Map page blank canvas | No GPS fix yet | Drive slowly until GPS fix quality > 0 |
| WebSocket "Connecting…" stays | Port 81 blocked | Check firewall/hotspot settings; ensure device is on same network |
| Python: Field.txt not found | Wrong FIELD_FOLDER path | Edit `FIELD_FOLDER` at top of `send_field_data.py` |
| Python: ezdxf not found | Library missing | Run `pip install ezdxf` |
| AB lines mode: 0 trees built | Name mismatch | Check AB Line Names on Settings page; names are case-sensitive |
| Option 3: ambiguous spacings | Grid has equal row/tree spacing | Script will prompt; choose which direction is rows |
