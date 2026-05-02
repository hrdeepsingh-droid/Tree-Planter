"""
make_pdf.py  —  Generates Tree Marker System overview PDF
Output:  tree_marker_system.pdf
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, KeepTogether
)

OUT = "C:/Users/msgil/GIT/tree-marker-alr/tree_marker_system.pdf"
W, H = A4

# ── Colours ───────────────────────────────────────────────────
GREEN   = colors.HexColor("#2e7d32")
LGREEN  = colors.HexColor("#e8f5e9")
BLUE    = colors.HexColor("#1565c0")
LBLUE   = colors.HexColor("#e3f2fd")
ORANGE  = colors.HexColor("#e65100")
LORANGE = colors.HexColor("#fff3e0")
GREY    = colors.HexColor("#424242")
LGREY   = colors.HexColor("#f5f5f5")
WHITE   = colors.white

# ── Styles ────────────────────────────────────────────────────
base = getSampleStyleSheet()

def S(name, **kw):
    return ParagraphStyle(name, **kw)

title_s   = S("title_s",   fontSize=24, textColor=WHITE,   leading=30, alignment=TA_CENTER, fontName="Helvetica-Bold")
sub_s     = S("sub_s",     fontSize=11, textColor=LGREY,   leading=16, alignment=TA_CENTER, fontName="Helvetica")
h1_s      = S("h1_s",      fontSize=14, textColor=WHITE,   leading=18, fontName="Helvetica-Bold", spaceAfter=2)
h2_s      = S("h2_s",      fontSize=12, textColor=BLUE,    leading=16, fontName="Helvetica-Bold", spaceBefore=10, spaceAfter=4)
h3_s      = S("h3_s",      fontSize=10, textColor=GREY,    leading=14, fontName="Helvetica-Bold", spaceBefore=6, spaceAfter=2)
body_s    = S("body_s",    fontSize=9,  textColor=GREY,    leading=14, alignment=TA_JUSTIFY, fontName="Helvetica")
bullet_s  = S("bullet_s",  fontSize=9,  textColor=GREY,    leading=14, fontName="Helvetica",
               leftIndent=12, bulletIndent=0, spaceBefore=1)
code_s    = S("code_s",    fontSize=8,  textColor=colors.HexColor("#1a237e"),
               fontName="Courier", leading=12, leftIndent=8, backColor=LGREY)
caption_s = S("caption_s", fontSize=8,  textColor=colors.HexColor("#757575"),
               leading=11, alignment=TA_CENTER, fontName="Helvetica-Oblique")
label_s   = S("label_s",   fontSize=9,  textColor=GREY, fontName="Helvetica-Bold", leading=13)
value_s   = S("value_s",   fontSize=9,  textColor=GREY, fontName="Helvetica",      leading=13)

def rule(color=BLUE, thickness=0.5):
    return HRFlowable(width="100%", thickness=thickness, color=color, spaceAfter=6, spaceBefore=2)

def gap(n=6):
    return Spacer(1, n)

def B(text, style=bullet_s):
    return Paragraph(f"<bullet>&bull;</bullet> {text}", style)

def section_banner(text, bg=BLUE):
    data = [[Paragraph(text, h1_s)]]
    t = Table(data, colWidths=[170*mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,-1), bg),
        ("TOPPADDING",    (0,0), (-1,-1), 8),
        ("BOTTOMPADDING", (0,0), (-1,-1), 8),
        ("LEFTPADDING",   (0,0), (-1,-1), 10),
        ("RIGHTPADDING",  (0,0), (-1,-1), 10),
        ("ROUNDEDCORNERS", [4]),
    ]))
    return t

def info_box(rows, bg=LGREY, border=BLUE):
    """rows = list of (label, value) tuples"""
    data = [[Paragraph(l, label_s), Paragraph(v, value_s)] for l, v in rows]
    t = Table(data, colWidths=[55*mm, 110*mm])
    t.setStyle(TableStyle([
        ("BACKGROUND",    (0,0), (-1,-1), bg),
        ("LINEBELOW",     (0,0), (-1,-2), 0.3, colors.HexColor("#cccccc")),
        ("TOPPADDING",    (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
        ("LEFTPADDING",   (0,0), (-1,-1), 8),
        ("RIGHTPADDING",  (0,0), (-1,-1), 8),
        ("BOX",           (0,0), (-1,-1), 1, border),
    ]))
    return t

def highlight_box(text, bg=LGREEN, border=GREEN):
    data = [[Paragraph(text, body_s)]]
    t = Table(data, colWidths=[170*mm])
    t.setStyle(TableStyle([
        ("BACKGROUND",    (0,0), (-1,-1), bg),
        ("BOX",           (0,0), (-1,-1), 1, border),
        ("TOPPADDING",    (0,0), (-1,-1), 8),
        ("BOTTOMPADDING", (0,0), (-1,-1), 8),
        ("LEFTPADDING",   (0,0), (-1,-1), 10),
        ("RIGHTPADDING",  (0,0), (-1,-1), 10),
    ]))
    return t

def flow_table(rows, col_widths, header_bg=BLUE):
    data = []
    styles_list = [
        ("BACKGROUND",    (0,0), (-1,0),  header_bg),
        ("TEXTCOLOR",     (0,0), (-1,0),  WHITE),
        ("FONTNAME",      (0,0), (-1,0),  "Helvetica-Bold"),
        ("FONTSIZE",      (0,0), (-1,-1), 8),
        ("TOPPADDING",    (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
        ("LEFTPADDING",   (0,0), (-1,-1), 6),
        ("RIGHTPADDING",  (0,0), (-1,-1), 6),
        ("ROWBACKGROUNDS",(0,1), (-1,-1), [WHITE, LGREY]),
        ("GRID",          (0,0), (-1,-1), 0.3, colors.HexColor("#bdbdbd")),
        ("VALIGN",        (0,0), (-1,-1), "TOP"),
    ]
    for r in rows:
        data.append([Paragraph(str(c), body_s) for c in r])
    t = Table(data, colWidths=col_widths)
    t.setStyle(TableStyle(styles_list))
    return t

# ══════════════════════════════════════════════════════════════
#  BUILD STORY
# ══════════════════════════════════════════════════════════════
story = []

# ── Cover banner ──────────────────────────────────────────────
cover_data = [[
    Paragraph("GPS Tree Marker System", title_s),
    Paragraph("Sunraysia Acres Pty Ltd", sub_s),
    Paragraph("Visual Feedback Options — Option B &amp; C", sub_s),
    Paragraph("KML Grid Overlay  +  Real-Time Web Dashboard", sub_s),
]]
cover = Table([[
    Paragraph("GPS Tree Marker System", title_s),
]], colWidths=[170*mm])
cover.setStyle(TableStyle([
    ("BACKGROUND",    (0,0), (-1,-1), GREEN),
    ("TOPPADDING",    (0,0), (-1,-1), 18),
    ("BOTTOMPADDING", (0,0), (-1,-1), 6),
    ("LEFTPADDING",   (0,0), (-1,-1), 10),
    ("RIGHTPADDING",  (0,0), (-1,-1), 10),
]))
story.append(cover)

sub_banner_data = [
    [Paragraph("Sunraysia Acres Pty Ltd  —  Mildura, Victoria", sub_s)],
    [Paragraph("Visual Feedback Options: KML Grid Overlay + Real-Time Web Dashboard", sub_s)],
]
sub_banner = Table(sub_banner_data, colWidths=[170*mm])
sub_banner.setStyle(TableStyle([
    ("BACKGROUND",    (0,0), (-1,-1), colors.HexColor("#1b5e20")),
    ("TOPPADDING",    (0,0), (-1,-1), 5),
    ("BOTTOMPADDING", (0,0), (-1,-1), 10),
    ("LEFTPADDING",   (0,0), (-1,-1), 10),
    ("RIGHTPADDING",  (0,0), (-1,-1), 10),
]))
story.append(sub_banner)
story.append(gap(12))

# ── 1. Project Overview ───────────────────────────────────────
story.append(section_banner("1.  Project Overview"))
story.append(gap(6))
story.append(Paragraph(
    "The Tree Marker system is a GPS-triggered paint marker that automatically fires a solenoid "
    "every time the tractor crosses a pre-calculated tree planting position. It piggybacks the "
    "existing AgOpenGPS RTK-GPS setup already on the tractor — no second GPS unit is needed.",
    body_s))
story.append(gap(8))

story.append(info_box([
    ("Farm",           "Sunraysia Acres Pty Ltd, Mildura / Sunraysia, Victoria"),
    ("Tractor",        "1994–1997 Case IH 3230, 12V, hydrostatic power steering"),
    ("GPS accuracy",   "Ardusimple SimpleRTK2B (u-blox F9P), RTK fix, ~2 cm"),
    ("Controller",     "KinCony ALR board (ESP32-S3) — mounts on tractor"),
    ("Marker",         "Existing electric paint solenoid, 12V — relay wired in series"),
    ("Row spacing",    "6.7 m between rows"),
    ("Tree spacing",   "3.0 m between trees within each row"),
    ("Grid size",      "10 rows × 50 trees = 500 tree positions per field"),
]))
story.append(gap(10))

# Data flow
story.append(Paragraph("How it works — data flow", h3_s))
flow_rows = [
    ["Step", "What happens", "Device"],
    ["1", "F9P RTK GPS locks to ~2 cm fix", "Ardusimple on AIO board"],
    ["2", "AIO Teensy 4.1 forwards NMEA over Ethernet UDP → port 9999", "AIO v4.5 Standard"],
    ["3", "AgIO (Windows tablet) receives position — also broadcasts to subnet", "Tablet in cab"],
    ["4", "KinCony ALR receives $GNGGA sentence over WiFi UDP", "ALR board on tractor"],
    ["5", "ALR compares GPS position against pre-calculated tree grid", "ESP32-S3 firmware"],
    ["6", "Within 35 cm of a tree point → GPIO 48 HIGH for 600 ms", "ALR relay"],
    ["7", "Relay fires solenoid → paint mark on ground", "Paint sprayer"],
]
story.append(flow_table(flow_rows, [10*mm, 100*mm, 55*mm]))
story.append(gap(4))
story.append(Paragraph(
    "The tree grid is pre-calculated from a field survey origin point and stored in firmware. "
    "Grid maths uses flat-earth distance — accurate to &lt;1 mm at orchard scales.",
    caption_s))
story.append(gap(14))

# ── 2. Why Visual Feedback ────────────────────────────────────
story.append(section_banner("2.  Why Visual Feedback Matters", bg=GREEN))
story.append(gap(6))
story.append(Paragraph(
    "The relay fires automatically and the OLED on the ALR board confirms each hit. However, "
    "for a commercial planting run across 500+ positions, the operator needs two additional things:",
    body_s))
story.append(gap(6))

needs_rows = [
    ["Need", "Without feedback", "With Options B + C"],
    ["Pre-run check",
     "No way to verify grid alignment before starting",
     "KML overlay in Google Earth confirms grid matches real field boundaries"],
    ["During run",
     "Operator can't tell if a mark was missed",
     "Live web dashboard shows each hit in real time — gaps are immediately visible"],
    ["Post-run record",
     "No permanent record of what was marked",
     "Dashboard hit history + MQTT log = permanent GPS-stamped record of every mark"],
    ["Missed-tree recovery",
     "Must re-drive entire row",
     "Dashboard shows exactly which tree was missed — drive back to that point only"],
]
story.append(flow_table(needs_rows, [30*mm, 65*mm, 70*mm]))
story.append(gap(14))

# ── 3. Option C — KML ─────────────────────────────────────────
story.append(section_banner("3.  Option C — KML Tree Grid Overlay", bg=ORANGE))
story.append(gap(6))
story.append(highlight_box(
    "<b>What it is:</b>  A standard KML file containing all 500 tree positions as placemarks "
    "and 10 row lines. Opens in Google Earth on any device. Used for pre-run alignment checks "
    "and post-run review. The file is auto-generated from the same grid parameters as the "
    "firmware — they are always in sync.",
    bg=LORANGE, border=ORANGE))
story.append(gap(8))

story.append(Paragraph("What the KML contains", h3_s))
story.append(B("500 yellow dot placemarks — one per tree, labelled R0T0 … R9T49"))
story.append(B("10 green row lines — one per row, showing direction of travel"))
story.append(B("Document name and grid parameters embedded in description"))
story.append(gap(8))

story.append(Paragraph("How to use it — pre-run alignment check", h3_s))
pre_rows = [
    ["Step", "Action"],
    ["1", "Open tree_grid.kml in Google Earth on phone or tablet"],
    ["2", "Navigate to the field — the 500 dots appear on satellite imagery"],
    ["3", "Compare dot positions against existing tree rows / fence lines"],
    ["4", "If alignment is off, adjust ORIGIN_LAT, ORIGIN_LON or ROW_BEARING in firmware and regenerate"],
    ["5", "When dots match reality, origin and bearing are confirmed — proceed to field run"],
]
story.append(flow_table(pre_rows, [10*mm, 155*mm]))
story.append(gap(8))

story.append(Paragraph("How to use it — post-run review", h3_s))
story.append(B("Open the KML in Google Earth after the run"))
story.append(B("Cross-reference with the web dashboard hit history (Option B) to identify any missed positions"))
story.append(B("Screenshot the overlay for farm records / neighbour sharing"))
story.append(gap(8))

story.append(Paragraph("Important note on coordinates", h3_s))
story.append(highlight_box(
    "The KML currently uses <b>placeholder coordinates</b> (origin -34.200000, 142.150000). "
    "Before the farm run, a field survey must establish the real ORIGIN_LAT, ORIGIN_LON, and "
    "ROW_BEARING from the actual orchard. Once those are measured and entered, re-run "
    "<i>make_kml.py</i> (one command) and the KML updates automatically.",
    bg=LORANGE, border=ORANGE))
story.append(gap(8))

story.append(Paragraph("Field survey procedure", h3_s))
survey_rows = [
    ["#", "Measurement", "Tool", "Where to enter"],
    ["1", "Stand the tractor at tree position (0,0) — first tree of first row. "
          "Record lat/lon from AgOpenGPS display.",
     "AgOpenGPS + F9P RTK", "ORIGIN_LAT, ORIGIN_LON in firmware"],
    ["2", "Drive to the end of row 0 — last tree. Note the AgOpenGPS AB line bearing.",
     "AgOpenGPS AB line bearing readout", "ROW_BEARING in firmware (degrees, 0=N, 90=E)"],
    ["3", "Count actual rows and trees if different from 10×50.",
     "Walk the field", "NUM_ROWS, NUM_TREES in firmware"],
]
story.append(flow_table(survey_rows, [6*mm, 70*mm, 44*mm, 45*mm]))
story.append(gap(14))

# ── 4. Option B — Web Dashboard ───────────────────────────────
story.append(section_banner("4.  Option B — Real-Time Web Dashboard", bg=BLUE))
story.append(gap(6))
story.append(highlight_box(
    "<b>What it is:</b>  A single HTML file that opens in any browser (phone, tablet, laptop). "
    "It connects to the HiveMQ MQTT broker over a secure WebSocket and displays a live map. "
    "Every time the relay fires, a green marker drops on the map at the exact GPS coordinates "
    "of the hit. No app installation required.",
    bg=LBLUE, border=BLUE))
story.append(gap(8))

story.append(Paragraph("How it works", h3_s))
tech_rows = [
    ["Component", "Technology", "Role"],
    ["Map display",    "Leaflet.js (open-source, offline-capable)", "Renders satellite/street tile map with tree dots"],
    ["Live updates",   "MQTT over WebSocket (port 8884, TLS)",      "Receives hit events from HiveMQ Cloud in real time"],
    ["Broker",         "HiveMQ Cloud — eb65c13e....hivemq.cloud",   "Relays MQTT messages from ALR board to browser"],
    ["Planned trees",  "GeoJSON generated from grid parameters",    "All 500 positions shown as grey dots on load"],
    ["Hit markers",    "Green Leaflet markers, timestamped",         "Drop on map as each relay fires"],
    ["No server",      "Pure browser HTML/JS — no backend needed",  "Open the file directly, works on any device"],
]
story.append(flow_table(tech_rows, [28*mm, 62*mm, 75*mm]))
story.append(gap(8))

story.append(Paragraph("What you see on the dashboard", h3_s))
story.append(B("<b>Grey dots</b> — all 500 planned tree positions (loaded at startup from grid maths)"))
story.append(B("<b>Green markers</b> — confirmed hits, appearing in real time as the tractor drives"))
story.append(B("<b>Popup on click</b> — shows Row, Tree, distance from centre, timestamp, lat/lon"))
story.append(B("<b>Hit counter</b> — total marks fired this session"))
story.append(B("<b>Connection indicator</b> — green/red dot showing MQTT link status"))
story.append(gap(8))

story.append(Paragraph("How to use it during a run", h3_s))
run_rows = [
    ["Step", "Action"],
    ["1", "Ensure phone/tablet is on the same WiFi hotspot as the ALR board"],
    ["2", "Open tree_marker_dashboard.html in Chrome or Safari — map loads automatically"],
    ["3", "Drive the first row — green markers should appear every 3.0 m"],
    ["4", "If a grey dot stays grey after passing it, that tree was missed — note the Row/Tree number"],
    ["5", "After completing all rows, screenshot the dashboard for farm records"],
    ["6", "Hit history is also in HiveMQ Cloud — accessible from any browser after the run"],
]
story.append(flow_table(run_rows, [10*mm, 155*mm]))
story.append(gap(8))

story.append(Paragraph("MQTT message format (what the board sends)", h3_s))
story.append(Paragraph('Topic:  <font name="Courier">treemarker/hit</font>', body_s))
story.append(gap(3))
story.append(Paragraph(
    '{"row":3, "tree":12, "dist":0.18, "lat":-34.3171234, "lon":142.1562345, "hits":37}',
    code_s))
story.append(gap(4))
story.append(Paragraph(
    "Every hit publishes one JSON message. The dashboard subscribes to this topic and plots "
    "each message as a map marker. The full hit log is retained in HiveMQ for post-run review.",
    body_s))
story.append(gap(14))

# ── 5. Combined Workflow ───────────────────────────────────────
story.append(section_banner("5.  Combined Workflow — A to Z", bg=GREEN))
story.append(gap(6))
workflow_rows = [
    ["Phase", "Task", "Tool", "Status"],
    ["Field survey",
     "Drive to tree (0,0) with RTK lock. Record lat/lon + AB line bearing.",
     "AgOpenGPS on tablet", "Pending — do before farm run"],
    ["Firmware update",
     "Enter real ORIGIN_LAT, ORIGIN_LON, ROW_BEARING, NUM_ROWS, NUM_TREES. Flash board.",
     "Arduino IDE / arduino-cli", "Pending — after survey"],
    ["KML update",
     "Re-run make_kml.py with real coordinates. New KML in 5 seconds.",
     "Python script in repo", "Pending — after survey"],
    ["Pre-run check",
     "Open updated KML in Google Earth. Confirm 500 dots match real field.",
     "Google Earth on phone", "Pending — before run"],
    ["Dashboard open",
     "Open tree_marker_dashboard.html in browser on phone.",
     "Any browser", "Ready — build in progress"],
    ["Farm run",
     "Drive rows. Relay fires at each tree. Dashboard shows live hits.",
     "Tractor + ALR board", "Pending — farm visit"],
    ["Post-run review",
     "Screenshot dashboard. Cross-check KML vs green hit markers.",
     "Dashboard + Google Earth", "Pending — after run"],
    ["Records",
     "Full hit log in HiveMQ Cloud — row, tree, GPS coords, timestamp for every mark.",
     "HiveMQ Cloud console", "Automatic — already active"],
]
story.append(flow_table(workflow_rows, [24*mm, 65*mm, 45*mm, 31*mm]))
story.append(gap(14))

# ── 6. System Specs ───────────────────────────────────────────
story.append(section_banner("6.  System Specifications", bg=GREY))
story.append(gap(6))

col1 = [
    ("Firmware version",  "v1.0.0"),
    ("Board",             "KinCony ALR — ESP32-S3-WROOM-1U N16R8"),
    ("Relay",             "GPIO 48, AC250V / 10A, volt-free contact"),
    ("OLED",              "SSD1306 128×64, I2C (SDA 39, SCL 38)"),
    ("WiFi",              "2.4 GHz 802.11n — connects to phone hotspot"),
    ("UDP input",         "Port 9999 — $GNGGA from AgIO broadcast"),
    ("Hit radius",        "0.35 m from tree centre"),
    ("Relay pulse",       "600 ms ON per hit"),
    ("Cooldown",          "2000 ms minimum between same-spot hits"),
]
story.append(info_box(col1))
story.append(gap(8))

col2 = [
    ("MQTT broker",       "HiveMQ Cloud (TLS port 8883)"),
    ("Broker host",       "eb65c13ec8ab480a9c8492778fdddda8.s1.eu.hivemq.cloud"),
    ("MQTT client ID",    "tree-marker-alr-01"),
    ("Status topic",      "treemarker/status  (heartbeat every 30 s)"),
    ("Hit topic",         "treemarker/hit  (JSON per relay fire)"),
    ("OTA topic",         "treemarker/ota/url  (send GitHub .bin URL to update)"),
    ("Restart topic",     "treemarker/restart  (any payload reboots board)"),
    ("GitHub repo",       "github.com/mandeepmildura/tree-marker-alr"),
]
story.append(info_box(col2))
story.append(gap(14))

# ── 7. Next Steps ─────────────────────────────────────────────
story.append(KeepTogether([
    section_banner("7.  Next Steps", bg=GREEN),
    gap(6),
    flow_table([
        ["#", "Task", "Notes"],
        ["1", "Complete field survey at Mildura farm",
         "Measure ORIGIN_LAT/LON and ROW_BEARING with RTK fix"],
        ["2", "Update firmware config + reflash",
         "Use OTA update over MQTT — no USB cable needed at farm"],
        ["3", "Regenerate KML with real coordinates",
         "Run make_kml.py — takes 5 seconds"],
        ["4", "Open KML in Google Earth — verify grid alignment",
         "Do this before starting the planting run"],
        ["5", "Open web dashboard on phone",
         "Confirm MQTT connection indicator is green"],
        ["6", "Drive first row slowly — confirm relay fires every 3.0 m",
         "Watch dashboard for green hits and listen for relay clicks"],
        ["7", "Proceed with full planting run",
         "Dashboard provides real-time hit confirmation"],
    ], [8*mm, 80*mm, 77*mm]),
]))
story.append(gap(8))
story.append(Paragraph(
    "Document prepared by Claude Code  —  April 2026  —  For internal use, Sunraysia Acres Pty Ltd",
    caption_s))

# ── Build ─────────────────────────────────────────────────────
doc = SimpleDocTemplate(
    OUT, pagesize=A4,
    leftMargin=20*mm, rightMargin=20*mm,
    topMargin=16*mm, bottomMargin=16*mm,
)
doc.build(story)
print("Written:", OUT)
