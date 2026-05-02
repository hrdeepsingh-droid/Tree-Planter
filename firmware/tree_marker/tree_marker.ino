// ================================================================
//  Tree Row Marker — KinCony ALR (ESP32-S3)
//  Receives RTK position from AgOpenGPS via WiFi UDP
//  Fires on-board relay at each tree grid intersection
//
//  Repo:  https://github.com/mandeepMildura/tree-marker-alr
//  Board: KinCony ALR  https://www.kincony.com/esp32-lora-sx1278-gateway.html
//  IDE:   ESP32S3 Dev Module  (esp32 by Espressif, 2.x)
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <DNSServer.h>

// ── Firmware version (bumped on each release) ─────────────────
#define FW_VERSION "1.4.12"

// ================================================================
//  COMPILED-IN DEFAULTS — overridden by Preferences after first save
// ================================================================
#define DEF_WIFI_SSID     "Mandeep"
#define DEF_WIFI_PASS     "rockyjungle161"
#define DEF_MQTT_HOST     "eb65c13ec8ab480a9c8492778fdddda8.s1.eu.hivemq.cloud"
#define DEF_MQTT_PORT     8883
#define DEF_MQTT_USER     "mandeep"
#define DEF_MQTT_PASS     "Gill@1977"
#define DEF_MQTT_CLIENT   "tree-marker-alr-01"
#define DEF_ORIGIN_LAT    -34.3166591
#define DEF_ORIGIN_LON    142.1562539
#define DEF_ROW_BEARING   180.0
#define DEF_ROW_SPACING   6.7
#define DEF_TREE_SPACING  3.0
#define DEF_NUM_ROWS      10
#define DEF_NUM_TREES     50
#define DEF_HIT_RADIUS    0.35
#define DEF_RELAY_PULSE   600

// Antenna → nozzle offset (v1.4.5). Both default to 0 so existing
// setups behave identically to v1.4.4 until calibrated.
#define DEF_FORE_AFT      0.0       // metres: positive = nozzle BEHIND antenna
#define DEF_LATERAL       0.0       // metres: positive = nozzle RIGHT of antenna centreline

// ── Grid mode ─────────────────────────────────────────────────
#define MODE_SIMPLE  0      // origin + bearing + spacings (bench/sim)
#define MODE_AB      1      // cross-AB lines imported from AgOpenGPS

// Default AB-line names (match whatever you saved in AgOpenGPS)
#define DEF_ROW_NAME   "Row"
#define DEF_TREE_NAME  "Tree"

// Max intersections we pre-compute and store in RAM.
// 2000 × 8 bytes = 16KB — comfortable on ESP32-S3.
#define MAX_INTERSECTIONS 2000

// Max boundary polygon vertices. 256 × 8 bytes = 2KB, and typical
// orchard boundaries have <100 points.
#define MAX_BOUNDARY 256

// ── AgIO UDP ──────────────────────────────────────────────────
#define UDP_PORT  8888

// ── MQTT topics ───────────────────────────────────────────────
#define T_STATUS      "treemarker/status"
#define T_HIT         "treemarker/hit"
#define T_OTA_URL     "treemarker/ota/url"
#define T_OTA_STATUS  "treemarker/ota/status"
#define T_RESTART     "treemarker/restart"

// ── Pin map ───────────────────────────────────────────────────
#define RELAY_PIN         48
#define OLED_SDA          39
#define OLED_SCL          38
#define SCREEN_W         128
#define SCREEN_H          64
#define SETUP_BUTTON_PIN   0       // DL button — hold at boot to force AP mode

// ── AP / setup mode ──────────────────────────────────────────
#define AP_SSID         "TreeMarker-Setup"
#define STA_TIMEOUT_MS  20000

// ── GitHub release API ───────────────────────────────────────
#define GH_LATEST_URL \
  "https://api.github.com/repos/mandeepMildura/tree-marker-alr/releases/latest"

// ================================================================
//  MUTABLE CONFIG (loaded from Preferences at boot)
// ================================================================
char   gWifiSsid[64];
char   gWifiPass[64];
char   gMqttHost[128];
int    gMqttPort;
char   gMqttUser[64];
char   gMqttPass[64];
double gOriginLat;
double gOriginLon;
double gRowBearing;
double gRowSpacing;
double gTreeSpacing;
int    gNumRows;
int    gNumTrees;
double gHitRadius;
int    gRelayPulse;
int    gUdpPort;
double gForeAft;         // + = nozzle behind antenna
double gLateral;         // + = nozzle right of centreline
double gHeading = 0;     // degrees, computed from consecutive fixes
double prevFixLat = 0, prevFixLon = 0;
bool   hasPrevFix = false;

// ── AB-mode state (loaded from NVS) ───────────────────────────
int    gGridMode;                 // MODE_SIMPLE or MODE_AB
char   gRowLineName[48];          // name of row AB line in ABLines.txt
char   gTreeLineName[48];         // name of tree AB line in ABLines.txt
double gFieldUtmE;                // (legacy — unused since v1.4.8)
double gFieldUtmN;                // (legacy — unused since v1.4.8)
int    gFieldZone;                // (legacy — unused since v1.4.8)
double gAnchorLat = 0;            // StartFix lat from AOG Field.txt
double gAnchorLon = 0;            // StartFix lon from AOG Field.txt
double gRowE, gRowN, gRowHdg;     // selected row AB line: point + heading(deg)
double gTreeE, gTreeN, gTreeHdg;  // selected tree AB line: point + heading(deg)
bool   gHasField  = false;        // true once Field.txt offsets are stored
bool   gHasLines  = false;        // true once both AB lines are selected
double gAbOriginE = 0;            // intersection of the two AB lines (row0/tree0) in local E/N
double gAbOriginN = 0;            // set by resolveActiveABLines()

// Raw ABLines.txt text held in RAM so user can re-pick lines without
// re-uploading. Persisted in NVS as "ab_raw" (<=3800 chars).
String gAbLinesRaw;

// AB-mode geometry (declared up here so loadPrefs can populate them).
struct IPt { float e; float n; };
IPt  intersections[MAX_INTERSECTIONS];
int  numIntersections = 0;
IPt  boundary[MAX_BOUNDARY];
int  numBoundary = 0;
bool gHasBoundary = false;
int  lastRawIntersections = 0;  // pre-boundary-clip count, for UI

Preferences prefs;

void loadPrefs() {
  prefs.begin("tm", true);
  strlcpy(gWifiSsid,  prefs.getString("wifi_ssid",  DEF_WIFI_SSID).c_str(),  sizeof(gWifiSsid));
  strlcpy(gWifiPass,  prefs.getString("wifi_pass",  DEF_WIFI_PASS).c_str(),  sizeof(gWifiPass));
  strlcpy(gMqttHost,  prefs.getString("mqtt_host",  DEF_MQTT_HOST).c_str(),  sizeof(gMqttHost));
  gMqttPort    = prefs.getInt(   "mqtt_port",  DEF_MQTT_PORT);
  strlcpy(gMqttUser,  prefs.getString("mqtt_user",  DEF_MQTT_USER).c_str(),  sizeof(gMqttUser));
  strlcpy(gMqttPass,  prefs.getString("mqtt_pass",  DEF_MQTT_PASS).c_str(),  sizeof(gMqttPass));
  gOriginLat   = prefs.getDouble("origin_lat",  DEF_ORIGIN_LAT);
  gOriginLon   = prefs.getDouble("origin_lon",  DEF_ORIGIN_LON);
  gRowBearing  = prefs.getDouble("row_bearing", DEF_ROW_BEARING);
  gRowSpacing  = prefs.getDouble("row_spacing", DEF_ROW_SPACING);
  gTreeSpacing = prefs.getDouble("tree_space",  DEF_TREE_SPACING);
  gNumRows     = prefs.getInt(   "num_rows",    DEF_NUM_ROWS);
  gNumTrees    = prefs.getInt(   "num_trees",   DEF_NUM_TREES);
  gHitRadius   = prefs.getDouble("hit_radius",  DEF_HIT_RADIUS);
  gRelayPulse  = prefs.getInt(   "relay_pulse", DEF_RELAY_PULSE);
  gUdpPort     = prefs.getInt(   "udp_port",    UDP_PORT);
  gForeAft     = prefs.getDouble("fore_aft",    DEF_FORE_AFT);
  gLateral     = prefs.getDouble("lateral",     DEF_LATERAL);

  // AB-mode state
  gGridMode = prefs.getInt("grid_mode", MODE_SIMPLE);
  strlcpy(gRowLineName,  prefs.getString("row_name",  DEF_ROW_NAME ).c_str(), sizeof(gRowLineName));
  strlcpy(gTreeLineName, prefs.getString("tree_name", DEF_TREE_NAME).c_str(), sizeof(gTreeLineName));
  gFieldUtmE = prefs.getDouble("field_e", 0.0);
  gFieldUtmN = prefs.getDouble("field_n", 0.0);
  gFieldZone = prefs.getInt(   "field_z", 0);
  gAnchorLat = prefs.getDouble("anchor_lat", 0.0);
  gAnchorLon = prefs.getDouble("anchor_lon", 0.0);
  gHasField  = prefs.getBool(  "has_field", false);
  gHasLines  = prefs.getBool(  "has_lines", false);
  gRowE   = prefs.getDouble("row_e",   0);
  gRowN   = prefs.getDouble("row_n",   0);
  gRowHdg = prefs.getDouble("row_h",   0);
  gTreeE  = prefs.getDouble("tree_e",  0);
  gTreeN  = prefs.getDouble("tree_n",  0);
  gTreeHdg= prefs.getDouble("tree_h",  0);
  gAbLinesRaw = prefs.getString("ab_raw", "");
  gHasBoundary = prefs.getBool("has_bnd", false);
  numBoundary  = 0;
  if (gHasBoundary) {
    size_t want = prefs.getBytesLength("bnd_pts");
    if (want > 0 && want <= sizeof(boundary)) {
      prefs.getBytes("bnd_pts", boundary, want);
      numBoundary = want / sizeof(IPt);
    } else {
      gHasBoundary = false;
    }
  }
  prefs.end();
}

void saveAbPrefs() {
  prefs.begin("tm", false);
  prefs.putInt(   "grid_mode", gGridMode);
  prefs.putString("row_name",  gRowLineName);
  prefs.putString("tree_name", gTreeLineName);
  prefs.putDouble("field_e",   gFieldUtmE);
  prefs.putDouble("field_n",   gFieldUtmN);
  prefs.putInt(   "field_z",   gFieldZone);
  prefs.putDouble("anchor_lat", gAnchorLat);
  prefs.putDouble("anchor_lon", gAnchorLon);
  prefs.putBool(  "has_field", gHasField);
  prefs.putBool(  "has_lines", gHasLines);
  prefs.putDouble("row_e",  gRowE);
  prefs.putDouble("row_n",  gRowN);
  prefs.putDouble("row_h",  gRowHdg);
  prefs.putDouble("tree_e", gTreeE);
  prefs.putDouble("tree_n", gTreeN);
  prefs.putDouble("tree_h", gTreeHdg);
  // NVS string limit is ~4000 bytes — truncate defensively.
  String r = gAbLinesRaw;
  if (r.length() > 3800) r = r.substring(0, 3800);
  prefs.putString("ab_raw", r);
  prefs.putBool("has_bnd", gHasBoundary);
  if (gHasBoundary && numBoundary > 0) {
    prefs.putBytes("bnd_pts", boundary, numBoundary * sizeof(IPt));
  } else {
    prefs.remove("bnd_pts");
  }
  prefs.end();
}

void saveGridPrefs() {
  prefs.begin("tm", false);
  prefs.putDouble("origin_lat",  gOriginLat);
  prefs.putDouble("origin_lon",  gOriginLon);
  prefs.putDouble("row_bearing", gRowBearing);
  prefs.putDouble("row_spacing", gRowSpacing);
  prefs.putDouble("tree_space",  gTreeSpacing);
  prefs.putInt(   "num_rows",    gNumRows);
  prefs.putInt(   "num_trees",   gNumTrees);
  prefs.putDouble("hit_radius",  gHitRadius);
  prefs.putInt(   "relay_pulse", gRelayPulse);
  prefs.putDouble("fore_aft",    gForeAft);
  prefs.putDouble("lateral",     gLateral);
  prefs.end();
}

void saveWifiPrefs() {
  prefs.begin("tm", false);
  prefs.putString("wifi_ssid", gWifiSsid);
  prefs.putString("wifi_pass", gWifiPass);
  prefs.putInt(   "udp_port",  gUdpPort);
  prefs.end();
}

// ── Objects ───────────────────────────────────────────────────
WiFiUDP           udp;
WiFiClientSecure  tlsClient;
PubSubClient      mqtt(tlsClient);
Adafruit_SSD1306  oled(SCREEN_W, SCREEN_H, &Wire, -1);
WebServer         webServer(80);
DNSServer         dnsServer;
bool              apMode = false;
bool              otaRebootPending = false;
unsigned long     otaRebootAt = 0;

// ── Grid ──────────────────────────────────────────────────────
const double M_PER_DEG_LAT = 111320.0;
double mPerDegLon;
double bearingRad, perpBearingRad;

struct Point { double lat; double lon; };
Point grid[20][100];   // [row][tree]  max 20x100

// (intersections[], boundary[], IPt, counts — all declared up top near
// the global config so loadPrefs() can populate them on boot.)

// Ring buffer of the last 20 fires — for the dashboard "Recent Hits" card
// and field calibration. Ephemeral (cleared on reboot).
#define HIT_LOG_CAP 20
struct HitLog { int row; int tree; float dist; double lat; double lon; float heading; unsigned long ms; };
HitLog hitLog[HIT_LOG_CAP];
int    hitLogHead  = 0;   // next write slot (oldest gets overwritten)
int    hitLogCount = 0;

// Forward decls — full definitions live further down next to the rest
// of the AB-mode helpers. Needed because checkGrid() / handleGrid() are
// above them and Arduino's auto-prototyping isn't reliable with
// reference parameters.
void latLonToLocal(double lat, double lon, double& localE, double& localN);
void localToLatLon(double localE, double localN, double& lat, double& lon);
void utmToLatLon(double utmE, double utmN, int zone, double& lat, double& lon);

// ── State ─────────────────────────────────────────────────────
bool   relayActive    = false;
unsigned long relayStart    = 0;
unsigned long lastHit       = 0;
int    lastRow   = -1, lastTree  = -1;
// Closest-approach tracker (v1.4.4+). Replaces the older inside-radius debounce.
// approachIdx: flat tree index (r*numTrees+t) we're currently approaching.
// approachMinD: smallest distance seen since approachIdx was locked.
// approachFired: set after a fire so the same tree can't re-fire until the
//                tractor moves out of range (nearIdx < 0 clears everything).
int    approachIdx   = -1;
float  approachMinD  = 1e9f;
bool   approachFired = false;
double lastDist  =  0;
double curLat    =  0, curLon   =  0;
bool   gpsFix    = false;
unsigned long lastFixMs     = 0;
int    totalHits            = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastMachineSend = 0;
bool   otaPending   = false;
String otaUrl       = "";

// ================================================================
//  WEB PAGE (served from ESP32 at http://board-ip/)
// ================================================================
const char PAGE[] PROGMEM = R"rawpage(<!DOCTYPE html>
<html lang=en>
<head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Tree Marker ALR — Sunraysia Acres</title>
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;700&family=Manrope:wght@400;500;600;700&display=swap" rel=stylesheet>
<style>
:root{--bg:#fcf9f4;--sl:#f6f3ee;--sc:#f0ede8;--sh:#ebe8e3;--sw:#e5e2dd;--wh:#ffffff;--pr:#006e2f;--pb:#22c55e;--tx:#1c1c19;--tv:#3d4a3d;--mu:#6d7b6c;--ol:#bccbb9;--er:#ba1a1a}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Manrope',-apple-system,sans-serif;background:var(--bg);color:var(--tx);min-height:100vh}
.hl{font-family:'Space Grotesk',sans-serif}
header{background:rgba(252,249,244,.92);backdrop-filter:blur(12px);position:sticky;top:0;z-index:50;border-bottom:1px solid var(--ol)}
.hi{max-width:920px;margin:auto;display:flex;align-items:center;gap:12px;height:56px;padding:0 20px}
.brand{font-family:'Space Grotesk',sans-serif;font-size:15px;font-weight:700;letter-spacing:-.3px;line-height:1.2}
.brand small{display:block;font-size:10px;font-weight:400;color:var(--mu);letter-spacing:.5px;text-transform:uppercase}
.pill{display:flex;align-items:center;gap:5px;padding:3px 10px;border-radius:99px;background:#dcfce7;border:1px solid #86efac;font-size:11px;font-weight:700;color:var(--pr);transition:all .3s}
.pill.off{background:#fee2e2;border-color:#fca5a5;color:var(--er)}
.sdot{width:7px;height:7px;border-radius:50%;background:var(--pb);flex-shrink:0}
.pill.off .sdot{background:var(--er)}
nav{display:flex;gap:2px;margin-left:12px}
.nb{background:none;border:none;cursor:pointer;font-family:'Space Grotesk',sans-serif;font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu);padding:6px 12px;border-radius:6px;transition:all .2s}
.nb.active{background:var(--sh);color:var(--pr)}
.nb:hover:not(.active){background:var(--sl)}
.fw{margin-left:auto;font-size:11px;color:var(--mu);font-family:'Space Grotesk',sans-serif}
main{max-width:920px;margin:0 auto;padding:22px 20px 48px}
.sec{display:none}.sec.active{display:block}
.sech{margin-bottom:18px}
.sech h2{font-size:26px;font-weight:700;letter-spacing:-.5px}
.slbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1.2px;color:var(--pr);margin-bottom:3px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
@media(max-width:600px){.g2,.g4{grid-template-columns:1fr 1fr}}
.card{background:var(--wh);border-radius:12px;padding:18px;border:1px solid var(--ol);margin-bottom:14px}
.clbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:var(--mu);margin-bottom:12px;display:flex;align-items:center;gap:6px}
.clbl::before{content:'';display:inline-block;width:3px;height:13px;background:var(--pb);border-radius:2px}
.sr{display:flex;align-items:center;justify-content:space-between;padding:7px 0;border-bottom:1px solid var(--sl);font-size:13px}
.sr:last-child{border:none;padding-bottom:0}
.sk{color:var(--tv);font-weight:500}
.sv{font-weight:700;font-family:'Space Grotesk',sans-serif}
.sv.ok{color:var(--pr)}.sv.bad{color:var(--er)}.sv.bl{color:#0369a1}
.bs{background:var(--sl);border-radius:8px;padding:12px 14px}
.bs .lbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu)}
.bs .num{font-family:'Space Grotesk',sans-serif;font-size:30px;font-weight:800;line-height:1.1}
.bs .num.gr{color:var(--pr)}
.dvd{height:1px;background:var(--ol);margin:14px 0}
.btn{display:flex;align-items:center;justify-content:space-between;width:100%;padding:13px 16px;border-radius:8px;border:none;cursor:pointer;font-family:'Space Grotesk',sans-serif;font-size:13px;font-weight:700;letter-spacing:.2px;transition:all .15s;margin-bottom:8px}
.btn:last-child{margin-bottom:0}
.btn:active{transform:scale(.98)}
.btn-p{background:var(--pr);color:#fff}.btn-p:hover{background:#005321}
.btn-g{background:#dcfce7;color:var(--pr);border:1px solid #86efac}.btn-g:hover{background:#bbf7d0}
.btn-w{background:#735a42;color:#fff}.btn-w:hover{background:#5c4534}
.bi{font-size:17px}
.fg{margin-bottom:12px}
.fg label{display:block;font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:.8px;color:var(--mu);margin-bottom:5px}
.fg input{width:100%;background:var(--sl);border:1px solid transparent;border-bottom-color:var(--ol);border-radius:6px 6px 0 0;padding:9px 12px;font-family:'Manrope',sans-serif;font-size:13px;font-weight:600;color:var(--tx);outline:none;transition:all .2s}
.fg input:focus{background:var(--wh);border-bottom-color:var(--pr);border-color:var(--ol)}
.note{font-size:11px;color:var(--mu);margin-top:8px}
.map-compass{background:rgba(255,255,255,.92);border:2px solid var(--pr);border-radius:50%;box-shadow:0 2px 8px rgba(0,0,0,.2);pointer-events:none;margin:0 0 16px 16px !important}
.tractor-arrow{background:transparent !important;border:none !important}
.tractor-lbl{position:absolute;top:26px;left:50%;transform:translateX(-50%);white-space:nowrap;background:rgba(3,105,161,.95);color:#fff;font-family:'Space Grotesk',sans-serif;font-size:11px;font-weight:700;padding:2px 7px;border-radius:4px;box-shadow:0 1px 3px rgba(0,0,0,.3);letter-spacing:.3px}
.nozzle-dot{background:transparent !important;border:none !important}
.mode-banner{font-size:12px;padding:10px 12px;border-radius:8px;margin-bottom:14px;line-height:1.55;border-left:4px solid var(--pr);background:#ecfdf5}
.mode-banner.ab{background:#eff6ff;border-left-color:#0369a1}
.mode-banner b{font-family:'Space Grotesk',sans-serif;letter-spacing:.3px}
.mode-banner code{font-family:'Space Grotesk',sans-serif;background:rgba(255,255,255,.6);padding:1px 5px;border-radius:4px;font-weight:600}
.subhead{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:var(--tv);margin:4px 0 8px}
.fg.simple-only.disabled{opacity:.45}
.fg.simple-only.disabled input{background:var(--sc);cursor:not-allowed}
.btn:disabled{opacity:.45;cursor:not-allowed}.btn:disabled:active{transform:none}
</style>
</head>
<body>
<header>
<div class=hi>
  <div class=brand>Tree Marker ALR<small>Sunraysia Acres</small></div>
  <div class="pill off" id=spill><div class=sdot></div><span id=slbl>Offline</span></div>
  <nav>
    <button class="nb active" onclick="showTab('status',this)">Status</button>
    <button class=nb onclick="showTab('map',this)">Map</button>
    <button class=nb onclick="showTab('field',this)">Field</button>
    <button class=nb onclick="showTab('config',this)">Config</button>
  </nav>
  <div class=fw id=hfw></div>
</div>
</header>
<main>

<!-- STATUS -->
<div id=t-status class="sec active">
  <div class=sech>
    <div class=slbl>Live Telemetry</div>
    <h2 class=hl>System Status</h2>
  </div>
  <div class=g2>
    <div class=card>
      <div class=clbl>System</div>
      <div class=sr><span class=sk>WiFi</span><span class="sv ok" id=s-wifi>—</span></div>
      <div class=sr><span class=sk>IP Address</span><span class="sv bl" id=s-ip>—</span></div>
      <div class=sr><span class=sk>MQTT</span><span class="sv ok" id=s-mqtt>—</span></div>
      <div class=sr><span class=sk>GPS Fix</span><span class="sv bad" id=s-gps>—</span></div>
      <div class=sr><span class=sk>Position</span><span class="sv bl" id=s-pos style="font-size:11px">—</span></div>
      <div class=sr><span class=sk>Uptime</span><span class=sv id=s-up>—</span></div>
      <div class=sr><span class=sk>Firmware</span><span class=sv id=s-fw>—</span></div>
    </div>
    <div class=card>
      <div class=clbl>Actions</div>
      <button class="btn btn-g" onclick=testRelay()><span>Fire Relay (wiring test)</span><span class=bi>&#9889;</span></button>
      <div class=dvd></div>
      <div class=clbl>Active Grid</div>
      <div class=sr><span class=sk>Origin</span><span class="sv bl" id=c-origin style="font-size:11px">—</span></div>
      <div class=sr><span class=sk>Bearing</span><span class=sv id=c-brg>—</span></div>
      <div class=sr><span class=sk>Row / Tree spacing</span><span class=sv id=c-spacing>—</span></div>
      <div class=sr><span class=sk>Grid size</span><span class=sv id=c-grid>&mdash;</span></div>
      <div class=sr><span class=sk>Hit radius</span><span class=sv id=c-hr>&mdash;</span></div>
      <div class=sr><span class=sk>Relay pulse</span><span class=sv id=c-rp>&mdash;</span></div>
      <div class=sr><span class=sk>Mode</span><span class=sv id=c-mode>&mdash;</span></div>
      <div class=sr><span class=sk>Intersections</span><span class=sv id=c-ni>&mdash;</span></div>
      <div class=sr><span class=sk>Heading</span><span class=sv id=c-hdg>&mdash;</span></div>
      <div class=sr><span class=sk>Nozzle offset</span><span class=sv id=c-off style="font-size:11px">&mdash;</span></div>
    </div>
  </div>
  <div class=card>
    <div class=clbl>Session Statistics</div>
    <div class=g4>
      <div class=bs><div class=lbl>Hits</div><div class="num gr" id=s-hits>0</div></div>
      <div class=bs><div class=lbl>Last Row</div><div class=num id=s-lrow>—</div></div>
      <div class=bs><div class=lbl>Last Tree</div><div class=num id=s-ltree>—</div></div>
      <div class=bs><div class=lbl>Last Dist</div><div class=num id=s-ldist style="font-size:20px">—</div></div>
    </div>
  </div>
  <div class=card>
    <div class=clbl>Recent Hits <span style="color:var(--mu);font-weight:400;font-size:9px;margin-left:6px;letter-spacing:1px">LIVE · LAST 20</span>
      <a href="/api/hits?csv=1" download="tree-marker-hits.csv" style="float:right;font-size:10px;font-weight:700;color:var(--pr);text-decoration:none">⬇ CSV</a>
    </div>
    <div id=hitlog style="max-height:260px;overflow-y:auto;font-family:'Space Grotesk',sans-serif;font-size:12px">
      <div style="color:var(--mu);padding:10px 0">No hits yet.</div>
    </div>
  </div>
</div>

<!-- MAP -->
<div id=t-map class=sec>
  <div class=sech>
    <div class=slbl>Satellite View</div>
    <h2 class=hl>Field Map</h2>
  </div>
  <div class=card style="padding:0;overflow:hidden">
    <div id=map style="height:520px;width:100%;background:#f0ede8"></div>
  </div>
  <p class=note style="margin-top:10px;line-height:1.8">
    <span style="color:#ca8a04;font-weight:700">&#x25CF; Yellow</span> = planned trees &nbsp;
    <span style="color:var(--pb);font-weight:700">&#x25CF; Green</span> = hits fired &nbsp;
    <span style="color:#0369a1;font-weight:700">&#x25B2; Blue arrow</span> = tractor (points in direction of travel, heading shown below) &nbsp;
    <span style="color:#a16207;font-weight:700">&#x25CF; Amber</span> = virtual nozzle (only shown when offset &ne; 0 &mdash; this is where paint lands) &nbsp;
    <span style="color:#b91c1c;font-weight:700">&mdash; Red dashed</span> = Row AB line (AB mode) &nbsp;
    <span style="color:#7e22ce;font-weight:700">&mdash; Purple dashed</span> = Tree AB line (AB mode) &nbsp;
    <span style="color:var(--pr);font-weight:700">&#x2B1A; Green dashed</span> = field boundary &nbsp;
    <span style="color:var(--tv);font-weight:700">&#x1F9ED; Compass</span> = map is north-up (bottom-left corner)
  </p>
</div>

<!-- FIELD -->
<div id=t-field class=sec>
  <div class=sech>
    <div class=slbl>AgOpenGPS Field Import</div>
    <h2 class=hl>Field Source</h2>
  </div>
  <div class=card>
    <div class=clbl>Current State</div>
    <div class=sr><span class=sk>Grid mode</span><span class=sv id=fd-mode>&mdash;</span></div>
    <div class=sr><span class=sk>Intersections</span><span class="sv gr" id=fd-n>&mdash;</span></div>
    <div class=sr><span class=sk>Anchor (StartFix)</span><span class="sv bl" id=fd-offset style="font-size:11px">&mdash;</span></div>
    <div class=sr><span class=sk>Active row line</span><span class=sv id=fd-rline>&mdash;</span></div>
    <div class=sr><span class=sk>Active tree line</span><span class=sv id=fd-tline>&mdash;</span></div>
    <div class=sr><span class=sk>Lines in file</span><span class="sv bl" id=fd-avail style="font-size:11px">&mdash;</span></div>
    <div class=sr><span class=sk>Boundary</span><span class=sv id=fd-bndy>&mdash;</span></div>
    <div class=dvd></div>
    <button class="btn btn-p" onclick=setMode(1)><span>Use AB mode (from AgOpenGPS)</span><span class=bi>&rarr;</span></button>
    <button class="btn btn-g" onclick=setMode(0)><span>Use Simple mode (origin + bearing)</span><span class=bi>&rarr;</span></button>
  </div>
  <div class=card>
    <div class=clbl>Upload from AgOpenGPS Field Folder</div>
    <div class=fg><label>ABLines.txt</label><input id=fd-ab type=file accept=".txt"></div>
    <div class=fg><label>Field.txt</label><input id=fd-ft type=file accept=".txt"></div>
    <div class=fg><label>Boundary.txt <span style="color:var(--mu);font-weight:400;text-transform:none;letter-spacing:0">(optional &mdash; clips grid to field shape)</span></label><input id=fd-bnd type=file accept=".txt"></div>
    <button class="btn btn-p" onclick=fieldUpload()><span>Upload &amp; Apply</span><span class=bi>&#8593;</span></button>
    <p class=note id=fd-status>Save two AB lines in AgOpenGPS (one row direction, one tree direction), plus optionally the field boundary. Copy ABLines.txt, Field.txt, and Boundary.txt from your field folder and drop them here.</p>
  </div>
  <div class=card>
    <div class=clbl>Active Line Names</div>
    <div class=g2>
      <div class=fg><label>Row Line Name</label><input id=fd-rname type=text></div>
      <div class=fg><label>Tree Line Name</label><input id=fd-tname type=text></div>
    </div>
    <button class="btn btn-p" onclick=applyLines()><span>Re-pick &amp; Rebuild Grid</span><span class=bi>&#10003;</span></button>
    <p class=note>The names must match exactly what you saved them as in AgOpenGPS.</p>
  </div>
</div>

<!-- CONFIG -->
<div id=t-config class=sec>
  <div class=sech>
    <div class=slbl>Device Configuration</div>
    <h2 class=hl>Grid Parameters</h2>
  </div>
  <div class=card>
    <div class=clbl>Field Grid</div>
    <div id=grid-mode-banner class=mode-banner></div>
    <div class="subhead" id=simple-subhead>Origin &amp; Bearing</div>
    <div id=simple-fields class=g2>
      <div class="fg simple-only"><label>Origin Latitude</label><input id=f-lat type=number step=0.0000001></div>
      <div class="fg simple-only"><label>Origin Longitude</label><input id=f-lon type=number step=0.0000001></div>
      <div class="fg simple-only" style="grid-column:1/-1"><label>Row Bearing (&#176;) &mdash; 0=N 90=E 180=S</label><input id=f-brg type=number step=0.1></div>
    </div>
    <div class="subhead" style="margin-top:14px">Plant Pattern &amp; Detection</div>
    <div class=g2>
      <div class=fg><label>Row Spacing (m)</label><input id=f-rs type=number step=0.1></div>
      <div class=fg><label>Tree Spacing (m)</label><input id=f-ts type=number step=0.1></div>
      <div class="fg simple-only"><label>Hit Radius (m)</label><input id=f-hr type=number step=0.01></div>
      <div class=fg><label>Number of Rows (max 20)</label><input id=f-nr type=number step=1 min=1 max=20></div>
      <div class=fg><label>Trees per Row (max 100)</label><input id=f-nt type=number step=1 min=1 max=100></div>
      <div class=fg style="grid-column:1/-1"><label>Relay Pulse (ms) &mdash; always editable</label><input id=f-rp type=number step=50></div>
    </div>
    <button class="btn btn-p" id=save-grid-btn onclick=saveGrid() style="margin-top:10px"><span>Save Grid + Apply Now</span><span class=bi>&#10003;</span></button>
    <p class=note id=save-grid-note>Grid rebuilds immediately &mdash; no reboot needed.</p>
  </div>
  <div class=card>
    <div class=clbl>Antenna &rarr; Nozzle Offset</div>
    <div class=g2>
      <div class=fg><label>Fore/Aft (m) &mdash; positive = BEHIND antenna</label><input id=f-fa type=number step=0.01></div>
      <div class=fg><label>Lateral (m) &mdash; positive = RIGHT of antenna</label><input id=f-latoff type=number step=0.01></div>
    </div>
    <button class="btn btn-p" onclick=saveGrid()><span>Save Offset + Apply</span><span class=bi>&#10003;</span></button>
    <p class=note>Bidirectional calibration: drive the same row both ways, measure the distance between marks, divide by 2 &mdash; that's your residual error. Adjust fore/aft (or lateral) and retest until marks land on top of each other. Current heading is shown on the Status tab.</p>
  </div>
  <div class=card>
    <div class=clbl>Network (requires reboot)</div>
    <div class=g2>
      <div class=fg><label>Network Name (SSID)</label><input id=f-ssid type=text></div>
      <div class=fg><label>Password</label><input id=f-wpass type=password placeholder="(leave blank to keep current)"></div>
      <div class=fg><label>AgIO UDP Port</label><input id=f-udp type=number step=1 min=1 max=65535></div>
    </div>
    <button class="btn btn-w" onclick=saveWifi()><span>Save Network + Reboot</span><span class=bi>&#8635;</span></button>
    <p class=note>Board will reconnect after reboot. Default AgIO port is 8888.</p>
  </div>
  <div class=card>
    <div class=clbl>Firmware Update (OTA)</div>
    <button class="btn btn-g" onclick=checkLatest() style="margin-bottom:10px">
      <span id=lat-btn>Check Latest Release on GitHub</span><span class=bi>&#128269;</span></button>
    <div class=fg><label>Firmware .bin URL (from GitHub Releases)</label><input id=f-ota type=text placeholder="https://github.com/.../releases/download/vX.Y.Z/tree_marker.ino.bin"></div>
    <button class="btn btn-p" onclick=flashOta()><span>Flash From URL</span><span class=bi>&#8659;</span></button>
    <div class=dvd></div>
    <div class=fg><label>Or upload a .bin file from this computer</label>
      <input id=f-file type=file accept=".bin"></div>
    <button class="btn btn-w" onclick=uploadBin()><span>Upload &amp; Flash</span><span class=bi>&#128228;</span></button>
    <p class=note id=ota-status>Board will reboot automatically after flashing.</p>
  </div>
</div>

</main>
<script>
var showTab=(t,btn)=>{
  document.querySelectorAll('.sec').forEach(e=>e.classList.remove('active'));
  document.querySelectorAll('.nb').forEach(e=>e.classList.remove('active'));
  document.getElementById('t-'+t).classList.add('active');
  btn.classList.add('active');
  if(t==='config')fillForm(window._cfg||{});
  if(t==='field')fetchField();
  if(t==='status')fetchHits();
  if(t==='map')initMap();
};
var _leafletReady=false,_mapInst=null,_mapLayers={grid:null,hits:null,tractor:null,nozzle:null,boundary:null,abLines:null};
var _compassSvg='<svg viewBox="0 0 60 60" width=54 height=54 xmlns="http://www.w3.org/2000/svg">'+
  '<circle cx=30 cy=30 r=27 fill="rgba(255,255,255,.95)"/>'+
  '<polygon points="30,6 24,30 36,30" fill="#ba1a1a"/>'+
  '<polygon points="30,54 24,30 36,30" fill="#6d7b6c"/>'+
  '<circle cx=30 cy=30 r=3 fill="#1c1c19"/>'+
  '<text x=30 y=16 font-size=11 font-weight=700 fill="#fff" text-anchor="middle" font-family="Space Grotesk,sans-serif">N</text>'+
  '<text x=30 y=50 font-size=9 font-weight=700 fill="#1c1c19" text-anchor="middle" font-family="Space Grotesk,sans-serif">S</text>'+
  '<text x=50 y=34 font-size=9 font-weight=700 fill="#1c1c19" text-anchor="middle" font-family="Space Grotesk,sans-serif">E</text>'+
  '<text x=10 y=34 font-size=9 font-weight=700 fill="#1c1c19" text-anchor="middle" font-family="Space Grotesk,sans-serif">W</text>'+
  '</svg>';
var _hdgLabel=(h)=>{var d=((h%360)+360)%360;var dirs=['N','NE','E','SE','S','SW','W','NW'];var idx=Math.round(d/45)%8;return d.toFixed(0)+'\u00B0 '+dirs[idx];};
var _tractorSvg=(h)=>'<div style="position:relative;width:28px;height:28px"><div style="width:28px;height:28px;transform:rotate('+(h||0)+'deg);transform-origin:50% 50%">'+
  '<svg viewBox="0 0 28 28" width=28 height=28 xmlns="http://www.w3.org/2000/svg">'+
    '<circle cx=14 cy=14 r=11 fill="#38bdf8" stroke="#0369a1" stroke-width=2.5 fill-opacity=.85/>'+
    '<polygon points="14,3 7,16 14,13 21,16" fill="#0c2447" stroke="#fff" stroke-width=1 stroke-linejoin=round/>'+
  '</svg></div><div class="tractor-lbl">'+_hdgLabel(h||0)+'</div></div>';
var loadLeaflet=()=>new Promise((ok,ng)=>{
  if(_leafletReady){ok();return;}
  var css=document.createElement('link');css.rel='stylesheet';
  css.href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
  document.head.appendChild(css);
  var js=document.createElement('script');
  js.src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
  js.onload=()=>{_leafletReady=true;ok();};js.onerror=()=>ng('leaflet load fail');
  document.head.appendChild(js);
});
var initMap=async()=>{
  if(_mapInst){setTimeout(()=>_mapInst.invalidateSize(),100);return;}
  try{await loadLeaflet();}catch(e){document.getElementById('map').innerHTML='<p style="padding:20px;color:var(--er)">Failed to load Leaflet (need internet). '+e+'</p>';return;}
  _mapInst=L.map('map',{maxZoom:20,zoomControl:true});
  var sat=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    {attribution:'&copy; Esri',maxNativeZoom:19,maxZoom:20}).addTo(_mapInst);
  var dark=L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    {attribution:'&copy; CartoDB',maxNativeZoom:19,maxZoom:20});
  var osm=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    {attribution:'&copy; OpenStreetMap',maxNativeZoom:19,maxZoom:20});
  L.control.layers({'Satellite':sat,'Dark':dark,'Street':osm},{},{position:'topright'}).addTo(_mapInst);
  var Compass=L.Control.extend({onAdd:function(){var d=L.DomUtil.create('div','map-compass');d.innerHTML=_compassSvg;return d;},onRemove:function(){}});
  new Compass({position:'bottomleft'}).addTo(_mapInst);
  _mapInst.setView([-34.3,142.15],16);
  await refreshMapGrid();
};
var refreshMapGrid=async()=>{
  if(!_mapInst)return;
  try{
    var r=await fetch('/api/grid').then(r=>r.json());
    if(_mapLayers.grid)_mapInst.removeLayer(_mapLayers.grid);
    if(_mapLayers.boundary)_mapInst.removeLayer(_mapLayers.boundary);
    if(_mapLayers.abLines)_mapInst.removeLayer(_mapLayers.abLines);
    // Boundary polygon (if present) — draw under the trees
    if(r.boundary&&r.boundary.length>=3){
      _mapLayers.boundary=L.polygon(r.boundary,
        {color:'#006e2f',weight:2,fillColor:'#006e2f',fillOpacity:0.08,dashArray:'6 4'})
        .bindTooltip('Field boundary ('+r.boundary.length+' vertices)').addTo(_mapInst);
    }
    // AB lines imported from AgOpenGPS — makes alignment errors visible
    var abg=null;
    if((r.rowLine&&r.rowLine.length>=2)||(r.treeLine&&r.treeLine.length>=2)){
      abg=L.layerGroup();
      if(r.rowLine&&r.rowLine.length>=2){
        L.polyline(r.rowLine,{color:'#b91c1c',weight:3,opacity:0.85,dashArray:'10 5'})
          .bindTooltip('Row AB line (trees run along this)',{sticky:true}).addTo(abg);
      }
      if(r.treeLine&&r.treeLine.length>=2){
        L.polyline(r.treeLine,{color:'#7e22ce',weight:3,opacity:0.85,dashArray:'10 5'})
          .bindTooltip('Tree AB line (rows stack along this)',{sticky:true}).addTo(abg);
      }
      abg.addTo(_mapInst);
      _mapLayers.abLines=abg;
    }
    var lg=L.layerGroup();
    r.points.forEach(p=>{
      L.circleMarker([p[0],p[1]],{radius:4,color:'#92400e',weight:1,fillColor:'#facc15',fillOpacity:0.8})
        .bindTooltip('R'+p[2]+' T'+p[3],{direction:'top',offset:[0,-4]}).addTo(lg);
    });
    lg.addTo(_mapInst);
    _mapLayers.grid=lg;
    // Fit to boundary if we have one (tighter fit), else to tree grid
    var fitSrc=(r.boundary&&r.boundary.length>=3)?r.boundary:r.points.map(p=>[p[0],p[1]]);
    if(fitSrc.length>0){
      _mapInst.fitBounds(L.latLngBounds(fitSrc).pad(0.15));
    }
  }catch(e){console.error('grid fetch failed',e);}
};
var updateMapTractor=(lat,lon,fix,hdg,fa,latOff)=>{
  if(!_mapInst)return;
  if(_mapLayers.tractor){_mapInst.removeLayer(_mapLayers.tractor);_mapLayers.tractor=null;}
  if(_mapLayers.nozzle){_mapInst.removeLayer(_mapLayers.nozzle);_mapLayers.nozzle=null;}
  if(!fix)return;
  var h=hdg||0;
  var ic=L.divIcon({className:'tractor-arrow',iconSize:[28,28],iconAnchor:[14,14],html:_tractorSvg(h)});
  _mapLayers.tractor=L.marker([lat,lon],{icon:ic,title:'Tractor heading '+_hdgLabel(h)}).addTo(_mapInst);
  // Draw virtual nozzle position (where relay will fire) if an offset is configured
  if((Math.abs(fa||0)>0.01)||(Math.abs(latOff||0)>0.01)){
    var rad=h*Math.PI/180;
    var sn=Math.sin(rad), cs=Math.cos(rad);
    // fa positive = BEHIND antenna; lat_off positive = RIGHT of antenna
    var nE=-(fa||0)*sn + (latOff||0)*cs;
    var nN=-(fa||0)*cs - (latOff||0)*sn;
    var dLat=nN/111320, dLon=nE/(111320*Math.cos(lat*Math.PI/180));
    var nlat=lat+dLat, nlon=lon+dLon;
    var nSvg='<svg viewBox="0 0 18 18" width=18 height=18 xmlns="http://www.w3.org/2000/svg">'+
      '<circle cx=9 cy=9 r=6 fill="#facc15" stroke="#a16207" stroke-width=2/>'+
      '<circle cx=9 cy=9 r=2 fill="#a16207"/></svg>';
    var nic=L.divIcon({className:'nozzle-dot',iconSize:[18,18],iconAnchor:[9,9],html:nSvg});
    var grp=L.layerGroup();
    L.polyline([[lat,lon],[nlat,nlon]],{color:'#a16207',weight:2,dashArray:'3 3'}).addTo(grp);
    L.marker([nlat,nlon],{icon:nic,title:'Virtual nozzle (paint point)'})
      .bindTooltip('Nozzle (F/A '+(fa||0).toFixed(2)+'m, Lat '+(latOff||0).toFixed(2)+'m)',{direction:'top',offset:[0,-9]}).addTo(grp);
    grp.addTo(_mapInst);
    _mapLayers.nozzle=grp;
  }
};
var updateMapHits=(hits)=>{
  if(!_mapInst)return;
  if(_mapLayers.hits)_mapInst.removeLayer(_mapLayers.hits);
  var lg=L.layerGroup();
  hits.forEach(h=>{
    L.circleMarker([h.lat,h.lon],{radius:5,color:'#166534',weight:1,fillColor:'#4ade80',fillOpacity:0.95})
      .bindTooltip('R'+h.row+' T'+h.tree+' · '+h.dist.toFixed(2)+'m',{direction:'top',offset:[0,-5]}).addTo(lg);
  });
  lg.addTo(_mapInst);
  _mapLayers.hits=lg;
};
var _lastHitCount=-1;
var fetchHits=()=>{
  fetch('/api/hits').then(r=>r.json()).then(rows=>{
    var el=document.getElementById('hitlog');
    if(!rows.length){el.innerHTML='<div style="color:var(--mu);padding:10px 0">No hits yet.</div>';return;}
    var age=(s)=>s<60?s+'s ago':Math.floor(s/60)+'m '+(s%60)+'s ago';
    var html='';
    rows.forEach((h,i)=>{
      var bg=i===0?'background:#dcfce7':'background:'+(i%2?'var(--sl)':'transparent');
      html+='<div style="display:flex;justify-content:space-between;padding:7px 10px;border-bottom:1px solid var(--sl);'+bg+'">'+
        '<span style="font-weight:700;color:var(--pr)">R'+h.row+' T'+h.tree+'</span>'+
        '<span style="color:var(--tv)">'+h.dist.toFixed(3)+' m</span>'+
        '<span style="color:var(--mu);font-size:11px">'+age(h.age)+'</span>'+
        '</div>';
    });
    el.innerHTML=html;
    updateMapHits(rows);
  }).catch(()=>{});
};
var fetchField=()=>{
  fetch('/field/state').then(r=>r.json()).then(d=>{
    document.getElementById('fd-mode').textContent=d.mode===1?'AB (from AOG)':'Simple (origin+bearing)';
    var nlbl=d.hasBoundary&&d.rawIntersections>d.intersections
      ?(d.intersections+' ('+d.rawIntersections+' before boundary clip)')
      :(''+d.intersections);
    document.getElementById('fd-n').textContent=nlbl;
    var f=d.field;
    document.getElementById('fd-offset').textContent=d.hasField?
      (f.anchorLat.toFixed(7)+', '+f.anchorLon.toFixed(7)):'(not loaded)';
    document.getElementById('fd-rline').textContent=d.hasLines?d.rowName:(d.rowName+' (not found)');
    document.getElementById('fd-tline').textContent=d.hasLines?d.treeName:(d.treeName+' (not found)');
    document.getElementById('fd-avail').textContent=d.availableLines||'(no file uploaded)';
    document.getElementById('fd-bndy').textContent=d.hasBoundary?(d.boundary+' vertices (active)'):'(none)';
    document.getElementById('fd-rname').value=d.rowName;
    document.getElementById('fd-tname').value=d.treeName;
  }).catch(()=>{});
};
var readFileAsText=(inp)=>new Promise((ok,ng)=>{
  var f=inp.files[0];if(!f){ok('');return;}
  var r=new FileReader();
  r.onload=()=>ok(r.result);r.onerror=()=>ng('read fail');
  r.readAsText(f);
});
var fieldUpload=async ()=>{
  var st=document.getElementById('fd-status');
  var ab=await readFileAsText(document.getElementById('fd-ab'));
  var fl=await readFileAsText(document.getElementById('fd-ft'));
  var bd=await readFileAsText(document.getElementById('fd-bnd'));
  if(!ab||!fl){alert('Pick both ABLines.txt and Field.txt (Boundary.txt is optional)');return;}
  st.textContent='Uploading...';
  var body={ablines:ab,field:fl};
  if(bd)body.boundary=bd;
  fetch('/field/upload',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)})
    .then(async r=>{
      var txt=await r.text();
      var d; try{d=JSON.parse(txt);}catch(e){d={ok:false,error:txt||('HTTP '+r.status)};}
      return {d,status:r.status};
    }).then(({d})=>{
      if(d.ok){
        var msg='OK. '+d.intersections+' intersections';
        if(d.boundary>0)msg+=' ('+d.rawIntersections+' before boundary clip, boundary='+d.boundary+' vertices)';
        msg+='. Lines in file: '+d.linesFound;
        st.textContent=msg;
        st.style.color='';
        fetchField();poll();refreshMapGrid();
      } else {
        st.style.color='var(--er)';
        st.textContent='Upload failed: '+(d.error||'unknown error');
        if(d.detail) st.textContent+=' — got: '+d.detail;
      }
    }).catch(e=>{st.style.color='var(--er)';st.textContent='Upload failed: '+e;});
};
var applyLines=()=>{
  var rn=document.getElementById('fd-rname').value.trim();
  var tn=document.getElementById('fd-tname').value.trim();
  if(!rn||!tn){alert('Fill both names');return;}
  fetch('/field/apply',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({rowName:rn,treeName:tn,mode:1})})
    .then(r=>r.json()).then(d=>{
      document.getElementById('fd-status').textContent='Rebuilt. linesOk='+d.linesOk+' intersections='+d.intersections;
      fetchField();poll();refreshMapGrid();
    });
};
var setMode=(m)=>{
  fetch('/field/apply',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:m})})
    .then(r=>r.json()).then(d=>{fetchField();poll();refreshMapGrid();});
};
var _formFilled=false;
var fillForm=(c)=>{
  if(!c||!c.lat)return;
  ['f-lat','f-lon','f-brg','f-rs','f-ts','f-nr','f-nt','f-hr','f-rp','f-ssid','f-udp','f-fa','f-latoff'].forEach(id=>{
    var el=document.getElementById(id);if(!el)return;
    var key=({'f-lat':'lat','f-lon':'lon','f-brg':'brg','f-rs':'rs','f-ts':'ts',
              'f-nr':'rows','f-nt':'trees','f-hr':'hr','f-rp':'rp',
              'f-ssid':'ssid','f-udp':'udp','f-fa':'fa','f-latoff':'lat_off'})[id];
    if(c[key]!==undefined)el.value=c[key];
  });
  _formFilled=true;
};
var uptime=(s)=>Math.floor(s/3600)+'h '+Math.floor((s%3600)/60)+'m '+s%60+'s';
var setv=(id,txt,cls)=>{const e=document.getElementById(id);if(e){e.textContent=txt;if(cls!==undefined)e.className='sv '+(cls?'ok':'bad');}};
var poll=()=>{
  fetch('/api/status').then(r=>r.json()).then(d=>{
    const ok=d.wifi,mo=d.mqtt,go=d.fix;
    const p=document.getElementById('spill');
    p.className='pill'+(ok?'':' off');
    document.getElementById('slbl').textContent=ok?'Online':'Offline';
    setv('s-wifi',ok?'Connected':'Disconnected',ok);
    setv('s-ip',d.ip,true);
    setv('s-mqtt',mo?'Connected':'Disconnected',mo);
    setv('s-gps',go?'RTK Fix':'No Fix',go);
    document.getElementById('s-pos').textContent=go?d.lat.toFixed(7)+', '+d.lon.toFixed(7):'-';
    document.getElementById('s-up').textContent=uptime(d.uptime);
    document.getElementById('s-fw').textContent='v'+d.fw;
    document.getElementById('hfw').textContent='fw v'+d.fw;
    document.getElementById('s-hits').textContent=d.hits;
    document.getElementById('s-lrow').textContent=d.lastRow>=0?d.lastRow:'-';
    document.getElementById('s-ltree').textContent=d.lastTree>=0?d.lastTree:'-';
    document.getElementById('s-ldist').textContent=d.lastDist>0?d.lastDist.toFixed(3)+'m':'-';
    const c=d.cfg;
    document.getElementById('c-origin').textContent=c.lat.toFixed(7)+', '+c.lon.toFixed(7);
    document.getElementById('c-brg').textContent=c.brg+' deg';
    document.getElementById('c-spacing').textContent=c.rs+'m / '+c.ts+'m';
    document.getElementById('c-grid').textContent=c.rows+' x '+c.trees;
    document.getElementById('c-hr').textContent=c.hr+' m';
    document.getElementById('c-rp').textContent=c.rp+' ms';
    document.getElementById('c-mode').textContent=c.mode===1?'AB':'Simple';
    document.getElementById('c-ni').textContent=c.nInt;
    document.getElementById('c-hdg').textContent=(d.hdg!==undefined?d.hdg.toFixed(1)+'°':'—');
    document.getElementById('c-off').textContent=
      'F/A '+(c.fa!==undefined?c.fa.toFixed(2):0)+'m, Lat '+(c.lat_off!==undefined?c.lat_off.toFixed(2):0)+'m';
    window._cfg=c;
    if(!_formFilled)fillForm(c);
    applyModeUi(c);
    // Refresh hit log when count changes or once on initial load.
    if(d.hits!==_lastHitCount){_lastHitCount=d.hits;fetchHits();}
    // Live tractor marker on the map (only effective if map tab has loaded)
    updateMapTractor(d.lat,d.lon,d.fix,d.hdg,c&&c.fa,c&&c.lat_off);
  }).catch(()=>{});
};
// Toggle the Simple-only fields and banner based on current mode.
// When mode=AB and AB lines are resolved, Origin lat/lon/brg are derived
// from the AB anchor + Row line heading — disable the inputs and show the
// active values so the user isn't misled by the stale stored Simple values.
var applyModeUi=(c)=>{
  var abActive=(c.mode===1 && c.hasLines);
  var banner=document.getElementById('grid-mode-banner');
  var subhead=document.getElementById('simple-subhead');
  var btn=document.getElementById('save-grid-btn');
  var note=document.getElementById('save-grid-note');
  if(!banner) return;
  document.querySelectorAll('.fg.simple-only').forEach(el=>{
    el.classList.toggle('disabled',abActive);
    var inp=el.querySelector('input'); if(inp) inp.disabled=abActive;
  });
  // Save button stays enabled in AB mode so Relay Pulse can still be saved.
  if(btn) btn.disabled=false;
  if(abActive){
    banner.className='mode-banner ab';
    banner.innerHTML='<b>Mode: AB (from AgOpenGPS)</b><br>'+
      '<b>Origin &amp; Bearing</b> come from the AB-line intersection &mdash; disabled here. '+
      '<b>Row/Tree Spacing</b> and <b>counts</b> stay editable because ABLines.txt doesn\'t carry spacing info. '+
      '<b>Relay Pulse</b> is always editable.<br>'+
      '<span style="color:var(--mu)">Convention: in AgOpenGPS, draw both AB lines so the A point sits on the first tree of the first row. '+
      'The grid then extends from that corner along each line.</span>';
    if(subhead) subhead.style.opacity='.5';
    if(note) note.textContent='Spacings and counts rebuild the grid instantly in AB mode. Origin/Bearing come from the AB lines.';
  } else {
    banner.className='mode-banner';
    banner.innerHTML='<b>Mode: Simple</b> &mdash; all Grid Parameters below are active. '+
      'Upload AB lines on the Field tab to switch to AB mode.';
    if(subhead) subhead.style.opacity='';
    if(note) note.textContent='Grid rebuilds immediately — no reboot needed.';
  }
};
var testRelay=()=>{fetch('/relay',{method:'POST'}).then(()=>alert('Relay fired!'));};
var saveGrid=()=>{
  var num=(id)=>+document.getElementById(id).value;
  const b={lat:num('f-lat'),lon:num('f-lon'),brg:num('f-brg'),
    rs:num('f-rs'),ts:num('f-ts'),nr:num('f-nr'),nt:num('f-nt'),
    hr:num('f-hr'),rp:num('f-rp')};
  var fa=document.getElementById('f-fa'),lo=document.getElementById('f-latoff');
  if(fa&&fa.value!=='')b.fa=+fa.value;
  if(lo&&lo.value!=='')b.lat_off=+lo.value;
  fetch('/config/grid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
    .then(r=>r.text()).then(t=>{alert(t);poll();});
};
var saveWifi=()=>{
  const ssid=document.getElementById('f-ssid').value;
  const pass=document.getElementById('f-wpass').value;
  const udp=+document.getElementById('f-udp').value;
  if(!ssid){alert('SSID cannot be empty');return;}
  if(!confirm('Save network settings and reboot?'))return;
  fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:ssid,pass:pass,udp:udp})}).then(r=>r.text()).then(t=>alert(t));
};
var flashOta=()=>{
  const url=document.getElementById('f-ota').value.trim();
  if(!url){alert('Paste a .bin URL first');return;}
  if(!confirm('Flash firmware from:\n'+url+'\n\nBoard will reboot after flashing.'))return;
  const st=document.getElementById('ota-status');
  st.textContent='Sending to board...';
  fetch('/ota',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({url:url})})
    .then(r=>r.text()).then(t=>{st.textContent=t;})
    .catch(e=>{st.textContent='Error: '+e;});
};
var checkLatest=()=>{
  const b=document.getElementById('lat-btn'),st=document.getElementById('ota-status');
  b.textContent='Checking GitHub...';
  fetch('/ota/latest').then(r=>r.json()).then(d=>{
    if(d.error){b.textContent='Check Latest Release on GitHub';st.textContent='Error: '+d.error;return;}
    b.textContent='Latest: '+d.tag+' (current v'+d.current+')';
    if(d.url)document.getElementById('f-ota').value=d.url;
    st.textContent=d.tag&&('v'+d.current)!==d.tag?'Newer release available. Click Flash From URL.':'You are on the latest release.';
  }).catch(e=>{b.textContent='Check Latest Release on GitHub';st.textContent='Error: '+e;});
};
var uploadBin=()=>{
  const f=document.getElementById('f-file').files[0];
  if(!f){alert('Pick a .bin file first');return;}
  if(!confirm('Upload and flash '+f.name+' ('+Math.round(f.size/1024)+' KB)?\nBoard will reboot.'))return;
  const st=document.getElementById('ota-status');
  st.textContent='Uploading '+f.name+'...';
  const fd=new FormData();fd.append('firmware',f,f.name);
  fetch('/ota/upload',{method:'POST',body:fd})
    .then(r=>r.text()).then(t=>{st.textContent=t;})
    .catch(e=>{st.textContent='Upload error: '+e;});
};
poll();setInterval(poll,2000);
</script>
</body></html>)rawpage";

// ================================================================
//  AP SETUP PAGE — served when board can't reach saved WiFi
//  or when DL button held at boot. Minimal, no external fonts.
// ================================================================
const char SETUP_PAGE[] PROGMEM = R"rawpage(<!DOCTYPE html>
<html lang=en>
<head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Tree Marker — WiFi Setup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,Segoe UI,sans-serif}
body{background:#fcf9f4;color:#1c1c19;padding:20px;min-height:100vh}
.wrap{max-width:460px;margin:0 auto}
h1{font-size:22px;margin-bottom:4px}
.sub{color:#6d7b6c;font-size:13px;margin-bottom:20px}
.card{background:#fff;border:1px solid #bccbb9;border-radius:12px;padding:18px;margin-bottom:14px}
.lbl{font-size:10px;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#6d7b6c;margin-bottom:10px}
label{display:block;font-size:11px;font-weight:700;text-transform:uppercase;color:#6d7b6c;margin:10px 0 4px}
input,select{width:100%;padding:10px 12px;border:1px solid #bccbb9;border-radius:6px;font-size:14px;background:#f6f3ee}
input:focus,select:focus{outline:none;border-color:#006e2f;background:#fff}
button{width:100%;padding:13px;border:none;border-radius:8px;background:#006e2f;color:#fff;font-size:14px;font-weight:700;cursor:pointer;margin-top:14px}
button:hover{background:#005321}
button.sec{background:#735a42}
button.sec:hover{background:#5c4534}
.note{font-size:11px;color:#6d7b6c;margin-top:10px;line-height:1.5}
.ssid-row{display:flex;justify-content:space-between;padding:8px 10px;border-bottom:1px solid #f0ede8;font-size:13px;cursor:pointer;border-radius:4px}
.ssid-row:hover{background:#f6f3ee}
.ssid-row .rssi{color:#6d7b6c;font-size:11px}
#scan-list{max-height:220px;overflow-y:auto;margin-top:6px}
</style></head><body><div class=wrap>
<h1>Tree Marker Setup</h1>
<p class=sub>Board can't reach saved WiFi. Pick a network and enter its password.</p>
<div class=card>
  <div class=lbl>Available Networks</div>
  <div id=scan-list>Scanning...</div>
  <button class=sec onclick=rescan()>Rescan</button>
</div>
<div class=card>
  <div class=lbl>Connect</div>
  <label>Network (SSID)</label><input id=ssid placeholder="Pick above or type">
  <label>Password</label><input id=pass type=password placeholder="WiFi password">
  <label>AgIO UDP Port</label><input id=udp type=number value=8888>
  <button onclick=save()>Save &amp; Reboot</button>
  <p class=note>Board will reboot and try to connect. If it fails again the setup AP comes back up.</p>
</div>
</div>
<script>
var scan=()=>{document.getElementById('scan-list').textContent='Scanning...';
  fetch('/scan').then(r=>r.json()).then(d=>{
    var el=document.getElementById('scan-list');el.innerHTML='';
    if(!d.length){el.textContent='No networks found.';return;}
    d.forEach(n=>{
      var row=document.createElement('div');row.className='ssid-row';
      row.innerHTML='<span>'+n.ssid+(n.sec?' &#128274;':'')+'</span><span class=rssi>'+n.rssi+' dBm</span>';
      row.onclick=()=>{document.getElementById('ssid').value=n.ssid;document.getElementById('pass').focus();};
      el.appendChild(row);
    });
  }).catch(()=>{document.getElementById('scan-list').textContent='Scan failed.';});};
var rescan=scan;
var save=()=>{
  var s=document.getElementById('ssid').value.trim();
  var p=document.getElementById('pass').value;
  var u=+document.getElementById('udp').value||8888;
  if(!s){alert('Pick or type an SSID');return;}
  fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:s,pass:p,udp:u})})
    .then(r=>r.text()).then(t=>alert(t));
};
scan();
</script></body></html>)rawpage";

// ── AgOpenGPS PGN 100 (0x64) — NAV position from AOG ─────────
// AOG broadcasts this 30-byte packet to modules in both real-RTK
// and simulator mode. Payload is 24 bytes:
//   [0..7]   longitude  (double, little-endian)
//   [8..15]  latitude   (double, little-endian)
//   [16..23] heading / speed / flags (not used here)
// Header: 0x80 0x81 <src=0x7F> 0x64 0x18 <payload..> <CRC>
bool parsePGN100(uint8_t* buf, int len, double& lat, double& lon) {
  if (len < 30) return false;
  if (buf[0] != 0x80 || buf[1] != 0x81) return false;
  if (buf[3] != 0x64) return false;     // PGN 100
  if (buf[4] != 0x18) return false;     // 24-byte payload
  memcpy(&lon, &buf[5],  8);
  memcpy(&lat, &buf[13], 8);
  if (lat < -90.0 || lat > 90.0)   return false;
  if (lon < -180.0 || lon > 180.0) return false;
  if (lat == 0.0 && lon == 0.0)    return false;
  return true;
}

// ── NMEA GGA parser ───────────────────────────────────────────
bool parseGGA(const char* s, double& lat, double& lon) {
  if (strncmp(s, "$GPGGA", 6) != 0 && strncmp(s, "$GNGGA", 6) != 0) return false;

  char buf[128];
  strncpy(buf, s, 127); buf[127] = '\0';

  char* f[16]; int n = 0;
  f[n++] = buf;
  for (char* p = buf; *p && n < 16; p++)
    if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
  if (n < 7) return false;
  if (f[6][0] == '0' || f[6][0] == '\0') return false;
  if (strlen(f[2]) < 4 || strlen(f[4]) < 5) return false;

  char d2[3] = { f[2][0], f[2][1], '\0' };
  lat = atof(d2) + atof(f[2] + 2) / 60.0;
  if (f[3][0] == 'S') lat = -lat;

  char d3[4] = { f[4][0], f[4][1], f[4][2], '\0' };
  lon = atof(d3) + atof(f[4] + 3) / 60.0;
  if (f[5][0] == 'W') lon = -lon;

  return true;
}

// ── Heading from consecutive fixes + antenna→nozzle offset ───
// Updates gHeading whenever we get a fix that moved at least 2 cm
// (below that, the direction would be dominated by GPS noise).
// Offset is applied by rotating a (fore_aft, lateral) vector into
// the world frame and shifting the check position accordingly.
void updateHeading(double lat, double lon) {
  if (hasPrevFix) {
    double dLat = lat - prevFixLat;
    double dLon = lon - prevFixLon;
    double mPerLon = M_PER_DEG_LAT * cos(lat * M_PI / 180.0);
    double dN = dLat * M_PER_DEG_LAT;
    double dE = dLon * mPerLon;
    if (sqrt(dN*dN + dE*dE) > 0.02) {
      double hdg = atan2(dE, dN) * 180.0 / M_PI;
      if (hdg < 0) hdg += 360.0;
      gHeading = hdg;
    }
  }
  prevFixLat = lat; prevFixLon = lon; hasPrevFix = true;
}

// Shifts the antenna position to the nozzle position in world frame.
// Convention:
//   gForeAft > 0  → nozzle is BEHIND the antenna (opposite heading direction)
//   gLateral > 0  → nozzle is RIGHT of centreline
// Heading h is in degrees, 0=N, 90=E, 180=S, 270=W.
void applyNozzleOffset(double lat, double lon, double& nozLat, double& nozLon) {
  if (gForeAft == 0.0 && gLateral == 0.0) {
    nozLat = lat; nozLon = lon; return;
  }
  double h = gHeading * M_PI / 180.0;
  // Forward unit vector (sin h, cos h); right unit vector (cos h, -sin h).
  double dE = -gForeAft * sin(h) + gLateral * cos(h);
  double dN = -gForeAft * cos(h) - gLateral * sin(h);
  double mPerLon = M_PER_DEG_LAT * cos(lat * M_PI / 180.0);
  nozLat = lat + dN / M_PER_DEG_LAT;
  nozLon = lon + dE / mPerLon;
}

// ── Flat-earth distance ────────────────────────────────────────
double distM(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * M_PER_DEG_LAT;
  double dLon = (lon2 - lon1) * mPerDegLon;
  return sqrt(dLat * dLat + dLon * dLon);
}

// ── Grid hit check — closest-approach detection ─────────────
// Rather than fire the moment the tractor enters a tree's radius,
// we track the minimum distance to the nearest tree and fire the
// instant that minimum stops decreasing. That's the geometric
// moment the nozzle was actually closest — nothing fuzzy about it.
//
// gHitRadius is reinterpreted as a *pass threshold*: closest
// approaches that exceed gHitRadius are ignored (e.g. driving
// between two rows, a tree in the next row might be 3m away at
// its closest but we don't want to fire). The search itself looks
// farther out so we can track a tree during approach.
void checkGrid(double lat, double lon) {
  if (relayActive) return;

  // Search radius wider than fire threshold so we can "see" the tree
  // on approach. 2.5× radius + 0.5m gives a comfortable buffer at
  // sim speed (1 m/s, 10 Hz fixes = 0.1 m per update).
  const double SEARCH_RANGE = gHitRadius * 2.5 + 0.5;

  int nearIdx = -1, nearR = -1, nearT = -1;
  double nearD = 1e9;

  if (gGridMode == MODE_AB) {
    if (!gHasLines || !gHasField || numIntersections == 0) return;
    double le, ln;
    latLonToLocal(lat, lon, le, ln);
    for (int i = 0; i < numIntersections; i++) {
      double de = le - intersections[i].e;
      double dn = ln - intersections[i].n;
      double d  = sqrt(de*de + dn*dn);
      if (d < SEARCH_RANGE && d < nearD) {
        nearD = d; nearIdx = i;
        nearR = i / gNumTrees;
        nearT = i % gNumTrees;
      }
    }
  } else {
    for (int r = 0; r < gNumRows; r++) {
      for (int t = 0; t < gNumTrees; t++) {
        double d = distM(lat, lon, grid[r][t].lat, grid[r][t].lon);
        if (d < SEARCH_RANGE && d < nearD) {
          nearD = d; nearR = r; nearT = t;
          nearIdx = r * gNumTrees + t;
        }
      }
    }
  }

  // No tree within search range → we're between rows / outside the
  // grid. Clear the approach tracker fully.
  if (nearIdx < 0) {
    approachIdx   = -1;
    approachMinD  = 1e9f;
    approachFired = false;
    return;
  }

  // Switched to a different nearest tree → lock focus on the new one.
  if (nearIdx != approachIdx) {
    approachIdx   = nearIdx;
    approachMinD  = nearD;
    approachFired = false;
    return;
  }

  // Already fired on this tree this pass → wait for tractor to leave range.
  if (approachFired) return;

  // Same tree, distance still dropping → update the minimum. Small
  // epsilon (5 mm) ignores GPS noise at steady distance.
  if (nearD < approachMinD - 0.005) {
    approachMinD = (float)nearD;
    return;
  }

  // Same tree, distance starting to grow → closest point was one fix ago.
  // Fire only if that closest approach was actually within the pass
  // threshold (gHitRadius). A 1 cm hysteresis avoids flapping on noise.
  if (nearD > approachMinD + 0.01 && approachMinD <= gHitRadius) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = true;
    relayStart  = millis();
    lastHit     = millis();
    lastRow     = nearR;
    lastTree    = nearT;
    lastDist    = approachMinD;
    totalHits++;
    approachFired = true;

    Serial.printf("HIT  Row %-2d  Tree %-3d  closest=%.3fm\n",
                  nearR, nearT, approachMinD);

    hitLog[hitLogHead] = { nearR, nearT, (float)approachMinD, lat, lon, (float)gHeading, millis() };
    hitLogHead = (hitLogHead + 1) % HIT_LOG_CAP;
    if (hitLogCount < HIT_LOG_CAP) hitLogCount++;

    char payload[80];
    snprintf(payload, sizeof(payload),
      "{\"row\":%d,\"tree\":%d,\"dist\":%.3f,\"lat\":%.7f,\"lon\":%.7f}",
      nearR, nearT, approachMinD, lat, lon);
    mqtt.publish(T_HIT, payload, true);
  }
}

// ── OLED ──────────────────────────────────────────────────────
void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);

  if (WiFi.status() != WL_CONNECTED) {
    oled.println("WiFi...");
  } else if (!gpsFix) {
    oled.println(WiFi.localIP().toString());
    oled.println(mqtt.connected() ? "MQTT OK" : "MQTT...");
    oled.println("Waiting for AgIO");
  } else {
    oled.printf("%.6f\n", curLat);
    oled.printf("%.6f\n", curLon);
    oled.println(mqtt.connected() ? "MQTT OK" : "MQTT--");
  }

  if (lastRow >= 0) {
    oled.printf("Last R%d T%d  #%d\n", lastRow, lastTree, totalHits);
  }

  if (relayActive) {
    oled.setTextSize(2);
    oled.setCursor(16, 48);
    oled.print("MARK!");
  }

  if (otaPending) {
    oled.setTextSize(1);
    oled.setCursor(0, 56);
    oled.print("OTA updating...");
  }

  oled.display();
}

// ── Build tree grid ───────────────────────────────────────────
void buildGrid() {
  mPerDegLon     = M_PER_DEG_LAT * cos(gOriginLat * M_PI / 180.0);
  bearingRad     = gRowBearing   * M_PI / 180.0;
  perpBearingRad = bearingRad    + M_PI / 2.0;

  for (int r = 0; r < gNumRows; r++) {
    for (int t = 0; t < gNumTrees; t++) {
      double along  = t * gTreeSpacing;
      double across = r * gRowSpacing;
      grid[r][t].lat = gOriginLat +
        (along * cos(bearingRad) + across * cos(perpBearingRad)) / M_PER_DEG_LAT;
      grid[r][t].lon = gOriginLon +
        (along * sin(bearingRad) + across * sin(perpBearingRad)) / mPerDegLon;
    }
  }
  Serial.printf("Grid: %d rows x %d trees  R=%.2fm  T=%.2fm  HIT=%.2fm\n",
    gNumRows, gNumTrees, gRowSpacing, gTreeSpacing, gHitRadius);
}

// ================================================================
//  AB MODE — cross-AB intersection grid imported from AgOpenGPS
//  (intersections[] / numIntersections declared above near grid[])
// ================================================================

// ── WGS84 → UTM forward projection ────────────────────────────
// Standard UTM formulas. Zone comes from AOG's Field.txt.
// Good to ~1 cm over any ordinary orchard block.
void latLonToUTM(double lat, double lon, int zone, double& utmE, double& utmN) {
  const double a  = 6378137.0;
  const double e2 = 0.00669437999014;
  const double k0 = 0.9996;
  double latR = lat * M_PI / 180.0;
  double lonR = lon * M_PI / 180.0;
  double lon0 = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;
  double N = a / sqrt(1.0 - e2 * sin(latR) * sin(latR));
  double T = tan(latR) * tan(latR);
  double C = e2 / (1.0 - e2) * cos(latR) * cos(latR);
  double A = cos(latR) * (lonR - lon0);
  double M = a * ((1.0 - e2/4.0 - 3.0*e2*e2/64.0) * latR
              - (3.0*e2/8.0 + 3.0*e2*e2/32.0) * sin(2.0*latR)
              + (15.0*e2*e2/256.0) * sin(4.0*latR));
  utmE = k0 * N * (A + (1.0 - T + C) * A*A*A / 6.0) + 500000.0;
  utmN = k0 * (M + N * tan(latR) * (A*A/2.0
             + (5.0 - T + 9.0*C) * A*A*A*A / 24.0));
  if (lat < 0) utmN += 10000000.0;
}

// GPS lat/lon → field-local easting/northing (same frame as AB lines).
// Flat-earth projection anchored at Field.txt $StartFix. Sub-millimetre
// accurate within 5km of the anchor, which easily covers any single field.
void latLonToLocal(double lat, double lon, double& localE, double& localN) {
  const double MPD = 111320.0;
  localN = (lat - gAnchorLat) * MPD;
  localE = (lon - gAnchorLon) * MPD * cos(gAnchorLat * M_PI / 180.0);
}

// ── WGS84 UTM inverse projection ──────────────────────────────
// Converts (utmE, utmN) within a given zone back to lat/lon. Used
// by /api/grid to return intersection coordinates that the browser
// map tab can plot without needing its own UTM code.
void utmToLatLon(double utmE, double utmN, int zone, double& lat, double& lon) {
  const double a  = 6378137.0;
  const double e2 = 0.00669437999014;
  const double k0 = 0.9996;
  const double e4 = e2 * e2;
  const double e6 = e4 * e2;
  const double ep2 = e2 / (1.0 - e2);
  // Heuristic: utmN > 5e6 implies southern hemisphere with 10 000 000 m offset
  // applied by the forward projection. Works for every non-equatorial field.
  bool southern = (utmN > 5000000.0);
  double y = utmN - (southern ? 10000000.0 : 0.0);
  double x = utmE - 500000.0;
  double lon0 = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;
  double M = y / k0;
  double mu = M / (a * (1.0 - e2/4.0 - 3.0*e4/64.0 - 5.0*e6/256.0));
  double e1 = (1.0 - sqrt(1.0 - e2)) / (1.0 + sqrt(1.0 - e2));
  double phi1 = mu
    + (3.0*e1/2.0   - 27.0*e1*e1*e1/32.0)                  * sin(2.0*mu)
    + (21.0*e1*e1/16.0 - 55.0*e1*e1*e1*e1/32.0)            * sin(4.0*mu)
    + (151.0*e1*e1*e1/96.0)                                * sin(6.0*mu)
    + (1097.0*e1*e1*e1*e1/512.0)                           * sin(8.0*mu);
  double sinPhi1 = sin(phi1);
  double cosPhi1 = cos(phi1);
  double tanPhi1 = tan(phi1);
  double N1 = a / sqrt(1.0 - e2 * sinPhi1 * sinPhi1);
  double T1 = tanPhi1 * tanPhi1;
  double C1 = ep2 * cosPhi1 * cosPhi1;
  double R1 = a * (1.0 - e2) / pow(1.0 - e2 * sinPhi1 * sinPhi1, 1.5);
  double D  = x / (N1 * k0);
  double latR = phi1 - (N1 * tanPhi1 / R1) *
    (D*D/2.0
     - (5.0 + 3.0*T1 + 10.0*C1 - 4.0*C1*C1 - 9.0*ep2) * D*D*D*D/24.0
     + (61.0 + 90.0*T1 + 298.0*C1 + 45.0*T1*T1 - 252.0*ep2 - 3.0*C1*C1) * D*D*D*D*D*D/720.0);
  double lonR = lon0 +
    (D - (1.0 + 2.0*T1 + C1) * D*D*D/6.0
       + (5.0 - 2.0*C1 + 28.0*T1 - 3.0*C1*C1 + 8.0*ep2 + 24.0*T1*T1) * D*D*D*D*D/120.0
    ) / cosPhi1;
  lat = latR * 180.0 / M_PI;
  lon = lonR * 180.0 / M_PI;
}

// Local E/N → lat/lon (inverse of latLonToLocal).
void localToLatLon(double localE, double localN, double& lat, double& lon) {
  const double MPD = 111320.0;
  lat = gAnchorLat + localN / MPD;
  lon = gAnchorLon + localE / (MPD * cos(gAnchorLat * M_PI / 180.0));
}

// ── AOG Field.txt parser ──────────────────────────────────────
// Modern AOG (v6+) stores $Offsets as 0,0 and the real anchor as StartFix.
// All coordinates in ABLines.txt / Boundary.txt are meters relative to this
// anchor, so we project with a flat-earth approximation from it. Example:
//   $FieldDir
//   MyField
//   $Offsets
//   0,0
//   Convergence
//   0
//   StartFix
//   -34.3145508,142.1541669
bool parseFieldTxt(const String& raw, double& anchorLat, double& anchorLon) {
  int idx = raw.indexOf("StartFix");
  if (idx < 0) return false;
  int nl = raw.indexOf('\n', idx);
  if (nl < 0) return false;
  int nl2 = raw.indexOf('\n', nl + 1);
  if (nl2 < 0) nl2 = raw.length();
  String line = raw.substring(nl + 1, nl2);
  line.trim();
  int c = line.indexOf(',');
  if (c < 0) return false;
  anchorLat = line.substring(0, c).toDouble();
  anchorLon = line.substring(c + 1).toDouble();
  return (fabs(anchorLat) > 0.001 && fabs(anchorLon) > 0.001);
}

// ── AOG ABLines.txt line finder ───────────────────────────────
// File format (header):
//   $LineCount
//   2
//   Row Direction,135.450,970.68,252.82
//   Tree Direction,45.450,-964.53,-252.07
// Each AB line: name,heading(deg),easting,northing
bool findABLine(const String& raw, const char* name,
                double& outE, double& outN, double& outHdg) {
  int cursor = 0;
  String target = String(name);
  target.trim();
  while (cursor < (int)raw.length()) {
    int nl = raw.indexOf('\n', cursor);
    if (nl < 0) nl = raw.length();
    String line = raw.substring(cursor, nl);
    cursor = nl + 1;
    line.trim();
    if (line.length() == 0)         continue;
    if (line.startsWith("$"))       continue;     // headers
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0) continue;
    // Some rows may legitimately have 3 fields (no trailing comma) — allow.
    String lname = line.substring(0, c1); lname.trim();
    if (lname != target) continue;
    double hdg = line.substring(c1 + 1, c2).toDouble();
    double e, n;
    if (c3 < 0) {
      e = line.substring(c2 + 1).toDouble();
      n = 0.0;  // malformed — skip
      continue;
    } else {
      e = line.substring(c2 + 1, c3).toDouble();
      n = line.substring(c3 + 1).toDouble();
    }
    outHdg = hdg; outE = e; outN = n;
    return true;
  }
  return false;
}

// ── List all AB line names in the raw text (for UI display) ──
// Writes up to maxOut comma-joined names into out (for dashboard).
void listABLineNames(const String& raw, char* out, size_t outSize) {
  out[0] = '\0';
  size_t used = 0;
  int cursor = 0;
  bool first = true;
  while (cursor < (int)raw.length() && used + 2 < outSize) {
    int nl = raw.indexOf('\n', cursor);
    if (nl < 0) nl = raw.length();
    String line = raw.substring(cursor, nl);
    cursor = nl + 1;
    line.trim();
    if (line.length() == 0 || line.startsWith("$")) continue;
    int c1 = line.indexOf(',');
    if (c1 < 0) continue;
    String name = line.substring(0, c1); name.trim();
    if (name.length() == 0) continue;
    if (!first) {
      if (used + 2 >= outSize) break;
      out[used++] = ','; out[used++] = ' '; out[used] = '\0';
    }
    size_t want = name.length();
    if (used + want >= outSize) break;
    memcpy(out + used, name.c_str(), want);
    used += want; out[used] = '\0';
    first = false;
  }
}

// ── AOG Boundary.txt parser ──────────────────────────────────
// AgOpenGPS stores field boundaries in a few slightly different shapes
// depending on version, but the common denominator is lines of
// easting,northing(,heading,lat,lon...) in local coordinates. We accept
// any non-$-prefixed line whose first two comma-separated fields parse
// as numbers, taking those as E/N. This handles v5-style, v6-style, and
// hand-edited files without fuss.
int parseBoundaryTxt(const String& raw, IPt* out, int maxOut) {
  int count = 0;
  int cursor = 0;
  while (cursor < (int)raw.length() && count < maxOut) {
    int nl = raw.indexOf('\n', cursor);
    if (nl < 0) nl = raw.length();
    String line = raw.substring(cursor, nl);
    cursor = nl + 1;
    line.trim();
    if (line.length() == 0 || line.startsWith("$")) continue;
    int c1 = line.indexOf(',');
    if (c1 < 0) continue;
    int c2 = line.indexOf(',', c1 + 1);
    String eStr = line.substring(0, c1);
    String nStr = (c2 < 0) ? line.substring(c1 + 1) : line.substring(c1 + 1, c2);
    eStr.trim(); nStr.trim();
    // Reject non-numeric rows (e.g. "1" as a count line)
    if (eStr.length() == 0 || nStr.length() == 0) continue;
    double e = eStr.toDouble();
    double n = nStr.toDouble();
    // If both parsed to literal 0, it's probably a header/count row — skip
    if (e == 0.0 && n == 0.0 && !(eStr == "0" && nStr == "0")) continue;
    out[count].e = (float)e;
    out[count].n = (float)n;
    count++;
  }
  return count;
}

// ── Ray-casting point-in-polygon test ────────────────────────
// Works on the boundary[] array (local E/N). Returns true if (e,n) is
// inside the polygon, edges inclusive-ish (doesn't matter at cm scale).
bool pointInBoundary(double e, double n) {
  if (!gHasBoundary || numBoundary < 3) return true;
  bool inside = false;
  for (int i = 0, j = numBoundary - 1; i < numBoundary; j = i++) {
    double xi = boundary[i].e, yi = boundary[i].n;
    double xj = boundary[j].e, yj = boundary[j].n;
    if (((yi > n) != (yj > n)) &&
        (e < (xj - xi) * (n - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

// ── Build the AB-mode intersection grid ───────────────────────
// Origin (row 0 / tree 0) = intersection of the Row and Tree AB lines,
// computed once in resolveActiveABLines() as gAbOriginE/N.
// Trees step ALONG the Row line (gTreeSpacing between trees).
// Rows step ALONG the Tree line (gRowSpacing between rows).
// Works correctly even if the two AB lines aren't exactly 90° apart.
void buildAbIntersections() {
  numIntersections = 0;
  lastRawIntersections = 0;
  if (!gHasLines) return;

  double rowHdgR  = gRowHdg  * M_PI / 180.0;
  double treeHdgR = gTreeHdg * M_PI / 180.0;
  double rowDE  = sin(rowHdgR),  rowDN  = cos(rowHdgR);
  double treeDE = sin(treeHdgR), treeDN = cos(treeHdgR);

  for (int r = 0; r < gNumRows; r++) {
    double baseE = gAbOriginE + r * gRowSpacing * treeDE;
    double baseN = gAbOriginN + r * gRowSpacing * treeDN;
    for (int t = 0; t < gNumTrees; t++) {
      if (numIntersections >= MAX_INTERSECTIONS) break;
      double iE = baseE + t * gTreeSpacing * rowDE;
      double iN = baseN + t * gTreeSpacing * rowDN;
      lastRawIntersections++;
      if (gHasBoundary && !pointInBoundary(iE, iN)) continue;
      intersections[numIntersections].e = (float)iE;
      intersections[numIntersections].n = (float)iN;
      numIntersections++;
    }
  }
  if (gHasBoundary) {
    Serial.printf("[AB] Built %d intersections (%d → %d after boundary clip, origin=%.2f,%.2f, row='%s' hdg=%.2f, tree='%s' hdg=%.2f)\n",
                  numIntersections, lastRawIntersections, numIntersections,
                  gAbOriginE, gAbOriginN,
                  gRowLineName, gRowHdg, gTreeLineName, gTreeHdg);
  } else {
    Serial.printf("[AB] Built %d intersections (origin=%.2f,%.2f, row='%s' hdg=%.2f, tree='%s' hdg=%.2f)\n",
                  numIntersections, gAbOriginE, gAbOriginN,
                  gRowLineName, gRowHdg, gTreeLineName, gTreeHdg);
  }
}

// ── Resolve named lines against gAbLinesRaw; update state ────
// Called after upload, after "apply", and on boot if we have raw.
// Also computes gAbOriginE/N = true intersection of the two AB lines,
// which is the origin (row 0 / tree 0) of the generated grid.
void resolveActiveABLines() {
  gHasLines = false;
  if (gAbLinesRaw.length() == 0) return;
  double rE, rN, rH, tE, tN, tH;
  bool rowOk  = findABLine(gAbLinesRaw, gRowLineName,  rE, rN, rH);
  bool treeOk = findABLine(gAbLinesRaw, gTreeLineName, tE, tN, tH);
  if (!rowOk || !treeOk) {
    Serial.printf("[AB] Could not find both lines (row=%s:%d, tree=%s:%d)\n",
                  gRowLineName, rowOk, gTreeLineName, treeOk);
    return;
  }
  gRowE = rE; gRowN = rN; gRowHdg = rH;
  gTreeE = tE; gTreeN = tN; gTreeHdg = tH;

  // Solve line-line intersection of the two AB lines to get the true
  // row-0 / tree-0 anchor. Works for any angle between them (AOG users
  // don't always draw them exactly perpendicular).
  double rowHdgR  = gRowHdg  * M_PI / 180.0;
  double treeHdgR = gTreeHdg * M_PI / 180.0;
  double rowDE  = sin(rowHdgR),  rowDN  = cos(rowHdgR);
  double treeDE = sin(treeHdgR), treeDN = cos(treeHdgR);
  double denom  = rowDE * treeDN - rowDN * treeDE;
  if (fabs(denom) < 1e-9) {
    Serial.println("[AB] Row and Tree AB lines are parallel — grid cannot be built");
    return;
  }
  double s = ((gTreeE - gRowE) * treeDN - (gTreeN - gRowN) * treeDE) / denom;
  gAbOriginE = gRowE + s * rowDE;
  gAbOriginN = gRowN + s * rowDN;
  gHasLines = true;
}

// ── PGN 239 — section state to AgOpenGPS ─────────────────────
void sendMachinePGN() {
  uint8_t sections = relayActive ? 0x01 : 0x00;
  uint8_t pkt[12];
  pkt[0]=0x80; pkt[1]=0x81; pkt[2]=0x7E; pkt[3]=239; pkt[4]=8;
  pkt[5]=sections; pkt[6]=0; pkt[7]=0; pkt[8]=0; pkt[9]=0; pkt[10]=0;
  uint8_t crc=0; for(int i=2;i<=10;i++) crc^=pkt[i]; pkt[11]=crc;

  IPAddress bcast = WiFi.localIP();
  IPAddress mask  = WiFi.subnetMask();
  for (int i = 0; i < 4; i++) bcast[i] |= ~mask[i];
  udp.beginPacket(bcast, 8888);
  udp.write(pkt, 12);
  udp.endPacket();
}

// ── MQTT callback ─────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("MQTT rx [%s]: %s\n", topic, msg.c_str());

  if (strcmp(topic, T_OTA_URL) == 0 && msg.length() > 10) {
    otaUrl = msg; otaPending = true;
    mqtt.publish(T_OTA_STATUS, "queued");
    return;
  }
  if (strcmp(topic, T_RESTART) == 0) {
    mqtt.publish(T_STATUS, "{\"event\":\"restarting\"}");
    delay(500); ESP.restart();
  }
}

// ── MQTT connect / reconnect ──────────────────────────────────
void mqttConnect() {
  if (mqtt.connected()) return;
  Serial.print("MQTT connecting...");
  if (mqtt.connect(DEF_MQTT_CLIENT, gMqttUser, gMqttPass,
                   T_STATUS, 0, true, "{\"online\":false}")) {
    Serial.println(" OK");
    mqtt.subscribe(T_OTA_URL);
    mqtt.subscribe(T_RESTART);
    char will[80];
    snprintf(will, sizeof(will),
      "{\"online\":true,\"fw\":\"%s\",\"ip\":\"%s\"}",
      FW_VERSION, WiFi.localIP().toString().c_str());
    mqtt.publish(T_STATUS, will, true);
  } else {
    Serial.printf(" failed rc=%d\n", mqtt.state());
  }
}

// ── OTA update ────────────────────────────────────────────────
void doOTA() {
  mqtt.publish(T_OTA_STATUS, "starting");
  updateOLED();
  WiFiClientSecure otaClient;
  otaClient.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  Serial.printf("OTA from: %s\n", otaUrl.c_str());
  t_httpUpdate_return ret = httpUpdate.update(otaClient, otaUrl);
  switch (ret) {
    case HTTP_UPDATE_OK:
      mqtt.publish(T_OTA_STATUS, "success"); break;
    case HTTP_UPDATE_FAILED:
      mqtt.publish(T_OTA_STATUS,
        ("failed: " + String(httpUpdate.getLastErrorString())).c_str()); break;
    case HTTP_UPDATE_NO_UPDATES:
      mqtt.publish(T_OTA_STATUS, "no-update"); break;
  }
  otaPending = false; otaUrl = "";
}

// ================================================================
//  WEB SERVER HANDLERS
// ================================================================

void handleRoot() {
  if (apMode) webServer.send_P(200, "text/html", SETUP_PAGE);
  else        webServer.send_P(200, "text/html", PAGE);
}

void handleStatus() {
  // In AB mode, the Active Grid card should show the *real* origin and
  // bearing — i.e. the intersection of the two AB lines and the Row line
  // heading. Fall back to stored Simple-mode values otherwise.
  double activeLat = gOriginLat, activeLon = gOriginLon, activeBrg = gRowBearing;
  if (gGridMode == MODE_AB && gHasLines && gHasField) {
    localToLatLon(gAbOriginE, gAbOriginN, activeLat, activeLon);
    activeBrg = gRowHdg;
  }
  char json[800];
  snprintf(json, sizeof(json),
    "{\"wifi\":%s,\"mqtt\":%s,\"fix\":%s,"
    "\"ip\":\"%s\",\"fw\":\"%s\",\"uptime\":%lu,"
    "\"hits\":%d,\"lastRow\":%d,\"lastTree\":%d,\"lastDist\":%.3f,"
    "\"lat\":%.7f,\"lon\":%.7f,\"hdg\":%.1f,"
    "\"cfg\":{\"lat\":%.7f,\"lon\":%.7f,\"brg\":%.1f,"
    "\"rs\":%.1f,\"ts\":%.1f,\"rows\":%d,\"trees\":%d,"
    "\"hr\":%.2f,\"rp\":%d,\"ssid\":\"%s\",\"udp\":%d,"
    "\"mode\":%d,\"nInt\":%d,\"hasField\":%s,\"hasLines\":%s,"
    "\"anchorLat\":%.7f,\"anchorLon\":%.7f,\"rowHdg\":%.1f,"
    "\"fa\":%.2f,\"lat_off\":%.2f}}",
    WiFi.status()==WL_CONNECTED?"true":"false",
    mqtt.connected()?"true":"false",
    gpsFix?"true":"false",
    WiFi.localIP().toString().c_str(),
    FW_VERSION, millis()/1000,
    totalHits, lastRow, lastTree, lastDist,
    curLat, curLon, gHeading,
    activeLat, activeLon, activeBrg,
    gRowSpacing, gTreeSpacing, gNumRows, gNumTrees,
    gHitRadius, gRelayPulse, gWifiSsid, gUdpPort,
    gGridMode, numIntersections,
    gHasField ? "true" : "false",
    gHasLines ? "true" : "false",
    gAnchorLat, gAnchorLon, gRowHdg,
    gForeAft, gLateral
  );
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", json);
}

void handleRelay() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  if (!relayActive) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = true;
    relayStart  = millis();
    Serial.println("WEB: relay test fired");
  }
  webServer.send(200, "text/plain", "OK");
}

// Simple JSON field extraction helper (no full JSON parser needed)
float jsonFloat(const String& body, const char* key) {
  int i = body.indexOf(String("\"") + key + "\"");
  if (i < 0) return 0;
  i = body.indexOf(':', i) + 1;
  return body.substring(i).toFloat();
}
String jsonStr(const String& body, const char* key) {
  int i = body.indexOf(String("\"") + key + "\"");
  if (i < 0) return "";
  i = body.indexOf('"', body.indexOf(':', i)) + 1;
  int j = body.indexOf('"', i);
  return body.substring(i, j);
}

void handleConfigGrid() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String body = webServer.arg("plain");
  gOriginLat   = jsonFloat(body, "lat");
  gOriginLon   = jsonFloat(body, "lon");
  gRowBearing  = jsonFloat(body, "brg");
  gRowSpacing  = jsonFloat(body, "rs");
  gTreeSpacing = jsonFloat(body, "ts");
  gNumRows     = (int)jsonFloat(body, "nr");
  gNumTrees    = (int)jsonFloat(body, "nt");
  gHitRadius   = jsonFloat(body, "hr");
  gRelayPulse  = (int)jsonFloat(body, "rp");
  // Optional nozzle offset (absent = leave unchanged, not zero)
  if (body.indexOf("\"fa\"") >= 0)      gForeAft = jsonFloat(body, "fa");
  if (body.indexOf("\"lat_off\"") >= 0) gLateral = jsonFloat(body, "lat_off");

  // Clamp to safe grid array bounds
  if (gNumRows  < 1)  gNumRows  = 1;
  if (gNumRows  > 20) gNumRows  = 20;
  if (gNumTrees < 1)  gNumTrees = 1;
  if (gNumTrees > 100) gNumTrees = 100;

  saveGridPrefs();
  buildGrid();
  if (gGridMode == MODE_AB && gHasLines) buildAbIntersections();
  Serial.println("WEB: grid config updated");
  webServer.send(200, "text/plain",
    "Grid saved and rebuilt: " + String(gNumRows) + " rows x " + String(gNumTrees) + " trees");
}

void handleConfigWifi() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String body = webServer.arg("plain");
  String ssid = jsonStr(body, "ssid");
  String pass = jsonStr(body, "pass");
  if (ssid.length() == 0) {
    webServer.send(400, "text/plain", "SSID cannot be empty"); return;
  }
  strlcpy(gWifiSsid, ssid.c_str(), sizeof(gWifiSsid));
  if (pass.length() > 0) strlcpy(gWifiPass, pass.c_str(), sizeof(gWifiPass));
  int newPort = (int)jsonFloat(body, "udp");
  if (newPort > 0 && newPort != gUdpPort) {
    gUdpPort = newPort;
    if (!apMode) { udp.stop(); udp.begin(gUdpPort); }
    Serial.printf("WEB: UDP port set to %d\n", gUdpPort);
  }
  saveWifiPrefs();
  webServer.send(200, "text/plain", "Network saved — rebooting in 2s...");
  delay(2000);
  ESP.restart();
}

void handleOtaWeb() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String url = jsonStr(webServer.arg("plain"), "url");
  if (url.length() < 10) {
    webServer.send(400, "text/plain", "Missing url"); return;
  }
  otaUrl     = url;
  otaPending = true;
  webServer.send(200, "text/plain", "OTA queued — flashing now, board will reboot");
  Serial.println("WEB: OTA triggered: " + url);
}

// ── Setup page (served in AP mode at http://192.168.4.1/) ────
void handleSetup() {
  webServer.send_P(200, "text/html", SETUP_PAGE);
}

// ── WiFi scan JSON for setup page ────────────────────────────
void handleScan() {
  int n = WiFi.scanNetworks();
  String out = "[";
  for (int i = 0; i < n && i < 20; i++) {
    if (i) out += ",";
    out += "{\"ssid\":\"";
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    out += ssid;
    out += "\",\"rssi\":";
    out += String(WiFi.RSSI(i));
    out += ",\"sec\":";
    out += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "false" : "true";
    out += "}";
  }
  out += "]";
  WiFi.scanDelete();
  webServer.send(200, "application/json", out);
}

// ── Direct .bin upload OTA ────────────────────────────────────
// The ESP32 WebServer upload handler runs chunk-by-chunk during
// the POST. We also register a completion handler that fires when
// the full upload is done.
void handleOtaUploadEnd() {
  if (Update.hasError()) {
    webServer.send(500, "text/plain",
      String("Upload failed: ") + Update.errorString());
  } else {
    webServer.send(200, "text/plain",
      "Upload OK — rebooting into new firmware");
    otaRebootPending = true;
    otaRebootAt = millis() + 1000;
  }
}

void handleOtaUploadChunk() {
  HTTPUpload& up = webServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("OTA upload start: %s\n", up.filename.c_str());
    if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-start");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA upload OK: %u bytes\n", up.totalSize);
      if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-success");
    } else {
      Update.printError(Serial);
      if (mqtt.connected()) mqtt.publish(T_OTA_STATUS, "upload-failed");
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    Serial.println("OTA upload aborted");
  }
}

// ── GitHub latest release lookup ─────────────────────────────
// Hits the releases/latest API, pulls tag_name and the .bin asset URL.
void handleOtaLatest() {
  if (WiFi.status() != WL_CONNECTED || apMode) {
    webServer.send(503, "application/json",
      "{\"error\":\"No internet in setup mode\"}");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setUserAgent("tree-marker-alr");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, GH_LATEST_URL)) {
    webServer.send(500, "application/json",
      "{\"error\":\"http.begin failed\"}");
    return;
  }
  int code = http.GET();
  if (code != 200) {
    http.end();
    char err[80];
    snprintf(err, sizeof(err), "{\"error\":\"HTTP %d\"}", code);
    webServer.send(502, "application/json", err);
    return;
  }
  String body = http.getString();
  http.end();

  // Parse tag_name
  String tag = "";
  int ti = body.indexOf("\"tag_name\"");
  if (ti >= 0) {
    int q1 = body.indexOf('"', body.indexOf(':', ti)) + 1;
    int q2 = body.indexOf('"', q1);
    if (q1 > 0 && q2 > q1) tag = body.substring(q1, q2);
  }
  // Find first browser_download_url ending in .bin
  String binUrl = "";
  int search = 0;
  while (true) {
    int k = body.indexOf("\"browser_download_url\"", search);
    if (k < 0) break;
    int q1 = body.indexOf('"', body.indexOf(':', k)) + 1;
    int q2 = body.indexOf('"', q1);
    if (q1 > 0 && q2 > q1) {
      String u = body.substring(q1, q2);
      if (u.endsWith(".bin")) { binUrl = u; break; }
    }
    search = q2 + 1;
  }
  char out[600];
  snprintf(out, sizeof(out),
    "{\"tag\":\"%s\",\"url\":\"%s\",\"current\":\"%s\"}",
    tag.c_str(), binUrl.c_str(), FW_VERSION);
  webServer.send(200, "application/json", out);
}

// ================================================================
//  AB FIELD IMPORT — accepts AOG ABLines.txt + Field.txt contents
// ================================================================

// POST /field/upload — JSON body: {"ablines":"...raw text...", "field":"...raw..."}
// Stores raw text in NVS, parses field offsets, resolves AB lines,
// rebuilds intersection grid.
// Escape a string for embedding inside a JSON string literal.
static String jsonEscape(const String& s) {
  String out; out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if      (c == '"')  out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((unsigned char)c < 0x20) { char b[8]; snprintf(b,sizeof(b),"\\u%04x",c); out += b; }
    else out += c;
  }
  return out;
}

// Helper: send a JSON error reply so the client never has to parse plain text.
static void sendJsonErr(int code, const char* err, const String& detail = "") {
  String body = String("{\"ok\":false,\"error\":\"") + jsonEscape(err) + "\"";
  if (detail.length() > 0) body += ",\"detail\":\"" + jsonEscape(detail) + "\"";
  body += "}";
  webServer.send(code, "application/json", body);
}

void handleFieldUpload() {
  if (webServer.method() != HTTP_POST) { sendJsonErr(405, "POST only"); return; }
  String body = webServer.arg("plain");
  if (body.length() == 0) { sendJsonErr(400, "Empty request body"); return; }

  // Extract "ablines" and "field" strings from JSON by locating keys.
  // Not a full JSON parser — expects simple {"ablines":"...","field":"..."}
  // with escaped newlines (\n) and quotes (\") in the strings.
  auto extract = [](const String& src, const char* key) -> String {
    String hit = String("\"") + key + "\"";
    int k = src.indexOf(hit);
    if (k < 0) return "";
    int q1 = src.indexOf('"', src.indexOf(':', k) + 1);
    if (q1 < 0) return "";
    String out; out.reserve(2048);
    int i = q1 + 1;
    while (i < (int)src.length()) {
      char c = src[i++];
      if (c == '\\' && i < (int)src.length()) {
        char n = src[i++];
        if      (n == 'n')  out += '\n';
        else if (n == 't')  out += '\t';
        else if (n == 'r')  out += '\r';
        else if (n == '"')  out += '"';
        else if (n == '\\') out += '\\';
        else                out += n;
      } else if (c == '"') {
        return out;
      } else {
        out += c;
      }
    }
    return out;
  };

  String ab     = extract(body, "ablines");
  String field  = extract(body, "field");
  String bndRaw = extract(body, "boundary");  // optional
  if (ab.length() == 0 || field.length() == 0) {
    sendJsonErr(400, "Missing ABLines.txt or Field.txt in payload"); return;
  }

  double aLat, aLon;
  if (!parseFieldTxt(field, aLat, aLon)) {
    // Include a preview of what we received so the user can see what went wrong.
    String head = field; head.trim();
    if (head.length() > 120) head = head.substring(0, 120) + "...";
    sendJsonErr(400,
      "Could not parse Field.txt — expected a 'StartFix' line followed by 'lat,lon'. "
      "Is this the Field.txt from your AgOpenGPS field folder?",
      head);
    return;
  }

  gAnchorLat = aLat; gAnchorLon = aLon;
  gFieldUtmE = 0; gFieldUtmN = 0; gFieldZone = 0;  // deprecated
  gHasField  = true;
  gAbLinesRaw = ab;

  // Boundary is optional — accept an empty/absent one as "clear boundary".
  if (bndRaw.length() > 0) {
    numBoundary = parseBoundaryTxt(bndRaw, boundary, MAX_BOUNDARY);
    gHasBoundary = (numBoundary >= 3);
    Serial.printf("[AB] Boundary parsed: %d vertices, active=%d\n",
                  numBoundary, gHasBoundary);
  } else {
    numBoundary = 0;
    gHasBoundary = false;
  }

  resolveActiveABLines();
  if (gHasLines) {
    gGridMode = MODE_AB;
    buildAbIntersections();
  }
  saveAbPrefs();

  char names[256];
  listABLineNames(gAbLinesRaw, names, sizeof(names));
  char out[520];
  snprintf(out, sizeof(out),
    "{\"ok\":true,\"field\":{\"anchorLat\":%.7f,\"anchorLon\":%.7f},"
    "\"linesFound\":\"%s\",\"rowOk\":%s,\"treeOk\":%s,"
    "\"intersections\":%d,\"rawIntersections\":%d,"
    "\"boundary\":%d,\"mode\":%d}",
    gAnchorLat, gAnchorLon, names,
    gHasLines ? "true" : "false", gHasLines ? "true" : "false",
    numIntersections, lastRawIntersections, numBoundary, gGridMode);
  webServer.send(200, "application/json", out);
  Serial.printf("[AB] Upload OK: anchor=(%.7f,%.7f), %d intersections, boundary=%d\n",
                gAnchorLat, gAnchorLon, numIntersections, numBoundary);
}

// POST /field/apply — {"rowName":"Row","treeName":"Tree","mode":1}
// Re-picks active lines by name and toggles mode. Intersections rebuild.
void handleFieldApply() {
  if (webServer.method() != HTTP_POST) {
    webServer.send(405, "text/plain", "POST only"); return;
  }
  String body = webServer.arg("plain");
  String rn = jsonStr(body, "rowName");
  String tn = jsonStr(body, "treeName");
  int newMode = (int)jsonFloat(body, "mode");
  if (rn.length() > 0) strlcpy(gRowLineName,  rn.c_str(), sizeof(gRowLineName));
  if (tn.length() > 0) strlcpy(gTreeLineName, tn.c_str(), sizeof(gTreeLineName));
  if (newMode == MODE_SIMPLE || newMode == MODE_AB) gGridMode = newMode;

  resolveActiveABLines();
  if (gGridMode == MODE_AB && gHasLines) buildAbIntersections();
  else                                   numIntersections = 0;
  saveAbPrefs();

  char out[200];
  snprintf(out, sizeof(out),
    "{\"ok\":true,\"mode\":%d,\"linesOk\":%s,\"intersections\":%d}",
    gGridMode, gHasLines ? "true" : "false", numIntersections);
  webServer.send(200, "application/json", out);
}

// GET /api/grid — planned tree positions as lat/lon, either from
// the Simple grid[][] or from intersections[] in AB mode. Designed
// for the dashboard Map tab to plot without needing its own maths.
// Builds the full response string in memory first (up to ~30KB at
// MAX_INTERSECTIONS=2000) — chunked streaming was flaky with many
// small sendContent calls, and we have plenty of RAM.
void handleGrid() {
  String out;
  out.reserve(32768);
  char head[96];
  snprintf(head, sizeof(head),
    "{\"mode\":%d,\"rows\":%d,\"trees\":%d,\"points\":[",
    gGridMode, gNumRows, gNumTrees);
  out += head;

  bool first = true;
  char buf[72];

  if (gGridMode == MODE_AB && gHasLines && gHasField) {
    for (int i = 0; i < numIntersections; i++) {
      double lat, lon;
      localToLatLon(intersections[i].e, intersections[i].n, lat, lon);
      int r = i / gNumTrees;
      int t = i % gNumTrees;
      int n = snprintf(buf, sizeof(buf), "%s[%.7f,%.7f,%d,%d]",
                       first ? "" : ",", lat, lon, r, t);
      out.concat(buf, n);
      first = false;
    }
  } else {
    for (int r = 0; r < gNumRows; r++) {
      for (int t = 0; t < gNumTrees; t++) {
        int n = snprintf(buf, sizeof(buf), "%s[%.7f,%.7f,%d,%d]",
                         first ? "" : ",", grid[r][t].lat, grid[r][t].lon, r, t);
        out.concat(buf, n);
        first = false;
      }
    }
  }

  out += "]";

  // Optional boundary polygon (AB mode only)
  out += ",\"boundary\":[";
  if (gGridMode == MODE_AB && gHasField && gHasBoundary) {
    for (int i = 0; i < numBoundary; i++) {
      double lat, lon;
      localToLatLon(boundary[i].e, boundary[i].n, lat, lon);
      int n = snprintf(buf, sizeof(buf), "%s[%.7f,%.7f]",
                       i ? "," : "", lat, lon);
      out.concat(buf, n);
    }
  }
  out += "]";

  // AB-line segments so the dashboard can overlay them on the map and
  // the user can see whether the Row/Tree lines match the actual trees.
  // Each segment starts ~one spacing-unit before the grid origin and
  // extends one spacing-unit beyond the last tree/row for visibility.
  auto writeAbSeg = [&](double dE, double dN, double extent) {
    double pad = (extent < 10.0) ? 10.0 : extent * 0.1;
    double aE = gAbOriginE - pad * dE;
    double aN = gAbOriginN - pad * dN;
    double bE = gAbOriginE + (extent + pad) * dE;
    double bN = gAbOriginN + (extent + pad) * dN;
    double lat1, lon1, lat2, lon2;
    localToLatLon(aE, aN, lat1, lon1);
    localToLatLon(bE, bN, lat2, lon2);
    char seg[160];
    int n = snprintf(seg, sizeof(seg), "[%.7f,%.7f],[%.7f,%.7f]",
                     lat1, lon1, lat2, lon2);
    out.concat(seg, n);
  };
  out += ",\"rowLine\":[";
  if (gGridMode == MODE_AB && gHasLines && gHasField) {
    double rowHdgR = gRowHdg * M_PI / 180.0;
    writeAbSeg(sin(rowHdgR), cos(rowHdgR),
               (double)(gNumTrees > 1 ? gNumTrees - 1 : 1) * gTreeSpacing);
  }
  out += "],\"treeLine\":[";
  if (gGridMode == MODE_AB && gHasLines && gHasField) {
    double treeHdgR = gTreeHdg * M_PI / 180.0;
    writeAbSeg(sin(treeHdgR), cos(treeHdgR),
               (double)(gNumRows > 1 ? gNumRows - 1 : 1) * gRowSpacing);
  }
  out += "]}";

  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "application/json", out);
}

// GET /api/hits — last 20 fires, newest first. Supports ?csv=1 for
// calibration use: drives a spreadsheet-friendly export with heading
// so you can average forward-pass vs return-pass marks to compute the
// residual antenna→nozzle offset error.
void handleHits() {
  bool csv = webServer.hasArg("csv");
  String out; out.reserve(2200);
  unsigned long now = millis();
  if (csv) {
    out = "row,tree,dist,lat,lon,heading,age_s\n";
    for (int i = 0; i < hitLogCount; i++) {
      int idx = (hitLogHead - 1 - i + HIT_LOG_CAP) % HIT_LOG_CAP;
      const HitLog& h = hitLog[idx];
      unsigned long ageS = (now - h.ms) / 1000UL;
      char buf[160];
      snprintf(buf, sizeof(buf),
        "%d,%d,%.3f,%.7f,%.7f,%.1f,%lu\n",
        h.row, h.tree, h.dist, h.lat, h.lon, h.heading, ageS);
      out += buf;
    }
    webServer.sendHeader("Content-Disposition", "attachment; filename=tree-marker-hits.csv");
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.send(200, "text/csv", out);
    return;
  }
  out = "[";
  for (int i = 0; i < hitLogCount; i++) {
    int idx = (hitLogHead - 1 - i + HIT_LOG_CAP) % HIT_LOG_CAP;
    const HitLog& h = hitLog[idx];
    unsigned long ageS = (now - h.ms) / 1000UL;
    char buf[220];
    snprintf(buf, sizeof(buf),
      "%s{\"row\":%d,\"tree\":%d,\"dist\":%.3f,\"lat\":%.7f,\"lon\":%.7f,\"hdg\":%.1f,\"age\":%lu}",
      i ? "," : "", h.row, h.tree, h.dist, h.lat, h.lon, h.heading, ageS);
    out += buf;
  }
  out += "]";
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", out);
}

// GET /field/state — what the dashboard Field tab shows
void handleFieldState() {
  char names[256];
  listABLineNames(gAbLinesRaw, names, sizeof(names));
  char out[700];
  snprintf(out, sizeof(out),
    "{\"mode\":%d,\"hasField\":%s,\"hasLines\":%s,"
    "\"field\":{\"anchorLat\":%.7f,\"anchorLon\":%.7f},"
    "\"rowName\":\"%s\",\"treeName\":\"%s\","
    "\"availableLines\":\"%s\",\"intersections\":%d,"
    "\"rawIntersections\":%d,\"hasBoundary\":%s,\"boundary\":%d,"
    "\"abSize\":%d}",
    gGridMode, gHasField ? "true" : "false", gHasLines ? "true" : "false",
    gAnchorLat, gAnchorLon,
    gRowLineName, gTreeLineName, names,
    numIntersections, lastRawIntersections,
    gHasBoundary ? "true" : "false", numBoundary,
    (int)gAbLinesRaw.length());
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", out);
}

// ── Start SoftAP captive-portal setup mode ───────────────────
void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  delay(200);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP mode: %s @ %s\n", AP_SSID, ip.toString().c_str());

  // Catch-all DNS → AP IP (makes phone/laptop auto-open the setup page)
  dnsServer.start(53, "*", ip);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextSize(1);
  oled.println("SETUP MODE");
  oled.println(AP_SSID);
  oled.println(ip.toString());
  oled.println("No password");
  oled.display();
}

// ── Try STA. Returns true if connected within STA_TIMEOUT_MS. ─
bool tryStationMode() {
  if (strlen(gWifiSsid) == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(gWifiSsid, gWifiPass);
  Serial.printf("WiFi STA: %s", gWifiSsid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > STA_TIMEOUT_MS) {
      Serial.println(" TIMEOUT");
      WiFi.disconnect(true, false);
      return false;
    }
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK  %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.printf("\n=== Tree Marker  v%s ===\n", FW_VERSION);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.printf("Tree Marker v%s\n", FW_VERSION);
    oled.println("Starting...");
    oled.display();
  }

  loadPrefs();
  buildGrid();
  if (gGridMode == MODE_AB) {
    resolveActiveABLines();
    if (gHasLines) buildAbIntersections();
  }

  // If DL button is held at boot, force AP setup mode.
  bool forceAP = (digitalRead(SETUP_BUTTON_PIN) == LOW);
  if (forceAP) Serial.println("DL button held — forcing AP setup mode");

  if (forceAP || !tryStationMode()) {
    startAP();
  }

  // Web server routes — same endpoints in both modes; /scan + /setup
  // are what the AP setup page uses.
  webServer.on("/",            HTTP_GET,  handleRoot);
  webServer.on("/setup",       HTTP_GET,  handleSetup);
  webServer.on("/scan",        HTTP_GET,  handleScan);
  webServer.on("/api/status",  HTTP_GET,  handleStatus);
  webServer.on("/api/hits",    HTTP_GET,  handleHits);
  webServer.on("/api/grid",    HTTP_GET,  handleGrid);
  webServer.on("/relay",       HTTP_POST, handleRelay);
  webServer.on("/config/grid", HTTP_POST, handleConfigGrid);
  webServer.on("/config/wifi", HTTP_POST, handleConfigWifi);
  webServer.on("/ota",         HTTP_POST, handleOtaWeb);
  webServer.on("/ota/latest",  HTTP_GET,  handleOtaLatest);
  webServer.on("/ota/upload",  HTTP_POST, handleOtaUploadEnd, handleOtaUploadChunk);
  webServer.on("/field/upload",HTTP_POST, handleFieldUpload);
  webServer.on("/field/apply", HTTP_POST, handleFieldApply);
  webServer.on("/field/state", HTTP_GET,  handleFieldState);
  // Captive-portal catch-all: anything unknown in AP mode redirects to setup
  webServer.onNotFound([]() {
    if (apMode) { webServer.send_P(200, "text/html", SETUP_PAGE); }
    else        { webServer.send(404, "text/plain", "Not found"); }
  });
  webServer.begin();

  if (apMode) {
    Serial.println("Web UI: http://192.168.4.1/ (setup)");
  } else {
    Serial.printf("Web UI: http://%s/\n", WiFi.localIP().toString().c_str());
    udp.begin(gUdpPort);
    Serial.printf("UDP listening on port %d\n", gUdpPort);

    tlsClient.setInsecure();
    mqtt.setServer(gMqttHost, gMqttPort);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    mqttConnect();
  }
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  // Relay off after pulse
  if (relayActive && millis() - relayStart > (unsigned long)gRelayPulse) {
    digitalWrite(RELAY_PIN, LOW);
    relayActive = false;
  }

  // OTA
  if (otaPending) doOTA();
  if (otaRebootPending && millis() >= otaRebootAt) {
    Serial.println("Rebooting after upload OTA...");
    delay(100);
    ESP.restart();
  }

  // Web server (always)
  webServer.handleClient();

  // AP setup mode: only run web + DNS. Skip GPS / MQTT / OLED updates below.
  if (apMode) {
    dnsServer.processNextRequest();
    static unsigned long lastApOled = 0;
    if (millis() - lastApOled > 500) {
      lastApOled = millis();
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.setTextSize(1);
      oled.println("SETUP MODE");
      oled.println(AP_SSID);
      oled.println("192.168.4.1");
      oled.display();
    }
    return;
  }

  // MQTT keep-alive
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttConnect();
    mqtt.loop();
  }

  // Read UDP from AgIO / AgOpenGPS
  int pkt = udp.parsePacket();
  if (pkt > 0) {
    uint8_t buf[256];
    int len = udp.read(buf, sizeof(buf) - 1);
    double lat, lon;
    if (parsePGN100(buf, len, lat, lon)) {
      // AOG NAV PGN 100 — works in RTK and simulator mode
      gpsFix = true; curLat = lat; curLon = lon;
      lastFixMs = millis();
      updateHeading(lat, lon);
      double nozLat, nozLon;
      applyNozzleOffset(lat, lon, nozLat, nozLon);
      checkGrid(nozLat, nozLon);
    } else {
      // Fallback: NMEA text ($GNGGA / $GPGGA) if AgIO NMEA-out enabled
      buf[len] = '\0';
      char* line = strtok((char*)buf, "\r\n");
      while (line) {
        if (parseGGA(line, lat, lon)) {
          gpsFix = true; curLat = lat; curLon = lon;
          lastFixMs = millis();
          updateHeading(lat, lon);
          double nozLat, nozLon;
          applyNozzleOffset(lat, lon, nozLat, nozLon);
          checkGrid(nozLat, nozLon);
        }
        line = strtok(nullptr, "\r\n");
      }
    }
  }

  // GPS fix timeout
  if (gpsFix && millis() - lastFixMs > 3000) {
    gpsFix = false;
    Serial.println("AgIO position lost");
  }

  // MQTT heartbeat every 30s
  if (millis() - lastHeartbeat > 30000) {
    lastHeartbeat = millis();
    char hb[120];
    snprintf(hb, sizeof(hb),
      "{\"online\":true,\"fw\":\"%s\",\"uptime\":%lu,\"hits\":%d,\"fix\":%s}",
      FW_VERSION, millis()/1000, totalHits, gpsFix?"true":"false");
    mqtt.publish(T_STATUS, hb, true);
  }

  // PGN 239 to AgOpenGPS every 100ms
  if (WiFi.status() == WL_CONNECTED && millis() - lastMachineSend > 100) {
    sendMachinePGN();
    lastMachineSend = millis();
  }

  // OLED every 400ms
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 400) {
    updateOLED();
    lastOled = millis();
  }
}
