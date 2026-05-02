// TreeMarker — merged firmware for KinCony ALR (ESP32-S3-WROOM-1U N16R8)
//
// Merges tree-marker-alr (MQTT / OLED / OTA / AgIO UDP GPS) with
// TreeMarker (NVS web UI / AB-line grid / DXF points / chunked UDP / calibration log)
//
// Board: ESP32S3 Dev Module (esp32 by Espressif 2.x)
// Flash: 16 MB  |  PSRAM: OPI  |  USB-CDC on Boot: Enabled
//
// Libraries (Sketch → Manage Libraries):
//   ArduinoJson       v6   by Benoit Blanchon
//   PubSubClient      by Nick O'Leary
//   Adafruit SSD1306  by Adafruit
//   Adafruit GFX      by Adafruit

#define FW_VERSION "2.0.0"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPUpdate.h>
#include <math.h>

// ─── KinCony ALR pin map ──────────────────────────────────────────────────────
#define RELAY_PIN    48    // on-board relay, active HIGH
#define OLED_SDA     39
#define OLED_SCL     38
#define DL_BUTTON     0    // hold LOW at boot → AP setup mode

// ─── Network ports ────────────────────────────────────────────────────────────
#define GPS_UDP_PORT    9999   // AgIO $GNGGA / $GNRMC broadcast
#define FIELD_UDP_PORT  8899   // sender.py JSON field data

// ─── OLED ─────────────────────────────────────────────────────────────────────
#define OLED_W   128
#define OLED_H    64
#define OLED_ADDR 0x3C

// ─── MQTT topics ──────────────────────────────────────────────────────────────
#define T_STATUS  "treemarker/status"
#define T_HIT     "treemarker/hit"
#define T_OTA_URL "treemarker/ota/url"
#define T_OTA_ST  "treemarker/ota/status"
#define T_RESTART "treemarker/restart"

// ─── First-boot defaults ──────────────────────────────────────────────────────
#define DEF_SSID         "AgOpenGPS"
#define DEF_PASS         "treeplant"
#define DEF_RELAY_MS     800
#define DEF_COOLDOWN_MS  3000
#define DEF_FORE_AFT     2.0f
#define DEF_LATERAL      0.0f
#define DEF_ROW_SPACING  6.7f
#define DEF_TREE_SPACING 3.0f
#define DEF_ROW_BEARING  90.0f
#define DEF_ORIG_LAT     -34.3166591
#define DEF_ORIG_LON     142.1562539
#define DEF_NUM_ROWS     10
#define DEF_NUM_TREES    50
#define DEF_LINES_EACH   40
#define DEF_MQTT_PORT    8883
#define DEF_ROW_NAME     "Row Direction"
#define DEF_TREE_NAME    "Tree Direction"

// ─── Capacities ───────────────────────────────────────────────────────────────
#define MAX_TREES      4000
#define MAX_ABLINES     200
#define MAX_BOUNDARY    500
#define MAX_CHUNKS       50
#define CHUNK_SIZE       80
#define CAL_LOG_SIZE     20
#define MQTT_HB_MS    30000   // heartbeat interval

// ─── Types ────────────────────────────────────────────────────────────────────
struct Point2D { double x, y; };
struct ABLine  { char name[64]; double headingRad, easting, northing; };
struct CalEntry{ double e, n; unsigned long ts; };
enum   GridMode{ GRID_SIMPLE = 0, GRID_ABLINES = 1, GRID_POINTS = 2 };

// ─── Settings (NVS-backed) ────────────────────────────────────────────────────
Preferences prefs;

char     wifiSSID[64], wifiPass[64];
char     mqttHost[128], mqttUser[64], mqttPass[64];
int      mqttPort;
int      relayMs, cooldownMs;
float    foreAft, lateral;
float    rowSpacing, treeSpacing;
float    rowBearing;     // degrees T, simple-grid mode
double   origLat, origLon;
int      numRows, numTrees;
int      linesEach;
char     rowLineName[64], treeLineName[64];
GridMode gridMode;

// ─── Runtime objects ──────────────────────────────────────────────────────────
WebServer          server(80);
WiFiUDP            gpsUdp;
WiFiUDP            fieldUdp;
WiFiClientSecure   tlsClient;
PubSubClient       mqtt(tlsClient);
Adafruit_SSD1306   oled(OLED_W, OLED_H, &Wire, -1);
bool               oledOk = false;

// UTM reference
int    utmZone     = 0;
double utmEastOff  = 0.0;
double utmNorthOff = 0.0;

// Grid
Point2D treePos[MAX_TREES];
int     treeCount    = 0;
ABLine  abLines[MAX_ABLINES];
int     abLineCount  = 0;
Point2D boundary[MAX_BOUNDARY];
int     boundaryCount= 0;
String  sourceMode   = "none";

// Chunked UDP reassembly
struct ChunkBuffer {
    bool    active = false;
    int     totalChunks = 0, totalPoints = 0, chunksReceived = 0;
    bool    received[MAX_CHUNKS];
    Point2D pts[MAX_TREES];
    void reset() {
        active = false;
        totalChunks = totalPoints = chunksReceived = 0;
        memset(received, 0, sizeof(received));
    }
} chunkBuf;

// GPS state
double  gpsLat = 0, gpsLon = 0;
float   gpsHeading = 0, gpsSpeed = 0;
bool    gpsValid  = false;
int     gpsQuality= 0;
double  nozzleE = 0, nozzleN = 0;

// Trigger state
int           hitCount   = 0;
bool          armed      = false;
int           nearestIdx = -1;
float         nearestDist= 1e9f;
unsigned long lastFire   = 0;
unsigned long relayOffAt = 0;

// Calibration log
CalEntry calLog[CAL_LOG_SIZE];
int      calLogHead  = 0;
int      calLogCount = 0;

// MQTT / OTA
unsigned long lastMqttHB = 0;
bool          otaPending = false;
String        otaUrl;

// ─── Forward declarations ─────────────────────────────────────────────────────
void publishHit(int treeIdx, float dist);
void buildSimpleGrid();
void buildABGrid();

// ─── UTM conversion (WGS84) ───────────────────────────────────────────────────
void latLonToUTM(double lat, double lon, int zone, double &E, double &N) {
    const double a  = 6378137.0, f = 1.0/298.257223563;
    const double e2 = 2*f - f*f, ep2 = e2/(1-e2), k0 = 0.9996;
    double lon0 = ((zone-1)*6.0 - 180.0 + 3.0) * M_PI / 180.0;
    double latR = lat*M_PI/180.0, lonR = lon*M_PI/180.0;
    double sl = sin(latR), cl = cos(latR);
    double Nv = a/sqrt(1 - e2*sl*sl);
    double T = tan(latR)*tan(latR), C = ep2*cl*cl, A = cl*(lonR-lon0);
    double M = a*((1 - e2/4   - 3*e2*e2/64    - 5*e2*e2*e2/256)   *latR
                 -(3*e2/8     + 3*e2*e2/32    + 45*e2*e2*e2/1024) *sin(2*latR)
                 +(15*e2*e2/256 + 45*e2*e2*e2/1024)               *sin(4*latR)
                 -(35*e2*e2*e2/3072)                               *sin(6*latR));
    E = k0*Nv*(A + (1-T+C)*A*A*A/6
                 + (5-18*T+T*T+72*C-58*ep2)*A*A*A*A*A/120) + 500000.0;
    N = k0*(M + Nv*tan(latR)*(A*A/2
                 + (5-T+9*C+4*C*C)*A*A*A*A/24
                 + (61-58*T+T*T+600*C-330*ep2)*A*A*A*A*A*A/720));
    if (lat < 0) N += 10000000.0;
}

void latLonToLocal(double lat, double lon, double &le, double &ln) {
    if (utmZone == 0) { le = 0; ln = 0; return; }
    double E, N;
    latLonToUTM(lat, lon, utmZone, E, N);
    le = E - utmEastOff;
    ln = N - utmNorthOff;
}

// ─── Point-in-polygon (ray casting) ──────────────────────────────────────────
bool pointInPolygon(double x, double y) {
    if (boundaryCount < 3) return true;
    int c = 0;
    for (int i = 0, j = boundaryCount-1; i < boundaryCount; j = i++) {
        if (((boundary[i].y > y) != (boundary[j].y > y)) &&
            (x < (boundary[j].x - boundary[i].x) * (y - boundary[i].y)
                 / (boundary[j].y - boundary[i].y) + boundary[i].x))
            c++;
    }
    return c & 1;
}

// ─── Grid builders ────────────────────────────────────────────────────────────
void buildSimpleGrid() {
    // Derive UTM zone from origin longitude
    utmZone = (int)((origLon + 180.0) / 6.0) + 1;
    double E0, N0;
    latLonToUTM(origLat, origLon, utmZone, E0, N0);
    utmEastOff  = E0;
    utmNorthOff = N0;
    // Origin is now (0,0); build rectangular grid
    double brg  = rowBearing * M_PI / 180.0;
    double rd_e = sin(brg), rd_n = cos(brg);    // along-row unit vector
    double cr_e = cos(brg), cr_n = -sin(brg);   // cross-row unit vector (90° CW)
    treeCount = 0;
    for (int r = 0; r < numRows && treeCount < MAX_TREES; r++) {
        for (int t = 0; t < numTrees && treeCount < MAX_TREES; t++) {
            treePos[treeCount++] = {
                t * treeSpacing * rd_e + r * rowSpacing * cr_e,
                t * treeSpacing * rd_n + r * rowSpacing * cr_n
            };
        }
    }
    sourceMode = "simple";
    gridMode   = GRID_SIMPLE;
    Serial.printf("Simple grid: %d trees (%d rows × %d)\n", treeCount, numRows, numTrees);
}

void buildABGrid() {
    ABLine *rowLine = nullptr, *treeLine = nullptr;
    for (int i = 0; i < abLineCount; i++) {
        if (strcmp(abLines[i].name, rowLineName)  == 0) rowLine  = &abLines[i];
        if (strcmp(abLines[i].name, treeLineName) == 0) treeLine = &abLines[i];
    }
    if (!rowLine  || !treeLine) {
        Serial.printf("WARN: AB lines '%s'/'%s' not found in %d loaded\n",
                      rowLineName, treeLineName, abLineCount);
        return;
    }
    double rd_e = sin(rowLine->headingRad),  rd_n = cos(rowLine->headingRad);
    double td_e = sin(treeLine->headingRad), td_n = cos(treeLine->headingRad);
    double rn_e = -rd_n, rn_n =  rd_e;   // perpendicular normals
    double tn_e = -td_n, tn_n =  td_e;
    treeCount = 0;
    for (int ri = -linesEach; ri <= linesEach; ri++) {
        double rp_e = rowLine->easting  + ri * rowSpacing  * rn_e;
        double rp_n = rowLine->northing + ri * rowSpacing  * rn_n;
        for (int ti = -linesEach; ti <= linesEach; ti++) {
            double tp_e = treeLine->easting  + ti * treeSpacing * tn_e;
            double tp_n = treeLine->northing + ti * treeSpacing * tn_n;
            double det  = -rd_e*td_n + rd_n*td_e;
            if (fabs(det) < 1e-10) continue;
            double dx = tp_e - rp_e, dy = tp_n - rp_n;
            double s  = (-dx*td_n + dy*td_e) / det;
            double ie = rp_e + s*rd_e, in_ = rp_n + s*rd_n;
            if (boundaryCount >= 3 && !pointInPolygon(ie, in_)) continue;
            if (treeCount < MAX_TREES) treePos[treeCount++] = {ie, in_};
        }
    }
    sourceMode = "ablines";
    gridMode   = GRID_ABLINES;
    Serial.printf("AB grid: %d intersections\n", treeCount);
}

// ─── NVS ─────────────────────────────────────────────────────────────────────
void loadSettings() {
    prefs.begin("tm", true);
    prefs.getString("ssid",     wifiSSID,     sizeof(wifiSSID));
    prefs.getString("pass",     wifiPass,     sizeof(wifiPass));
    prefs.getString("mqttHost", mqttHost,     sizeof(mqttHost));
    mqttPort    = prefs.getInt   ("mqttPort",  DEF_MQTT_PORT);
    prefs.getString("mqttUser", mqttUser,     sizeof(mqttUser));
    prefs.getString("mqttPass", mqttPass,     sizeof(mqttPass));
    relayMs     = prefs.getInt   ("relayMs",   DEF_RELAY_MS);
    cooldownMs  = prefs.getInt   ("cooldownMs",DEF_COOLDOWN_MS);
    foreAft     = prefs.getFloat ("foreAft",   DEF_FORE_AFT);
    lateral     = prefs.getFloat ("lateral",   DEF_LATERAL);
    rowSpacing  = prefs.getFloat ("rowSp",     DEF_ROW_SPACING);
    treeSpacing = prefs.getFloat ("treeSp",    DEF_TREE_SPACING);
    rowBearing  = prefs.getFloat ("rowBrg",    DEF_ROW_BEARING);
    origLat     = prefs.getDouble("origLat",   DEF_ORIG_LAT);
    origLon     = prefs.getDouble("origLon",   DEF_ORIG_LON);
    numRows     = prefs.getInt   ("numRows",   DEF_NUM_ROWS);
    numTrees    = prefs.getInt   ("numTrees",  DEF_NUM_TREES);
    linesEach   = prefs.getInt   ("linesEach", DEF_LINES_EACH);
    gridMode    = (GridMode)prefs.getInt("gridMode", GRID_SIMPLE);
    prefs.getString("rowLine",  rowLineName,  sizeof(rowLineName));
    prefs.getString("treeLine", treeLineName, sizeof(treeLineName));
    prefs.end();

    if (!strlen(wifiSSID))     strcpy(wifiSSID,     DEF_SSID);
    if (!strlen(wifiPass))     strcpy(wifiPass,     DEF_PASS);
    if (!strlen(rowLineName))  strcpy(rowLineName,  DEF_ROW_NAME);
    if (!strlen(treeLineName)) strcpy(treeLineName, DEF_TREE_NAME);
}

void saveSettings() {
    prefs.begin("tm", false);
    prefs.putString("ssid",     wifiSSID);
    prefs.putString("pass",     wifiPass);
    prefs.putString("mqttHost", mqttHost);
    prefs.putInt   ("mqttPort", mqttPort);
    prefs.putString("mqttUser", mqttUser);
    prefs.putString("mqttPass", mqttPass);
    prefs.putInt   ("relayMs",  relayMs);
    prefs.putInt   ("cooldownMs", cooldownMs);
    prefs.putFloat ("foreAft",  foreAft);
    prefs.putFloat ("lateral",  lateral);
    prefs.putFloat ("rowSp",    rowSpacing);
    prefs.putFloat ("treeSp",   treeSpacing);
    prefs.putFloat ("rowBrg",   rowBearing);
    prefs.putDouble("origLat",  origLat);
    prefs.putDouble("origLon",  origLon);
    prefs.putInt   ("numRows",  numRows);
    prefs.putInt   ("numTrees", numTrees);
    prefs.putInt   ("linesEach",linesEach);
    prefs.putInt   ("gridMode", (int)gridMode);
    prefs.putString("rowLine",  rowLineName);
    prefs.putString("treeLine", treeLineName);
    prefs.end();
}

// ─── OLED ─────────────────────────────────────────────────────────────────────
void oledShow(const String &l1, const String &l2 = "",
              const String &l3 = "", const String &l4 = "") {
    if (!oledOk) return;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0,  0); oled.println(l1);
    oled.setCursor(0, 16); oled.println(l2);
    oled.setCursor(0, 32); oled.println(l3);
    oled.setCursor(0, 48); oled.println(l4);
    oled.display();
}

void updateOLED() {
    if (!oledOk) return;
    oledShow(
        String("TreeMarker v") + FW_VERSION,
        gpsValid ? (String("GPS q=") + gpsQuality + " hdg=" + (int)gpsHeading)
                 : "GPS: no fix",
        String("Trees:") + treeCount + " Hits:" + hitCount,
        armed ? ">>> ARMED <<<" : ("Mode:" + sourceMode)
    );
}

// ─── GPS via UDP (AgIO broadcast, port 9999) ──────────────────────────────────
static float nmeaDeg(const char *f, const char *h) {
    if (!f || strlen(f) < 3) return 0.0f;
    float r = atof(f);
    int   d = (int)(r / 100.0f);
    float v = d + (r - d*100.0f) / 60.0f;
    if (h[0]=='S' || h[0]=='W') v = -v;
    return v;
}

void processNMEA(const char *raw) {
    static char buf[128];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *star = strrchr(buf, '*');
    if (star) *star = '\0';

    char *f[20]; int fc = 0;
    char *tok = strtok(buf, ",");
    while (tok && fc < 20) { f[fc++] = tok; tok = strtok(nullptr, ","); }
    if (fc < 2) return;

    if (strstr(f[0], "GGA") && fc >= 7) {
        gpsQuality = atoi(f[6]);
        if (gpsQuality > 0) {
            gpsLat   = nmeaDeg(f[2], f[3]);
            gpsLon   = nmeaDeg(f[4], f[5]);
            gpsValid = true;
            double le, ln;
            latLonToLocal(gpsLat, gpsLon, le, ln);
            float headR = gpsHeading * (float)M_PI / 180.0f;
            nozzleE = le - foreAft * sin(headR) + lateral * cos(headR);
            nozzleN = ln - foreAft * cos(headR) - lateral * sin(headR);
        } else {
            gpsValid = false;
        }

    } else if (strstr(f[0], "RMC") && fc >= 9) {
        if (f[2][0] == 'A') {
            gpsSpeed   = atof(f[7]) * 0.51444f;
            gpsHeading = atof(f[8]);
        }
    }
}

void readGpsUDP() {
    int len = gpsUdp.parsePacket();
    if (len <= 0) return;
    static char buf[512];
    int n = gpsUdp.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';

    // Buffer may contain multiple newline-separated sentences
    char *p = buf;
    while (*p) {
        char *nl = strpbrk(p, "\r\n");
        if (nl) *nl = '\0';
        if (*p == '$') processNMEA(p);
        if (nl) { p = nl + 1; while (*p == '\r' || *p == '\n') p++; }
        else break;
    }
}

// ─── Field data via UDP (sender.py, port 8899) ────────────────────────────────
void processFieldUDP() {
    int len = fieldUdp.parsePacket();
    if (len <= 0) return;
    static char buf[8192];
    int n = fieldUdp.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';

    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, buf) != DeserializationError::Ok) {
        Serial.println("Field JSON parse error");
        return;
    }

    // UTM reference block
    if (doc.containsKey("utmZone")) {
        utmZone    = doc["utmZone"]  | 0;
        utmEastOff = doc["eastOff"]  | 0.0;
        utmNorthOff= doc["northOff"] | 0.0;
        Serial.printf("UTM zone=%d  E-off=%.1f  N-off=%.1f\n",
                      utmZone, utmEastOff, utmNorthOff);
    }

    const char *mode = doc["mode"] | "";

    // ── AB lines ──────────────────────────────────────────────────────────────
    if (strcmp(mode, "ablines") == 0) {
        abLineCount = 0;
        for (JsonObject ln : doc["lines"].as<JsonArray>()) {
            if (abLineCount >= MAX_ABLINES) break;
            ABLine &ab = abLines[abLineCount++];
            strlcpy(ab.name, ln["name"] | "", sizeof(ab.name));
            ab.headingRad = ((double)(ln["heading"] | 0.0f)) * M_PI / 180.0;
            ab.easting    = ln["easting"]  | 0.0;
            ab.northing   = ln["northing"] | 0.0;
        }
        Serial.printf("AB lines received: %d\n", abLineCount);

        if (doc.containsKey("boundary")) {
            boundaryCount = 0;
            for (JsonObject pt : doc["boundary"].as<JsonArray>()) {
                if (boundaryCount >= MAX_BOUNDARY) break;
                boundary[boundaryCount++] = {pt["e"] | 0.0, pt["n"] | 0.0};
            }
        }
        buildABGrid();

    // ── Points ────────────────────────────────────────────────────────────────
    } else if (strcmp(mode, "points") == 0) {
        int totalChunks = doc["totalChunks"] | 1;
        int totalPoints = doc["totalPoints"] | 0;
        int chunkIdx    = doc["chunkIdx"]    | 0;

        // Header-only packet (chunkIdx == -1)
        if (chunkIdx < 0) {
            if (!chunkBuf.active || chunkBuf.totalChunks != totalChunks) {
                chunkBuf.reset();
                chunkBuf.active      = true;
                chunkBuf.totalChunks = totalChunks;
                chunkBuf.totalPoints = totalPoints;
            }
            return;
        }

        if (totalChunks == 1) {
            treeCount = 0;
            for (JsonObject pt : doc["points"].as<JsonArray>()) {
                if (treeCount >= MAX_TREES) break;
                treePos[treeCount++] = {pt["e"] | 0.0, pt["n"] | 0.0};
            }
            sourceMode = "points";
            gridMode   = GRID_POINTS;
            Serial.printf("Points (single): %d\n", treeCount);
            return;
        }

        // Multi-chunk assembly
        if (!chunkBuf.active || chunkBuf.totalChunks != totalChunks) {
            chunkBuf.reset();
            chunkBuf.active      = true;
            chunkBuf.totalChunks = totalChunks;
            chunkBuf.totalPoints = totalPoints;
        }
        if (chunkIdx < MAX_CHUNKS && !chunkBuf.received[chunkIdx]) {
            chunkBuf.received[chunkIdx] = true;
            chunkBuf.chunksReceived++;
            int base = chunkIdx * CHUNK_SIZE;
            for (JsonObject pt : doc["points"].as<JsonArray>()) {
                if (base >= MAX_TREES) break;
                chunkBuf.pts[base++] = {pt["e"] | 0.0, pt["n"] | 0.0};
            }
        }
        if (chunkBuf.chunksReceived >= totalChunks) {
            int nc = min(chunkBuf.totalPoints, MAX_TREES);
            memcpy(treePos, chunkBuf.pts, nc * sizeof(Point2D));
            treeCount  = nc;
            sourceMode = "points";
            gridMode   = GRID_POINTS;
            chunkBuf.reset();
            Serial.printf("Points (chunked): %d\n", treeCount);
        }

    // ── Standalone boundary update ─────────────────────────────────────────────
    } else if (strcmp(mode, "boundary") == 0) {
        boundaryCount = 0;
        for (JsonObject pt : doc["points"].as<JsonArray>()) {
            if (boundaryCount >= MAX_BOUNDARY) break;
            boundary[boundaryCount++] = {pt["e"] | 0.0, pt["n"] | 0.0};
        }
        if (gridMode == GRID_ABLINES && abLineCount > 0) buildABGrid();
    }
}

// ─── Trigger logic (minimum-distance / closest-point detection) ───────────────
void checkTrigger() {
    if (!gpsValid || treeCount == 0) return;

    unsigned long now = millis();

    if (relayOffAt && now >= relayOffAt) {
        digitalWrite(RELAY_PIN, LOW);
        relayOffAt = 0;
    }

    if (now - lastFire < (unsigned long)cooldownMs) return;

    // Find nearest tree
    float bestDist = 1e9f; int bestIdx = -1;
    for (int i = 0; i < treeCount; i++) {
        float de = (float)(nozzleE - treePos[i].x);
        float dn = (float)(nozzleN - treePos[i].y);
        float d  = sqrtf(de*de + dn*dn);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
    }

    if (!armed && bestDist <= 0.5f) {
        armed = true; nearestIdx = bestIdx; nearestDist = bestDist;
        return;
    }
    if (!armed) return;

    if (bestIdx != nearestIdx) {
        if (bestDist > 0.8f) armed = false;
        else { nearestIdx = bestIdx; nearestDist = bestDist; }
        return;
    }

    if (bestDist <= nearestDist) { nearestDist = bestDist; return; }

    // Distance increasing — nozzle just passed the closest point
    if (bestDist > nearestDist + 0.02f) {
        digitalWrite(RELAY_PIN, HIGH);
        relayOffAt = now + (unsigned long)relayMs;
        lastFire   = now;
        hitCount++;
        armed = false;

        int slot = calLogHead % CAL_LOG_SIZE;
        calLog[slot] = {nozzleE, nozzleN, now};
        calLogHead++;
        if (calLogCount < CAL_LOG_SIZE) calLogCount++;

        Serial.printf("FIRE tree=%d dist=%.3f hits=%d\n",
                      nearestIdx, nearestDist, hitCount);
        publishHit(nearestIdx, nearestDist);
    }
}

// ─── MQTT ─────────────────────────────────────────────────────────────────────
void publishHit(int idx, float dist) {
    if (!mqtt.connected() || !strlen(mqttHost)) return;
    char p[256];
    snprintf(p, sizeof(p),
        "{\"tree\":%d,\"dist\":%.3f,\"hits\":%d,"
        "\"lat\":%.7f,\"lon\":%.7f,\"e\":%.3f,\"n\":%.3f}",
        idx, dist, hitCount, gpsLat, gpsLon, nozzleE, nozzleN);
    mqtt.publish(T_HIT, p);
}

void publishStatus() {
    if (!mqtt.connected() || !strlen(mqttHost)) return;
    char p[256];
    snprintf(p, sizeof(p),
        "{\"fw\":\"%s\",\"hits\":%d,\"trees\":%d,"
        "\"gps\":%s,\"mode\":\"%s\",\"uptime\":%lu}",
        FW_VERSION, hitCount, treeCount,
        gpsValid ? "true" : "false",
        sourceMode.c_str(), millis()/1000UL);
    mqtt.publish(T_STATUS, p);
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
    String t(topic), msg((char*)payload, len);
    if (t == T_OTA_URL) {
        otaPending = true; otaUrl = msg;
        Serial.printf("OTA queued: %s\n", msg.c_str());
        mqtt.publish(T_OTA_ST, "queued");
    } else if (t == T_RESTART) {
        ESP.restart();
    }
}

bool mqttConnect() {
    if (!strlen(mqttHost) || mqtt.connected()) return mqtt.connected();
    mqtt.setServer(mqttHost, mqttPort);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    tlsClient.setInsecure();
    String cid = String("treemarker-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqtt.connect(cid.c_str(), mqttUser, mqttPass)) {
        mqtt.subscribe(T_OTA_URL);
        mqtt.subscribe(T_RESTART);
        Serial.println("MQTT connected");
        return true;
    }
    return false;
}

void doOTA() {
    if (!otaPending) return;
    otaPending = false;
    mqtt.publish(T_OTA_ST, "starting");
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    t_httpUpdate_return ret = httpUpdate.update(tlsClient, otaUrl);
    if (ret == HTTP_UPDATE_OK)
        mqtt.publish(T_OTA_ST, "success");
    else {
        String err = "failed: " + httpUpdate.getLastErrorString();
        mqtt.publish(T_OTA_ST, err.c_str());
    }
}

// ─── Web UI ───────────────────────────────────────────────────────────────────
static const char CSS[] PROGMEM =
    "<style>"
    "body{font-family:Arial,sans-serif;max-width:840px;margin:20px auto;padding:0 12px}"
    "h2{background:#2a7;color:#fff;padding:10px 14px;border-radius:5px;margin:0 0 10px}"
    "h3{color:#2a7;border-bottom:2px solid #2a7;padding-bottom:3px}"
    "table{border-collapse:collapse;width:100%;margin-bottom:14px}"
    "td,th{border:1px solid #ccc;padding:5px 9px;text-align:left}"
    "th{background:#f0f0f0}"
    "input[type=text],input[type=number],input[type=password]"
      "{width:98%;padding:4px;box-sizing:border-box}"
    "select{width:99%;padding:4px}"
    ".btn{background:#2a7;color:#fff;padding:7px 20px;border:none;"
         "border-radius:4px;cursor:pointer;font-size:1em}"
    ".btn-red{background:#c33}"
    ".ok{color:#2a7;font-weight:bold}.warn{color:#c33;font-weight:bold}"
    "nav{margin:8px 0}nav a{margin-right:12px;color:#2a7;text-decoration:none}"
    "</style>";

String pageHead(const char *title) {
    String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                 "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                 "<title>TreeMarker</title>");
    h += FPSTR(CSS);
    h += F("</head><body><h2>TreeMarker v");
    h += FW_VERSION;
    h += F(" &mdash; ");
    h += title;
    h += F("</h2><nav>"
           "<a href='/'>&#8962; Status</a>"
           "<a href='/spray'>Spray</a>"
           "<a href='/nozzle'>Nozzle</a>"
           "<a href='/simple'>Simple Grid</a>"
           "<a href='/abnames'>AB Names</a>"
           "<a href='/wifi'>WiFi</a>"
           "<a href='/mqtt'>MQTT</a>"
           "</nav><hr>");
    return h;
}

// ── / ─────────────────────────────────────────────────────────────────────────
void handleRoot() {
    String pg = pageHead("Status");
    pg += F("<h3>System</h3><table>");
    pg += "<tr><th>Hits fired</th><td><b>" + String(hitCount) + "</b></td></tr>";
    pg += "<tr><th>GPS fix</th><td class='" +
          String(gpsValid ? "ok'>&#10003; quality " + String(gpsQuality)
                           : "warn'>&#10007; no fix") + "</td></tr>";
    if (gpsValid) {
        pg += "<tr><th>Heading</th><td>" + String(gpsHeading,1) + " &deg;T</td></tr>";
        pg += "<tr><th>Speed</th><td>"   + String(gpsSpeed,2)   + " m/s</td></tr>";
        pg += "<tr><th>Nozzle E</th><td>"+ String(nozzleE,3)    + " m</td></tr>";
        pg += "<tr><th>Nozzle N</th><td>"+ String(nozzleN,3)    + " m</td></tr>";
    }
    pg += "<tr><th>Grid mode</th><td>" + sourceMode + "</td></tr>";
    pg += "<tr><th>Tree positions</th><td>" + String(treeCount) + "</td></tr>";
    pg += "<tr><th>Boundary pts</th><td>"  + String(boundaryCount) + "</td></tr>";
    pg += "<tr><th>UTM zone</th><td>" +
          (utmZone ? String(utmZone) : String("(not set)")) + "</td></tr>";
    pg += "<tr><th>MQTT</th><td class='" +
          String(mqtt.connected() ? "ok'>Connected" : "warn'>Disconnected") + "</td></tr>";
    pg += "<tr><th>Armed</th><td>" +
          String(armed ? "<span class='warn'>ARMED</span>" : "no") + "</td></tr>";
    pg += F("</table>");

    pg += F("<h3>Calibration Log</h3>"
            "<table><tr><th>#</th><th>Nozzle E (m)</th><th>Nozzle N (m)</th><th>Age (s)</th></tr>");
    unsigned long now = millis();
    for (int i = 0; i < calLogCount; i++) {
        int idx = (calLogHead - 1 - i + CAL_LOG_SIZE*2) % CAL_LOG_SIZE;
        pg += "<tr><td>" + String(i+1) + "</td><td>" +
              String(calLog[idx].e, 3) + "</td><td>" +
              String(calLog[idx].n, 3) + "</td><td>" +
              String((now - calLog[idx].ts) / 1000UL) + "</td></tr>";
    }
    if (!calLogCount) pg += F("<tr><td colspan='4'>No marks yet</td></tr>");
    pg += F("</table>");

    pg += F("<br><form action='/reboot' method='post'>"
            "<input type='submit' class='btn btn-red' value='Reboot' "
            "onclick=\"return confirm('Reboot now?')\">"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}

// ── /spray ────────────────────────────────────────────────────────────────────
void handleSprayGet() {
    String pg = pageHead("Spray Settings");
    pg += F("<form action='/spray' method='post'><table>"
            "<tr><th>Relay ON (ms)</th>"
            "<td><input type='number' name='relayMs' min='50' max='5000' value='");
    pg += relayMs;
    pg += F("'></td></tr><tr><th>Cooldown (ms)</th>"
            "<td><input type='number' name='cooldownMs' min='100' max='30000' value='");
    pg += cooldownMs;
    pg += F("'></td></tr></table><br>"
            "<input type='submit' class='btn' value='Save'></form></body></html>");
    server.send(200, "text/html", pg);
}
void handleSprayPost() {
    if (server.hasArg("relayMs"))    relayMs    = server.arg("relayMs").toInt();
    if (server.hasArg("cooldownMs")) cooldownMs = server.arg("cooldownMs").toInt();
    saveSettings();
    server.sendHeader("Location", "/"); server.send(303);
}

// ── /nozzle ───────────────────────────────────────────────────────────────────
void handleNozzleGet() {
    String pg = pageHead("Nozzle Offset");
    pg += F("<p><b>Calibration:</b> Drive the same row in both directions. "
            "Half the gap between the two spray marks = fore/aft error. "
            "Positive fore/aft = nozzle is behind the GPS antenna. "
            "Positive lateral = nozzle is right of antenna centreline.</p>"
            "<form action='/nozzle' method='post'><table>"
            "<tr><th>Fore/Aft (m)</th>"
            "<td><input type='number' step='0.01' name='foreAft' value='");
    pg += String(foreAft, 2);
    pg += F("'></td></tr><tr><th>Lateral (m)</th>"
            "<td><input type='number' step='0.01' name='lateral' value='");
    pg += String(lateral, 2);
    pg += F("'></td></tr></table><br>"
            "<input type='submit' class='btn' value='Save'></form></body></html>");
    server.send(200, "text/html", pg);
}
void handleNozzlePost() {
    if (server.hasArg("foreAft")) foreAft  = server.arg("foreAft").toFloat();
    if (server.hasArg("lateral")) lateral  = server.arg("lateral").toFloat();
    saveSettings();
    server.sendHeader("Location", "/"); server.send(303);
}

// ── /simple ───────────────────────────────────────────────────────────────────
void handleSimpleGet() {
    String pg = pageHead("Simple Grid");
    pg += F("<p>Rectangular grid defined by an origin GPS coordinate. "
            "Row 0 / Tree 0 is the origin. Rows extend perpendicular to the bearing.</p>"
            "<form action='/simple' method='post'><table>");
    pg += "<tr><th>Origin Latitude</th><td><input type='number' step='0.0000001' name='origLat' value='" + String(origLat, 7) + "'></td></tr>";
    pg += "<tr><th>Origin Longitude</th><td><input type='number' step='0.0000001' name='origLon' value='" + String(origLon, 7) + "'></td></tr>";
    pg += "<tr><th>Row bearing (&deg;T)</th><td><input type='number' step='0.1' name='rowBrg' value='" + String(rowBearing, 1) + "'></td></tr>";
    pg += "<tr><th>Row spacing (m)</th><td><input type='number' step='0.01' name='rowSp' value='" + String(rowSpacing, 2) + "'></td></tr>";
    pg += "<tr><th>Tree spacing (m)</th><td><input type='number' step='0.01' name='treeSp' value='" + String(treeSpacing, 2) + "'></td></tr>";
    pg += "<tr><th>Number of rows</th><td><input type='number' name='numRows' min='1' max='500' value='" + String(numRows) + "'></td></tr>";
    pg += "<tr><th>Trees per row</th><td><input type='number' name='numTrees' min='1' max='500' value='" + String(numTrees) + "'></td></tr>";
    pg += F("</table><br><input type='submit' class='btn' value='Save &amp; Build Grid'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}
void handleSimplePost() {
    if (server.hasArg("origLat"))  origLat     = server.arg("origLat").toDouble();
    if (server.hasArg("origLon"))  origLon     = server.arg("origLon").toDouble();
    if (server.hasArg("rowBrg"))   rowBearing  = server.arg("rowBrg").toFloat();
    if (server.hasArg("rowSp"))    rowSpacing  = server.arg("rowSp").toFloat();
    if (server.hasArg("treeSp"))   treeSpacing = server.arg("treeSp").toFloat();
    if (server.hasArg("numRows"))  numRows     = server.arg("numRows").toInt();
    if (server.hasArg("numTrees")) numTrees    = server.arg("numTrees").toInt();
    saveSettings();
    buildSimpleGrid();
    server.sendHeader("Location", "/"); server.send(303);
}

// ── /abnames ──────────────────────────────────────────────────────────────────
void handleABNamesGet() {
    String pg = pageHead("AB Line Names");
    pg += F("<p>Names must exactly match AgOpenGPS AB line names (case-sensitive). "
            "Used when sender.py sends in AB-lines mode.</p>"
            "<form action='/abnames' method='post'><table>");
    pg += "<tr><th>Row AB line</th><td><input type='text' name='rowLine' value='" + String(rowLineName) + "'></td></tr>";
    pg += "<tr><th>Tree AB line</th><td><input type='text' name='treeLine' value='" + String(treeLineName) + "'></td></tr>";
    pg += "<tr><th>Lines each side of AB origin</th><td><input type='number' name='linesEach' min='1' max='200' value='" + String(linesEach) + "'></td></tr>";
    pg += F("</table><br><input type='submit' class='btn' value='Save'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}
void handleABNamesPost() {
    if (server.hasArg("rowLine"))
        strlcpy(rowLineName,  server.arg("rowLine").c_str(),  sizeof(rowLineName));
    if (server.hasArg("treeLine"))
        strlcpy(treeLineName, server.arg("treeLine").c_str(), sizeof(treeLineName));
    if (server.hasArg("linesEach"))
        linesEach = server.arg("linesEach").toInt();
    saveSettings();
    server.sendHeader("Location", "/"); server.send(303);
}

// ── /wifi ─────────────────────────────────────────────────────────────────────
void handleWifiGet() {
    String pg = pageHead("WiFi");
    pg += F("<form action='/wifi' method='post'><table>"
            "<tr><th>SSID</th><td><input type='text' name='ssid' value='");
    pg += String(wifiSSID);
    pg += F("'></td></tr><tr><th>Password</th>"
            "<td><input type='password' name='pass' placeholder='(blank = keep current)'>"
            "</td></tr></table><br>"
            "<input type='submit' class='btn' value='Save &amp; Reboot'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}
void handleWifiPost() {
    if (server.hasArg("ssid"))
        strlcpy(wifiSSID, server.arg("ssid").c_str(), sizeof(wifiSSID));
    if (server.hasArg("pass") && server.arg("pass").length() > 0)
        strlcpy(wifiPass, server.arg("pass").c_str(), sizeof(wifiPass));
    saveSettings();
    server.send(200, "text/html",
        "<html><body><h2>TreeMarker</h2><p>Saved. Rebooting&hellip;</p></body></html>");
    delay(1000);
    ESP.restart();
}

// ── /mqtt ─────────────────────────────────────────────────────────────────────
void handleMqttGet() {
    String pg = pageHead("MQTT Settings");
    pg += F("<p>HiveMQ Cloud or any MQTT broker with TLS on port 8883. "
            "Leave host blank to disable MQTT entirely.</p>"
            "<form action='/mqtt' method='post'><table>"
            "<tr><th>Broker host</th><td><input type='text' name='mqttHost' value='");
    pg += String(mqttHost);
    pg += F("'></td></tr><tr><th>Port</th>"
            "<td><input type='number' name='mqttPort' value='");
    pg += mqttPort;
    pg += F("'></td></tr><tr><th>Username</th>"
            "<td><input type='text' name='mqttUser' value='");
    pg += String(mqttUser);
    pg += F("'></td></tr><tr><th>Password</th>"
            "<td><input type='password' name='mqttPass' placeholder='(blank = keep current)'>"
            "</td></tr></table><br>"
            "<input type='submit' class='btn' value='Save'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}
void handleMqttPost() {
    if (server.hasArg("mqttHost"))
        strlcpy(mqttHost, server.arg("mqttHost").c_str(), sizeof(mqttHost));
    if (server.hasArg("mqttPort"))
        mqttPort = server.arg("mqttPort").toInt();
    if (server.hasArg("mqttUser"))
        strlcpy(mqttUser, server.arg("mqttUser").c_str(), sizeof(mqttUser));
    if (server.hasArg("mqttPass") && server.arg("mqttPass").length() > 0)
        strlcpy(mqttPass, server.arg("mqttPass").c_str(), sizeof(mqttPass));
    saveSettings();
    server.sendHeader("Location", "/"); server.send(303);
}

// ── /reboot ───────────────────────────────────────────────────────────────────
void handleReboot() {
    server.send(200, "text/html",
        "<html><body><h2>TreeMarker</h2><p>Rebooting&hellip;</p></body></html>");
    delay(500);
    ESP.restart();
}

// ─── setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
    pinMode(DL_BUTTON, INPUT_PULLUP);

    Wire.begin(OLED_SDA, OLED_SCL);
    oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (oledOk) { oled.clearDisplay(); oled.display(); }
    // Try 0x3D if 0x3C failed (some SSD1306 clones)
    if (!oledOk) {
        oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);
        if (oledOk) { oled.clearDisplay(); oled.display(); }
    }

    oledShow("TreeMarker v" FW_VERSION, "Booting...");
    loadSettings();

    // AP mode: hold DL button at boot
    bool apMode = (digitalRead(DL_BUTTON) == LOW);

    if (!apMode) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiSSID, wifiPass);
        oledShow("TreeMarker v" FW_VERSION, "Connecting WiFi...", wifiSSID);
        int t = 0;
        while (WiFi.status() != WL_CONNECTED && t < 40) {
            delay(500); t++; Serial.print('.');
        }
        if (WiFi.status() != WL_CONNECTED) apMode = true;
    }

    if (apMode) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("TreeMarker-Setup", "setup1234");
        String ip = WiFi.softAPIP().toString();
        Serial.printf("\nAP mode  IP=%s\n", ip.c_str());
        oledShow("TreeMarker v" FW_VERSION, "AP Mode",
                 "SSID: TreeMarker-Setup", "IP: " + ip);
    } else {
        String ip = WiFi.localIP().toString();
        Serial.printf("\nWiFi OK  IP=%s\n", ip.c_str());
        oledShow("TreeMarker v" FW_VERSION, "WiFi OK", "IP: " + ip, "Waiting for GPS...");
    }

    gpsUdp.begin(GPS_UDP_PORT);
    fieldUdp.begin(FIELD_UDP_PORT);

    if (strlen(mqttHost)) mqttConnect();

    server.on("/",        HTTP_GET,  handleRoot);
    server.on("/spray",   HTTP_GET,  handleSprayGet);
    server.on("/spray",   HTTP_POST, handleSprayPost);
    server.on("/nozzle",  HTTP_GET,  handleNozzleGet);
    server.on("/nozzle",  HTTP_POST, handleNozzlePost);
    server.on("/simple",  HTTP_GET,  handleSimpleGet);
    server.on("/simple",  HTTP_POST, handleSimplePost);
    server.on("/abnames", HTTP_GET,  handleABNamesGet);
    server.on("/abnames", HTTP_POST, handleABNamesPost);
    server.on("/wifi",    HTTP_GET,  handleWifiGet);
    server.on("/wifi",    HTTP_POST, handleWifiPost);
    server.on("/mqtt",    HTTP_GET,  handleMqttGet);
    server.on("/mqtt",    HTTP_POST, handleMqttPost);
    server.on("/reboot",  HTTP_POST, handleReboot);
    server.begin();

    // Build simple grid immediately so the board is ready without sender.py
    if (gridMode == GRID_SIMPLE) buildSimpleGrid();

    Serial.printf("TreeMarker v%s ready  GPS=%d  Field=%d\n",
                  FW_VERSION, GPS_UDP_PORT, FIELD_UDP_PORT);
}

// ─── loop ─────────────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();
    readGpsUDP();
    processFieldUDP();
    checkTrigger();

    // MQTT heartbeat + reconnect
    if (strlen(mqttHost)) {
        if (!mqtt.connected()) mqttConnect();
        mqtt.loop();
        unsigned long now = millis();
        if (now - lastMqttHB > MQTT_HB_MS) {
            publishStatus();
            lastMqttHB = now;
        }
    }

    // OTA (non-blocking: flag set in mqttCallback, executed here outside ISR context)
    if (otaPending) doOTA();

    // OLED refresh every 500 ms
    static unsigned long lastOled = 0;
    unsigned long now = millis();
    if (now - lastOled > 500) { updateOLED(); lastOled = now; }
}
