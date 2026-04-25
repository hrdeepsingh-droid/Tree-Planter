// TreeMarker.ino  –  ESP32 tree-planting mark-out system
// Receives field data via UDP, reads NMEA GPS on Serial2,
// fires relay at GPIO26 when nozzle passes a tree position.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <math.h>

// ─── Fixed pins ───────────────────────────────────────────────────────────────
#define RELAY_PIN     26
#define LED_PIN        2
#define GPS_RX_PIN    16
#define GPS_TX_PIN    17
#define UDP_PORT    8899

// ─── First-boot defaults (written to NVS only when key is absent) ─────────────
#define DEF_SSID          "AgOpenGPS"
#define DEF_PASS          "treeplant"
#define DEF_RELAY_MS      800
#define DEF_COOLDOWN_MS   3000
#define DEF_FORE_AFT      2.0f
#define DEF_LATERAL       0.0f
#define DEF_ROW_SPACING   6.7f
#define DEF_TREE_SPACING  3.0f
#define DEF_LINES_EACH    40
#define DEF_ROW_NAME      "Row Direction"
#define DEF_TREE_NAME     "Tree Direction"

// ─── Capacities ───────────────────────────────────────────────────────────────
#define MAX_TREES      4000
#define MAX_ABLINES     200
#define MAX_BOUNDARY    500
#define MAX_CHUNKS       50   // max UDP chunks for points mode
#define CHUNK_SIZE       80   // points per chunk (must match Python sender)
#define CAL_LOG_SIZE     20

// ─── Types ────────────────────────────────────────────────────────────────────
struct Point2D { double x, y; };

struct ABLine {
    char   name[64];
    double headingRad;
    double easting;
    double northing;
};

struct CalEntry { double e, n; unsigned long ts; };

// ─── Settings (NVS-backed) ────────────────────────────────────────────────────
Preferences prefs;

char    wifiSSID[64];
char    wifiPass[64];
int     relayMs;
int     cooldownMs;
float   foreAft;
float   lateral;
float   rowSpacing;
float   treeSpacing;
int     linesEach;
char    rowLineName[64];
char    treeLineName[64];

// ─── Runtime state ────────────────────────────────────────────────────────────
WebServer server(80);
WiFiUDP   udp;

// UTM reference received from Python sender
int    utmZone    = 0;
double utmEastOff = 0.0;
double utmNorthOff= 0.0;

// Tree positions
Point2D treePos[MAX_TREES];
int     treeCount = 0;

// Boundary polygon
Point2D boundary[MAX_BOUNDARY];
int     boundaryCount = 0;

// AB lines received from AgOpenGPS sender
ABLine  abLines[MAX_ABLINES];
int     abLineCount = 0;

String  sourceMode = "none";   // "ablines" or "points"

// Chunked reassembly buffer for points mode
struct ChunkBuffer {
    bool    active        = false;
    int     totalChunks   = 0;
    int     totalPoints   = 0;
    Point2D pts[MAX_TREES];
    bool    received[MAX_CHUNKS];
    int     chunksReceived= 0;
    void reset() {
        active = false; totalChunks = 0; totalPoints = 0; chunksReceived = 0;
        memset(received, 0, sizeof(received));
    }
} chunkBuf;

// GPS state
double  gpsLat     = 0.0;
double  gpsLon     = 0.0;
float   gpsHeading = 0.0f;   // degrees true north, from RMC
float   gpsSpeed   = 0.0f;   // m/s
bool    gpsValid   = false;
double  nozzleE    = 0.0;    // local easting of computed nozzle position
double  nozzleN    = 0.0;

// Relay / trigger state
int           hitCount   = 0;
bool          armed      = false;
int           nearestIdx = -1;
float         nearestDist= 1e9f;
unsigned long lastFire   = 0;
unsigned long relayOffAt = 0;

// Calibration log (ring buffer)
CalEntry calLog[CAL_LOG_SIZE];
int      calLogHead  = 0;
int      calLogCount = 0;

// ─── UTM → local coordinates (WGS84) ─────────────────────────────────────────
void latLonToLocal(double lat, double lon, double &le, double &ln) {
    const double a   = 6378137.0;
    const double f   = 1.0 / 298.257223563;
    const double e2  = 2*f - f*f;
    const double ep2 = e2 / (1.0 - e2);
    const double k0  = 0.9996;

    double lon0 = ((utmZone - 1) * 6.0 - 180.0 + 3.0) * M_PI / 180.0;
    double latR = lat * M_PI / 180.0;
    double lonR = lon * M_PI / 180.0;

    double sinLat = sin(latR), cosLat = cos(latR);
    double N  = a / sqrt(1.0 - e2 * sinLat * sinLat);
    double T  = tan(latR) * tan(latR);
    double C  = ep2 * cosLat * cosLat;
    double A  = cosLat * (lonR - lon0);

    double M  = a * (
          (1.0 - e2/4.0 - 3.0*e2*e2/64.0  - 5.0*e2*e2*e2/256.0)  * latR
        - (3.0*e2/8.0   + 3.0*e2*e2/32.0  + 45.0*e2*e2*e2/1024.0) * sin(2*latR)
        + (15.0*e2*e2/256.0 + 45.0*e2*e2*e2/1024.0)                * sin(4*latR)
        - (35.0*e2*e2*e2/3072.0)                                    * sin(6*latR));

    double E_utm = k0 * N * (A
        + (1.0 - T + C)                               * A*A*A / 6.0
        + (5.0 - 18.0*T + T*T + 72.0*C - 58.0*ep2)   * A*A*A*A*A / 120.0)
        + 500000.0;

    double N_utm = k0 * (M + N * tan(latR) * (
          A*A / 2.0
        + (5.0 - T + 9.0*C + 4.0*C*C)                           * A*A*A*A / 24.0
        + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*ep2)           * A*A*A*A*A*A / 720.0));

    if (lat < 0.0) N_utm += 10000000.0;

    le = E_utm - utmEastOff;
    ln = N_utm - utmNorthOff;
}

// ─── Point-in-polygon (ray casting) ──────────────────────────────────────────
bool pointInPolygon(double x, double y) {
    if (boundaryCount < 3) return true;   // no boundary → everywhere valid
    int crossings = 0;
    for (int i = 0, j = boundaryCount - 1; i < boundaryCount; j = i++) {
        double xi = boundary[i].x, yi = boundary[i].y;
        double xj = boundary[j].x, yj = boundary[j].y;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
            crossings++;
    }
    return (crossings & 1) == 1;
}

// ─── Build tree grid from two AB line families ────────────────────────────────
void buildGridFromABLines() {
    ABLine *rowLine  = nullptr;
    ABLine *treeLine = nullptr;
    for (int i = 0; i < abLineCount; i++) {
        if (strcmp(abLines[i].name, rowLineName)  == 0) rowLine  = &abLines[i];
        if (strcmp(abLines[i].name, treeLineName) == 0) treeLine = &abLines[i];
    }
    if (!rowLine) {
        Serial.printf("WARN: row AB line '%s' not found\n", rowLineName);
        return;
    }
    if (!treeLine) {
        Serial.printf("WARN: tree AB line '%s' not found\n", treeLineName);
        return;
    }

    // Direction unit vectors along each line
    double rd_e = sin(rowLine->headingRad),  rd_n = cos(rowLine->headingRad);
    double td_e = sin(treeLine->headingRad), td_n = cos(treeLine->headingRad);

    // Perpendicular (normal) unit vectors for offsetting parallel lines
    double rn_e = -rd_n, rn_n =  rd_e;
    double tn_e = -td_n, tn_n =  td_e;

    treeCount = 0;

    for (int ri = -linesEach; ri <= linesEach; ri++) {
        // Row-family line ri: passes through point rp, direction rd
        double rp_e = rowLine->easting  + ri * rowSpacing * rn_e;
        double rp_n = rowLine->northing + ri * rowSpacing * rn_n;

        for (int ti = -linesEach; ti <= linesEach; ti++) {
            // Tree-family line ti: passes through point tp, direction td
            double tp_e = treeLine->easting  + ti * treeSpacing * tn_e;
            double tp_n = treeLine->northing + ti * treeSpacing * tn_n;

            // Solve:  rp + s*rd = tp + t*td
            //   s*rd_e - t*td_e = tp_e - rp_e
            //   s*rd_n - t*td_n = tp_n - rp_n
            double det = -rd_e * td_n + rd_n * td_e;
            if (fabs(det) < 1e-10) continue;   // parallel lines

            double dx = tp_e - rp_e;
            double dy = tp_n - rp_n;
            double s  = (-dx * td_n + dy * td_e) / det;

            double ie = rp_e + s * rd_e;
            double in_ = rp_n + s * rd_n;

            if (boundaryCount >= 3 && !pointInPolygon(ie, in_)) continue;

            if (treeCount < MAX_TREES)
                treePos[treeCount++] = {ie, in_};
        }
    }
    Serial.printf("Grid built: %d intersections\n", treeCount);
}

// ─── NVS helpers ─────────────────────────────────────────────────────────────
void loadSettings() {
    prefs.begin("tm", true);
    prefs.getString("ssid",     wifiSSID,     sizeof(wifiSSID));
    prefs.getString("pass",     wifiPass,     sizeof(wifiPass));
    relayMs     = prefs.getInt  ("relayMs",   DEF_RELAY_MS);
    cooldownMs  = prefs.getInt  ("cooldownMs",DEF_COOLDOWN_MS);
    foreAft     = prefs.getFloat("foreAft",   DEF_FORE_AFT);
    lateral     = prefs.getFloat("lateral",   DEF_LATERAL);
    rowSpacing  = prefs.getFloat("rowSp",     DEF_ROW_SPACING);
    treeSpacing = prefs.getFloat("treeSp",    DEF_TREE_SPACING);
    linesEach   = prefs.getInt  ("linesEach", DEF_LINES_EACH);
    prefs.getString("rowLine",  rowLineName,  sizeof(rowLineName));
    prefs.getString("treeLine", treeLineName, sizeof(treeLineName));
    prefs.end();

    if (strlen(wifiSSID)     == 0) strcpy(wifiSSID,     DEF_SSID);
    if (strlen(wifiPass)     == 0) strcpy(wifiPass,     DEF_PASS);
    if (strlen(rowLineName)  == 0) strcpy(rowLineName,  DEF_ROW_NAME);
    if (strlen(treeLineName) == 0) strcpy(treeLineName, DEF_TREE_NAME);
}

void saveSettings() {
    prefs.begin("tm", false);
    prefs.putString("ssid",     wifiSSID);
    prefs.putString("pass",     wifiPass);
    prefs.putInt   ("relayMs",  relayMs);
    prefs.putInt   ("cooldownMs", cooldownMs);
    prefs.putFloat ("foreAft",  foreAft);
    prefs.putFloat ("lateral",  lateral);
    prefs.putFloat ("rowSp",    rowSpacing);
    prefs.putFloat ("treeSp",   treeSpacing);
    prefs.putInt   ("linesEach",linesEach);
    prefs.putString("rowLine",  rowLineName);
    prefs.putString("treeLine", treeLineName);
    prefs.end();
}

// ─── NMEA parsing ─────────────────────────────────────────────────────────────
static char nmeaBuf[128];
static int  nmeaIdx = 0;

static float nmea_deg(const char *field, const char *hemi) {
    if (!field || strlen(field) < 3) return 0.0f;
    float raw = atof(field);
    int   deg = (int)(raw / 100.0f);
    float val = deg + (raw - deg * 100.0f) / 60.0f;
    if (hemi[0] == 'S' || hemi[0] == 'W') val = -val;
    return val;
}

void processNMEA(const char *sentence) {
    static char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Strip checksum
    char *star = strrchr(buf, '*');
    if (star) *star = '\0';

    char *fields[20];
    int   fc = 0;
    char *tok = strtok(buf, ",");
    while (tok && fc < 20) { fields[fc++] = tok; tok = strtok(nullptr, ","); }
    if (fc < 2) return;

    if (strstr(fields[0], "GGA") && fc >= 7) {
        float lat = nmea_deg(fields[2], fields[3]);
        float lon = nmea_deg(fields[4], fields[5]);
        int   fix = atoi(fields[6]);
        if (fix > 0 && utmZone > 0) {
            gpsLat   = lat;
            gpsLon   = lon;
            gpsValid = true;

            double le, ln;
            latLonToLocal(lat, lon, le, ln);

            // Shift antenna position to nozzle using current heading
            float headR = gpsHeading * (float)M_PI / 180.0f;
            // Nozzle is foreAft metres behind (subtract along heading)
            // and lateral metres to the right (add perpendicular)
            nozzleE = le - foreAft * sin(headR) + lateral * cos(headR);
            nozzleN = ln - foreAft * cos(headR) - lateral * sin(headR);
        }
    } else if (strstr(fields[0], "RMC") && fc >= 9) {
        if (fields[2][0] == 'A') {
            gpsSpeed   = atof(fields[7]) * 0.51444f;   // knots → m/s
            gpsHeading = atof(fields[8]);
        } else {
            gpsValid = false;
        }
    }
}

void readGPS() {
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n' || c == '\r') {
            if (nmeaIdx > 5) {
                nmeaBuf[nmeaIdx] = '\0';
                if (nmeaBuf[0] == '$') processNMEA(nmeaBuf);
            }
            nmeaIdx = 0;
        } else if (nmeaIdx < (int)sizeof(nmeaBuf) - 1) {
            nmeaBuf[nmeaIdx++] = c;
        }
    }
}

// ─── Trigger logic (minimum-distance detection) ───────────────────────────────
void checkTrigger() {
    if (!gpsValid || treeCount == 0) return;

    unsigned long now = millis();

    // Release relay after configured duration
    if (relayOffAt && now >= relayOffAt) {
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN,   LOW);
        relayOffAt = 0;
    }

    // Respect cooldown between consecutive fires
    if (now - lastFire < (unsigned long)cooldownMs) return;

    // Find nearest tree to nozzle position
    float bestDist = 1e9f;
    int   bestIdx  = -1;
    for (int i = 0; i < treeCount; i++) {
        float de = (float)(nozzleE - treePos[i].x);
        float dn = (float)(nozzleN - treePos[i].y);
        float d  = sqrtf(de*de + dn*dn);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
    }

    // ARM when nozzle enters 0.5 m radius of nearest tree
    if (!armed && bestDist <= 0.5f) {
        armed       = true;
        nearestIdx  = bestIdx;
        nearestDist = bestDist;
        return;
    }

    if (!armed) return;

    // Track if the closest tree has changed while we are armed
    if (bestIdx != nearestIdx) {
        // We drifted to a different nearest; reset
        if (bestDist > 0.8f) { armed = false; return; }
        nearestIdx  = bestIdx;
        nearestDist = bestDist;
        return;
    }

    // Distance is still decreasing: update minimum
    if (bestDist <= nearestDist) {
        nearestDist = bestDist;
        return;
    }

    // Distance has started increasing → nozzle just crossed the closest point
    if (bestDist > nearestDist + 0.02f) {
        // Fire relay
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(LED_PIN,   HIGH);
        relayOffAt = now + (unsigned long)relayMs;
        lastFire   = now;
        hitCount++;
        armed      = false;

        // Log calibration entry
        int slot = calLogHead % CAL_LOG_SIZE;
        calLog[slot] = {nozzleE, nozzleN, now};
        calLogHead++;
        if (calLogCount < CAL_LOG_SIZE) calLogCount++;

        Serial.printf("FIRE  tree=%d  dist=%.3f  hits=%d\n",
                      nearestIdx, nearestDist, hitCount);
    }
}

// ─── UDP packet processing ────────────────────────────────────────────────────
void processUDP() {
    int pktLen = udp.parsePacket();
    if (pktLen <= 0) return;

    static char udpBuf[8192];
    int len = udp.read(udpBuf, sizeof(udpBuf) - 1);
    if (len <= 0) return;
    udpBuf[len] = '\0';

    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, udpBuf) != DeserializationError::Ok) {
        Serial.println("UDP JSON parse error");
        return;
    }

    // UTM reference block (present in every first / single packet)
    if (doc.containsKey("utmZone")) {
        utmZone    = doc["utmZone"] | 0;
        utmEastOff = doc["eastOff"]  | 0.0;
        utmNorthOff= doc["northOff"] | 0.0;
        Serial.printf("UTM zone=%d  E-off=%.1f  N-off=%.1f\n",
                      utmZone, utmEastOff, utmNorthOff);
    }

    const char *mode = doc["mode"] | "";

    // ── AB lines mode ──────────────────────────────────────────────────────────
    if (strcmp(mode, "ablines") == 0) {
        sourceMode = "ablines";
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
            Serial.printf("Boundary: %d pts\n", boundaryCount);
        }

        buildGridFromABLines();

    // ── Points mode ────────────────────────────────────────────────────────────
    } else if (strcmp(mode, "points") == 0) {
        sourceMode = "points";

        int totalChunks = doc["totalChunks"] | 1;
        int totalPoints = doc["totalPoints"] | 0;
        int chunkIdx    = doc["chunkIdx"]    | 0;

        if (chunkIdx < 0) {
            // Header-only packet; initialise reassembly buffer
            if (!chunkBuf.active || chunkBuf.totalChunks != totalChunks) {
                chunkBuf.reset();
                chunkBuf.active      = true;
                chunkBuf.totalChunks = totalChunks;
                chunkBuf.totalPoints = totalPoints;
            }
            return;
        }

        if (totalChunks == 1) {
            // Single-packet delivery
            treeCount = 0;
            for (JsonObject pt : doc["points"].as<JsonArray>()) {
                if (treeCount >= MAX_TREES) break;
                treePos[treeCount++] = {pt["e"] | 0.0, pt["n"] | 0.0};
            }
            Serial.printf("Points (single): %d loaded\n", treeCount);
            return;
        }

        // Multi-chunk: initialise buffer on first real chunk if header missed
        if (!chunkBuf.active || chunkBuf.totalChunks != totalChunks) {
            chunkBuf.reset();
            chunkBuf.active      = true;
            chunkBuf.totalChunks = totalChunks;
            chunkBuf.totalPoints = totalPoints;
        }

        if (chunkIdx >= 0 && chunkIdx < MAX_CHUNKS && !chunkBuf.received[chunkIdx]) {
            chunkBuf.received[chunkIdx] = true;
            chunkBuf.chunksReceived++;

            int base = chunkIdx * CHUNK_SIZE;
            for (JsonObject pt : doc["points"].as<JsonArray>()) {
                if (base >= MAX_TREES) break;
                chunkBuf.pts[base++] = {pt["e"] | 0.0, pt["n"] | 0.0};
            }
        }

        if (chunkBuf.chunksReceived >= totalChunks) {
            int n = min(chunkBuf.totalPoints, MAX_TREES);
            memcpy(treePos, chunkBuf.pts, n * sizeof(Point2D));
            treeCount = n;
            chunkBuf.reset();
            Serial.printf("Points (chunked): %d loaded\n", treeCount);
        }

    // ── Standalone boundary packet ─────────────────────────────────────────────
    } else if (strcmp(mode, "boundary") == 0) {
        boundaryCount = 0;
        for (JsonObject pt : doc["points"].as<JsonArray>()) {
            if (boundaryCount >= MAX_BOUNDARY) break;
            boundary[boundaryCount++] = {pt["e"] | 0.0, pt["n"] | 0.0};
        }
        Serial.printf("Boundary updated: %d pts\n", boundaryCount);
        if (sourceMode == "ablines" && abLineCount > 0) buildGridFromABLines();
    }
}

// ─── Web interface helpers ────────────────────────────────────────────────────
static const char CSS[] PROGMEM =
    "<style>"
    "body{font-family:Arial,sans-serif;max-width:820px;margin:20px auto;padding:0 12px}"
    "h2{background:#2a7;color:#fff;padding:10px 14px;border-radius:5px;margin:0 0 12px}"
    "h3{color:#2a7;border-bottom:2px solid #2a7;padding-bottom:4px}"
    "table{border-collapse:collapse;width:100%;margin-bottom:16px}"
    "td,th{border:1px solid #ccc;padding:6px 10px;text-align:left}"
    "th{background:#f0f0f0;font-weight:bold}"
    "input[type=text],input[type=number],input[type=password]{width:98%;padding:5px;box-sizing:border-box}"
    ".btn{background:#2a7;color:#fff;padding:7px 20px;border:none;border-radius:4px;cursor:pointer;font-size:1em}"
    ".btn-red{background:#c33}"
    ".ok{color:#2a7;font-weight:bold}.warn{color:#c33;font-weight:bold}"
    "nav a{margin-right:14px;color:#2a7}"
    "</style>";

String pageHead(const char *title) {
    String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
                 "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                 "<title>TreeMarker</title>");
    h += FPSTR(CSS);
    h += F("</head><body><h2>TreeMarker &mdash; ");
    h += title;
    h += F("</h2><nav>"
           "<a href='/'>&#8962; Status</a>"
           "<a href='/spray'>Spray</a>"
           "<a href='/nozzle'>Nozzle</a>"
           "<a href='/grid'>Grid</a>"
           "<a href='/abnames'>AB Names</a>"
           "<a href='/wifi'>WiFi</a>"
           "</nav><hr>");
    return h;
}

// ─── /  Status page ───────────────────────────────────────────────────────────
void handleRoot() {
    String pg = pageHead("Status");

    pg += F("<h3>System Status</h3><table>");
    pg += "<tr><th>Hits fired</th><td><b>" + String(hitCount) + "</b></td></tr>";
    pg += "<tr><th>GPS valid</th><td class='" +
          String(gpsValid ? "ok'>&#10003; Yes" : "warn'>&#10007; No") + "</td></tr>";
    if (gpsValid) {
        pg += "<tr><th>Heading</th><td>" + String(gpsHeading, 1) + " &deg;T</td></tr>";
        pg += "<tr><th>Speed</th><td>"   + String(gpsSpeed,   2) + " m/s</td></tr>";
        pg += "<tr><th>Nozzle E</th><td>"+ String(nozzleE,    3) + " m</td></tr>";
        pg += "<tr><th>Nozzle N</th><td>"+ String(nozzleN,    3) + " m</td></tr>";
    }
    pg += "<tr><th>Source mode</th><td>" + sourceMode + "</td></tr>";
    pg += "<tr><th>Tree positions</th><td>" + String(treeCount)    + "</td></tr>";
    pg += "<tr><th>Boundary points</th><td>"+ String(boundaryCount)+ "</td></tr>";
    pg += "<tr><th>UTM zone</th><td>" + (utmZone ? String(utmZone) : String("(waiting)")) + "</td></tr>";
    pg += "<tr><th>Armed</th><td>" + String(armed ? "<span class='warn'>ARMED</span>" : "no") + "</td></tr>";
    pg += F("</table>");

    pg += F("<h3>Calibration Log (last 20 marks)</h3>"
            "<table><tr><th>#</th><th>Nozzle E (m)</th><th>Nozzle N (m)</th><th>Age (s)</th></tr>");
    unsigned long now = millis();
    for (int i = 0; i < calLogCount; i++) {
        int idx = (calLogHead - 1 - i + CAL_LOG_SIZE * 2) % CAL_LOG_SIZE;
        CalEntry &e = calLog[idx];
        pg += "<tr><td>" + String(i+1) + "</td><td>" +
              String(e.e, 3) + "</td><td>" + String(e.n, 3) + "</td><td>" +
              String((now - e.ts) / 1000UL) + "</td></tr>";
    }
    if (calLogCount == 0) pg += F("<tr><td colspan='4'>No marks yet</td></tr>");
    pg += F("</table>");

    pg += F("<br><form action='/reboot' method='post'>"
            "<input type='submit' class='btn btn-red' value='Reboot ESP32' "
            "onclick=\"return confirm('Reboot now?')\">"
            "</form></body></html>");

    server.send(200, "text/html", pg);
}

// ─── /spray ───────────────────────────────────────────────────────────────────
void handleSprayGet() {
    String pg = pageHead("Spray Settings");
    pg += F("<form action='/spray' method='post'><table>"
            "<tr><th>Relay ON duration (ms)</th>"
            "<td><input type='number' name='relayMs' min='50' max='5000' value='");
    pg += relayMs;
    pg += F("'></td></tr>"
            "<tr><th>Cooldown between triggers (ms)</th>"
            "<td><input type='number' name='cooldownMs' min='100' max='30000' value='");
    pg += cooldownMs;
    pg += F("'></td></tr>"
            "</table><br><input type='submit' class='btn' value='Save'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}

void handleSprayPost() {
    if (server.hasArg("relayMs"))    relayMs    = server.arg("relayMs").toInt();
    if (server.hasArg("cooldownMs")) cooldownMs = server.arg("cooldownMs").toInt();
    saveSettings();
    server.sendHeader("Location", "/");
    server.send(303);
}

// ─── /nozzle ──────────────────────────────────────────────────────────────────
void handleNozzleGet() {
    String pg = pageHead("Nozzle Offset");
    pg += F("<p><b>Calibration procedure:</b><br>"
            "1. Plant a known mark manually, then drive the line in <em>both</em> directions.<br>"
            "2. Measure the distance between the two spray marks.<br>"
            "3. The fore/aft error = half that distance.<br>"
            "4. If the mark is late (behind target) when heading north, <em>increase</em> fore/aft.<br>"
            "5. Positive fore/aft = nozzle is behind the GPS antenna.<br>"
            "6. Positive lateral = nozzle is to the right of the antenna centreline.</p>");
    pg += F("<form action='/nozzle' method='post'><table>"
            "<tr><th>Fore/Aft offset (m)</th>"
            "<td><input type='number' step='0.01' name='foreAft' value='");
    pg += String(foreAft, 2);
    pg += F("'></td></tr>"
            "<tr><th>Lateral offset (m)</th>"
            "<td><input type='number' step='0.01' name='lateral' value='");
    pg += String(lateral, 2);
    pg += F("'></td></tr>"
            "</table><br><input type='submit' class='btn' value='Save'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}

void handleNozzlePost() {
    if (server.hasArg("foreAft")) foreAft  = server.arg("foreAft").toFloat();
    if (server.hasArg("lateral")) lateral  = server.arg("lateral").toFloat();
    saveSettings();
    server.sendHeader("Location", "/");
    server.send(303);
}

// ─── /grid ────────────────────────────────────────────────────────────────────
void handleGridGet() {
    String pg = pageHead("Grid Settings");
    pg += F("<form action='/grid' method='post'><table>"
            "<tr><th>Row spacing (m)</th>"
            "<td><input type='number' step='0.01' name='rowSp' value='");
    pg += String(rowSpacing, 2);
    pg += F("'></td></tr>"
            "<tr><th>Tree spacing in-row (m)</th>"
            "<td><input type='number' step='0.01' name='treeSp' value='");
    pg += String(treeSpacing, 2);
    pg += F("'></td></tr>"
            "<tr><th>Lines each side of AB origin</th>"
            "<td><input type='number' name='linesEach' min='1' max='200' value='");
    pg += linesEach;
    pg += F("'></td></tr>"
            "</table><p><em>Saving will automatically rebuild the grid if AB line data is loaded.</em></p>"
            "<br><input type='submit' class='btn' value='Save &amp; Rebuild'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}

void handleGridPost() {
    if (server.hasArg("rowSp"))    rowSpacing  = server.arg("rowSp").toFloat();
    if (server.hasArg("treeSp"))   treeSpacing = server.arg("treeSp").toFloat();
    if (server.hasArg("linesEach"))linesEach   = server.arg("linesEach").toInt();
    saveSettings();
    if (sourceMode == "ablines" && abLineCount > 0) buildGridFromABLines();
    server.sendHeader("Location", "/");
    server.send(303);
}

// ─── /abnames ─────────────────────────────────────────────────────────────────
void handleABNamesGet() {
    String pg = pageHead("AB Line Names");
    pg += F("<p>These names must exactly match the AB line names in AgOpenGPS.</p>"
            "<form action='/abnames' method='post'><table>"
            "<tr><th>Row AB line name</th>"
            "<td><input type='text' name='rowLine' value='");
    pg += String(rowLineName);
    pg += F("'></td></tr>"
            "<tr><th>Tree AB line name</th>"
            "<td><input type='text' name='treeLine' value='");
    pg += String(treeLineName);
    pg += F("'></td></tr>"
            "</table><br><input type='submit' class='btn' value='Save'>"
            "</form></body></html>");
    server.send(200, "text/html", pg);
}

void handleABNamesPost() {
    if (server.hasArg("rowLine"))
        strlcpy(rowLineName,  server.arg("rowLine").c_str(),  sizeof(rowLineName));
    if (server.hasArg("treeLine"))
        strlcpy(treeLineName, server.arg("treeLine").c_str(), sizeof(treeLineName));
    saveSettings();
    server.sendHeader("Location", "/");
    server.send(303);
}

// ─── /wifi ────────────────────────────────────────────────────────────────────
void handleWifiGet() {
    String pg = pageHead("WiFi Credentials");
    pg += F("<form action='/wifi' method='post'><table>"
            "<tr><th>SSID</th>"
            "<td><input type='text' name='ssid' value='");
    pg += String(wifiSSID);
    pg += F("'></td></tr>"
            "<tr><th>Password</th>"
            "<td><input type='password' name='pass' placeholder='(leave blank to keep current)'></td></tr>"
            "</table><br><input type='submit' class='btn' value='Save &amp; Reboot'>"
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
        "<html><body><h2>TreeMarker</h2><p>Credentials saved. Rebooting&hellip;</p></body></html>");
    delay(1000);
    ESP.restart();
}

// ─── /reboot ──────────────────────────────────────────────────────────────────
void handleReboot() {
    server.send(200, "text/html",
        "<html><body><h2>TreeMarker</h2><p>Rebooting&hellip;</p></body></html>");
    delay(500);
    ESP.restart();
}

// ─── setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    pinMode(RELAY_PIN, OUTPUT);  digitalWrite(RELAY_PIN, LOW);
    pinMode(LED_PIN,   OUTPUT);  digitalWrite(LED_PIN,   LOW);

    loadSettings();
    Serial.printf("\nTreeMarker  SSID=%s\n", wifiSSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPass);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        delay(500);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        tries++;
        Serial.print('.');
    }
    digitalWrite(LED_PIN, LOW);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi failed – AP mode");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("TreeMarker-Setup", "setup1234");
        Serial.printf("AP IP=%s\n", WiFi.softAPIP().toString().c_str());
    }

    udp.begin(UDP_PORT);

    server.on("/",        HTTP_GET,  handleRoot);
    server.on("/spray",   HTTP_GET,  handleSprayGet);
    server.on("/spray",   HTTP_POST, handleSprayPost);
    server.on("/nozzle",  HTTP_GET,  handleNozzleGet);
    server.on("/nozzle",  HTTP_POST, handleNozzlePost);
    server.on("/grid",    HTTP_GET,  handleGridGet);
    server.on("/grid",    HTTP_POST, handleGridPost);
    server.on("/abnames", HTTP_GET,  handleABNamesGet);
    server.on("/abnames", HTTP_POST, handleABNamesPost);
    server.on("/wifi",    HTTP_GET,  handleWifiGet);
    server.on("/wifi",    HTTP_POST, handleWifiPost);
    server.on("/reboot",  HTTP_POST, handleReboot);
    server.begin();

    Serial.println("Web server started on port 80");
    Serial.printf("UDP listening on port %d\n", UDP_PORT);
}

// ─── loop ─────────────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();
    processUDP();
    readGPS();
    checkTrigger();
}
