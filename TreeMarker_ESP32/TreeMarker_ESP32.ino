// TreeMarker_ESP32.ino
// GPS-triggered relay mark-out system for tree planting.
// Receives field grid data via UDP, tracks RTK nozzle position,
// fires relay at GPIO26 when nozzle passes each tree position.
//
// Board  : ESP32 Dev Module
// Libs   : ArduinoJson v6, WebSockets by Markus Sattler
// Fixed pins : RELAY 26 | LED 2 | GPS_RX 16 | GPS_TX 17
// Fixed ports: UDP field 8888 | HTTP 80 | WS 81

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <math.h>

// ─── Fixed hardware ───────────────────────────────────────────────────────────
#define RELAY_PIN       26
#define LED_PIN          2
#define GPS_RX_PIN      16
#define GPS_TX_PIN      17
#define UDP_FIELD_PORT  8888
#define WS_PORT         81
#define ARM_DISTANCE_M  0.5f

// ─── First-boot defaults ──────────────────────────────────────────────────────
#define DEF_SSID         "AgOpenGPS"
#define DEF_PASS         "treeplant"
#define DEF_RELAY_ON_MS  800
#define DEF_COOLDOWN_MS  3000
#define DEF_FORE_AFT_M   2.0f
#define DEF_LATERAL_M    0.0f
#define DEF_ROW_SPACING  6.7f
#define DEF_TREE_SPACING 3.0f
#define DEF_LINES_EACH   40
#define DEF_ROW_LINE     "Row Direction"
#define DEF_TREE_LINE    "Tree Direction"

// ─── Capacities ───────────────────────────────────────────────────────────────
#define MAX_TREES     2000
#define MAX_ABLINES    200
#define MAX_BOUNDARY   500
#define MAX_CHUNKS      50
#define CHUNK_SIZE      80
#define CAL_LOG_SIZE    20

// ─── Types ────────────────────────────────────────────────────────────────────
struct Pt2 { float e, n; };   // local easting/northing (float = 1mm ok)

struct ABLine {
    char  name[64];
    float headingDeg;
    float easting;
    float northing;
};

struct CalMark { float e, n; };

// ─── Config (NVS-backed) ──────────────────────────────────────────────────────
struct Config {
    char  wifiSSID[64];
    char  wifiPass[64];
    int   relayOnMs;
    int   cooldownMs;
    float foreAftM;
    float lateralM;
    float rowSpacing;
    float treeSpacing;
    int   linesEachSide;
    char  rowLineName[64];
    char  treeLineName[64];
} cfg;

Preferences prefs;

void loadConfig() {
    prefs.begin("tm", true);
    prefs.getString("ssid",    cfg.wifiSSID,    sizeof(cfg.wifiSSID));
    prefs.getString("pass",    cfg.wifiPass,    sizeof(cfg.wifiPass));
    cfg.relayOnMs    = prefs.getInt  ("relayOnMs",  DEF_RELAY_ON_MS);
    cfg.cooldownMs   = prefs.getInt  ("cooldownMs", DEF_COOLDOWN_MS);
    cfg.foreAftM     = prefs.getFloat("foreAftM",   DEF_FORE_AFT_M);
    cfg.lateralM     = prefs.getFloat("lateralM",   DEF_LATERAL_M);
    cfg.rowSpacing   = prefs.getFloat("rowSpacing", DEF_ROW_SPACING);
    cfg.treeSpacing  = prefs.getFloat("treeSp",     DEF_TREE_SPACING);
    cfg.linesEachSide= prefs.getInt  ("linesEach",  DEF_LINES_EACH);
    prefs.getString("rowLine",  cfg.rowLineName,  sizeof(cfg.rowLineName));
    prefs.getString("treeLine", cfg.treeLineName, sizeof(cfg.treeLineName));
    prefs.end();
    if (!strlen(cfg.wifiSSID))    strcpy(cfg.wifiSSID,    DEF_SSID);
    if (!strlen(cfg.wifiPass))    strcpy(cfg.wifiPass,    DEF_PASS);
    if (!strlen(cfg.rowLineName)) strcpy(cfg.rowLineName, DEF_ROW_LINE);
    if (!strlen(cfg.treeLineName))strcpy(cfg.treeLineName,DEF_TREE_LINE);
}

void saveConfig() {
    prefs.begin("tm", false);
    prefs.putString("ssid",     cfg.wifiSSID);
    prefs.putString("pass",     cfg.wifiPass);
    prefs.putInt   ("relayOnMs",cfg.relayOnMs);
    prefs.putInt   ("cooldownMs",cfg.cooldownMs);
    prefs.putFloat ("foreAftM", cfg.foreAftM);
    prefs.putFloat ("lateralM", cfg.lateralM);
    prefs.putFloat ("rowSpacing",cfg.rowSpacing);
    prefs.putFloat ("treeSp",   cfg.treeSpacing);
    prefs.putInt   ("linesEach",cfg.linesEachSide);
    prefs.putString("rowLine",  cfg.rowLineName);
    prefs.putString("treeLine", cfg.treeLineName);
    prefs.end();
}

// ─── Grid state ───────────────────────────────────────────────────────────────
Pt2     trees[MAX_TREES];
bool    treeFired[MAX_TREES];
int     treeCount    = 0;
ABLine  abLines[MAX_ABLINES];
int     abLineCount  = 0;
Pt2     boundary[MAX_BOUNDARY];
int     boundaryCount= 0;
int     rowLineIdx   = -1;    // index into abLines[]
bool    gridReady    = false;
String  sourceMode   = "none";

// Active row heading (set regardless of source mode)
float   activeRowHeadingDeg = 0;
float   activeRowOriginE    = 0;
float   activeRowOriginN    = 0;

// Field UTM reference
double  fieldEasting  = 0;
double  fieldNorthing = 0;
int     fieldZone     = 0;

// Chunked assembly (writes directly into trees[])
bool    chunkReceived[MAX_CHUNKS];
int     chunkTotalChunks = 0;
int     chunkTotalPoints = 0;
int     chunkGotCount    = 0;
bool    chunkActive      = false;

// ─── GPS state ────────────────────────────────────────────────────────────────
double  gpsLat = 0, gpsLon = 0;
float   currentHeadingDeg = 0;
float   currentSpeedMS    = 0;
bool    gpsValid          = false;
int     gpsQuality        = 0;
float   antennaE = 0, antennaN = 0;
float   nozzleE  = 0, nozzleN  = 0;

// ─── Guidance ─────────────────────────────────────────────────────────────────
float   crossTrackM     = 0;
float   distNextMarkM   = -1;
float   headingErrorDeg = 0;
int     currentRowIndex = 0;

// ─── Trigger state ────────────────────────────────────────────────────────────
int           totalHits    = 0;
bool          triggerArmed = false;
int           armedTreeIdx = -1;
float         armedMinDist = 1e9f;
unsigned long lastFireMs   = 0;
unsigned long relayOffMs   = 0;

// ─── Calibration log ──────────────────────────────────────────────────────────
CalMark calLog[CAL_LOG_SIZE];
int     calLogHead  = 0;
int     calLogCount = 0;
char    lastMarkStr[48] = "--";

// ─── Objects ──────────────────────────────────────────────────────────────────
WebServer        httpServer(80);
WiFiUDP          fieldUdp;
WebSocketsServer wsServer(WS_PORT);

// ═══════════════════════════════════════════════════════════════════════════════
//  UTM CONVERSION (WGS84)
// ═══════════════════════════════════════════════════════════════════════════════
void latLonToUTM(double lat, double lon, int zone, double &E, double &N) {
    const double a  = 6378137.0, f = 1.0/298.257223563;
    const double e2 = 2*f - f*f, ep2 = e2/(1-e2), k0 = 0.9996;
    double lon0 = ((zone-1)*6.0 - 180.0 + 3.0) * M_PI / 180.0;
    double latR = lat*M_PI/180.0, lonR = lon*M_PI/180.0;
    double sl = sin(latR), cl = cos(latR);
    double Nv = a/sqrt(1 - e2*sl*sl);
    double T  = tan(latR)*tan(latR), C = ep2*cl*cl, A = cl*(lonR-lon0);
    double M  = a*((1-e2/4-3*e2*e2/64-5*e2*e2*e2/256)*latR
                  -(3*e2/8+3*e2*e2/32+45*e2*e2*e2/1024)*sin(2*latR)
                  +(15*e2*e2/256+45*e2*e2*e2/1024)*sin(4*latR)
                  -(35*e2*e2*e2/3072)*sin(6*latR));
    E = k0*Nv*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*ep2)*A*A*A*A*A/120)+500000.0;
    N = k0*(M+Nv*tan(latR)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                            +(61-58*T+T*T+600*C-330*ep2)*A*A*A*A*A*A/720));
    if (lat < 0) N += 10000000.0;
}

void latLonToLocal(double lat, double lon, float &le, float &ln) {
    if (fieldZone == 0) { le = 0; ln = 0; return; }
    double E, N;
    latLonToUTM(lat, lon, fieldZone, E, N);
    le = (float)(E - fieldEasting);
    ln = (float)(N - fieldNorthing);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GEOMETRY HELPERS
// ═══════════════════════════════════════════════════════════════════════════════
bool pointInPolygon(float x, float y) {
    if (boundaryCount < 3) return true;
    int c = 0;
    for (int i = 0, j = boundaryCount-1; i < boundaryCount; j = i++) {
        float xi = boundary[i].e, yi = boundary[i].n;
        float xj = boundary[j].e, yj = boundary[j].n;
        if (((yi > y) != (yj > y)) &&
            (x < (xj-xi)*(y-yi)/(yj-yi)+xi)) c++;
    }
    return c & 1;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GRID BUILDERS
// ═══════════════════════════════════════════════════════════════════════════════
void buildGridFromABLines() {
    rowLineIdx = -1;
    int treeLineIdx = -1;
    for (int i = 0; i < abLineCount; i++) {
        if (strcmp(abLines[i].name, cfg.rowLineName)  == 0) rowLineIdx  = i;
        if (strcmp(abLines[i].name, cfg.treeLineName) == 0) treeLineIdx = i;
    }
    if (rowLineIdx < 0 || treeLineIdx < 0) {
        Serial.printf("WARN: AB lines '%s'/'%s' not found among %d loaded\n",
                      cfg.rowLineName, cfg.treeLineName, abLineCount);
        return;
    }

    activeRowHeadingDeg = abLines[rowLineIdx].headingDeg;
    activeRowOriginE    = abLines[rowLineIdx].easting;
    activeRowOriginN    = abLines[rowLineIdx].northing;

    double rh  = abLines[rowLineIdx].headingDeg  * M_PI / 180.0;
    double th  = abLines[treeLineIdx].headingDeg * M_PI / 180.0;
    double rd_e = sin(rh), rd_n = cos(rh);
    double td_e = sin(th), td_n = cos(th);
    double rn_e = -rd_n, rn_n =  rd_e;
    double tn_e = -td_n, tn_n =  td_e;

    treeCount = 0;
    memset(treeFired, 0, sizeof(treeFired));

    for (int ri = -cfg.linesEachSide; ri <= cfg.linesEachSide; ri++) {
        double rp_e = abLines[rowLineIdx].easting  + ri*cfg.rowSpacing*rn_e;
        double rp_n = abLines[rowLineIdx].northing + ri*cfg.rowSpacing*rn_n;
        for (int ti = -cfg.linesEachSide; ti <= cfg.linesEachSide; ti++) {
            double tp_e = abLines[treeLineIdx].easting  + ti*cfg.treeSpacing*tn_e;
            double tp_n = abLines[treeLineIdx].northing + ti*cfg.treeSpacing*tn_n;
            double det = -rd_e*td_n + rd_n*td_e;
            if (fabs(det) < 1e-10) continue;
            double dx = tp_e-rp_e, dy = tp_n-rp_n;
            double s  = (-dx*td_n + dy*td_e)/det;
            float  ie = (float)(rp_e + s*rd_e);
            float  in_= (float)(rp_n + s*rd_n);
            if (boundaryCount >= 3 && !pointInPolygon(ie, in_)) continue;
            if (treeCount < MAX_TREES) trees[treeCount++] = {ie, in_};
        }
    }
    gridReady  = true;
    sourceMode = "AgOpenGPS AB lines";
    Serial.printf("AB grid built: %d intersections\n", treeCount);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  NMEA PARSING
// ═══════════════════════════════════════════════════════════════════════════════
static char nmeaBuf[128];
static int  nmeaIdx = 0;

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
    strncpy(buf, raw, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    char *star = strrchr(buf, '*');
    if (star) *star = '\0';

    char *f[22]; int fc = 0;
    char *tok = strtok(buf, ",");
    while (tok && fc < 22) { f[fc++] = tok; tok = strtok(nullptr, ","); }
    if (fc < 2) return;

    if (strstr(f[0], "GGA") && fc >= 7) {
        int q = atoi(f[6]);
        if (q > 0) {
            gpsQuality = q;
            gpsLat     = nmeaDeg(f[2], f[3]);
            gpsLon     = nmeaDeg(f[4], f[5]);
            gpsValid   = true;

            // Convert to local coords
            latLonToLocal(gpsLat, gpsLon, antennaE, antennaN);

            // Shift to nozzle position using current heading
            float hRad = currentHeadingDeg * (float)M_PI / 180.0f;
            float fwdE = sinf(hRad), fwdN = cosf(hRad);   // forward unit vector
            float rgtE = cosf(hRad), rgtN = -sinf(hRad);  // right unit vector
            nozzleE = antennaE - cfg.foreAftM*fwdE + cfg.lateralM*rgtE;
            nozzleN = antennaN - cfg.foreAftM*fwdN + cfg.lateralM*rgtN;
        } else {
            gpsValid = false;
        }

    } else if (strstr(f[0], "RMC") && fc >= 9) {
        if (f[2][0] == 'A') {
            currentSpeedMS    = atof(f[7]) * 0.51444f;
            currentHeadingDeg = atof(f[8]);
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
        } else if (nmeaIdx < (int)sizeof(nmeaBuf)-1) {
            nmeaBuf[nmeaIdx++] = c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GUIDANCE CALCULATIONS
// ═══════════════════════════════════════════════════════════════════════════════
void calcGuidance() {
    if (!gridReady || rowLineIdx < 0) return;

    float h  = activeRowHeadingDeg * (float)M_PI / 180.0f;
    float rgtE = cosf(h), rgtN = -sinf(h);  // perpendicular-right unit vector

    // Cross-track: signed distance from nozzle to nearest row line
    float base_ct = (nozzleE - activeRowOriginE)*rgtE
                  + (nozzleN - activeRowOriginN)*rgtN;
    currentRowIndex = (int)roundf(base_ct / cfg.rowSpacing);
    crossTrackM     = base_ct - currentRowIndex * cfg.rowSpacing;

    // Heading error: deviation from row heading, normalised to ±180
    headingErrorDeg = currentHeadingDeg - activeRowHeadingDeg;
    while (headingErrorDeg >  180.0f) headingErrorDeg -= 360.0f;
    while (headingErrorDeg < -180.0f) headingErrorDeg += 360.0f;

    // Distance to next mark: nearest un-fired tree ahead on current row
    float fwdE = sinf(h + headingErrorDeg*(float)M_PI/180.0f);   // actual travel direction
    float fwdN = cosf(h + headingErrorDeg*(float)M_PI/180.0f);
    float minAhead = 1e9f;
    for (int i = 0; i < treeCount; i++) {
        if (treeFired[i]) continue;
        float dE  = trees[i].e - nozzleE;
        float dN  = trees[i].n - nozzleN;
        // Cross-track of this tree relative to current row
        float tct = dE*rgtE + dN*rgtN - currentRowIndex*cfg.rowSpacing;
        if (fabsf(tct) > cfg.rowSpacing * 0.6f) continue;  // different row
        float proj = dE*fwdE + dN*fwdN;
        if (proj > 0 && proj < minAhead) minAhead = proj;
    }
    distNextMarkM = (minAhead < 1e8f) ? minAhead : -1.0f;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TRIGGER LOGIC  (minimum-distance / closest-point detection)
// ═══════════════════════════════════════════════════════════════════════════════
void checkTrigger() {
    if (!gpsValid || treeCount == 0) return;

    unsigned long now = millis();

    // Release relay after duration
    if (relayOffMs && now >= relayOffMs) {
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN,   LOW);
        relayOffMs = 0;
    }

    if (now - lastFireMs < (unsigned long)cfg.cooldownMs) return;

    // Find nearest tree
    float bestDist = 1e9f; int bestIdx = -1;
    for (int i = 0; i < treeCount; i++) {
        if (treeFired[i]) continue;
        float de = nozzleE - trees[i].e;
        float dn = nozzleN - trees[i].n;
        float d  = sqrtf(de*de + dn*dn);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
    }
    if (bestIdx < 0) return;

    // ARM when within ARM_DISTANCE_M
    if (!triggerArmed && bestDist <= ARM_DISTANCE_M) {
        triggerArmed = true;
        armedTreeIdx = bestIdx;
        armedMinDist = bestDist;
        return;
    }
    if (!triggerArmed) return;

    // Different nearest tree while armed
    if (bestIdx != armedTreeIdx) {
        if (bestDist > ARM_DISTANCE_M * 1.6f) triggerArmed = false;
        else { armedTreeIdx = bestIdx; armedMinDist = bestDist; }
        return;
    }

    // Track minimum distance
    if (bestDist <= armedMinDist) { armedMinDist = bestDist; return; }

    // Distance increasing — fire
    if (bestDist > armedMinDist + 0.02f) {
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(LED_PIN,   HIGH);
        relayOffMs   = now + (unsigned long)cfg.relayOnMs;
        lastFireMs   = now;
        triggerArmed = false;
        treeFired[armedTreeIdx] = true;
        totalHits++;

        // Calibration log
        calLog[calLogHead % CAL_LOG_SIZE] = {nozzleE, nozzleN};
        calLogHead++;
        if (calLogCount < CAL_LOG_SIZE) calLogCount++;
        snprintf(lastMarkStr, sizeof(lastMarkStr), "E%.2f N%.2f", nozzleE, nozzleN);

        Serial.printf("FIRE idx=%d dist=%.3f hits=%d\n",
                      armedTreeIdx, armedMinDist, totalHits);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WEBSOCKET
// ═══════════════════════════════════════════════════════════════════════════════
static int lastFiredIdx = -1;  // set in checkTrigger region, broadcast once

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    // We only broadcast; nothing to handle from clients
    (void)num; (void)type; (void)payload; (void)length;
}

void broadcastGPS() {
    if (!gpsValid) return;
    char buf[384];
    snprintf(buf, sizeof(buf),
        "{\"nozzleE\":%.3f,\"nozzleN\":%.3f,"
        "\"antennaE\":%.3f,\"antennaN\":%.3f,"
        "\"headingDeg\":%.1f,\"speedKmh\":%.2f,"
        "\"crossTrackM\":%.4f,\"distNextMarkM\":%.2f,"
        "\"headingErrorDeg\":%.2f,\"currentRowIndex\":%d,"
        "\"totalHits\":%d,\"gridReady\":%s,"
        "\"rowHeadingDeg\":%.2f,\"timestamp\":%lu}",
        nozzleE, nozzleN, antennaE, antennaN,
        currentHeadingDeg, currentSpeedMS * 3.6f,
        crossTrackM, distNextMarkM,
        headingErrorDeg, currentRowIndex,
        totalHits, gridReady ? "true" : "false",
        activeRowHeadingDeg, millis());
    wsServer.broadcastTXT(buf);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UDP FIELD DATA
// ═══════════════════════════════════════════════════════════════════════════════
void processFieldUDP() {
    int len = fieldUdp.parsePacket();
    if (len <= 0) return;

    static char buf[8192];
    int n = fieldUdp.read(buf, sizeof(buf)-1);
    if (n <= 0) return;
    buf[n] = '\0';

    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, buf) != DeserializationError::Ok) {
        Serial.println("UDP JSON parse error");
        return;
    }

    // Field UTM reference
    if (doc.containsKey("fieldZone")) {
        fieldZone     = doc["fieldZone"]     | 0;
        fieldEasting  = doc["fieldEasting"]  | 0.0;
        fieldNorthing = doc["fieldNorthing"] | 0.0;
        Serial.printf("Field UTM zone=%d  E=%.1f  N=%.1f\n",
                      fieldZone, fieldEasting, fieldNorthing);
    }

    const char *mode = doc["mode"] | "";

    // ── AB lines ──────────────────────────────────────────────────────────────
    if (strcmp(mode, "ablines") == 0) {
        abLineCount = 0;
        for (JsonObject ln : doc["allLines"].as<JsonArray>()) {
            if (abLineCount >= MAX_ABLINES) break;
            strlcpy(abLines[abLineCount].name, ln["name"] | "", 64);
            abLines[abLineCount].headingDeg = ln["heading"]  | 0.0f;
            abLines[abLineCount].easting    = ln["easting"]  | 0.0f;
            abLines[abLineCount].northing   = ln["northing"] | 0.0f;
            abLineCount++;
        }
        Serial.printf("AB lines received: %d\n", abLineCount);

        boundaryCount = 0;
        if (doc.containsKey("boundary")) {
            for (JsonObject pt : doc["boundary"].as<JsonArray>()) {
                if (boundaryCount >= MAX_BOUNDARY) break;
                boundary[boundaryCount++] = {pt["e"]|0.0f, pt["n"]|0.0f};
            }
        }
        buildGridFromABLines();

    // ── Points (chunked or single) ────────────────────────────────────────────
    } else if (strcmp(mode, "points") == 0) {
        // Header packet: has totalChunks/totalPoints but no chunkIdx key
        if (!doc.containsKey("chunkIdx")) {
            chunkTotalChunks = doc["totalChunks"] | 1;
            chunkTotalPoints = doc["totalPoints"] | 0;
            chunkGotCount    = 0;
            chunkActive      = true;
            memset(chunkReceived, 0, sizeof(chunkReceived));
            treeCount = 0;
            memset(treeFired, 0, sizeof(treeFired));
            Serial.printf("Points header: %d chunks, %d pts\n",
                          chunkTotalChunks, chunkTotalPoints);
            return;
        }

        int ci = doc["chunkIdx"] | 0;
        int tc = doc["totalChunks"] | 1;

        if (!chunkActive || chunkTotalChunks != tc) {
            // Single-packet or no header received
            chunkTotalChunks = tc;
            chunkTotalPoints = doc["totalPoints"] | 0;
            chunkGotCount    = 0;
            chunkActive      = true;
            memset(chunkReceived, 0, sizeof(chunkReceived));
            treeCount = 0;
            memset(treeFired, 0, sizeof(treeFired));
        }

        if (ci >= 0 && ci < MAX_CHUNKS && !chunkReceived[ci]) {
            chunkReceived[ci] = true;
            chunkGotCount++;
            int base = ci * CHUNK_SIZE;
            for (JsonObject pt : doc["points"].as<JsonArray>()) {
                if (base >= MAX_TREES) break;
                trees[base++] = {pt["e"]|0.0f, pt["n"]|0.0f};
            }
            if (base > treeCount) treeCount = base;
        }

        if (chunkGotCount >= chunkTotalChunks) {
            treeCount  = min(chunkTotalPoints, MAX_TREES);
            chunkActive = false;
            gridReady   = true;
            sourceMode  = "AutoCAD DXF";
            // No AB line data in points mode; guidance needs at least a heading
            // Use activeRowHeadingDeg as-is (0 until user sets via settings)
            Serial.printf("Points complete: %d trees\n", treeCount);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HTTP — API
// ═══════════════════════════════════════════════════════════════════════════════
void handleApiGrid() {
    // Return nearby trees + grid metadata as JSON
    DynamicJsonDocument doc(16384);
    doc["rowHeadingDeg"]  = activeRowHeadingDeg;
    doc["rowSpacing"]     = cfg.rowSpacing;
    doc["treeSpacing"]    = cfg.treeSpacing;
    doc["rowOriginE"]     = activeRowOriginE;
    doc["rowOriginN"]     = activeRowOriginN;
    doc["gridReady"]      = gridReady;
    doc["sourceMode"]     = sourceMode;
    doc["treeCount"]      = treeCount;

    JsonArray arr = doc.createNestedArray("trees");
    // Send all trees (client filters visible ones)
    for (int i = 0; i < treeCount && i < 1500; i++) {
        JsonArray t = arr.createNestedArray();
        t.add(trees[i].e);
        t.add(trees[i].n);
        t.add(treeFired[i] ? 1 : 0);
    }

    JsonArray cal = doc.createNestedArray("calLog");
    for (int i = 0; i < calLogCount; i++) {
        int idx = (calLogHead - 1 - i + CAL_LOG_SIZE*2) % CAL_LOG_SIZE;
        JsonArray c = cal.createNestedArray();
        c.add(calLog[idx].e);
        c.add(calLog[idx].n);
    }

    String out;
    serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HTTP — SETTINGS PAGE
// ═══════════════════════════════════════════════════════════════════════════════
void sendSettingsPage() {
    httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    httpServer.send(200, "text/html", "");

    httpServer.sendContent(F(
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>TreeMarker</title><style>"
        "body{margin:0;font-family:Arial,sans-serif;background:#1b2b1b;color:#e0e0e0}"
        "h1{background:#1a4a1a;color:#7fff7f;margin:0;padding:12px 16px;font-size:1.3em}"
        "h2{color:#7fff7f;font-size:1em;margin:0 0 8px}"
        ".card{background:#243324;border-radius:8px;padding:14px 16px;margin:12px;"
               "box-shadow:0 2px 8px #0008}"
        "label{display:block;margin:6px 0 2px;font-size:.85em;color:#aaa}"
        "input[type=text],input[type=number],input[type=password]"
               "{width:100%;padding:6px;box-sizing:border-box;background:#1b2b1b;"
               "border:1px solid #3a5a3a;border-radius:4px;color:#e0e0e0}"
        ".row2{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
        ".btn{background:#2a6a2a;color:#fff;padding:8px 18px;border:none;"
              "border-radius:4px;cursor:pointer;font-size:.95em;margin-top:8px}"
        ".btn:hover{background:#3a8a3a}"
        ".btn-red{background:#6a2a2a}.btn-red:hover{background:#8a3a3a}"
        ".btn-blue{background:#2a4a7a}.btn-blue:hover{background:#3a6a9a}"
        ".stat{display:inline-block;background:#1a3a1a;border-radius:4px;"
               "padding:4px 10px;margin:3px;font-size:.9em}"
        ".tip{font-size:.78em;color:#8a8;margin-top:6px;font-style:italic}"
        "table{width:100%;border-collapse:collapse;font-size:.85em}"
        "th,td{border:1px solid #3a5a3a;padding:5px 8px;text-align:left}"
        "th{background:#1a3a1a;color:#7fff7f}"
        "a{color:#7fff7f}nav{padding:10px 12px;background:#1a3a1a}"
        "</style></head><body>"
        "<h1>&#127807; TreeMarker</h1>"
        "<nav><a href='/map'>&#128506; Live Map</a></nav>"));

    // Status card
    char statBuf[400];
    snprintf(statBuf, sizeof(statBuf),
        "<div class='card'><h2>Status</h2>"
        "<span class='stat'>Hits: <b>%d</b></span>"
        "<span class='stat'>Last: %s</span>"
        "<span class='stat'>Hdg: %.1f&deg;</span>"
        "<span class='stat'>Speed: %.1f km/h</span>"
        "<span class='stat'>Source: %s</span>"
        "<span class='stat'>Trees: %d</span>"
        "<span class='stat'>Boundary: %s</span>"
        "<span class='stat'>GPS: q=%d</span>"
        "</div>",
        totalHits, lastMarkStr,
        currentHeadingDeg, currentSpeedMS*3.6f,
        sourceMode.c_str(), treeCount,
        boundaryCount >= 3 ? "Active" : "None",
        gpsQuality);
    httpServer.sendContent(statBuf);

    // Spray settings
    char formBuf[512];
    snprintf(formBuf, sizeof(formBuf),
        "<div class='card'><h2>Spray Settings</h2>"
        "<form action='/save-spray' method='post'>"
        "<div class='row2'>"
        "<div><label>Relay ON (ms)</label>"
        "<input type='number' name='relayOnMs' min='100' max='5000' value='%d'></div>"
        "<div><label>Cooldown (ms)</label>"
        "<input type='number' name='cooldownMs' min='500' max='10000' value='%d'></div>"
        "</div><button class='btn' type='submit'>Save</button></form></div>",
        cfg.relayOnMs, cfg.cooldownMs);
    httpServer.sendContent(formBuf);

    // Nozzle offset
    snprintf(formBuf, sizeof(formBuf),
        "<div class='card'><h2>Nozzle Offset</h2>"
        "<form action='/save-offset' method='post'>"
        "<div class='row2'>"
        "<div><label>Fore/Aft (m)</label>"
        "<input type='number' step='0.001' name='foreAftM' value='%.3f'></div>"
        "<div><label>Lateral (m)</label>"
        "<input type='number' step='0.001' name='lateralM' value='%.3f'></div>"
        "</div>"
        "<p class='tip'>Calibration: drive one row forward then return on the same row. "
        "Measure displacement between forward and return marks at the same tree. "
        "Divide by 2 = offset error. Check the calibration log below for precise readings.</p>"
        "<button class='btn' type='submit'>Save</button></form></div>",
        cfg.foreAftM, cfg.lateralM);
    httpServer.sendContent(formBuf);

    // Grid settings
    snprintf(formBuf, sizeof(formBuf),
        "<div class='card'><h2>Grid Settings</h2>"
        "<form action='/save-grid' method='post'>"
        "<div class='row2'>"
        "<div><label>Row spacing (m)</label>"
        "<input type='number' step='0.01' name='rowSpacing' value='%.2f'></div>"
        "<div><label>Tree spacing (m)</label>"
        "<input type='number' step='0.01' name='treeSpacing' value='%.2f'></div>"
        "</div>"
        "<label>Lines each side of AB origin</label>"
        "<input type='number' name='linesEachSide' min='5' max='100' value='%d'>"
        "<button class='btn' type='submit'>Save &amp; Rebuild</button></form></div>",
        cfg.rowSpacing, cfg.treeSpacing, cfg.linesEachSide);
    httpServer.sendContent(formBuf);

    // AB line names
    snprintf(formBuf, sizeof(formBuf),
        "<div class='card'><h2>AB Line Names</h2>"
        "<form action='/save-lines' method='post'>"
        "<label>Row AB line name</label>"
        "<input type='text' name='rowLineName' value='%s'>"
        "<label>Tree AB line name</label>"
        "<input type='text' name='treeLineName' value='%s'>"
        "<button class='btn' type='submit'>Save</button></form></div>",
        cfg.rowLineName, cfg.treeLineName);
    httpServer.sendContent(formBuf);

    // WiFi
    snprintf(formBuf, sizeof(formBuf),
        "<div class='card'><h2>WiFi</h2>"
        "<form action='/save-wifi' method='post'>"
        "<label>SSID</label><input type='text' name='wifiSSID' value='%s'>"
        "<label>Password</label><input type='password' name='wifiPass' "
        "placeholder='(leave blank to keep current)'>"
        "<p class='tip'>Reboot required after saving.</p>"
        "<button class='btn' type='submit'>Save</button></form></div>",
        cfg.wifiSSID);
    httpServer.sendContent(formBuf);

    // Calibration log
    httpServer.sendContent(F(
        "<div class='card'><h2>Calibration Log</h2>"
        "<table><tr><th>#</th><th>Easting (m)</th><th>Northing (m)</th></tr>"));
    if (calLogCount == 0) {
        httpServer.sendContent(F("<tr><td colspan='3'>No marks yet</td></tr>"));
    } else {
        for (int i = 0; i < calLogCount; i++) {
            int idx = (calLogHead - 1 - i + CAL_LOG_SIZE*2) % CAL_LOG_SIZE;
            char row[128];
            snprintf(row, sizeof(row),
                "<tr><td>%d</td><td>%.3f</td><td>%.3f</td></tr>",
                i+1, calLog[idx].e, calLog[idx].n);
            httpServer.sendContent(row);
        }
    }
    httpServer.sendContent(F("</table>"
        "<form action='/clear-log' method='post' style='margin-top:8px'>"
        "<button class='btn btn-red' type='submit'>Clear Log</button></form>"
        "</div>"));

    // Reboot
    httpServer.sendContent(F(
        "<div class='card'>"
        "<form action='/reboot' method='post'>"
        "<button class='btn btn-red' type='submit' "
        "onclick=\"return confirm('Reboot now?')\">Reboot ESP32</button>"
        "</form></div></body></html>"));

    httpServer.sendContent("");   // terminate chunked
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HTTP — MAP PAGE
// ═══════════════════════════════════════════════════════════════════════════════
void sendMapPage() {
    httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    httpServer.send(200, "text/html", "");

    httpServer.sendContent(F(
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>TreeMarker Map</title><style>"
        "body{margin:0;background:#111;color:#e0e0e0;font-family:Arial,sans-serif;"
             "display:flex;flex-direction:column;height:100vh;overflow:hidden}"
        "#topbar{display:flex;justify-content:space-between;align-items:center;"
                "background:#1a2a1a;padding:6px 12px;font-size:.85em}"
        "#conn{padding:3px 10px;border-radius:10px;font-weight:bold}"
        ".conn-ok{background:#1a4a1a;color:#7fff7f}"
        ".conn-no{background:#4a1a1a;color:#ff7f7f}"
        "#map{flex:1;display:block;width:100%}"
        "#panels{display:grid;grid-template-columns:repeat(5,1fr);"
                "background:#141a14;padding:8px;gap:6px}"
        ".panel{background:#1d2d1d;border-radius:6px;padding:8px;text-align:center}"
        ".panel .lbl{font-size:.65em;color:#888;text-transform:uppercase;letter-spacing:.05em}"
        ".panel .val{font-size:1.5em;font-weight:bold;line-height:1.2;margin-top:3px}"
        ".panel .sub{font-size:.7em;color:#888}"
        ".ct-ok{color:#00ff66}.ct-warn{color:#ffff00}.ct-bad{color:#ff4444}"
        "#backlink{background:#1a2a1a;text-align:center;padding:6px;"
                  "font-size:.8em}<a style='color:#7fff7f'>a</a>}"
        "a{color:#7fff7f}"
        "</style></head><body>"));

    httpServer.sendContent(F(
        "<div id='topbar'>"
        "<span>&#127807; TreeMarker — Live Map</span>"
        "<span id='conn' class='conn-no'>Connecting...</span>"
        "</div>"
        "<canvas id='map'></canvas>"
        "<div id='panels'>"
        "<div class='panel'><div class='lbl'>Cross-Track</div>"
        "<div class='val ct-ok' id='ct'>--</div><div class='sub' id='ctdir'>&nbsp;</div></div>"
        "<div class='panel'><div class='lbl'>Next Mark</div>"
        "<div class='val' id='nm'>--</div><div class='sub'>m</div></div>"
        "<div class='panel'><div class='lbl'>Heading Error</div>"
        "<div class='val' id='he'>--</div><div class='sub' id='hedir'>&nbsp;</div></div>"
        "<div class='panel'><div class='lbl'>Speed</div>"
        "<div class='val' id='spd'>--</div><div class='sub'>km/h</div></div>"
        "<div class='panel'><div class='lbl'>Total Marks</div>"
        "<div class='val' id='hits'>0</div><div class='sub'>&nbsp;</div></div>"
        "</div>"
        "<div id='backlink'><a href='/'>&#9664; Settings</a></div>"));

    httpServer.sendContent(F("<script>\n"
        "const canvas = document.getElementById('map');\n"
        "const ctx = canvas.getContext('2d');\n"
        "let grid = null, pos = null, firedSet = new Set();\n"
        "let prevHits = 0;\n"

        "function resize() {\n"
        "  canvas.width = canvas.offsetWidth;\n"
        "  canvas.height = canvas.offsetHeight;\n"
        "  draw();\n"
        "}\n"
        "window.addEventListener('resize', resize);\n"

        "// Fetch static grid data\n"
        "function fetchGrid() {\n"
        "  fetch('/api/grid').then(r=>r.json()).then(d=>{\n"
        "    grid = d;\n"
        "    d.trees.forEach((t,i)=>{ if(t[2]) firedSet.add(i); });\n"
        "    draw();\n"
        "  }).catch(()=>setTimeout(fetchGrid,3000));\n"
        "}\n"

        "// WebSocket\n"
        "function connect() {\n"
        "  const ws = new WebSocket('ws://'+location.hostname+':81');\n"
        "  ws.onopen = ()=>{ document.getElementById('conn').textContent='Live';\n"
        "    document.getElementById('conn').className='conn-ok'; };\n"
        "  ws.onclose = ()=>{ document.getElementById('conn').textContent='Reconnecting...';\n"
        "    document.getElementById('conn').className='conn-no';\n"
        "    setTimeout(connect,2000); };\n"
        "  ws.onmessage = e=>{ pos=JSON.parse(e.data); updatePanels(); draw();\n"
        "    if(pos.totalHits > prevHits){ prevHits=pos.totalHits; fetchGrid(); } };\n"
        "}\n"));

    httpServer.sendContent(F(
        "function updatePanels() {\n"
        "  if(!pos) return;\n"
        "  const ct = pos.crossTrackM;\n"
        "  const ctEl = document.getElementById('ct');\n"
        "  ctEl.textContent = Math.abs(ct).toFixed(3);\n"
        "  const ad = Math.abs(ct);\n"
        "  ctEl.className = 'val ' + (ad<0.05?'ct-ok':ad<0.15?'ct-warn':'ct-bad');\n"
        "  document.getElementById('ctdir').textContent = ct>=0?'RIGHT':'LEFT';\n"
        "  const nm = pos.distNextMarkM;\n"
        "  document.getElementById('nm').textContent = nm>=0?nm.toFixed(2):'--';\n"
        "  const he = pos.headingErrorDeg;\n"
        "  document.getElementById('he').textContent = Math.abs(he).toFixed(1);\n"
        "  document.getElementById('hedir').textContent = he>=0?'RIGHT':'LEFT';\n"
        "  document.getElementById('spd').textContent = pos.speedKmh.toFixed(1);\n"
        "  document.getElementById('hits').textContent = pos.totalHits;\n"
        "}\n"

        "function draw() {\n"
        "  const W=canvas.width, H=canvas.height;\n"
        "  ctx.fillStyle='#111'; ctx.fillRect(0,0,W,H);\n"
        "  if(!pos) return;\n"
        "  const rowH = (grid?grid.rowHeadingDeg:pos.rowHeadingDeg||0)*Math.PI/180;\n"
        "  const rs = grid?grid.rowSpacing:6.7;\n"
        "  const scale = Math.min(W,H)/Math.max(rs*7*1.2,1);\n"
        "  const cx=W/2, cy=H/2;\n"
        "  const nE=pos.nozzleE, nN=pos.nozzleN;\n"
        "  // Local→canvas: row direction = +x, cross-row = +y\n"
        "  function toC(e,n){const dE=e-nE,dN=n-nN;\n"
        "    return[cx+(dE*Math.sin(rowH)+dN*Math.cos(rowH))*scale,\n"
        "           cy+(dE*Math.cos(rowH)-dN*Math.sin(rowH))*scale];\n"
        "  }\n"
        "  const rowIdx = pos.currentRowIndex||0;\n"
        "  // Draw 7 rows\n"
        "  for(let i=rowIdx-3;i<=rowIdx+3;i++){\n"
        "    const yy = cy+(i-rowIdx)*rs*scale;\n"
        "    ctx.strokeStyle = i===rowIdx?'#00cc44':'#1a3a1a';\n"
        "    ctx.lineWidth = i===rowIdx?2:1;\n"
        "    ctx.beginPath(); ctx.moveTo(0,yy); ctx.lineTo(W,yy); ctx.stroke();\n"
        "  }\n"));

    httpServer.sendContent(F(
        "  // Draw trees\n"
        "  if(grid&&grid.trees){\n"
        "    grid.trees.forEach((t,i)=>{\n"
        "      const[sx,sy]=toC(t[0],t[1]);\n"
        "      if(sx<-20||sx>W+20||sy<-20||sy>H+20)return;\n"
        "      ctx.beginPath(); ctx.arc(sx,sy,4,0,Math.PI*2);\n"
        "      if(firedSet.has(i)||t[2]){"
        "        ctx.fillStyle='#ffff00'; ctx.fill();\n"
        "      } else {\n"
        "        ctx.strokeStyle='#ffffff'; ctx.lineWidth=1; ctx.stroke();\n"
        "      }\n"
        "    });\n"
        "  }\n"
        "  // Antenna dot\n"
        "  const[ax,ay]=toC(pos.antennaE,pos.antennaN);\n"
        "  ctx.beginPath(); ctx.arc(ax,ay,3,0,Math.PI*2);\n"
        "  ctx.fillStyle='#aaaaaa'; ctx.fill();\n"
        "  // Nozzle triangle pointing in direction of travel\n"
        "  const hErr=(pos.headingErrorDeg||0)*Math.PI/180;\n"
        "  ctx.save(); ctx.translate(cx,cy); ctx.rotate(hErr);\n"
        "  ctx.fillStyle='#00ff44';\n"
        "  ctx.beginPath(); ctx.moveTo(14,0); ctx.lineTo(-8,-7); ctx.lineTo(-8,7);\n"
        "  ctx.closePath(); ctx.fill();\n"
        "  ctx.restore();\n"
        "  // Crosshair\n"
        "  ctx.strokeStyle='#444'; ctx.lineWidth=1;\n"
        "  ctx.beginPath(); ctx.moveTo(cx-10,cy); ctx.lineTo(cx+10,cy);\n"
        "  ctx.moveTo(cx,cy-10); ctx.lineTo(cx,cy+10); ctx.stroke();\n"
        "}\n"

        "resize(); fetchGrid(); connect();\n"
        "</script></body></html>"));

    httpServer.sendContent("");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HTTP — FORM HANDLERS
// ═══════════════════════════════════════════════════════════════════════════════
void handleSaveSpray() {
    if (httpServer.hasArg("relayOnMs")) cfg.relayOnMs  = httpServer.arg("relayOnMs").toInt();
    if (httpServer.hasArg("cooldownMs"))cfg.cooldownMs = httpServer.arg("cooldownMs").toInt();
    saveConfig();
    httpServer.sendHeader("Location", "/"); httpServer.send(303);
}

void handleSaveOffset() {
    if (httpServer.hasArg("foreAftM")) cfg.foreAftM = httpServer.arg("foreAftM").toFloat();
    if (httpServer.hasArg("lateralM")) cfg.lateralM = httpServer.arg("lateralM").toFloat();
    saveConfig();
    httpServer.sendHeader("Location", "/"); httpServer.send(303);
}

void handleSaveGrid() {
    if (httpServer.hasArg("rowSpacing"))    cfg.rowSpacing    = httpServer.arg("rowSpacing").toFloat();
    if (httpServer.hasArg("treeSpacing"))   cfg.treeSpacing   = httpServer.arg("treeSpacing").toFloat();
    if (httpServer.hasArg("linesEachSide")) cfg.linesEachSide = httpServer.arg("linesEachSide").toInt();
    saveConfig();
    if (gridReady && abLineCount > 0) buildGridFromABLines();
    httpServer.sendHeader("Location", "/"); httpServer.send(303);
}

void handleSaveLines() {
    if (httpServer.hasArg("rowLineName"))
        strlcpy(cfg.rowLineName,  httpServer.arg("rowLineName").c_str(),  sizeof(cfg.rowLineName));
    if (httpServer.hasArg("treeLineName"))
        strlcpy(cfg.treeLineName, httpServer.arg("treeLineName").c_str(), sizeof(cfg.treeLineName));
    saveConfig();
    httpServer.sendHeader("Location", "/"); httpServer.send(303);
}

void handleSaveWifi() {
    if (httpServer.hasArg("wifiSSID"))
        strlcpy(cfg.wifiSSID, httpServer.arg("wifiSSID").c_str(), sizeof(cfg.wifiSSID));
    if (httpServer.hasArg("wifiPass") && httpServer.arg("wifiPass").length() > 0)
        strlcpy(cfg.wifiPass, httpServer.arg("wifiPass").c_str(), sizeof(cfg.wifiPass));
    saveConfig();
    httpServer.send(200, "text/html",
        "<html><body style='background:#1b2b1b;color:#e0e0e0;font-family:Arial'>"
        "<p>Saved. Rebooting&hellip;</p></body></html>");
    delay(1000); ESP.restart();
}

void handleClearLog() {
    calLogHead = 0; calLogCount = 0;
    memset(calLog, 0, sizeof(calLog));
    strcpy(lastMarkStr, "--");
    httpServer.sendHeader("Location", "/"); httpServer.send(303);
}

void handleReboot() {
    httpServer.send(200, "text/html",
        "<html><body style='background:#1b2b1b;color:#e0e0e0;font-family:Arial'>"
        "<p>Rebooting&hellip;</p></body></html>");
    delay(500); ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
    pinMode(LED_PIN,   OUTPUT); digitalWrite(LED_PIN,   LOW);

    loadConfig();
    Serial.printf("\nTreeMarker  SSID=%s\n", cfg.wifiSSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 40) {
        delay(500); t++;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        Serial.print('.');
    }
    digitalWrite(LED_PIN, LOW);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Open browser: http://%s/\n", WiFi.localIP().toString().c_str());
        Serial.printf("Live map:     http://%s/map\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi failed — AP mode");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("TreeMarker-Setup", "setup1234");
        Serial.printf("AP IP=%s\n", WiFi.softAPIP().toString().c_str());
    }

    fieldUdp.begin(UDP_FIELD_PORT);

    wsServer.begin();
    wsServer.onEvent(webSocketEvent);

    httpServer.on("/",            HTTP_GET,  sendSettingsPage);
    httpServer.on("/map",         HTTP_GET,  sendMapPage);
    httpServer.on("/api/grid",    HTTP_GET,  handleApiGrid);
    httpServer.on("/save-spray",  HTTP_POST, handleSaveSpray);
    httpServer.on("/save-offset", HTTP_POST, handleSaveOffset);
    httpServer.on("/save-grid",   HTTP_POST, handleSaveGrid);
    httpServer.on("/save-lines",  HTTP_POST, handleSaveLines);
    httpServer.on("/save-wifi",   HTTP_POST, handleSaveWifi);
    httpServer.on("/clear-log",   HTTP_POST, handleClearLog);
    httpServer.on("/reboot",      HTTP_POST, handleReboot);
    httpServer.begin();

    Serial.printf("HTTP port 80  WS port %d  UDP port %d\n",
                  WS_PORT, UDP_FIELD_PORT);
}

void loop() {
    httpServer.handleClient();
    wsServer.loop();
    processFieldUDP();
    readGPS();

    // After each GPS fix: update guidance, check trigger, broadcast
    static bool prevValid = false;
    if (gpsValid && (!prevValid || true)) {   // broadcast every fix
        calcGuidance();
        checkTrigger();
        broadcastGPS();
    }
    prevValid = gpsValid;
}
