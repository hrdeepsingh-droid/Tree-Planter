"""
sim_gps.py — Fake AgIO GPS broadcaster for tree-marker-alr desk testing

Broadcasts $GNGGA sentences to 172.20.10.255:9999 (same as AgIO would).
Starts at ORIGIN (row 0, tree 0) and walks east at ~1 m/s along the row.
The ALR board should fire its relay every 3.0 m (tree spacing).

Run:  py -3 sim_gps.py
Stop: Ctrl-C
"""

import socket, time, math

# ── Match these to tree_marker.ino config ─────────────────────────
ORIGIN_LAT   = -34.3166591
ORIGIN_LON   =  142.1562539
ROW_BEARING  = 180.0       # degrees — must match tree_marker.ino
TREE_SPACING =  3.0        # metres between trees
NUM_TREES    =  50
SPEED_MPS    =  1.0        # simulated walk speed (m/s)
UDP_IP       = "192.168.1.232"   # ALR board direct IP (NETGEAR94)
UDP_PORT     = 8888
INTERVAL     = 0.1         # seconds between sentences (10 Hz)
# ──────────────────────────────────────────────────────────────────

METRES_PER_DEG_LAT = 111320.0

def metres_to_deg_lon(lat):
    return 111320.0 * math.cos(math.radians(lat))

def nmea_checksum(sentence):
    cs = 0
    for c in sentence:
        cs ^= ord(c)
    return f"{cs:02X}"

def make_gngga(lat, lon):
    # Convert decimal degrees to NMEA ddmm.mmmmm format
    def to_nmea(deg, is_lat):
        d = int(abs(deg))
        m = (abs(deg) - d) * 60.0
        if is_lat:
            return f"{d:02d}{m:08.5f}", "S" if deg < 0 else "N"
        else:
            return f"{d:03d}{m:08.5f}", "W" if deg < 0 else "E"

    t = time.gmtime()
    utc = f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00"
    la, ns = to_nmea(lat, True)
    lo, ew = to_nmea(lon, False)
    # quality 4 = RTK fixed
    body = f"GNGGA,{utc},{la},{ns},{lo},{ew},4,12,0.7,333.4,M,0.0,M,,"
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print(f"Simulating GPS walk along row 0 -> broadcasting to {UDP_IP}:{UDP_PORT}")
print(f"Origin: {ORIGIN_LAT}, {ORIGIN_LON}")
print(f"Expect relay hits every {TREE_SPACING} m  ({NUM_TREES} trees)")
print("Press Ctrl-C to stop\n")

dist = 0.0          # metres east of origin
step = SPEED_MPS * INTERVAL

brg_rad = math.radians(ROW_BEARING)

try:
    while dist <= TREE_SPACING * NUM_TREES:
        # Walk along ROW_BEARING from origin
        dlat_m = dist * math.cos(brg_rad)
        dlon_m = dist * math.sin(brg_rad)
        lat = ORIGIN_LAT + dlat_m / METRES_PER_DEG_LAT
        lon = ORIGIN_LON + dlon_m / metres_to_deg_lon(ORIGIN_LAT)
        sentence = make_gngga(lat, lon)
        sock.sendto(sentence.encode(), (UDP_IP, UDP_PORT))
        tree_idx = dist / TREE_SPACING
        print(f"  dist={dist:6.2f}m  tree~{tree_idx:.1f}  lat={lat:.7f}  lon={lon:.7f}", end="\r")
        dist += step
        time.sleep(INTERVAL)
    print(f"\nDone — walked {NUM_TREES} tree spacings ({TREE_SPACING * NUM_TREES:.0f} m)")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sock.close()
