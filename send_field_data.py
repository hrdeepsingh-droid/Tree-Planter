#!/usr/bin/env python3
"""
send_field_data.py  –  TreeMarker field data sender
Sends tree grid data to the ESP32 via UDP.

Edit the SETTINGS section below before running.
Run:  python send_field_data.py
"""

import json
import math
import os
import socket
import sys
import time
from collections import defaultdict

# ═══════════════════════════════════════════════════════════════════════════════
#  SETTINGS  –  edit these
# ═══════════════════════════════════════════════════════════════════════════════
FIELD_FOLDER          = "C:/AgOpenGPS/Fields/MyField"
DXF_FILE              = "C:/CAD/tree_layout.dxf"
ESP32_IP              = "192.168.43.100"
ESP32_PORT            = 8899
INCLUDE_BOUNDARY      = True
DXF_REAL_WORLD_COORDS = True    # True = DXF in UTM coords, False = already local
DUPLICATE_TOLERANCE_M = 0.05    # metres for point deduplication
# ═══════════════════════════════════════════════════════════════════════════════

CHUNK_SIZE    = 80
SEND_ATTEMPTS = 3
SEND_DELAY    = 0.3   # seconds between attempts


# ───────────────────────────────────────────────────────────────────────────────
#  Field.txt  (always read first for UTM offsets)
# ───────────────────────────────────────────────────────────────────────────────
def read_field_txt() -> tuple[float, float, int]:
    """Return (fieldEasting, fieldNorthing, fieldZone) from Field.txt $Offsets section."""
    path = os.path.join(FIELD_FOLDER, "Field.txt")
    if not os.path.exists(path):
        _die(f"Field.txt not found: {path}\n"
             "Edit FIELD_FOLDER at the top of this script.")
    east = north = 0.0
    zone = 0
    in_offsets = False
    vals: list[str] = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line == "$Offsets":
                in_offsets = True
                vals = []
                continue
            if in_offsets:
                if line.startswith("$"):
                    break
                vals.append(line)
    if len(vals) >= 3:
        try:
            east  = float(vals[0])
            north = float(vals[1])
            zone  = int(vals[2])
        except ValueError:
            pass
    if zone == 0:
        _die("Could not read $Offsets from Field.txt.\n"
             "Expected format:\n  $Offsets\n  500000.00\n  6200000.00\n  54")
    return east, north, zone


# ───────────────────────────────────────────────────────────────────────────────
#  ABLines.txt
# ───────────────────────────────────────────────────────────────────────────────
def read_ablines_txt() -> list[dict]:
    """Parse ABLines.txt. Returns list of {name, heading, easting, northing}."""
    path = os.path.join(FIELD_FOLDER, "ABLines.txt")
    if not os.path.exists(path):
        _die(f"ABLines.txt not found: {path}\n"
             "Edit FIELD_FOLDER at the top of this script.")
    lines = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            row = raw.strip()
            if not row or row.startswith("$"):
                continue
            parts = [p.strip() for p in row.split(",")]
            if len(parts) < 4:
                continue
            try:
                lines.append({
                    "name":     parts[0],
                    "heading":  float(parts[1]),
                    "easting":  float(parts[2]),
                    "northing": float(parts[3]),
                })
            except ValueError:
                pass
    return lines


# ───────────────────────────────────────────────────────────────────────────────
#  Boundary.txt
# ───────────────────────────────────────────────────────────────────────────────
def read_boundary_txt() -> list[dict]:
    """Parse Boundary.txt. Returns list of {e, n}, skipping True/False flag lines."""
    path = os.path.join(FIELD_FOLDER, "Boundary.txt")
    if not os.path.exists(path):
        return []
    pts = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            row = raw.strip()
            if not row or row.lower() in ("true", "false"):
                continue
            parts = [p.strip() for p in row.split(",")]
            if len(parts) < 2:
                continue
            try:
                pts.append({"e": float(parts[0]), "n": float(parts[1])})
            except ValueError:
                pass
    return pts


# ───────────────────────────────────────────────────────────────────────────────
#  UDP sender
# ───────────────────────────────────────────────────────────────────────────────
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _send(data: dict, label: str = "") -> None:
    payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
    for i in range(SEND_ATTEMPTS):
        try:
            _sock.sendto(payload, (ESP32_IP, ESP32_PORT))
        except OSError as e:
            print(f"  [attempt {i+1}] send error: {e}")
        if i < SEND_ATTEMPTS - 1:
            time.sleep(SEND_DELAY)
    tag = f" [{label}]" if label else ""
    print(f"  Sent{tag}: {len(payload):,} bytes")


def _die(msg: str) -> None:
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)


# ───────────────────────────────────────────────────────────────────────────────
#  Option 1 — AgOpenGPS AB lines
# ───────────────────────────────────────────────────────────────────────────────
def send_agopengps() -> None:
    print("\n─── Option 1: AgOpenGPS AB lines ───────────────────────────")
    east, north, zone = read_field_txt()
    print(f"  Field.txt: zone={zone}  E={east:.1f}  N={north:.1f}")

    ab_lines = read_ablines_txt()
    if not ab_lines:
        _die("ABLines.txt is empty or could not be parsed.")
    print(f"  AB lines ({len(ab_lines)}):")
    for ab in ab_lines:
        print(f"    '{ab['name']}'  hdg={ab['heading']:.1f}°  "
              f"E={ab['easting']:.1f}  N={ab['northing']:.1f}")

    boundary = []
    if INCLUDE_BOUNDARY:
        boundary = read_boundary_txt()
        print(f"  Boundary: {len(boundary)} pts" if boundary
              else "  Boundary.txt not found (optional)")

    packet = {
        "mode":         "ablines",
        "fieldZone":    zone,
        "fieldEasting": east,
        "fieldNorthing":north,
        "allLines":     ab_lines,
    }
    if boundary:
        packet["boundary"] = boundary

    payload_str = json.dumps(packet, separators=(",", ":"))
    if len(payload_str.encode()) <= 7000:
        print(f"\n  Sending single packet to {ESP32_IP}:{ESP32_PORT} …")
        _send(packet, "AB lines")
    else:
        # Chunk the AB lines array (unusual but possible for very large fields)
        total_chunks = math.ceil(len(ab_lines) / CHUNK_SIZE)
        print(f"\n  Sending in {total_chunks} chunks …")
        for ci in range(total_chunks):
            chunk = ab_lines[ci*CHUNK_SIZE:(ci+1)*CHUNK_SIZE]
            pkt = {**packet, "allLines": chunk,
                   "chunkIdx": ci, "totalChunks": total_chunks}
            if "boundary" in pkt and ci > 0:
                del pkt["boundary"]  # only in first chunk
            _send(pkt, f"chunk {ci+1}/{total_chunks}")

    print(f"\nDone. Open browser: http://{ESP32_IP}/")


# ───────────────────────────────────────────────────────────────────────────────
#  DXF helpers (shared by Options 2 & 3)
# ───────────────────────────────────────────────────────────────────────────────
def _load_ezdxf():
    try:
        import ezdxf
        return ezdxf
    except ImportError:
        _die("ezdxf is not installed.\n  Install with:  pip install ezdxf")


def _extract_dxf(ezdxf):
    """Returns (circles, segments) from DXF modelspace."""
    if not os.path.exists(DXF_FILE):
        _die(f"DXF file not found: {DXF_FILE}\n"
             "Edit DXF_FILE at the top of this script.")
    doc = ezdxf.readfile(DXF_FILE)
    msp = doc.modelspace()
    circles, segments = [], []

    for e in msp:
        t = e.dxftype()
        if t == "CIRCLE":
            c = e.dxf.center
            circles.append((float(c.x), float(c.y)))
        elif t == "LINE":
            s, en = e.dxf.start, e.dxf.end
            segments.append(((float(s.x), float(s.y)), (float(en.x), float(en.y))))
        elif t == "LWPOLYLINE":
            pts = list(e.get_points("xy"))
            for i in range(len(pts)-1):
                segments.append((pts[i], pts[i+1]))
            if e.is_closed and len(pts) > 2:
                segments.append((pts[-1], pts[0]))
        elif t == "POLYLINE":
            verts = list(e.vertices)
            for i in range(len(verts)-1):
                a, b = verts[i].dxf.location, verts[i+1].dxf.location
                segments.append(((float(a.x), float(a.y)), (float(b.x), float(b.y))))
    return circles, segments


def _seg_intersect(p1, p2, p3, p4):
    d1x, d1y = p2[0]-p1[0], p2[1]-p1[1]
    d2x, d2y = p4[0]-p3[0], p4[1]-p3[1]
    cross = d1x*d2y - d1y*d2x
    if abs(cross) < 1e-12:
        return None
    dx, dy = p3[0]-p1[0], p3[1]-p1[1]
    t = (dx*d2y - dy*d2x) / cross
    u = (dx*d1y - dy*d1x) / cross
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (p1[0]+t*d1x, p1[1]+t*d1y)
    return None


def _dedup(pts: list, tol: float) -> list:
    out = []
    for p in pts:
        for q in out:
            if abs(p[0]-q[0]) < tol and abs(p[1]-q[1]) < tol:
                break
        else:
            out.append(p)
    return out


def _dxf_to_local(pts, east, north):
    if DXF_REAL_WORLD_COORDS:
        return [(x-east, y-north) for x, y in pts]
    return list(pts)


def _send_points(local_pts, east, north, zone):
    """Send a list of (e, n) local points in chunked UDP packets."""
    total = len(local_pts)
    total_chunks = math.ceil(total / CHUNK_SIZE)

    # Header packet
    header = {
        "mode":         "points",
        "fieldZone":    zone,
        "fieldEasting": east,
        "fieldNorthing":north,
        "totalChunks":  total_chunks,
        "totalPoints":  total,
    }
    _send(header, "header")

    for ci in range(total_chunks):
        start = ci * CHUNK_SIZE
        end   = min(start + CHUNK_SIZE, total)
        pts   = [{"e": local_pts[i][0], "n": local_pts[i][1]}
                 for i in range(start, end)]
        pkt = {
            "mode":        "points",
            "chunkIdx":    ci,
            "totalChunks": total_chunks,
            "totalPoints": total,
            "points":      pts,
        }
        _send(pkt, f"chunk {ci+1}/{total_chunks}")
    print(f"\n  All {total_chunks} chunks sent.")
    print(f"  Open browser: http://{ESP32_IP}/")


# ───────────────────────────────────────────────────────────────────────────────
#  Option 2 — DXF points only
# ───────────────────────────────────────────────────────────────────────────────
def send_dxf_points_only() -> None:
    print("\n─── Option 2: AutoCAD DXF — tree positions ─────────────────")
    ezdxf = _load_ezdxf()
    east, north, zone = read_field_txt()
    print(f"  Field.txt: zone={zone}  E={east:.1f}  N={north:.1f}")

    circles, segments = _extract_dxf(ezdxf)
    print(f"  DXF: {len(circles)} circles, {len(segments)} segments")

    print("  Computing intersections …", end="", flush=True)
    intersections = []
    n = len(segments)
    for i in range(n):
        for j in range(i+1, n):
            pt = _seg_intersect(segments[i][0], segments[i][1],
                                segments[j][0], segments[j][1])
            if pt:
                intersections.append(pt)
    print(f" {len(intersections)} found")

    combined = circles + intersections
    combined = _dedup(combined, DUPLICATE_TOLERANCE_M)
    local    = _dxf_to_local(combined, east, north)
    print(f"  After dedup: {len(local)} points")

    print("\n  Sample (first 5 local coords):")
    for p in local[:5]:
        print(f"    E={p[0]:.3f}  N={p[1]:.3f}")

    ans = input(f"\n  Send {len(local)} points to {ESP32_IP}:{ESP32_PORT}? [y/N] ").strip().lower()
    if ans != "y":
        print("  Cancelled.")
        return

    _send_points(local, east, north, zone)


# ───────────────────────────────────────────────────────────────────────────────
#  Option 3 — DXF points + auto-generate AB lines
# ───────────────────────────────────────────────────────────────────────────────
def _seg_angle_deg(seg) -> float:
    """Segment bearing in [0, 180) degrees."""
    dx = seg[1][0] - seg[0][0]
    dy = seg[1][1] - seg[0][1]
    a  = math.degrees(math.atan2(dx, dy)) % 180.0
    return a


def _seg_length(seg) -> float:
    dx = seg[1][0] - seg[0][0]
    dy = seg[1][1] - seg[0][1]
    return math.sqrt(dx*dx + dy*dy)


def _median_spacing(segs, group_angle_deg, tol_deg=10.0) -> float:
    """
    Measure median perpendicular spacing between parallel lines in a group.
    Project midpoints of each segment onto the axis perpendicular to the group.
    Sort projections; median gap between adjacent projections = spacing.
    """
    perp_rad = math.radians(group_angle_deg + 90.0)
    pe, pn   = math.sin(perp_rad), math.cos(perp_rad)

    midpoints = []
    for seg in segs:
        mx = (seg[0][0] + seg[1][0]) / 2
        my = (seg[0][1] + seg[1][1]) / 2
        midpoints.append(mx*pe + my*pn)

    if len(midpoints) < 2:
        return 0.0

    # Cluster midpoints (many segments may belong to same line)
    midpoints.sort()
    CLUSTER_TOL = 0.5   # midpoints within 0.5m → same line
    clusters = []
    cur = [midpoints[0]]
    for m in midpoints[1:]:
        if m - cur[-1] < CLUSTER_TOL:
            cur.append(m)
        else:
            clusters.append(sum(cur)/len(cur))
            cur = [m]
    clusters.append(sum(cur)/len(cur))

    if len(clusters) < 2:
        return 0.0

    gaps = [clusters[i+1] - clusters[i] for i in range(len(clusters)-1)]
    gaps.sort()
    return gaps[len(gaps)//2]   # median gap


def auto_detect_ab_lines(segments, east, north):
    """
    Detect two dominant perpendicular directions from DXF segments.
    Returns dict with keys: row_angle_deg, tree_angle_deg,
                             row_spacing, tree_spacing,
                             row_seg (longest), tree_seg (longest)
    """
    if not segments:
        _die("No line/polyline segments found in DXF.")

    # Bin segment angles (weighted by length) into 1-degree bins [0, 180)
    bins = defaultdict(float)
    for seg in segments:
        length = _seg_length(seg)
        if length < 0.1:
            continue
        angle = _seg_angle_deg(seg)
        bins[int(angle)] += length

    if not bins:
        _die("No non-trivial segments found in DXF.")

    # Find strongest angle peak
    peak1 = max(bins, key=lambda k: bins[k])
    peak1_weight = bins[peak1]

    # Find peak most perpendicular to peak1 (i.e., peak1 ± 90°, wrapping at 180°)
    def perp_score(a, b):
        diff = abs(a - b) % 180
        return min(diff, 180 - diff)   # 0 = parallel, 90 = perpendicular

    peak2 = max(
        (k for k in bins if perp_score(k, peak1) > 45),
        key=lambda k: bins[k],
        default=None
    )
    if peak2 is None:
        _die("Could not find two perpendicular dominant directions in DXF.\n"
             "Ensure the DXF contains grid lines in two directions.")

    print(f"  Detected dominant angles: {peak1}° (w={peak1_weight:.1f})  "
          f"and {peak2}° (w={bins[peak2]:.1f})")

    # Classify segments into groups
    ANGLE_TOL = 10.0
    def angle_matches(seg, target):
        a = _seg_angle_deg(seg)
        diff = abs(a - target) % 180
        return min(diff, 180 - diff) < ANGLE_TOL

    group1 = [s for s in segments if angle_matches(s, peak1)]
    group2 = [s for s in segments if angle_matches(s, peak2)]

    sp1 = _median_spacing(group1, peak1)
    sp2 = _median_spacing(group2, peak2)

    print(f"  Group {peak1}°: {len(group1)} segments, median spacing = {sp1:.3f} m")
    print(f"  Group {peak2}°: {len(group2)} segments, median spacing = {sp2:.3f} m")

    # Warn if spacings are ambiguous (within 20% of each other)
    if sp1 > 0 and sp2 > 0:
        ratio = max(sp1, sp2) / min(sp1, sp2)
        if ratio < 1.20:
            print(f"\n  WARNING: Spacings are ambiguous ({sp1:.3f} vs {sp2:.3f} m, "
                  f"ratio {ratio:.2f}). Cannot reliably assign rows vs trees.")
            print(f"  Group A: angle={peak1}°, spacing={sp1:.3f} m")
            print(f"  Group B: angle={peak2}°, spacing={sp2:.3f} m")
            ans = input("  Which group is ROWS? Enter A or B: ").strip().upper()
            if ans == "A":
                row_angle, tree_angle = peak1, peak2
                row_sp, tree_sp = sp1, sp2
                row_grp, tree_grp = group1, group2
            elif ans == "B":
                row_angle, tree_angle = peak2, peak1
                row_sp, tree_sp = sp2, sp1
                row_grp, tree_grp = group2, group1
            else:
                _die("Invalid selection.")
        else:
            # Larger spacing = rows
            if sp1 >= sp2:
                row_angle, tree_angle = peak1, peak2
                row_sp, tree_sp = sp1, sp2
                row_grp, tree_grp = group1, group2
            else:
                row_angle, tree_angle = peak2, peak1
                row_sp, tree_sp = sp2, sp1
                row_grp, tree_grp = group2, group1
    else:
        row_angle, tree_angle = peak1, peak2
        row_sp, tree_sp = sp1, sp2
        row_grp, tree_grp = group1, group2

    print(f"\n  → Rows:  angle={row_angle}°, spacing={row_sp:.3f} m")
    print(f"  → Trees: angle={tree_angle}°, spacing={tree_sp:.3f} m")

    # Reference segment = longest in each group
    row_ref  = max(row_grp,  key=_seg_length, default=None)
    tree_ref = max(tree_grp, key=_seg_length, default=None)

    return {
        "row_angle":  row_angle,
        "tree_angle": tree_angle,
        "row_sp":     row_sp,
        "tree_sp":    tree_sp,
        "row_ref":    row_ref,
        "tree_ref":   tree_ref,
    }


def _seg_to_ab(seg, east_off, north_off) -> tuple[float, float, float]:
    """
    Convert a segment to (heading_deg, local_easting, local_northing).
    A = start, B = end; heading = degrees clockwise from north, vector A→B.
    Returns local coordinates of A by subtracting field offsets.
    """
    ax, ay = seg[0]
    bx, by = seg[1]
    heading = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
    if DXF_REAL_WORLD_COORDS:
        le = ax - east_off
        ln = ay - north_off
    else:
        le, ln = ax, ay
    return heading, le, ln


def write_ablines_file(dxf_dir, row_name, row_hdg, row_e, row_n,
                       tree_name, tree_hdg, tree_e, tree_n) -> str:
    out_path = os.path.join(dxf_dir, "ABLines_generated.txt")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("$LineCount\n2\n")
        fh.write(f"{row_name},{row_hdg:.4f},{row_e:.4f},{row_n:.4f}\n")
        fh.write(f"{tree_name},{tree_hdg:.4f},{tree_e:.4f},{tree_n:.4f}\n")
    return out_path


def send_dxf_with_ablines() -> None:
    print("\n─── Option 3: DXF points + auto-generate AB lines ──────────")
    ezdxf = _load_ezdxf()
    east, north, zone = read_field_txt()
    print(f"  Field.txt: zone={zone}  E={east:.1f}  N={north:.1f}")

    circles, segments = _extract_dxf(ezdxf)
    print(f"  DXF: {len(circles)} circles, {len(segments)} segments")

    # ── Direction detection ────────────────────────────────────────────────────
    detected = auto_detect_ab_lines(segments, east, north)

    # ── AB line file generation ────────────────────────────────────────────────
    row_name  = "Row Direction"
    tree_name = "Tree Direction"

    if detected["row_ref"] is None or detected["tree_ref"] is None:
        _die("Could not find reference segments for AB line generation.")

    row_hdg,  row_e,  row_n  = _seg_to_ab(detected["row_ref"],  east, north)
    tree_hdg, tree_e, tree_n = _seg_to_ab(detected["tree_ref"], east, north)

    dxf_dir  = os.path.dirname(os.path.abspath(DXF_FILE))
    out_path = write_ablines_file(
        dxf_dir,
        row_name,  row_hdg,  row_e,  row_n,
        tree_name, tree_hdg, tree_e, tree_n,
    )

    print(f"\n  ✓ ABLines_generated.txt written to:\n    {out_path}")
    print(f"\n  Copy this file to your AgOpenGPS field folder:")
    print(f"    {FIELD_FOLDER}")
    print(f"\n  In AgOpenGPS, set tool width = {detected['row_sp']:.2f} m "
          f"(row spacing) so parallel tracks generate correctly.")
    print(f"\n  Detected headings:")
    print(f"    Row Direction  : {row_hdg:.1f}°  (spacing {detected['row_sp']:.3f} m)")
    print(f"    Tree Direction : {tree_hdg:.1f}°  (spacing {detected['tree_sp']:.3f} m)")

    # ── Tree intersection points ───────────────────────────────────────────────
    print("\n  Computing intersections for DXF mode send …", end="", flush=True)
    intersections = []
    n = len(segments)
    for i in range(n):
        for j in range(i+1, n):
            pt = _seg_intersect(segments[i][0], segments[i][1],
                                segments[j][0], segments[j][1])
            if pt:
                intersections.append(pt)
    print(f" {len(intersections)} found")

    combined = circles + intersections
    combined = _dedup(combined, DUPLICATE_TOLERANCE_M)
    local    = _dxf_to_local(combined, east, north)
    print(f"  After dedup: {len(local)} tree positions")

    print("\n  Sample (first 5 local coords):")
    for p in local[:5]:
        print(f"    E={p[0]:.3f}  N={p[1]:.3f}")

    ans = input(f"\n  Send {len(local)} points to {ESP32_IP}:{ESP32_PORT}? [y/N] ").strip().lower()
    if ans != "y":
        print("  Cancelled.")
        return

    _send_points(local, east, north, zone)


# ───────────────────────────────────────────────────────────────────────────────
#  Main menu
# ───────────────────────────────────────────────────────────────────────────────
def main() -> None:
    print("=" * 58)
    print("  TreeMarker  –  Field Data Sender")
    print("=" * 58)
    print(f"  ESP32 target : {ESP32_IP}:{ESP32_PORT}")
    print(f"  Field folder : {FIELD_FOLDER}")
    print()
    print("  1  AgOpenGPS AB lines")
    print("       (reads Field.txt + ABLines.txt + Boundary.txt)")
    print()
    print("  2  AutoCAD DXF — tree positions only")
    print("       (circles + line intersections → points mode)")
    print()
    print("  3  AutoCAD DXF — tree positions + auto-generate AB lines")
    print("       (detects dominant directions, writes ABLines_generated.txt)")
    print()
    choice = input("Select [1/2/3]: ").strip()

    if choice == "1":
        send_agopengps()
    elif choice == "2":
        send_dxf_points_only()
    elif choice == "3":
        send_dxf_with_ablines()
    else:
        print("Invalid choice. Enter 1, 2 or 3.")
        sys.exit(1)


if __name__ == "__main__":
    main()
