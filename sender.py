#!/usr/bin/env python3
"""
TreeMarker field data sender
Sends AB line grid data or AutoCAD DXF tree positions to the ESP32 via UDP.

Edit the settings below before running.
"""

import json
import math
import os
import socket
import sys

# ═══════════════════════════════════════════════════════════════════════════════
#  SETTINGS  –  edit these before running
# ═══════════════════════════════════════════════════════════════════════════════

# Folder containing Field.txt, ABLines.txt, (optionally) Boundary.txt
FIELD_FOLDER = "C:/AgOpenGPS/Fields/MyField"

# Full path to the AutoCAD DXF file (Mode 2 only)
DXF_FILE = "C:/AutoCAD/tree_layout.dxf"

# IP address of the ESP32 on the field WiFi network
ESP32_IP = "192.168.1.100"

# UDP port – must match UDP_PORT in the sketch (default 8888)
ESP32_PORT = 8888

# Set True to also read and send Boundary.txt in AgOpenGPS mode
INCLUDE_BOUNDARY = True

# True  → DXF coordinates are real-world UTM (subtract Field.txt offsets)
# False → DXF coordinates are already local (AgOpenGPS-style, relative to offsets)
DXF_REAL_WORLD_COORDS = False

# Points closer than this (metres) are treated as duplicates in DXF mode
DUPLICATE_TOLERANCE_M = 0.05

# ═══════════════════════════════════════════════════════════════════════════════
#  INTERNAL CONSTANTS  –  must match sketch values
# ═══════════════════════════════════════════════════════════════════════════════
POINTS_PER_CHUNK = 80   # max tree positions per UDP payload
SEND_ATTEMPTS    = 3    # each packet sent this many times for reliability


# ───────────────────────────────────────────────────────────────────────────────
#  Field.txt parser
# ───────────────────────────────────────────────────────────────────────────────
def read_field_txt(folder: str) -> tuple[int, float, float]:
    """Return (utm_zone, easting_offset, northing_offset) from Field.txt."""
    path = os.path.join(folder, "Field.txt")
    if not os.path.exists(path):
        _die(
            f"Field.txt not found: {path}\n"
            "Edit FIELD_FOLDER at the top of this script."
        )
    zone = 0
    east_off = north_off = 0.0
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if "=" not in line:
                continue
            key, _, val = line.partition("=")
            key = key.strip().lower()
            val = val.strip()
            if key == "utmzone":
                zone = int(val)
            elif key == "easting":
                east_off = float(val)
            elif key == "northing":
                north_off = float(val)
    if zone == 0:
        _die("Could not read UTMZone from Field.txt – check file format.")
    return zone, east_off, north_off


# ───────────────────────────────────────────────────────────────────────────────
#  ABLines.txt parser
# ───────────────────────────────────────────────────────────────────────────────
def read_ablines_txt(folder: str) -> list[dict]:
    """
    Parse ABLines.txt.
    Expected row format (comma-separated, no header):
        Name, HeadingDeg, Easting, Northing
    Returns list of dicts with keys: name, heading, easting, northing
    """
    path = os.path.join(folder, "ABLines.txt")
    if not os.path.exists(path):
        _die(
            f"ABLines.txt not found: {path}\n"
            "Edit FIELD_FOLDER at the top of this script."
        )
    lines = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            row = raw.strip()
            if not row or row.startswith("#"):
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
                pass   # skip malformed rows
    return lines


# ───────────────────────────────────────────────────────────────────────────────
#  Boundary.txt parser
# ───────────────────────────────────────────────────────────────────────────────
def read_boundary_txt(folder: str) -> list[dict]:
    """
    Parse Boundary.txt.
    Expected row format (comma-separated):
        Easting, Northing
    Returns list of dicts with keys: e, n
    """
    path = os.path.join(folder, "Boundary.txt")
    if not os.path.exists(path):
        return []
    pts = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            row = raw.strip()
            if not row or row.startswith("#"):
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
    for attempt in range(SEND_ATTEMPTS):
        try:
            _sock.sendto(payload, (ESP32_IP, ESP32_PORT))
        except OSError as exc:
            print(f"  [attempt {attempt+1}] send error: {exc}")
    tag = f"  {label}  " if label else "  "
    print(f"{tag}{len(payload):,} bytes → {ESP32_IP}:{ESP32_PORT}")


def _die(msg: str) -> None:
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)


# ───────────────────────────────────────────────────────────────────────────────
#  Mode 1 – AgOpenGPS
# ───────────────────────────────────────────────────────────────────────────────
def mode_agopengps() -> None:
    print("\n─── Mode 1: AgOpenGPS ───────────────────────────────────────")

    zone, east_off, north_off = read_field_txt(FIELD_FOLDER)
    print(f"Field.txt  →  UTM zone {zone}  E-offset {east_off:.1f}  N-offset {north_off:.1f}")

    ab_lines = read_ablines_txt(FIELD_FOLDER)
    if not ab_lines:
        _die("ABLines.txt is empty or could not be parsed.")
    print(f"AB lines loaded ({len(ab_lines)}):")
    for ab in ab_lines:
        print(f"  '{ab['name']}'  heading={ab['heading']:.1f}°  "
              f"E={ab['easting']:.1f}  N={ab['northing']:.1f}")

    boundary = []
    if INCLUDE_BOUNDARY:
        boundary = read_boundary_txt(FIELD_FOLDER)
        if boundary:
            print(f"Boundary loaded: {len(boundary)} points")
        else:
            print("Boundary.txt not found or empty (optional – skipped)")

    packet: dict = {
        "mode":     "ablines",
        "utmZone":  zone,
        "eastOff":  east_off,
        "northOff": north_off,
        "lines":    ab_lines,
    }
    if boundary:
        packet["boundary"] = boundary

    print(f"\nSending to {ESP32_IP}:{ESP32_PORT} …")
    _send(packet, label="AB-lines packet")
    print("Done.\n")


# ───────────────────────────────────────────────────────────────────────────────
#  DXF geometry helpers
# ───────────────────────────────────────────────────────────────────────────────
def _seg_intersect(
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    p4: tuple[float, float],
) -> tuple[float, float] | None:
    """
    Parametric intersection of segments p1-p2 and p3-p4.
    Returns the intersection point, or None if parallel / outside segments.
    """
    d1x, d1y = p2[0] - p1[0], p2[1] - p1[1]
    d2x, d2y = p4[0] - p3[0], p4[1] - p3[1]
    cross = d1x * d2y - d1y * d2x
    if abs(cross) < 1e-12:
        return None
    dx, dy = p3[0] - p1[0], p3[1] - p1[1]
    t = (dx * d2y - dy * d2x) / cross
    u = (dx * d1y - dy * d1x) / cross
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return (p1[0] + t * d1x, p1[1] + t * d1y)
    return None


def _dedup(pts: list[tuple[float, float]], tol: float) -> list[tuple[float, float]]:
    """Remove points within tol metres of an already-kept point."""
    out: list[tuple[float, float]] = []
    for p in pts:
        for q in out:
            if abs(p[0] - q[0]) < tol and abs(p[1] - q[1]) < tol:
                break
        else:
            out.append(p)
    return out


# ───────────────────────────────────────────────────────────────────────────────
#  Mode 2 – AutoCAD DXF
# ───────────────────────────────────────────────────────────────────────────────
def mode_dxf() -> None:
    print("\n─── Mode 2: AutoCAD DXF ─────────────────────────────────────")

    try:
        import ezdxf  # type: ignore
    except ImportError:
        _die(
            "ezdxf library is not installed.\n"
            "Install it with:  pip install ezdxf"
        )

    if not os.path.exists(DXF_FILE):
        _die(
            f"DXF file not found: {DXF_FILE}\n"
            "Edit DXF_FILE at the top of this script."
        )

    zone, east_off, north_off = read_field_txt(FIELD_FOLDER)
    print(f"Field.txt  →  UTM zone {zone}  E-offset {east_off:.1f}  N-offset {north_off:.1f}")

    print(f"Reading DXF: {DXF_FILE}")
    doc = ezdxf.readfile(DXF_FILE)
    msp = doc.modelspace()

    circles: list[tuple[float, float]] = []
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []

    for entity in msp:
        etype = entity.dxftype()

        if etype == "CIRCLE":
            c = entity.dxf.center
            circles.append((float(c.x), float(c.y)))

        elif etype == "LINE":
            s, e = entity.dxf.start, entity.dxf.end
            segments.append(((float(s.x), float(s.y)), (float(e.x), float(e.y))))

        elif etype == "LWPOLYLINE":
            pts = list(entity.get_points("xy"))   # list of (x, y)
            for i in range(len(pts) - 1):
                segments.append((pts[i], pts[i + 1]))
            if entity.is_closed and len(pts) > 2:
                segments.append((pts[-1], pts[0]))

        elif etype == "POLYLINE":
            verts = list(entity.vertices)
            for i in range(len(verts) - 1):
                a = verts[i].dxf.location
                b = verts[i + 1].dxf.location
                segments.append(
                    ((float(a.x), float(a.y)), (float(b.x), float(b.y)))
                )

    print(f"  Found {len(circles)} circle centres, {len(segments)} segments")

    # Find all pairwise segment intersections
    print("  Computing segment intersections …", end="", flush=True)
    intersections: list[tuple[float, float]] = []
    n = len(segments)
    for i in range(n):
        for j in range(i + 1, n):
            pt = _seg_intersect(segments[i][0], segments[i][1],
                                segments[j][0], segments[j][1])
            if pt:
                intersections.append(pt)
    print(f" {len(intersections)} found")

    # Merge circle centres and intersections, deduplicate
    combined = circles + intersections
    combined = _dedup(combined, DUPLICATE_TOLERANCE_M)
    print(f"  After dedup ({DUPLICATE_TOLERANCE_M} m tolerance): {len(combined)} points")

    # Convert to local coordinates
    if DXF_REAL_WORLD_COORDS:
        local = [(x - east_off, y - north_off) for x, y in combined]
    else:
        local = combined

    # Show sample for confirmation
    print("\nSample points (first 5):")
    for i, p in enumerate(local[:5]):
        print(f"  {i+1}:  E={p[0]:.3f}  N={p[1]:.3f}")
    print(f"\nTotal: {len(local)} tree positions to send.")

    ans = input("Send to ESP32? [y/N] ").strip().lower()
    if ans != "y":
        print("Cancelled.")
        return

    total       = len(local)
    total_chunks = math.ceil(total / POINTS_PER_CHUNK)

    print(f"\nSending {total} points in {total_chunks} chunk(s) to {ESP32_IP}:{ESP32_PORT} …")

    # Header packet (chunkIdx=-1 signals header-only to the sketch)
    header = {
        "mode":        "points",
        "utmZone":     zone,
        "eastOff":     east_off,
        "northOff":    north_off,
        "totalChunks": total_chunks,
        "totalPoints": total,
        "chunkIdx":    -1,
        "points":      [],
    }
    _send(header, label="header")

    for chunk_idx in range(total_chunks):
        start = chunk_idx * POINTS_PER_CHUNK
        end   = min(start + POINTS_PER_CHUNK, total)
        pts   = [{"e": local[i][0], "n": local[i][1]} for i in range(start, end)]
        pkt   = {
            "mode":        "points",
            "totalChunks": total_chunks,
            "totalPoints": total,
            "chunkIdx":    chunk_idx,
            "points":      pts,
        }
        _send(pkt, label=f"chunk {chunk_idx + 1}/{total_chunks} ({len(pts)} pts)")

    print("\nAll chunks sent.")


# ───────────────────────────────────────────────────────────────────────────────
#  Entry point
# ───────────────────────────────────────────────────────────────────────────────
def main() -> None:
    print("=" * 55)
    print("  TreeMarker  –  Field Data Sender")
    print("=" * 55)
    print(f"  Target  : {ESP32_IP}:{ESP32_PORT}")
    print(f"  Field   : {FIELD_FOLDER}")
    print()
    print("  1  AgOpenGPS   (AB lines from Field.txt / ABLines.txt)")
    print("  2  AutoCAD DXF (circle centres + line intersections)")
    print()

    choice = input("Select mode [1/2]: ").strip()

    if choice == "1":
        mode_agopengps()
    elif choice == "2":
        mode_dxf()
    else:
        print("Invalid choice. Enter 1 or 2.")
        sys.exit(1)


if __name__ == "__main__":
    main()
