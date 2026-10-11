"""
aptdat_core.py
--------------
Core engine: parses an X-Plane apt.dat file for one airport and turns the
runway + pavement (taxiway/apron) layout into flat 3D geometry.

No GUI code here -- aptdat_app.py wraps this with a Tk GUI / CLI.

Supported apt.dat row codes:
  1 / 16 / 17     airport / seaplane base / heliport header (the ONLY header codes)
  1302            metadata (datum_lat / datum_lon = projection origin; icao/iata/faa/city for search)
  100             land runway
  101             water runway
  102             helipad
  110             pavement (taxiway / apron) header
  111,112,113,114 boundary nodes of a pavement ring
                    111/113 = plain node, 112/114 = node with bezier control point,
                    113/114 = last node of a ring (implicit close)
Everything else (windsocks (19), lighting, signs, ATC flow, taxi routes,
frequencies, parking, boundary, linear features/markings) is intentionally
skipped -- this tool only builds the *pavement layout*.
"""

import math
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

try:
    from earcut.earcut import earcut as _earcut
    _HAVE_EARCUT = True
except Exception:  # pragma: no cover - exercised only when the package is missing
    _earcut = None
    _HAVE_EARCUT = False

Vec2 = Tuple[float, float]

BEZIER_SUBDIVISIONS = 10     # segments per curved edge - smooth enough for airport fillets
_DUP_EPS = 1e-3              # metres: points closer than this are the same point
_COLLINEAR_EPS = 1e-4        # metres: a vertex this close to the line through its neighbours is redundant
_AREA_WARN = 1e-3            # relative triangulation-vs-polygon area error that triggers a warning


# --------------------------------------------------------------------------
# Data model
# --------------------------------------------------------------------------

@dataclass
class Ring:
    points: List[Vec2]          # local XY meters, bezier-flattened and cleaned, closed (no dup last point)


@dataclass
class Pavement:
    surface: int
    rings: List[Ring]           # rings[0] = outer boundary, rings[1:] = holes


@dataclass
class Runway:
    width: float
    surface: int
    end1_id: str
    end2_id: str
    end1: Vec2                  # local XY meters
    end2: Vec2
    water: bool = False


@dataclass
class Helipad:
    name: str
    surface: int
    center: Vec2
    length: float
    width: float
    heading_deg: float


@dataclass
class AirportData:
    icao: str                   # the header identifier used as the lookup key
    name: str = ""
    elevation_ft: float = 0.0
    ref_lat: float = 0.0
    ref_lon: float = 0.0
    runways: List[Runway] = field(default_factory=list)
    pavements: List[Pavement] = field(default_factory=list)
    helipads: List[Helipad] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)


class AirportEntry(NamedTuple):
    """One row of the airport list (header + a few 1302 metadata fields)."""
    ident: str                  # header identifier - what parse_airport() looks up
    name: str
    kind: str = "airport"       # airport | seaplane base | heliport
    icao: str = ""              # 1302 icao_code (often identical to ident)
    iata: str = ""
    faa: str = ""
    city: str = ""
    state: str = ""
    country: str = ""


# --------------------------------------------------------------------------
# Geodesy: local tangent-plane projection on the WGS84 ellipsoid, meters,
# centered on the airport reference point. Uses the true meridional and
# prime-vertical radii of curvature at the reference latitude, so distances
# are right to a few cm over an airport's extent (a flat 111320 m/deg
# overstates north-south distances by ~0.4% at mid latitudes).
# X = east (+), Y = north (+), matches Blender/OBJ ground plane (Z stays up).
# --------------------------------------------------------------------------

_WGS84_A = 6378137.0
_WGS84_E2 = 0.00669437999014


class Projector:
    def __init__(self, ref_lat: float, ref_lon: float):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        phi = math.radians(ref_lat)
        w = math.sqrt(1.0 - _WGS84_E2 * math.sin(phi) ** 2)
        prime_vertical = _WGS84_A / w
        meridional = _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
        deg = math.pi / 180.0
        self._m_per_deg_lon = deg * prime_vertical * math.cos(phi)
        self._m_per_deg_lat = deg * meridional

    def to_xy(self, lat: float, lon: float) -> Vec2:
        dlon = (lon - self.ref_lon + 180.0) % 360.0 - 180.0     # survives the antimeridian
        return (dlon * self._m_per_deg_lon, (lat - self.ref_lat) * self._m_per_deg_lat)


# --------------------------------------------------------------------------
# Small 2D helpers
# --------------------------------------------------------------------------

def _near(p: Vec2, q: Vec2, eps: float = _DUP_EPS) -> bool:
    return abs(p[0] - q[0]) <= eps and abs(p[1] - q[1]) <= eps


def _signed_area(ring: List[Vec2]) -> float:
    s = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def _orient(ring: List[Vec2], ccw: bool) -> List[Vec2]:
    return list(ring) if (_signed_area(ring) > 0) == ccw else list(reversed(ring))


def _dist_to_line(p: Vec2, a: Vec2, b: Vec2) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    return abs(dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / length


def _clean_ring(pts: List[Vec2]) -> List[Vec2]:
    """Remove duplicate points, zero-width spikes and redundant collinear
    vertices so that every vertex we keep is a real corner of the outline."""
    out: List[Vec2] = []
    for p in pts:
        if not out or not _near(p, out[-1]):
            out.append(p)
    while len(out) > 1 and _near(out[0], out[-1]):
        out.pop()
    changed = True
    while changed and len(out) >= 3:
        changed = False
        i = 0
        while len(out) >= 3 and i < len(out):
            a, p, b = out[i - 1], out[i], out[(i + 1) % len(out)]
            if _near(a, b) or _near(p, a) or _near(p, b) or _dist_to_line(p, a, b) <= _COLLINEAR_EPS:
                out.pop(i)
                changed = True
                i = max(i - 1, 0)
            else:
                i += 1
    return out


# --------------------------------------------------------------------------
# Bezier flattening for pavement boundary rings
# --------------------------------------------------------------------------

@dataclass
class _Node:
    pt: Vec2
    ctrl: Optional[Vec2]        # control point as stored in apt.dat (absolute), or None


@dataclass
class _Corner:
    pt: Vec2
    h_in: Optional[Vec2]        # incoming tangent handle (absolute), or None for a straight arrival
    h_out: Optional[Vec2]       # outgoing tangent handle (absolute), or None for a straight departure


def _cubic_bezier(p0, p1, p2, p3, n):
    pts = []
    for i in range(1, n + 1):
        t = i / n
        mt = 1 - t
        x = (mt**3) * p0[0] + 3 * (mt**2) * t * p1[0] + 3 * mt * (t**2) * p2[0] + (t**3) * p3[0]
        y = (mt**3) * p0[1] + 3 * (mt**2) * t * p1[1] + 3 * mt * (t**2) * p2[1] + (t**3) * p3[1]
        pts.append((x, y))
    return pts


def _merge_coincident(nodes: List[_Node], closed: bool) -> List[_Corner]:
    """Collapse runs of nodes that sit on the same point into one corner.

    WorldEditor writes a corner between two curves ("split" bezier node) as
    several nodes at the SAME coordinate, e.g.
        112 P ctrl_a   <- carries the incoming handle (mirror of ctrl_a)
        111 P
        112 P ctrl_b   <- carries the outgoing handle (ctrl_b)
    Treating those as separate nodes makes zero-length bezier segments that
    draw self-overlapping spikes, which wrecks triangulation. Merged, the run
    is a single corner: incoming handle from its FIRST node, outgoing handle
    from its LAST node.
    """
    corners: List[_Corner] = []
    for n in nodes:
        h_out = n.ctrl
        h_in = (2 * n.pt[0] - n.ctrl[0], 2 * n.pt[1] - n.ctrl[1]) if n.ctrl else None
        if corners and _near(corners[-1].pt, n.pt):
            corners[-1].h_out = h_out
        else:
            corners.append(_Corner(n.pt, h_in, h_out))
    if closed and len(corners) > 1 and _near(corners[0].pt, corners[-1].pt):
        # the run wraps around the end of the chain: it starts in the last
        # corner and finishes in the first one
        last = corners.pop()
        corners[0].h_in = last.h_in
    return corners


def _flatten_chain(nodes: List[_Node], closed: bool) -> List[Vec2]:
    """Turn a list of nodes (each optionally with a bezier control point) into
    a flat polyline, per the X-Plane apt.dat curve convention:
      outgoing tangent handle at a node = its control point as-is
      incoming tangent handle at a node = mirror of its control point through the node
    An edge whose two ends have no handle is a straight line.
    """
    corners = _merge_coincident(nodes, closed)
    if len(corners) < 2:
        return [c.pt for c in corners]
    out: List[Vec2] = [corners[0].pt]
    count = len(corners)
    for i in range(count if closed else count - 1):
        a = corners[i]
        b = corners[(i + 1) % count]
        if a.h_out is None and b.h_in is None:
            out.append(b.pt)
            continue
        h1 = a.h_out if a.h_out is not None else a.pt
        h2 = b.h_in if b.h_in is not None else b.pt
        out.extend(_cubic_bezier(a.pt, h1, h2, b.pt, BEZIER_SUBDIVISIONS))
    if closed and len(out) > 1 and _near(out[0], out[-1]):
        out.pop()
    return out


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------

# Airport header row codes. NOTE: 19 is a WINDSOCK, not a header.
_HEADER_PREFIXES = ("1 ", "1\t", "16 ", "16\t", "17 ", "17\t")
_KIND_BY_CODE = {"1": "airport", "16": "seaplane base", "17": "heliport"}
_META_FIELDS = {"icao_code": "icao", "iata_code": "iata", "faa_code": "faa",
                "city": "city", "state": "state", "country": "country"}


def _to_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def list_airports(path: str, progress: Optional[Callable[[float], None]] = None) -> List[AirportEntry]:
    """Return every airport/seaplane base/heliport in the file (single fast
    pass; only header and 1302 metadata rows are looked at). `progress`, if
    given, is called with a 0..1 fraction while scanning."""
    entries: List[AirportEntry] = []
    cur: Optional[dict] = None
    total = max(os.path.getsize(path), 1)
    done = 0
    last_report = 0

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            done += len(raw)
            s = raw.strip()
            if not s:
                continue
            if s.startswith(_HEADER_PREFIXES):
                if cur is not None:
                    entries.append(AirportEntry(**cur))
                toks = s.split()
                if len(toks) > 4:
                    cur = dict(ident=toks[4], name=" ".join(toks[5:]), kind=_KIND_BY_CODE[toks[0]])
                else:
                    cur = None
            elif cur is not None and s.startswith("1302 "):
                parts = s.split(None, 2)
                if len(parts) == 3 and parts[1] in _META_FIELDS:
                    cur[_META_FIELDS[parts[1]]] = parts[2].strip()
            if progress is not None and done - last_report > 4_000_000:
                last_report = done
                progress(min(done / total, 1.0))
    if cur is not None:
        entries.append(AirportEntry(**cur))
    if progress is not None:
        progress(1.0)
    return entries


class AirportIndex:
    """Searchable view over a list of AirportEntry. Matching is case-insensitive;
    every word of the query must appear somewhere in the airport's ident, ICAO,
    IATA, FAA code, name, city, state or country. Exact code matches rank first,
    then code prefixes, then name prefixes."""

    def __init__(self, entries: List[AirportEntry]):
        self.entries = entries
        self._rows = []
        for e in entries:
            low = (e.ident.lower(), e.icao.lower(), e.iata.lower(), e.faa.lower(), e.name.lower())
            hay = " ".join((low[0], low[1], low[2], low[3], low[4],
                            e.city.lower(), e.state.lower(), e.country.lower()))
            self._rows.append((e, hay, low))

    def __len__(self):
        return len(self.entries)

    def search(self, query: str, limit: int = 300) -> Tuple[List[AirportEntry], int]:
        """Return (best `limit` matches, total number of matches)."""
        q = query.strip().lower()
        if not q:
            ordered = sorted(self.entries, key=lambda e: e.ident)
            return ordered[:limit], len(ordered)
        tokens = q.split()
        scored = []
        for e, hay, (ident, icao, iata, faa, name) in self._rows:
            if all(t in hay for t in tokens):
                if q in (ident, icao, iata, faa):
                    score = 0
                elif ident.startswith(q) or icao.startswith(q):
                    score = 1
                elif iata.startswith(q) or faa.startswith(q):
                    score = 2
                elif name.startswith(q):
                    score = 3
                elif any(w.startswith(q) for w in name.split()):
                    score = 4
                else:
                    score = 5
                scored.append((score, e.ident, e))
        scored.sort(key=lambda t: (t[0], t[1]))
        return [t[2] for t in scored[:limit]], len(scored)


def resolve_airport_id(path: str, code: str, entries: Optional[List[AirportEntry]] = None) -> Optional[str]:
    """Map an ICAO / IATA / FAA code (or header ident) to the header ident."""
    q = code.strip().upper()
    if not q:
        return None
    entries = entries if entries is not None else list_airports(path)
    for field_name in ("ident", "icao", "iata", "faa"):
        for e in entries:
            if getattr(e, field_name).upper() == q:
                return e.ident
    return None


def _read_airport_lines(path: str, ident: str) -> Optional[List[str]]:
    """Read just the lines of one airport block (stops at the next header)."""
    target = ident.strip().upper()
    buf: List[str] = []
    inside = False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            if s.startswith(_HEADER_PREFIXES):
                if inside:
                    break
                toks = s.split()
                if len(toks) > 4 and toks[4].upper() == target:
                    inside = True
                    buf = [s]
                continue
            if inside:
                buf.append(s)
    return buf if inside else None


class _PavementBuilder:
    """Collects the node chains (outer ring + holes) of one 110 block."""

    def __init__(self, sink: list):
        self.sink = sink
        self.reset()

    def reset(self):
        self.active = False
        self.surface = 0
        self.chains: List[Tuple[List[_Node], bool]] = []
        self.cur: List[_Node] = []

    def start(self, surface: int):
        self.flush()
        self.active = True
        self.surface = surface

    def node(self, pt: Vec2, ctrl: Optional[Vec2], ends_chain: bool, closed: bool):
        self.cur.append(_Node(pt, ctrl))
        if ends_chain:
            self.chains.append((self.cur, closed))
            self.cur = []

    def flush(self):
        if self.active:
            if self.cur:                       # chain never terminated by 113/114: close it anyway
                self.chains.append((self.cur, True))
            if self.chains:
                self.sink.append((self.surface, self.chains))
        self.reset()


def _parse_block(lines: List[str]) -> AirportData:
    header = lines[0].split()
    ident = header[4]
    data = AirportData(icao=ident, name=" ".join(header[5:]) or ident,
                       elevation_ft=_to_float(header[1]) if len(header) > 1 else 0.0)

    datum_lat = datum_lon = None
    raw_runways = []    # (water, width, surface, id1, lat1, lon1, id2, lat2, lon2)
    raw_helipads = []   # (name, lat, lon, length, width, surface, heading)
    raw_pavements = []  # (surface, [(nodes_latlon, closed), ...])
    pav = _PavementBuilder(raw_pavements)

    for line in lines[1:]:
        toks = line.split()
        code = toks[0]
        if code.startswith("#"):
            continue
        try:
            if code == "1302":
                if len(toks) >= 3 and toks[1] == "datum_lat":
                    datum_lat = float(toks[2])
                elif len(toks) >= 3 and toks[1] == "datum_lon":
                    datum_lon = float(toks[2])
            elif code == "100":
                pav.flush()
                raw_runways.append((False, float(toks[1]), int(float(toks[2])),
                                    toks[8], float(toks[9]), float(toks[10]),
                                    toks[17], float(toks[18]), float(toks[19])))
            elif code == "101":
                # 101 width buoys id lat lon id lat lon
                pav.flush()
                raw_runways.append((True, float(toks[1]), -1,
                                    toks[3], float(toks[4]), float(toks[5]),
                                    toks[6], float(toks[7]), float(toks[8])))
            elif code == "102":
                # 102 designator lat lon heading length width surface markings shoulder smoothness lights
                pav.flush()
                raw_helipads.append((toks[1], float(toks[2]), float(toks[3]),
                                     float(toks[5]), float(toks[6]),
                                     int(float(toks[7])) if len(toks) > 7 else 0, float(toks[4])))
            elif code == "110":
                pav.start(int(float(toks[1])))
            elif code in ("111", "112", "113", "114", "115", "116"):
                if pav.active:
                    pt = (float(toks[1]), float(toks[2]))
                    ctrl = (float(toks[3]), float(toks[4])) if code in ("112", "114", "116") else None
                    pav.node(pt, ctrl, ends_chain=code in ("113", "114", "115", "116"),
                             closed=code in ("113", "114"))
            else:
                # anything else (19 windsock, 120 linear features, 130 boundary, ATC,
                # lighting, parking, ...) ends any in-progress pavement block.
                pav.flush()
        except (ValueError, IndexError):
            data.warnings.append(f"skipped malformed line: {line[:70]}")
    pav.flush()

    # --- projection origin ---
    if datum_lat is None or datum_lon is None:
        lats, lons = [], []
        for r in raw_runways:
            lats += [r[4], r[7]]
            lons += [r[5], r[8]]
        for h in raw_helipads:
            lats.append(h[1])
            lons.append(h[2])
        for _, chains in raw_pavements:
            for nodes, _ in chains:
                for n in nodes:
                    lats.append(n.pt[0])
                    lons.append(n.pt[1])
        if lats:
            datum_lat = sum(lats) / len(lats)
            first = lons[0]
            datum_lon = first + sum((l - first + 180.0) % 360.0 - 180.0 for l in lons) / len(lons)
            datum_lon = (datum_lon + 180.0) % 360.0 - 180.0
        else:
            datum_lat, datum_lon = 0.0, 0.0

    data.ref_lat, data.ref_lon = datum_lat, datum_lon
    proj = Projector(datum_lat, datum_lon)

    for water, width, surface, id1, lat1, lon1, id2, lat2, lon2 in raw_runways:
        data.runways.append(Runway(width=width, surface=surface, end1_id=id1, end2_id=id2,
                                   end1=proj.to_xy(lat1, lon1), end2=proj.to_xy(lat2, lon2), water=water))

    for hp_name, lat, lon, length, width, surface, heading in raw_helipads:
        data.helipads.append(Helipad(name=hp_name, surface=surface, center=proj.to_xy(lat, lon),
                                     length=length, width=width, heading_deg=heading))

    # pavement lat/lon node chains -> projected, bezier-flattened, cleaned rings
    for surface, chains in raw_pavements:
        rings: List[Ring] = []
        for nodes, closed in chains:
            projected = [_Node(pt=proj.to_xy(*n.pt), ctrl=(proj.to_xy(*n.ctrl) if n.ctrl else None))
                         for n in nodes]
            if len(projected) < 2:
                continue
            pts = _clean_ring(_flatten_chain(projected, closed=closed))
            if len(pts) >= 3 and abs(_signed_area(pts)) > 1e-6:
                rings.append(Ring(points=pts))
        if rings:
            data.pavements.append(Pavement(surface=surface, rings=rings))

    return data


def parse_airport(path: str, icao: str) -> Optional[AirportData]:
    """Parse one airport. `icao` may be the header identifier, or the ICAO /
    IATA / FAA code from the 1302 metadata. Returns None if it isn't in the file."""
    ident = icao.strip()
    lines = _read_airport_lines(path, ident)
    if lines is None:
        real = resolve_airport_id(path, ident)
        if real is not None:
            lines = _read_airport_lines(path, real)
    if lines is None:
        return None
    return _parse_block(lines)


# --------------------------------------------------------------------------
# Geometry building: runways/pavements -> triangulated 3D meshes
# --------------------------------------------------------------------------

class _Triangulation(NamedTuple):
    points: List[Vec2]                  # outer ring (CCW) followed by each hole (CW)
    rings: List[List[Vec2]]             # the same rings, separately
    tris: List[Tuple[int, int, int]]    # CCW triangles, indices into `points`
    deviation: float                    # |triangulated area - polygon area| / polygon area


def _triangulate(outer: List[Vec2], holes: List[List[Vec2]]) -> _Triangulation:
    if not _HAVE_EARCUT:
        raise RuntimeError("earcut package not available - run: pip install earcut")
    rings = [_orient(outer, True)] + [_orient(h, False) for h in holes]
    coords: List[float] = []
    hole_indices: List[int] = []
    points: List[Vec2] = []
    for i, ring in enumerate(rings):
        if i > 0:
            hole_indices.append(len(points))
        for (x, y) in ring:
            coords.append(x)
            coords.append(y)
            points.append((x, y))
    flat = _earcut(coords, hole_indices if hole_indices else None, 2)

    tris: List[Tuple[int, int, int]] = []
    tri_area = 0.0
    for k in range(0, len(flat) - 2, 3):
        i0, i1, i2 = flat[k], flat[k + 1], flat[k + 2]
        a, b, c = points[i0], points[i1], points[i2]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if cross == 0:
            continue
        if cross < 0:                   # guarantee CCW (normal +Z) regardless of earcut's output order
            i1, i2 = i2, i1
            cross = -cross
        tris.append((i0, i1, i2))
        tri_area += cross / 2.0
    poly_area = abs(_signed_area(rings[0])) - sum(abs(_signed_area(h)) for h in rings[1:])
    deviation = abs(tri_area - poly_area) / poly_area if poly_area > 1e-9 else 0.0
    return _Triangulation(points, rings, tris, deviation)


@dataclass
class Mesh:
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)
    faces: List[Tuple[int, int, int]] = field(default_factory=list)   # 0-based indices into `vertices`
    last_deviation: float = 0.0     # triangulation quality of the most recently added polygon

    def _emit_cap(self, tri: _Triangulation, z: float, flip: bool) -> None:
        # Only vertices that a triangle actually uses are written, so a
        # triangulation can never leave orphaned (loose) vertices behind.
        used = sorted({i for t in tri.tris for i in t})
        base = len(self.vertices)
        remap = {}
        for n, old in enumerate(used):
            remap[old] = base + n
            self.vertices.append((tri.points[old][0], tri.points[old][1], z))
        for (a, b, c) in tri.tris:
            if flip:
                self.faces.append((remap[a], remap[c], remap[b]))
            else:
                self.faces.append((remap[a], remap[b], remap[c]))

    def add_flat_polygon(self, ring_outer: List[Vec2], holes: List[List[Vec2]], z: float, flip=False) -> None:
        tri = _triangulate(ring_outer, holes)
        self.last_deviation = tri.deviation
        self._emit_cap(tri, z, flip)

    def add_extruded_polygon(self, ring_outer: List[Vec2], holes: List[List[Vec2]],
                             z_bottom: float, z_top: float) -> None:
        """Top + bottom caps plus side walls (all facing outward), for a slab with real thickness."""
        tri = _triangulate(ring_outer, holes)
        self.last_deviation = tri.deviation
        self._emit_cap(tri, z_top, flip=False)
        self._emit_cap(tri, z_bottom, flip=True)
        # rings are CCW (outer) / CW (holes): the right-hand side of every edge is "out of the material"
        for ring in tri.rings:
            n = len(ring)
            for i in range(n):
                a, b = ring[i], ring[(i + 1) % n]
                base = len(self.vertices)
                self.vertices.append((a[0], a[1], z_bottom))
                self.vertices.append((b[0], b[1], z_bottom))
                self.vertices.append((b[0], b[1], z_top))
                self.vertices.append((a[0], a[1], z_top))
                self.faces.append((base, base + 1, base + 2))
                self.faces.append((base, base + 2, base + 3))

    def finalize(self, ndigits: int = 4) -> "Mesh":
        """Weld coincident vertices (0.1 mm), drop degenerate/duplicate faces,
        and drop any vertex no face uses."""
        key_to_new: Dict[Tuple[float, float, float], int] = {}
        welded: List[Tuple[float, float, float]] = []
        remap: List[int] = []
        for (x, y, z) in self.vertices:
            key = (round(x, ndigits) + 0.0, round(y, ndigits) + 0.0, round(z, ndigits) + 0.0)
            j = key_to_new.get(key)
            if j is None:
                j = len(welded)
                key_to_new[key] = j
                welded.append(key)
            remap.append(j)
        seen = set()
        faces = []
        for (a, b, c) in self.faces:
            a, b, c = remap[a], remap[b], remap[c]
            if a == b or b == c or a == c:
                continue
            k = tuple(sorted((a, b, c)))
            if k in seen:
                continue
            seen.add(k)
            faces.append((a, b, c))
        used = sorted({i for f in faces for i in f})
        compact = {old: new for new, old in enumerate(used)}
        self.vertices = [welded[i] for i in used]
        self.faces = [(compact[a], compact[b], compact[c]) for (a, b, c) in faces]
        return self


def _runway_rectangle(rw: Runway) -> List[Vec2]:
    (x1, y1), (x2, y2) = rw.end1, rw.end2
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    if length == 0:
        return []
    ux, uy = dx / length, dy / length      # unit vector along runway
    px, py = -uy, ux                       # perpendicular unit vector
    hw = rw.width / 2.0
    p1 = (x1 + px * hw, y1 + py * hw)
    p2 = (x2 + px * hw, y2 + py * hw)
    p3 = (x2 - px * hw, y2 - py * hw)
    p4 = (x1 - px * hw, y1 - py * hw)
    return [p1, p2, p3, p4]


def _helipad_rectangle(hp: Helipad) -> List[Vec2]:
    cx, cy = hp.center
    hl, hw = hp.length / 2.0, hp.width / 2.0
    rad = math.radians(hp.heading_deg)
    ux, uy = math.sin(rad), math.cos(rad)     # heading 0 = north = +Y
    px, py = uy, -ux
    corners_local = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
    pts = []
    for lx, ly in corners_local:
        pts.append((cx + px * lx + ux * ly, cy + py * lx + uy * ly))
    return pts


def build_meshes(data: AirportData, slab_thickness: float = 0.0,
                 log: Optional[Callable[[str], None]] = None) -> Dict[str, Mesh]:
    """Returns a dict of {group_name: Mesh}, one mesh per logical group so the
    Blender/OBJ output comes in cleanly-named, separately-selectable pieces.
    Problems with individual polygons are reported through `log` (and never
    abort the whole airport)."""
    say = log or (lambda m: None)
    groups: Dict[str, Mesh] = {}

    def unique(name: str) -> str:
        if name not in groups:
            return name
        n = 2
        while f"{name}_{n}" in groups:
            n += 1
        return f"{name}_{n}"

    def add(name: str, outer: List[Vec2], holes: List[List[Vec2]]) -> None:
        m = Mesh()
        try:
            if slab_thickness > 0:
                m.add_extruded_polygon(outer, holes, -slab_thickness, 0.0)
            else:
                m.add_flat_polygon(outer, holes, 0.0)
        except RuntimeError:
            raise                                    # missing earcut: the user must fix that
        except Exception as e:                       # skip a malformed polygon rather than aborting
            say(f"  [warn] skipped {name}: {e}")
            return
        if not m.faces:
            say(f"  [warn] {name}: no triangles produced (degenerate polygon), skipped")
            return
        if m.last_deviation > _AREA_WARN:
            say(f"  [warn] {name}: triangulated area is {m.last_deviation * 100:.1f}% off the polygon area - "
                f"the outline in apt.dat is probably self-intersecting")
        groups[unique(name)] = m

    for rw in data.runways:
        rect = _runway_rectangle(rw)
        if not rect:
            say(f"  [warn] runway {rw.end1_id}-{rw.end2_id} has zero length, skipped")
            continue
        add(f"{'WaterRunway' if rw.water else 'Runway'}_{rw.end1_id}-{rw.end2_id}", rect, [])

    for hp in data.helipads:
        add(f"Helipad_{hp.name}", _helipad_rectangle(hp), [])

    for i, pav in enumerate(data.pavements):
        if not pav.rings:
            continue
        add(f"Pavement_surf{pav.surface}_{i:03d}", pav.rings[0].points, [r.points for r in pav.rings[1:]])

    for m in groups.values():
        m.finalize()
    return groups
