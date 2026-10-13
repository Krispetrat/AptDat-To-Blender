"""
aptdat_core.py
--------------
Core engine: parses an X-Plane apt.dat file for one airport and turns everything
the file describes into 3D geometry / placed objects / plain data.

No GUI code here -- aptdat_app.py wraps this with a Tk GUI / CLI.

Supported apt.dat row codes (1050 / 1100 / 1130 / 1200 specs):
  1 / 16 / 17     airport / seaplane base / heliport header (the ONLY header codes)
  1302            metadata (datum_lat / datum_lon = projection origin, icao/iata/faa/city/...)
  100 / 101 / 102 land runway (incl. shoulders, overruns, displaced thresholds, lights,
                  markings codes) / water runway / helipad
  110             pavement (taxiway / apron) header, with holes
  120             linear feature (painted taxi lines, hold lines, light strings)
  130             airport boundary
  111..116        boundary / line nodes. 111/113 plain, 112/114 with bezier control point,
                    113/114 close a loop, 115/116 end an open string. Nodes of 110/120/130
                    chains also carry a painted LINE TYPE and a LIGHT TYPE for the segment
                    that starts at the node (this is how pavement-edge markings are stored)
  14 / 15 / 18 / 19 / 20 / 21
                  viewpoint / legacy startup location / light beacon / windsock /
                  taxiway sign / lighting object (PAPI, VASI, ...)
  50-56, 1050-1056  communication frequencies
  1000-1004, 1100, 1101, 1110
                  ATC traffic flows (kept as data)
  1200-1206       taxi route network: nodes, edges, active zones, ground-vehicle edges
  1300 / 1301     startup locations (+ metadata)
  1400 / 1401     ground-truck parking / destinations
  1500            jetways (best effort, position + raw fields)

Every row code that is not understood is counted in AirportData.ignored so nothing
disappears silently.
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

# Heights (metres above the pavement plane) so overlays never z-fight with the pavement.
LINE_Z = 0.02                # painted lines, thresholds
BOUNDARY_Z = 0.01
LIGHT_Z = 0.04
NETWORK_Z = 0.06


# --------------------------------------------------------------------------
# Data model
# --------------------------------------------------------------------------

@dataclass
class Ring:
    points: List[Vec2]          # local XY meters, bezier-flattened and cleaned, closed (no dup last point)


@dataclass
class LineRun:
    """A stretch of a painted line and/or light string that has one style."""
    line_type: int              # painted line type code (0 = none)
    light_type: int             # light type code (0 = none)
    points: List[Vec2]          # flattened polyline, local XY meters
    closed: bool = False        # True when the run is a complete loop (no duplicated end point)


@dataclass
class Pavement:
    surface: int
    rings: List[Ring]           # rings[0] = outer boundary, rings[1:] = holes
    smoothness: float = 0.25
    texture_heading: float = 0.0
    description: str = ""
    edge_runs: List[LineRun] = field(default_factory=list)   # painted lines / lights on the pavement edge


@dataclass
class LinearFeature:
    """120 row: painted line / light string (taxi centre lines, hold lines, ...)."""
    name: str
    runs: List[LineRun]


@dataclass
class Boundary:
    """130 row: airport boundary (outer ring + optional holes)."""
    name: str
    rings: List[Ring]


@dataclass
class Runway:
    width: float
    surface: int
    end1_id: str
    end2_id: str
    end1: Vec2                  # local XY meters
    end2: Vec2
    water: bool = False
    shoulder: int = 0           # surface code + 100 * shoulder width in metres
    smoothness: float = 0.25
    center_lights: int = 0
    edge_lights: int = 0
    auto_signs: int = 0
    displaced1: float = 0.0     # displaced threshold length, end 1 / end 2 (m)
    displaced2: float = 0.0
    overrun1: float = 0.0       # overrun / blast-pad length, end 1 / end 2 (m)
    overrun2: float = 0.0
    props: Dict[str, object] = field(default_factory=dict)   # everything else (markings, approach lights, ...)


@dataclass
class Helipad:
    name: str
    surface: int
    center: Vec2
    length: float
    width: float
    heading_deg: float
    props: Dict[str, object] = field(default_factory=dict)


@dataclass
class Marker:
    """A single placed object: sign, windsock, beacon, startup location, ..."""
    kind: str
    name: str
    pos: Vec2
    heading_deg: Optional[float] = None
    height_m: float = 0.0
    props: Dict[str, object] = field(default_factory=dict)


@dataclass
class TaxiNode:
    id: int
    pos: Vec2
    usage: str                  # init | end | both | junc
    name: str = ""


@dataclass
class TaxiEdge:
    a: int
    b: int
    oneway: bool
    kind: str                   # runway | taxiway_A..F | vehicle
    name: str = ""
    ground_vehicle: bool = False
    active_zones: List[Tuple[str, List[str]]] = field(default_factory=list)   # (departure|arrival|ils, runway ends)
    controls: List[List[str]] = field(default_factory=list)                   # raw 1205 rows


@dataclass
class Frequency:
    kind: str                   # ATIS / unicom / clearance / ground / tower / approach / departure
    khz: int
    name: str = ""


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
    linear_features: List[LinearFeature] = field(default_factory=list)
    boundaries: List[Boundary] = field(default_factory=list)
    markers: List[Marker] = field(default_factory=list)
    taxi_nodes: List[TaxiNode] = field(default_factory=list)
    taxi_edges: List[TaxiEdge] = field(default_factory=list)
    frequencies: List[Frequency] = field(default_factory=list)
    flows: List[Dict[str, object]] = field(default_factory=list)       # raw ATC flow rules, in file order
    metadata: Dict[str, str] = field(default_factory=dict)             # 1302 rows
    projection: Dict[str, float] = field(default_factory=dict)         # to rebuild lat/lon from local metres
    ignored: Dict[str, int] = field(default_factory=dict)              # row codes that were not understood
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
# Bezier flattening for boundary chains (pavement / linear feature / airport boundary)
# --------------------------------------------------------------------------

Style = Tuple[int, int]         # (painted line type, light type) of the segment starting at a node


@dataclass
class _Node:
    pt: Vec2
    ctrl: Optional[Vec2]        # control point as stored in apt.dat (absolute), or None
    style: Style = (0, 0)       # line type / light type of the segment that starts here


@dataclass
class _Corner:
    pt: Vec2
    h_in: Optional[Vec2]        # incoming tangent handle (absolute), or None for a straight arrival
    h_out: Optional[Vec2]       # outgoing tangent handle (absolute), or None for a straight departure
    style: Style = (0, 0)       # style of the segment leaving this corner


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
    (and line/light style) from its LAST node.
    """
    corners: List[_Corner] = []
    for n in nodes:
        h_out = n.ctrl
        h_in = (2 * n.pt[0] - n.ctrl[0], 2 * n.pt[1] - n.ctrl[1]) if n.ctrl else None
        if corners and _near(corners[-1].pt, n.pt):
            corners[-1].h_out = h_out
            corners[-1].style = n.style
        else:
            corners.append(_Corner(n.pt, h_in, h_out, n.style))
    if closed and len(corners) > 1 and _near(corners[0].pt, corners[-1].pt):
        # the run wraps around the end of the chain: it starts in the last
        # corner and finishes in the first one
        last = corners.pop()
        corners[0].h_in = last.h_in
    return corners


def _flatten_segments(nodes: List[_Node], closed: bool) -> List[Tuple[Style, List[Vec2]]]:
    """Per-segment flattening: [(style, [start, ..., end]), ...] following the apt.dat
    curve convention:
      outgoing tangent handle at a node = its control point as-is
      incoming tangent handle at a node = mirror of its control point through the node
    An edge whose two ends have no handle is a straight line."""
    corners = _merge_coincident(nodes, closed)
    if len(corners) < 2:
        return []
    count = len(corners)
    segs: List[Tuple[Style, List[Vec2]]] = []
    for i in range(count if closed else count - 1):
        a = corners[i]
        b = corners[(i + 1) % count]
        if a.h_out is None and b.h_in is None:
            pts = [a.pt, b.pt]
        else:
            h1 = a.h_out if a.h_out is not None else a.pt
            h2 = b.h_in if b.h_in is not None else b.pt
            pts = [a.pt] + _cubic_bezier(a.pt, h1, h2, b.pt, BEZIER_SUBDIVISIONS)
        segs.append((a.style, pts))
    return segs


def _flatten_chain(nodes: List[_Node], closed: bool) -> List[Vec2]:
    """Turn a list of nodes (each optionally with a bezier control point) into a flat polyline."""
    segs = _flatten_segments(nodes, closed)
    if not segs:
        return [c.pt for c in _merge_coincident(nodes, closed)]
    out: List[Vec2] = [segs[0][1][0]]
    for _, pts in segs:
        out.extend(pts[1:])
    if closed and len(out) > 1 and _near(out[0], out[-1]):
        out.pop()
    return out


def _dedupe(points: List[Vec2]) -> List[Vec2]:
    out: List[Vec2] = []
    for p in points:
        if not out or not _near(p, out[-1]):
            out.append(p)
    return out


def _segments_to_runs(segs: List[Tuple[Style, List[Vec2]]], closed: bool) -> List[LineRun]:
    """Join consecutive segments that share a style into runs. Segments with style (0, 0)
    (no painted line, no lights) break the run. A closed chain that carries one style all the
    way round becomes a single closed run; a run that straddles the chain start is re-joined."""
    if not segs:
        return []
    runs: List[LineRun] = []
    cur: Optional[LineRun] = None
    for (lt, li), pts in segs:
        if lt == 0 and li == 0:
            cur = None
            continue
        if cur is not None and cur.line_type == lt and cur.light_type == li:
            cur.points.extend(pts[1:])
        else:
            cur = LineRun(lt, li, list(pts))
            runs.append(cur)

    first_style, last_style = segs[0][0], segs[-1][0]
    if closed and runs:
        if len(runs) == 1 and all(s == first_style for s, _ in segs) and first_style != (0, 0):
            runs[0].closed = True
        elif (len(runs) >= 2 and first_style == last_style and first_style != (0, 0)):
            last = runs.pop()
            runs[0].points = last.points + runs[0].points[1:]

    result: List[LineRun] = []
    for r in runs:
        pts = _dedupe(r.points)
        if r.closed and len(pts) > 1 and _near(pts[0], pts[-1]):
            pts.pop()
        if len(pts) >= (3 if r.closed else 2):
            r.points = pts
            result.append(r)
    return result


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


class _ChainBuilder:
    """Collects the node chains of one 110 (pavement), 120 (linear feature) or
    130 (boundary) block. For pavements/boundaries chain 0 is the outer ring and the
    following chains are holes; for linear features each chain is one line."""

    def __init__(self, sink: list):
        self.sink = sink
        self.reset()

    def reset(self):
        self.active = False
        self.code = ""
        self.header: List[str] = []
        self.chains: List[Tuple[List[_Node], bool]] = []
        self.cur: List[_Node] = []

    def start(self, code: str, header: List[str]):
        self.flush()
        self.active = True
        self.code = code
        self.header = header

    def node(self, node: _Node, ends_chain: bool, closed: bool):
        self.cur.append(node)
        if ends_chain:
            self.chains.append((self.cur, closed))
            self.cur = []

    def flush(self):
        if self.active:
            if self.cur:                       # chain never terminated by 113/114/115/116: close it anyway
                self.chains.append((self.cur, True))
            if self.chains:
                self.sink.append((self.code, self.header, self.chains))
        self.reset()


_NODE_CODES = ("111", "112", "113", "114", "115", "116")
_BEZIER_CODES = ("112", "114", "116")
_END_CODES = ("113", "114", "115", "116")
_CLOSE_CODES = ("113", "114")

_FREQ_KINDS = {0: "ATIS", 1: "unicom", 2: "clearance", 3: "ground", 4: "tower", 5: "approach", 6: "departure"}
_FLOW_CODES = ("1001", "1002", "1003", "1004", "1100", "1101", "1110")


def _f(toks: List[str], i: int, default: float = 0.0) -> float:
    return _to_float(toks[i], default) if i < len(toks) else default


def _i(toks: List[str], i: int, default: int = 0) -> int:
    try:
        return int(float(toks[i])) if i < len(toks) else default
    except ValueError:
        return default


def _node_style(toks: List[str], code: str) -> Style:
    """Line type / light type columns: after lat lon (111/113) or after lat lon ctrl_lat ctrl_lon (112/114).
    115/116 end a string and carry no style."""
    if code in ("115", "116"):
        return (0, 0)
    i = 5 if code in ("112", "114") else 3
    return (_i(toks, i), _i(toks, i + 1))


def _bearing(p: Vec2, q: Vec2) -> float:
    return math.degrees(math.atan2(q[0] - p[0], q[1] - p[1])) % 360.0


def _parse_block(lines: List[str]) -> AirportData:
    header = lines[0].split()
    ident = header[4]
    data = AirportData(icao=ident, name=" ".join(header[5:]) or ident,
                       elevation_ft=_to_float(header[1]) if len(header) > 1 else 0.0)

    datum_lat = datum_lon = None
    raw_runways: List[dict] = []
    raw_helipads: List[dict] = []
    raw_chains: list = []       # (code, header tokens, [(nodes_latlon, closed), ...])
    raw_markers: List[tuple] = []   # (kind, name, lat, lon, heading, height_m, props)
    raw_taxi_nodes: List[tuple] = []  # (id, lat, lon, usage, name)
    raw_taxi_edges: List[TaxiEdge] = []
    chain = _ChainBuilder(raw_chains)
    last_edge: Optional[TaxiEdge] = None
    last_startup: Optional[tuple] = None
    cur_flow: Optional[Dict[str, object]] = None

    def ignore(code: str):
        data.ignored[code] = data.ignored.get(code, 0) + 1

    for line in lines[1:]:
        toks = line.split()
        code = toks[0]
        if code.startswith("#"):
            continue
        try:
            if code in _NODE_CODES:
                if chain.active:
                    pt = (float(toks[1]), float(toks[2]))
                    ctrl = (float(toks[3]), float(toks[4])) if code in _BEZIER_CODES else None
                    chain.node(_Node(pt, ctrl, _node_style(toks, code)),
                               ends_chain=code in _END_CODES, closed=code in _CLOSE_CODES)
                continue

            # anything that is not a boundary node ends any in-progress chain block
            if code in ("110", "120", "130"):
                chain.start(code, toks[1:])
                continue
            chain.flush()

            if code == "1302":
                if len(toks) >= 3:
                    data.metadata[toks[1]] = line.split(None, 2)[2].strip()
                    if toks[1] == "datum_lat":
                        datum_lat = float(toks[2])
                    elif toks[1] == "datum_lon":
                        datum_lon = float(toks[2])
            elif code == "100":
                def end(o):
                    return dict(id=toks[o], lat=float(toks[o + 1]), lon=float(toks[o + 2]),
                                displaced=_f(toks, o + 3), overrun=_f(toks, o + 4),
                                markings=_i(toks, o + 5), approach=_i(toks, o + 6),
                                tdz=_i(toks, o + 7), reil=_i(toks, o + 8))
                raw_runways.append(dict(water=False, width=float(toks[1]), surface=_i(toks, 2),
                                        shoulder=_i(toks, 3), smoothness=_f(toks, 4, 0.25),
                                        center=_i(toks, 5), edge=_i(toks, 6), signs=_i(toks, 7),
                                        e1=end(8), e2=end(17)))
            elif code == "101":
                # 101 width buoys id lat lon id lat lon
                raw_runways.append(dict(water=True, width=float(toks[1]), surface=-1, buoys=_i(toks, 2),
                                        e1=dict(id=toks[3], lat=float(toks[4]), lon=float(toks[5])),
                                        e2=dict(id=toks[6], lat=float(toks[7]), lon=float(toks[8]))))
            elif code == "102":
                # 102 designator lat lon heading length width surface markings shoulder smoothness lights
                raw_helipads.append(dict(name=toks[1], lat=float(toks[2]), lon=float(toks[3]),
                                         heading=float(toks[4]), length=float(toks[5]), width=float(toks[6]),
                                         surface=_i(toks, 7), markings=_i(toks, 8), shoulder=_i(toks, 9),
                                         smoothness=_f(toks, 10, 0.25), edge_lights=_i(toks, 11)))
            elif code == "14":
                # 14 lat lon height_ft 0 name
                raw_markers.append(("viewpoint", " ".join(toks[5:]) or "Viewpoint", float(toks[1]), float(toks[2]),
                                    None, _f(toks, 3) * 0.3048, {"height_ft": _f(toks, 3)}))
            elif code == "15":
                # legacy startup location: 15 lat lon heading name
                raw_markers.append(("startup", " ".join(toks[4:]) or "Startup", float(toks[1]), float(toks[2]),
                                    float(toks[3]), 0.0, {"legacy": True}))
                last_startup = None
            elif code == "18":
                raw_markers.append(("beacon", " ".join(toks[4:]) or "Beacon", float(toks[1]), float(toks[2]),
                                    None, 0.0, {"beacon_type": _i(toks, 3)}))
            elif code == "19":
                raw_markers.append(("windsock", " ".join(toks[4:]) or "Windsock", float(toks[1]), float(toks[2]),
                                    None, 0.0, {"lit": _i(toks, 3)}))
            elif code == "20":
                # 20 lat lon heading 0 size text
                text = " ".join(toks[6:])
                raw_markers.append(("sign", text or "Sign", float(toks[1]), float(toks[2]),
                                    float(toks[3]), 0.0, {"size": _i(toks, 5), "text": text}))
            elif code == "21":
                # 21 lat lon type heading glideslope name
                raw_markers.append(("lightobj", " ".join(toks[6:]) or f"Light{_i(toks, 3)}", float(toks[1]),
                                    float(toks[2]), float(toks[4]), 0.0,
                                    {"light_type": _i(toks, 3), "glideslope_deg": _f(toks, 5, 3.0)}))
            elif code in ("50", "51", "52", "53", "54", "55", "56"):
                data.frequencies.append(Frequency(_FREQ_KINDS[int(code) - 50], _i(toks, 1) * 10, " ".join(toks[2:])))
            elif code in ("1050", "1051", "1052", "1053", "1054", "1055", "1056"):
                data.frequencies.append(Frequency(_FREQ_KINDS[int(code) - 1050], _i(toks, 1), " ".join(toks[2:])))
            elif code == "1000":
                cur_flow = {"name": " ".join(toks[1:]), "rules": []}
                data.flows.append(cur_flow)
            elif code in _FLOW_CODES:
                if cur_flow is None:
                    cur_flow = {"name": "", "rules": []}
                    data.flows.append(cur_flow)
                cur_flow["rules"].append({"code": int(code), "args": toks[1:]})
            elif code == "1200":
                pass                                  # header: the nodes/edges that follow are the network
            elif code == "1201":
                # 1201 lat lon usage id name
                raw_taxi_nodes.append((int(toks[4]), float(toks[1]), float(toks[2]), toks[3], " ".join(toks[5:])))
            elif code == "1202":
                # 1202 node1 node2 direction type name
                last_edge = TaxiEdge(int(toks[1]), int(toks[2]), toks[3].lower() == "oneway",
                                     toks[4] if len(toks) > 4 else "taxiway", " ".join(toks[5:]))
                raw_taxi_edges.append(last_edge)
            elif code == "1206":
                # 1206 node1 node2 direction name   (ground vehicles only)
                last_edge = TaxiEdge(int(toks[1]), int(toks[2]), toks[3].lower() == "oneway" if len(toks) > 3 else False,
                                     "vehicle", " ".join(toks[4:]), ground_vehicle=True)
                raw_taxi_edges.append(last_edge)
            elif code == "1204":
                if last_edge is not None and len(toks) >= 3:
                    last_edge.active_zones.append((toks[1], toks[2].split(",")))
            elif code == "1205":
                if last_edge is not None:
                    last_edge.controls.append(toks[1:])
            elif code == "1300":
                # 1300 lat lon heading type categories name
                props = {"location_type": toks[4] if len(toks) > 4 else "",
                         "categories": toks[5].split("|") if len(toks) > 5 else []}
                raw_markers.append(("startup", " ".join(toks[6:]) or "Startup", float(toks[1]), float(toks[2]),
                                    float(toks[3]), 0.0, props))
                last_startup = raw_markers[-1]
            elif code == "1301":
                # 1301 icao_width ops_type airlines...
                if last_startup is not None:
                    last_startup[6].update({"width_code": toks[1] if len(toks) > 1 else "",
                                            "ops_type": toks[2] if len(toks) > 2 else "",
                                            "airlines": toks[3:]})
            elif code in ("1400", "1401"):
                # 1400 lat lon heading truck_type number name... / 1401 lat lon heading truck_type name...
                kind = "truck_parking" if code == "1400" else "truck_dest"
                ttype = toks[4] if len(toks) > 4 else ""
                props = {"truck_type": ttype}
                rest = toks[5:]
                if code == "1400" and rest and rest[0].lstrip("-").isdigit():
                    props["count"] = int(rest[0])
                    rest = rest[1:]
                raw_markers.append((kind, " ".join(rest) or ttype or kind, float(toks[1]), float(toks[2]),
                                    float(toks[3]), 0.0, props))
            elif code == "1500":
                # jetway: position + heading; remaining columns kept raw (layout varies by spec version)
                raw_markers.append(("jetway", "Jetway", float(toks[1]), float(toks[2]),
                                    _f(toks, 3), 0.0, {"raw": toks[4:]}))
            elif code in ("99", "I", "A"):
                pass
            else:
                ignore(code)
        except (ValueError, IndexError):
            data.warnings.append(f"skipped malformed line: {line[:70]}")
    chain.flush()

    # --- projection origin ---
    if datum_lat is None or datum_lon is None:
        lats, lons = [], []
        for r in raw_runways:
            lats += [r["e1"]["lat"], r["e2"]["lat"]]
            lons += [r["e1"]["lon"], r["e2"]["lon"]]
        for h in raw_helipads:
            lats.append(h["lat"])
            lons.append(h["lon"])
        for m in raw_markers:
            lats.append(m[2])
            lons.append(m[3])
        for _, _, chains in raw_chains:
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
    data.projection = {"ref_lat": datum_lat, "ref_lon": datum_lon,
                       "m_per_deg_lat": proj._m_per_deg_lat, "m_per_deg_lon": proj._m_per_deg_lon}

    for r in raw_runways:
        e1, e2 = r["e1"], r["e2"]
        rw = Runway(width=r["width"], surface=r["surface"], end1_id=e1["id"], end2_id=e2["id"],
                    end1=proj.to_xy(e1["lat"], e1["lon"]), end2=proj.to_xy(e2["lat"], e2["lon"]),
                    water=r["water"])
        if r["water"]:
            rw.props = {"perimeter_buoys": r.get("buoys", 0)}
        else:
            rw.shoulder, rw.smoothness = r["shoulder"], r["smoothness"]
            rw.center_lights, rw.edge_lights, rw.auto_signs = r["center"], r["edge"], r["signs"]
            rw.displaced1, rw.displaced2 = e1["displaced"], e2["displaced"]
            rw.overrun1, rw.overrun2 = e1["overrun"], e2["overrun"]
            rw.props = {
                "end1_markings": e1["markings"], "end2_markings": e2["markings"],
                "end1_approach_lights": e1["approach"], "end2_approach_lights": e2["approach"],
                "end1_tdz_lights": e1["tdz"], "end2_tdz_lights": e2["tdz"],
                "end1_reil": e1["reil"], "end2_reil": e2["reil"],
            }
        data.runways.append(rw)

    for h in raw_helipads:
        data.helipads.append(Helipad(name=h["name"], surface=h["surface"], center=proj.to_xy(h["lat"], h["lon"]),
                                     length=h["length"], width=h["width"], heading_deg=h["heading"],
                                     props={"markings": h["markings"], "shoulder": h["shoulder"],
                                            "smoothness": h["smoothness"], "edge_lights": h["edge_lights"]}))

    def project_nodes(nodes: List[_Node]) -> List[_Node]:
        return [_Node(pt=proj.to_xy(*n.pt), ctrl=(proj.to_xy(*n.ctrl) if n.ctrl else None), style=n.style)
                for n in nodes]

    def rings_of(chains) -> Tuple[List[Ring], List[LineRun]]:
        rings: List[Ring] = []
        runs: List[LineRun] = []
        for nodes, closed in chains:
            projected = project_nodes(nodes)
            if len(projected) < 2:
                continue
            runs.extend(_segments_to_runs(_flatten_segments(projected, closed), closed))
            pts = _clean_ring(_flatten_chain(projected, closed=closed))
            if len(pts) >= 3 and abs(_signed_area(pts)) > 1e-6:
                rings.append(Ring(points=pts))
        return rings, runs

    for code, hdr, chains in raw_chains:
        if code == "110":
            rings, runs = rings_of(chains)
            if rings:
                data.pavements.append(Pavement(
                    surface=_i(hdr, 0), rings=rings, smoothness=_f(hdr, 1, 0.25),
                    texture_heading=_f(hdr, 2), description=" ".join(hdr[3:]), edge_runs=runs))
        elif code == "120":
            line_runs: List[LineRun] = []
            for nodes, closed in chains:
                projected = project_nodes(nodes)
                if len(projected) >= 2:
                    line_runs.extend(_segments_to_runs(_flatten_segments(projected, closed), closed))
            if line_runs:
                data.linear_features.append(LinearFeature(name=" ".join(hdr), runs=line_runs))
        elif code == "130":
            rings, _ = rings_of(chains)
            if rings:
                data.boundaries.append(Boundary(name=" ".join(hdr), rings=rings))

    for kind, name, lat, lon, heading, height_m, props in raw_markers:
        data.markers.append(Marker(kind=kind, name=name, pos=proj.to_xy(lat, lon), heading_deg=heading,
                                   height_m=height_m, props=props))

    for nid, lat, lon, usage, name in raw_taxi_nodes:
        data.taxi_nodes.append(TaxiNode(nid, proj.to_xy(lat, lon), usage, name))
    data.taxi_edges = raw_taxi_edges

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
    edges: List[Tuple[int, int]] = field(default_factory=list)        # wire edges (boundary, taxi network)
    category: str = "Layout"        # becomes the sub-collection in the Blender script
    color: Optional[Tuple[float, float, float]] = None
    props: Dict[str, object] = field(default_factory=dict)            # custom properties on the Blender object

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

    # ---- overlay helpers: painted lines, light dots, wire edges -------------------------

    def add_ribbon(self, pts: List[Vec2], width: float, z: float, closed: bool = False) -> None:
        """Flat strip of constant width along a polyline (faces up). Corners are mitred
        (mitre length capped so tight turns do not spike)."""
        pts = _dedupe(pts)
        if closed and len(pts) > 1 and _near(pts[0], pts[-1]):
            pts = pts[:-1]
        n = len(pts)
        if n < 2 or (closed and n < 3):
            return
        hw = width / 2.0
        count = n if closed else n - 1
        seg_n: List[Vec2] = []                      # left-hand unit normal of every segment
        for i in range(count):
            a, b = pts[i], pts[(i + 1) % n]
            dx, dy = b[0] - a[0], b[1] - a[1]
            ln = math.hypot(dx, dy) or 1.0
            seg_n.append((-dy / ln, dx / ln))
        base = len(self.vertices)
        for i in range(n):
            if closed:
                n0, n1 = seg_n[i - 1], seg_n[i]
            elif i == 0:
                n0 = n1 = seg_n[0]
            elif i == n - 1:
                n0 = n1 = seg_n[-1]
            else:
                n0, n1 = seg_n[i - 1], seg_n[i]
            mx, my = n0[0] + n1[0], n0[1] + n1[1]
            ml = math.hypot(mx, my)
            if ml < 1e-9:                           # a full U-turn: fall back to one segment's normal
                mx, my, ml = n1[0], n1[1], 1.0
            mx, my = mx / ml, my / ml
            scale = min(1.0 / max(mx * n1[0] + my * n1[1], 1e-6), 3.0)
            ox, oy = mx * hw * scale, my * hw * scale
            self.vertices.append((pts[i][0] + ox, pts[i][1] + oy, z))     # left
            self.vertices.append((pts[i][0] - ox, pts[i][1] - oy, z))     # right
        for i in range(count):
            j = (i + 1) % n
            l0, r0, l1, r1 = base + 2 * i, base + 2 * i + 1, base + 2 * j, base + 2 * j + 1
            self.faces.append((l0, r0, r1))
            self.faces.append((l0, r1, l1))

    def add_square(self, c: Vec2, size: float, z: float) -> None:
        h = size / 2.0
        base = len(self.vertices)
        self.vertices += [(c[0] - h, c[1] - h, z), (c[0] + h, c[1] - h, z),
                          (c[0] + h, c[1] + h, z), (c[0] - h, c[1] + h, z)]
        self.faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]

    def add_wire(self, pts: List[Vec2], z: float, closed: bool = False) -> None:
        pts = _dedupe(pts)
        if closed and len(pts) > 1 and _near(pts[0], pts[-1]):
            pts = pts[:-1]
        if len(pts) < 2:
            return
        base = len(self.vertices)
        self.vertices += [(p[0], p[1], z) for p in pts]
        for i in range(len(pts) - 1):
            self.edges.append((base + i, base + i + 1))
        if closed and len(pts) > 2:
            self.edges.append((base + len(pts) - 1, base))

    def add_edge(self, p: Tuple[float, float, float], q: Tuple[float, float, float]) -> None:
        base = len(self.vertices)
        self.vertices += [p, q]
        self.edges.append((base, base + 1))

    def add_pyramid(self, c: Vec2, width: float, length: float, height: float, heading_deg: float = 0.0) -> None:
        """Small marker solid; the long axis points along `heading_deg` (clockwise from north)."""
        th = math.radians(heading_deg)
        s, co = math.sin(th), math.cos(th)
        hw, hl = width / 2.0, length / 2.0
        corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]
        base = len(self.vertices)
        for lx, ly in corners:
            self.vertices.append((c[0] + lx * co + ly * s, c[1] - lx * s + ly * co, 0.0))
        self.vertices.append((c[0], c[1], height))
        a = base + 4
        self.faces += [(base, base + 1, a), (base + 1, base + 2, a), (base + 2, base + 3, a), (base + 3, base, a),
                       (base, base + 3, base + 2), (base, base + 2, base + 1)]

    def finalize(self, ndigits: int = 4) -> "Mesh":
        """Weld coincident vertices (0.1 mm), drop degenerate/duplicate faces and edges,
        and drop any vertex no face or edge uses."""
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
        seen_e = set()
        edges = []
        for (a, b) in self.edges:
            a, b = remap[a], remap[b]
            if a == b:
                continue
            k = (a, b) if a < b else (b, a)
            if k in seen_e:
                continue
            seen_e.add(k)
            edges.append((a, b))
        used = sorted({i for f in faces for i in f} | {i for e in edges for i in e})
        compact = {old: new for new, old in enumerate(used)}
        self.vertices = [welded[i] for i in used]
        self.faces = [(compact[a], compact[b], compact[c]) for (a, b, c) in faces]
        self.edges = [(compact[a], compact[b]) for (a, b) in edges]
        return self


# --------------------------------------------------------------------------
# Styles: names / colours / widths. Only codes that are certain are named; any other
# code still exports (as "t<code>") with a neutral colour and keeps its raw number.
# --------------------------------------------------------------------------

_YELLOW = (0.95, 0.80, 0.10)
_WHITE = (0.95, 0.95, 0.95)
_GREEN = (0.10, 0.80, 0.20)
_BLUE = (0.10, 0.30, 1.00)
_NEUTRAL = (0.70, 0.70, 0.70)

# line type -> (slug, colour, ribbon width m, dashed)
_LINE_STYLES: Dict[int, Tuple[str, Tuple[float, float, float], float, bool]] = {
    1: ("solid_yellow", _YELLOW, 0.20, False),
    2: ("broken_yellow", _YELLOW, 0.20, True),
    3: ("double_solid_yellow", _YELLOW, 0.45, False),
    4: ("runway_hold", _YELLOW, 0.90, False),
    5: ("other_hold", _YELLOW, 0.50, False),
    6: ("ils_hold", _YELLOW, 0.90, False),
}
for _base, _style in list(_LINE_STYLES.items()):
    # 51-56 are the same six styles drawn with a black border
    _LINE_STYLES[50 + _base] = (_style[0] + "_black_border",) + _style[1:]

_LIGHT_STYLES: Dict[int, Tuple[str, Tuple[float, float, float]]] = {
    101: ("taxi_centerline_green", _GREEN),
    102: ("taxi_edge_blue", _BLUE),
}

_DASH, _GAP = 3.0, 3.0

_SURFACES = {1: "asphalt", 2: "concrete", 3: "turf", 4: "dirt", 5: "gravel",
             12: "dry_lakebed", 13: "water", 14: "snow_ice", 15: "transparent"}
_SURFACE_COLORS = {1: (0.16, 0.16, 0.18), 2: (0.62, 0.62, 0.60), 3: (0.25, 0.45, 0.20), 4: (0.40, 0.30, 0.20),
                   5: (0.50, 0.47, 0.42), 12: (0.70, 0.60, 0.45), 13: (0.10, 0.30, 0.60), 14: (0.90, 0.92, 0.95)}
_DEFAULT_SURFACE_COLOR = (0.35, 0.35, 0.37)
_SHOULDER_DEFAULT_WIDTH = 3.0     # metres, used when the shoulder code carries a surface but no width


def surface_name(code: int) -> str:
    return _SURFACES.get(code, f"surface{code}")


def _surface_color(code: int):
    return _SURFACE_COLORS.get(code, _DEFAULT_SURFACE_COLOR)


def line_style_name(code: int) -> str:
    return _LINE_STYLES[code][0] if code in _LINE_STYLES else f"t{code}"


def light_style_name(code: int) -> str:
    return _LIGHT_STYLES[code][0] if code in _LIGHT_STYLES else f"t{code}"


def _sample_polyline(pts: List[Vec2], spacing: float, closed: bool = False) -> List[Vec2]:
    """Points every `spacing` metres along a polyline, starting at its first point."""
    path = list(pts) + ([pts[0]] if closed and pts else [])
    out: List[Vec2] = []
    next_at = 0.0
    travelled = 0.0
    for a, b in zip(path, path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            continue
        end = travelled + seg
        while (next_at < end - 1e-9) if closed else (next_at <= end + 1e-9):
            t = min(max((next_at - travelled) / seg, 0.0), 1.0)
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            next_at += spacing
        travelled = end
    return out


def _split_dashes(pts: List[Vec2], dash: float, gap: float, closed: bool = False) -> List[List[Vec2]]:
    """Cut a polyline into dash segments (for broken lines)."""
    path = list(pts) + ([pts[0]] if closed and pts else [])
    dashes: List[List[Vec2]] = []
    cur: List[Vec2] = []
    on = True
    left = dash
    for a, b in zip(path, path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            continue
        pos = 0.0
        if on and not cur:
            cur = [a]
        while seg - pos > left:
            pos += left
            t = pos / seg
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if on:
                cur.append(p)
                dashes.append(cur)
                cur = []
            else:
                cur = [p]
            on = not on
            left = dash if on else gap
        left -= seg - pos
        if on:
            cur.append(b)
    if on and len(cur) >= 2:
        dashes.append(cur)
    return dashes


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


@dataclass
class BuildOptions:
    slab_thickness: float = 0.0   # metres; 0 = flat plane
    lines: bool = True            # painted lines: taxi lines, hold lines, pavement-edge markings
    lights: bool = True           # light strings (centre line / edge lights ...)
    boundary: bool = True         # airport boundary (130)
    network: bool = True          # taxi route network + ground-vehicle routes
    runway_extras: bool = True    # shoulders, overruns / blast pads, displaced-threshold bars
    markers: bool = True          # signs, windsocks, beacon, startups, PAPI/VASI, trucks, viewpoint, ...
    light_spacing: float = 15.0   # metres between light dots on a light string


MARKER_LABELS = {
    "viewpoint": "Viewpoint", "startup": "Startup locations", "beacon": "Light beacon",
    "windsock": "Windsocks", "sign": "Signs", "lightobj": "Lighting objects (PAPI-VASI)",
    "truck_parking": "Truck parking", "truck_dest": "Truck destinations", "jetway": "Jetways",
    "runway_end": "Runway ends", "taxinode": "Taxi nodes",
}


def all_markers(data: AirportData, options: Optional[BuildOptions] = None) -> List[Marker]:
    """Every placed object: the parsed markers plus runway ends (with their lights/markings
    codes) and the taxi nodes that mark a start/end (init / end / both)."""
    opt = options or BuildOptions()
    if not opt.markers:
        return []
    out = list(data.markers)
    for rw in data.runways:
        if rw.water:
            continue
        hdg = _bearing(rw.end1, rw.end2)
        out.append(Marker("runway_end", rw.end1_id, rw.end1, hdg, 0.0,
                          {"surface": rw.surface, "displaced_threshold_m": rw.displaced1,
                           "overrun_m": rw.overrun1, "markings": rw.props.get("end1_markings", 0),
                           "approach_lights": rw.props.get("end1_approach_lights", 0),
                           "tdz_lights": rw.props.get("end1_tdz_lights", 0), "reil": rw.props.get("end1_reil", 0)}))
        out.append(Marker("runway_end", rw.end2_id, rw.end2, (hdg + 180.0) % 360.0, 0.0,
                          {"surface": rw.surface, "displaced_threshold_m": rw.displaced2,
                           "overrun_m": rw.overrun2, "markings": rw.props.get("end2_markings", 0),
                           "approach_lights": rw.props.get("end2_approach_lights", 0),
                           "tdz_lights": rw.props.get("end2_tdz_lights", 0), "reil": rw.props.get("end2_reil", 0)}))
    for n in data.taxi_nodes:
        if n.usage.lower() != "junc":
            out.append(Marker("taxinode", n.name or f"node{n.id}", n.pos, None, 0.0,
                              {"node_id": n.id, "usage": n.usage}))
    return out


def build_meshes(data: AirportData, slab_thickness: float = 0.0,
                 log: Optional[Callable[[str], None]] = None,
                 options: Optional[BuildOptions] = None) -> Dict[str, Mesh]:
    """Returns a dict of {group_name: Mesh}, one mesh per logical group so the
    Blender/OBJ output comes in cleanly-named, separately-selectable pieces.
    Problems with individual polygons are reported through `log` (and never
    abort the whole airport). Painted lines / light strings are merged per style
    (one mesh per line type) so a big airport stays a manageable number of objects."""
    opt = options if options is not None else BuildOptions(slab_thickness=slab_thickness)
    slab = opt.slab_thickness
    say = log or (lambda m: None)
    groups: Dict[str, Mesh] = {}

    def unique(name: str) -> str:
        if name not in groups:
            return name
        n = 2
        while f"{name}_{n}" in groups:
            n += 1
        return f"{name}_{n}"

    def add(name: str, outer: List[Vec2], holes: List[List[Vec2]], category: str, color, props=None) -> None:
        m = Mesh(category=category, color=color, props=dict(props or {}))
        try:
            if slab > 0:
                m.add_extruded_polygon(outer, holes, -slab, 0.0)
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

    # ---- runways -------------------------------------------------------------------
    for rw in data.runways:
        rect = _runway_rectangle(rw)
        if not rect:
            say(f"  [warn] runway {rw.end1_id}-{rw.end2_id} has zero length, skipped")
            continue
        tag = f"{rw.end1_id}-{rw.end2_id}"
        props = {"width_m": rw.width, "end1": rw.end1_id, "end2": rw.end2_id}
        if rw.water:
            props.update(rw.props)
            add(f"WaterRunway_{tag}", rect, [], "Runways", _surface_color(13), props)
            continue
        props.update({"surface": rw.surface, "surface_name": surface_name(rw.surface),
                      "shoulder_code": rw.shoulder, "smoothness": rw.smoothness,
                      "centerline_lights": rw.center_lights, "edge_lights": rw.edge_lights,
                      "auto_distance_signs": rw.auto_signs,
                      "displaced_threshold_1_m": rw.displaced1, "displaced_threshold_2_m": rw.displaced2,
                      "overrun_1_m": rw.overrun1, "overrun_2_m": rw.overrun2})
        props.update(rw.props)
        add(f"Runway_{tag}", rect, [], "Runways", _surface_color(rw.surface), props)

        if not opt.runway_extras:
            continue
        (x1, y1), (x2, y2) = rw.end1, rw.end2
        length = math.hypot(x2 - x1, y2 - y1)
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        px, py = -uy, ux
        hw = rw.width / 2.0
        for end_id, (ex, ey), sign, over, disp in ((rw.end1_id, rw.end1, -1.0, rw.overrun1, rw.displaced1),
                                                   (rw.end2_id, rw.end2, 1.0, rw.overrun2, rw.displaced2)):
            if over > 0.01:                                  # overrun / blast pad beyond the runway end
                ox, oy = ex + ux * sign * over, ey + uy * sign * over
                pad = [(ex + px * hw, ey + py * hw), (ox + px * hw, oy + py * hw),
                       (ox - px * hw, oy - py * hw), (ex - px * hw, ey - py * hw)]
                add(f"Overrun_{end_id}", pad, [], "Runways", _surface_color(rw.surface),
                    {"runway_end": end_id, "length_m": over})
        shoulder_surface, shoulder_w = rw.shoulder % 100, rw.shoulder // 100
        if shoulder_surface > 0:
            w = float(shoulder_w) if shoulder_w > 0 else _SHOULDER_DEFAULT_WIDTH
            for side in (1.0, -1.0):
                a0, a1 = hw, hw + w
                strip = [(x1 + px * side * a0, y1 + py * side * a0), (x2 + px * side * a0, y2 + py * side * a0),
                         (x2 + px * side * a1, y2 + py * side * a1), (x1 + px * side * a1, y1 + py * side * a1)]
                add(f"Shoulder_{tag}_{'L' if side > 0 else 'R'}", strip, [], "Runways", _surface_color(shoulder_surface),
                    {"runway": tag, "width_m": w, "surface": shoulder_surface,
                     "surface_name": surface_name(shoulder_surface)})

    # ---- helipads ------------------------------------------------------------------
    for hp in data.helipads:
        props = {"surface": hp.surface, "surface_name": surface_name(hp.surface),
                 "length_m": hp.length, "width_m": hp.width, "heading_deg": hp.heading_deg}
        props.update(hp.props)
        add(f"Helipad_{hp.name}", _helipad_rectangle(hp), [], "Helipads", _surface_color(hp.surface), props)

    # ---- pavements -----------------------------------------------------------------
    for i, pav in enumerate(data.pavements):
        if not pav.rings:
            continue
        props = {"surface": pav.surface, "surface_name": surface_name(pav.surface),
                 "smoothness": pav.smoothness, "texture_heading": pav.texture_heading}
        if pav.description:
            props["description"] = pav.description
        add(f"Pavement_surf{pav.surface}_{i:03d}", pav.rings[0].points, [r.points for r in pav.rings[1:]],
            "Pavements", _surface_color(pav.surface), props)

    # ---- displaced-threshold bars (thin ribbon across the runway) --------------------
    overlay: Dict[str, Mesh] = {}

    def overlay_mesh(name: str, category: str, color, props=None) -> Mesh:
        m = overlay.get(name)
        if m is None:
            m = overlay[name] = Mesh(category=category, color=color, props=dict(props or {}))
        return m

    if opt.runway_extras:
        for rw in data.runways:
            if rw.water or math.hypot(rw.end2[0] - rw.end1[0], rw.end2[1] - rw.end1[1]) == 0:
                continue
            (x1, y1), (x2, y2) = rw.end1, rw.end2
            length = math.hypot(x2 - x1, y2 - y1)
            ux, uy = (x2 - x1) / length, (y2 - y1) / length
            px, py = -uy, ux
            hw = rw.width / 2.0
            for end_id, (ex, ey), sign, disp in ((rw.end1_id, rw.end1, 1.0, rw.displaced1),
                                                 (rw.end2_id, rw.end2, -1.0, rw.displaced2)):
                if disp > 0.01:
                    cx, cy = ex + ux * sign * disp, ey + uy * sign * disp
                    m = overlay_mesh(f"DisplacedThreshold_{end_id}", "Lines", _WHITE,
                                     {"runway_end": end_id, "displaced_m": disp})
                    m.add_ribbon([(cx - px * hw, cy - py * hw), (cx + px * hw, cy + py * hw)], 1.5, LINE_Z)

    # ---- painted lines and light strings, merged per style ----------------------------
    def emit_runs(source: str, runs: List[LineRun]) -> None:
        for run in runs:
            if opt.lines and run.line_type:
                slug, color, width, dashed = _LINE_STYLES.get(run.line_type, (f"t{run.line_type}", _NEUTRAL, 0.30, False))
                name = f"Line_{source}_{slug}" if slug.startswith("t") and slug[1:].isdigit() else \
                       f"Line_{source}_t{run.line_type}_{slug}"
                m = overlay_mesh(name, "Lines", color, {"line_type": run.line_type, "source": source})
                if dashed:
                    for d in _split_dashes(run.points, _DASH, _GAP, run.closed):
                        m.add_ribbon(d, width, LINE_Z)
                else:
                    m.add_ribbon(run.points, width, LINE_Z, closed=run.closed)
            if opt.lights and run.light_type:
                slug, color = _LIGHT_STYLES.get(run.light_type, (f"t{run.light_type}", (1.0, 1.0, 0.85)))
                name = f"Lights_{source}_{slug}" if slug.startswith("t") and slug[1:].isdigit() else \
                       f"Lights_{source}_t{run.light_type}_{slug}"
                m = overlay_mesh(name, "Lights", color, {"light_type": run.light_type, "source": source,
                                                         "spacing_m": opt.light_spacing})
                for p in _sample_polyline(run.points, max(opt.light_spacing, 0.5), run.closed):
                    m.add_square(p, 0.4, LIGHT_Z)

    if opt.lines or opt.lights:
        for feat in data.linear_features:
            emit_runs("Feature", feat.runs)
        for pav in data.pavements:
            emit_runs("Edge", pav.edge_runs)

    # ---- airport boundary (wire) ------------------------------------------------------
    if opt.boundary:
        for i, b in enumerate(data.boundaries):
            m = Mesh(category="Boundary", color=(1.0, 0.3, 0.3), props={"name": b.name})
            for ring in b.rings:
                m.add_wire(ring.points, BOUNDARY_Z, closed=True)
            overlay[f"Boundary_{i:02d}"] = m

    # ---- taxi route network + ground-vehicle routes (wire) -----------------------------
    if opt.network and data.taxi_edges:
        by_id = {n.id: n for n in data.taxi_nodes}
        missing = 0
        for e in data.taxi_edges:
            na, nb = by_id.get(e.a), by_id.get(e.b)
            if na is None or nb is None:
                missing += 1
                continue
            if e.ground_vehicle:
                name = "TruckRoutes" + ("_oneway" if e.oneway else "")
                color = (0.9, 0.5, 0.1)
            else:
                name = f"TaxiRoutes_{e.kind}" + ("_oneway" if e.oneway else "")
                color = (0.2, 0.8, 0.9)
            m = overlay.get(name)
            if m is None:
                m = overlay[name] = Mesh(category="TaxiNetwork", color=color,
                                         props={"kind": e.kind, "oneway": e.oneway})
            m.add_edge((na.pos[0], na.pos[1], NETWORK_Z), (nb.pos[0], nb.pos[1], NETWORK_Z))
        if missing:
            say(f"  [warn] {missing} taxi edge(s) refer to nodes that are not in the file, skipped")

    for name, m in overlay.items():
        if m.vertices:
            groups[unique(name)] = m

    for m in groups.values():
        m.finalize()
    return {k: v for k, v in groups.items() if v.faces or v.edges}


def build_marker_meshes(markers: List[Marker]) -> Dict[str, Mesh]:
    """Markers as small pyramids (one mesh per marker kind) so they survive in a plain OBJ,
    which has no empties. Long axis = heading, if the marker has one."""
    groups: Dict[str, Mesh] = {}
    for mk in markers:
        m = groups.setdefault(f"Markers_{MARKER_LABELS.get(mk.kind, mk.kind).replace(' ', '_')}",
                              Mesh(category="Markers", color=(1.0, 0.45, 0.1)))
        height = mk.height_m if mk.height_m > 0.5 else 2.0
        has_heading = mk.heading_deg is not None
        m.add_pyramid(mk.pos, 1.0, 2.0 if has_heading else 1.0, height, mk.heading_deg or 0.0)
    for m in groups.values():
        m.finalize()
    return groups
