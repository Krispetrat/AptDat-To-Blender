"""
aptdat_core.py
--------------
Core engine: parses an X-Plane apt.dat file for one ICAO and turns the
runway + pavement (taxiway/apron) layout into flat 3D geometry.

No GUI code here -- aptdat_app.py wraps this with a Tk GUI / CLI.

Supported apt.dat row codes:
  1               airport header (elevation, ICAO, name)
  1302            metadata (datum_lat / datum_lon used as projection origin)
  100             runway (hard/turf/etc.)
  102             helipad
  110             pavement (taxiway / apron) header
  111,112,113,114,115,116   boundary nodes for pavement/boundary/linear features
                    111/113 = straight node, 112/114 = bezier node,
                    113/114 = closes the ring, 115/116 = ends an open chain
Everything else (lighting, signs, ATC flow, taxi routes, frequencies,
parking, boundary, linear features/markings) is intentionally skipped --
this tool only builds the *pavement layout*, not markings or metadata.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

try:
    from earcut import earcut as _earcut_mod
    _HAVE_EARCUT = True
except Exception:
    _HAVE_EARCUT = False

Vec2 = Tuple[float, float]

BEZIER_SUBDIVISIONS = 10  # segments per curved edge - smooth enough for airport fillets


# --------------------------------------------------------------------------
# Data model
# --------------------------------------------------------------------------

@dataclass
class Ring:
    points: List[Vec2]          # local XY meters, already bezier-flattened, closed (no dup last point)


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
    icao: str
    name: str = ""
    elevation_ft: float = 0.0
    ref_lat: float = 0.0
    ref_lon: float = 0.0
    runways: List[Runway] = field(default_factory=list)
    pavements: List[Pavement] = field(default_factory=list)
    helipads: List[Helipad] = field(default_factory=list)


# --------------------------------------------------------------------------
# Geodesy: simple local equirectangular projection, meters, centered on the
# airport reference point. Good enough for an airport's few-km extent.
# X = east (+), Y = north (+), matches Blender/OBJ ground plane (Z stays up).
# --------------------------------------------------------------------------

_M_PER_DEG_LAT = 111320.0


def _m_per_deg_lon(ref_lat_deg: float) -> float:
    return 111320.0 * math.cos(math.radians(ref_lat_deg))


class Projector:
    def __init__(self, ref_lat: float, ref_lon: float):
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self._mlon = _m_per_deg_lon(ref_lat)

    def to_xy(self, lat: float, lon: float) -> Vec2:
        x = (lon - self.ref_lon) * self._mlon
        y = (lat - self.ref_lat) * _M_PER_DEG_LAT
        return (x, y)


# --------------------------------------------------------------------------
# Bezier flattening for pavement boundary rings
# --------------------------------------------------------------------------

@dataclass
class _Node:
    pt: Vec2
    ctrl: Optional[Vec2]        # control point as stored in apt.dat, or None
    close: bool                 # True if this is a 113/114 (closes the ring)
    ends_chain: bool            # True if this is 113/114/115/116 (last node of chain)


def _cubic_bezier(p0, p1, p2, p3, n):
    pts = []
    for i in range(1, n + 1):
        t = i / n
        mt = 1 - t
        x = (mt**3) * p0[0] + 3 * (mt**2) * t * p1[0] + 3 * mt * (t**2) * p2[0] + (t**3) * p3[0]
        y = (mt**3) * p0[1] + 3 * (mt**2) * t * p1[1] + 3 * mt * (t**2) * p2[1] + (t**3) * p3[1]
        pts.append((x, y))
    return pts


def _flatten_chain(nodes: List[_Node], closed: bool) -> List[Vec2]:
    """Turn a list of nodes (each optionally with a bezier control point) into
    a flat polyline, per the X-Plane apt.dat curve convention:
      outgoing tangent handle at node A = A's control point as-is
      incoming tangent handle at node B = mirror of B's control point through B
    If neither A nor B has a control point, the segment is a straight line.
    """
    out: List[Vec2] = [nodes[0].pt]
    count = len(nodes)
    edge_count = count if closed else count - 1
    for i in range(edge_count):
        a = nodes[i]
        b = nodes[(i + 1) % count]
        if a.ctrl is None and b.ctrl is None:
            out.append(b.pt)
            continue
        # A node's stored control point is its own OUTGOING handle (literal);
        # when the node ends a segment, the incoming handle is its MIRROR.
        # (v1 had these swapped -> self-intersecting loops at taxiway fillets.)
        h1 = a.ctrl if a.ctrl else a.pt
        h2 = (2 * b.pt[0] - b.ctrl[0], 2 * b.pt[1] - b.ctrl[1]) if b.ctrl else b.pt
        out.extend(_cubic_bezier(a.pt, h1, h2, b.pt, BEZIER_SUBDIVISIONS))
    # drop an accidental duplicate closing point
    if closed and len(out) > 1 and out[0] == out[-1]:
        out.pop()
    return out


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------

def _split_airport_blocks(path: str):
    """Yield (icao, list_of_lines) for every airport (row code 1/16/17/19 = land/seaplane/heliport)
    found in the file, without loading everything else into memory at once."""
    header_codes = {"1", "16", "17", "19"}
    icao = None
    buf: List[str] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if not stripped:
                continue
            code = stripped.split(" ", 1)[0]
            if code in header_codes:
                if icao is not None:
                    yield icao, buf
                toks = stripped.split()
                # 1 <elev> <deprecated> <deprecated> <ICAO> <name...>
                icao = toks[4] if len(toks) > 4 else None
                buf = [stripped]
            else:
                if icao is not None:
                    buf.append(stripped)
    if icao is not None:
        yield icao, buf


def list_airports(path: str) -> List[Tuple[str, str]]:
    """Return [(icao, name), ...] found in the file (fast scan, header lines only)."""
    results = []
    header_codes = {"1", "16", "17", "19"}
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            stripped = raw.strip()
            if not stripped:
                continue
            code = stripped.split(" ", 1)[0]
            if code in header_codes:
                toks = stripped.split()
                if len(toks) > 4:
                    icao = toks[4]
                    name = " ".join(toks[5:]) if len(toks) > 5 else ""
                    results.append((icao, name))
    return results


def parse_airport(path: str, icao: str) -> Optional[AirportData]:
    icao = icao.strip().upper()
    lines = None
    for found_icao, buf in _split_airport_blocks(path):
        if found_icao and found_icao.upper() == icao:
            lines = buf
            break
    if lines is None:
        return None

    header = lines[0].split()
    elevation_ft = float(header[1]) if len(header) > 1 else 0.0
    name = " ".join(header[5:]) if len(header) > 5 else icao

    data = AirportData(icao=icao, name=name, elevation_ft=elevation_ft)

    datum_lat = datum_lon = None
    raw_runways = []   # (width, surface, id1, lat1, lon1, id2, lat2, lon2)
    raw_helipads = []  # (name, lat, lon, length, width, surface, heading)

    # pavement-block state
    pav_surface = None
    pav_rings: List[List[_Node]] = []
    cur_chain: List[_Node] = []
    in_pavement = False

    def flush_chain(closed: bool):
        nonlocal cur_chain
        if cur_chain:
            pav_rings.append((cur_chain, closed))
            cur_chain = []

    def flush_pavement():
        nonlocal pav_surface, pav_rings, in_pavement
        if in_pavement and pav_rings:
            data.pavements.append((pav_surface, pav_rings))
        pav_surface = None
        pav_rings = []
        in_pavement = False

    for line in lines[1:]:
        toks = line.split()
        code = toks[0]

        if code == "1302" and len(toks) >= 3:
            if toks[1] == "datum_lat":
                datum_lat = float(toks[2])
            elif toks[1] == "datum_lon":
                datum_lon = float(toks[2])
            continue

        if code == "100":
            flush_pavement()
            width = float(toks[1])
            surface = int(float(toks[2]))
            id1 = toks[8]
            lat1, lon1 = float(toks[9]), float(toks[10])
            id2 = toks[17]
            lat2, lon2 = float(toks[18]), float(toks[19])
            raw_runways.append((width, surface, id1, lat1, lon1, id2, lat2, lon2))
            continue

        if code == "102":
            flush_pavement()
            # 102 <name> <width> <length> <surface> <shoulder> <smoothness> <heading> <lat> <lon> ...
            try:
                hp_name = toks[1]
                width = float(toks[2])
                length = float(toks[3])
                surface = int(float(toks[4]))
                heading = float(toks[7])
                lat = float(toks[8])
                lon = float(toks[9])
                raw_helipads.append((hp_name, lat, lon, length, width, surface, heading))
            except (IndexError, ValueError):
                pass
            continue

        if code == "110":
            flush_pavement()
            in_pavement = True
            pav_surface = int(float(toks[1]))
            continue

        if code in ("111", "113"):
            if not in_pavement:
                continue
            pt = (float(toks[1]), float(toks[2]))
            cur_chain.append(_Node(pt=pt, ctrl=None, close=(code == "113"), ends_chain=(code == "113")))
            if code == "113":
                flush_chain(closed=True)
            continue

        if code in ("112", "114"):
            if not in_pavement:
                continue
            pt = (float(toks[1]), float(toks[2]))
            ctrl = (float(toks[3]), float(toks[4]))
            cur_chain.append(_Node(pt=pt, ctrl=ctrl, close=(code == "114"), ends_chain=(code == "114")))
            if code == "114":
                flush_chain(closed=True)
            continue

        if code == "115":
            if not in_pavement:
                continue
            pt = (float(toks[1]), float(toks[2]))
            cur_chain.append(_Node(pt=pt, ctrl=None, close=False, ends_chain=True))
            flush_chain(closed=False)
            continue

        if code == "116":
            if not in_pavement:
                continue
            pt = (float(toks[1]), float(toks[2]))
            ctrl = (float(toks[3]), float(toks[4]))
            cur_chain.append(_Node(pt=pt, ctrl=ctrl, close=False, ends_chain=True))
            flush_chain(closed=False)
            continue

        # anything else (120 linear features, 130 boundary, ATC/lighting/parking/etc.)
        # ends any in-progress pavement block.
        flush_pavement()

    flush_pavement()

    if datum_lat is None or datum_lon is None:
        # fall back to the average of runway endpoints
        pts = [(r[3], r[4]) for r in raw_runways] + [(r[6], r[7]) for r in raw_runways]
        if pts:
            datum_lat = sum(p[0] for p in pts) / len(pts)
            datum_lon = sum(p[1] for p in pts) / len(pts)
        else:
            datum_lat, datum_lon = 0.0, 0.0

    data.ref_lat, data.ref_lon = datum_lat, datum_lon
    proj = Projector(datum_lat, datum_lon)

    for width, surface, id1, lat1, lon1, id2, lat2, lon2 in raw_runways:
        data.runways.append(Runway(
            width=width, surface=surface, end1_id=id1, end2_id=id2,
            end1=proj.to_xy(lat1, lon1), end2=proj.to_xy(lat2, lon2),
        ))

    for hp_name, lat, lon, length, width, surface, heading in raw_helipads:
        data.helipads.append(Helipad(
            name=hp_name, surface=surface, center=proj.to_xy(lat, lon),
            length=length, width=width, heading_deg=heading,
        ))

    for surface, ring_chains in data.pavements if False else []:
        pass  # placeholder, real conversion below

    # convert pavement lat/lon node chains -> projected, bezier-flattened rings
    converted_pavements: List[Pavement] = []
    for surface, ring_chains in data.pavements:
        rings: List[Ring] = []
        for chain, closed in ring_chains:
            proj_nodes = [_Node(pt=proj.to_xy(*n.pt), ctrl=(proj.to_xy(*n.ctrl) if n.ctrl else None),
                                 close=n.close, ends_chain=n.ends_chain) for n in chain]
            if len(proj_nodes) < 2:
                continue
            flat = _flatten_chain(proj_nodes, closed=closed)
            if len(flat) >= 3:
                rings.append(Ring(points=flat))
        if rings:
            converted_pavements.append(Pavement(surface=surface, rings=rings))
    data.pavements = converted_pavements

    return data


# --------------------------------------------------------------------------
# Geometry building: runways/pavements -> triangulated 3D meshes
# --------------------------------------------------------------------------

@dataclass
class Mesh:
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)
    faces: List[Tuple[int, int, int]] = field(default_factory=list)   # 0-based indices into `vertices`

    def add_flat_polygon(self, ring_outer: List[Vec2], holes: List[List[Vec2]], z: float, flip=False) -> None:
        tris = _triangulate(ring_outer, holes)
        base = len(self.vertices)
        all_pts = ring_outer[:]
        # _triangulate returns indices into (outer + each hole) concatenated; rebuild that list identically
        combined = list(ring_outer)
        for h in holes:
            combined.extend(h)
        for (x, y) in combined:
            self.vertices.append((x, y, z))
        for (i0, i1, i2) in tris:
            if flip:
                self.faces.append((base + i0, base + i2, base + i1))
            else:
                self.faces.append((base + i0, base + i1, base + i2))

    def add_extruded_polygon(self, ring_outer: List[Vec2], holes: List[List[Vec2]], z_bottom: float, z_top: float) -> None:
        """Top + bottom caps plus side walls, for a slab with real thickness."""
        self.add_flat_polygon(ring_outer, holes, z_top, flip=False)
        self.add_flat_polygon(ring_outer, holes, z_bottom, flip=True)
        for ring in [ring_outer] + holes:
            n = len(ring)
            for i in range(n):
                a = ring[i]
                b = ring[(i + 1) % n]
                base = len(self.vertices)
                self.vertices.append((a[0], a[1], z_top))
                self.vertices.append((b[0], b[1], z_top))
                self.vertices.append((b[0], b[1], z_bottom))
                self.vertices.append((a[0], a[1], z_bottom))
                self.faces.append((base + 0, base + 1, base + 2))
                self.faces.append((base + 0, base + 2, base + 3))


def _triangulate(outer: List[Vec2], holes: List[List[Vec2]]) -> List[Tuple[int, int, int]]:
    if not _HAVE_EARCUT:
        raise RuntimeError("earcut package not available - run: pip install earcut")
    rings = [outer] + holes
    coords = []
    hole_indices = []
    running = 0
    for i, ring in enumerate(rings):
        if i > 0:
            hole_indices.append(running)
        for (x, y) in ring:
            coords.append(x)
            coords.append(y)
        running += len(ring)
    flat_indices = _earcut_mod.earcut(coords, hole_indices if hole_indices else None, 2)
    tris = []
    for i in range(0, len(flat_indices), 3):
        tris.append((flat_indices[i], flat_indices[i + 1], flat_indices[i + 2]))
    return tris


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
        x = cx + px * lx + ux * ly
        y = cy + py * lx + uy * ly
        pts.append((x, y))
    return pts


def build_meshes(data: AirportData, slab_thickness: float = 0.0):
    """Returns a dict of {group_name: Mesh}, one mesh per logical group so the
    Blender/OBJ output comes in cleanly-named, separately-selectable pieces."""
    groups: dict = {}

    def get(name: str) -> Mesh:
        if name not in groups:
            groups[name] = Mesh()
        return groups[name]

    for rw in data.runways:
        rect = _runway_rectangle(rw)
        if not rect:
            continue
        name = f"Runway_{rw.end1_id}-{rw.end2_id}"
        m = get(name)
        if slab_thickness > 0:
            m.add_extruded_polygon(rect, [], -slab_thickness, 0.0)
        else:
            m.add_flat_polygon(rect, [], 0.0)

    for hp in data.helipads:
        rect = _helipad_rectangle(hp)
        name = f"Helipad_{hp.name}"
        m = get(name)
        if slab_thickness > 0:
            m.add_extruded_polygon(rect, [], -slab_thickness, 0.0)
        else:
            m.add_flat_polygon(rect, [], 0.0)

    for i, pav in enumerate(data.pavements):
        if not pav.rings:
            continue
        outer = pav.rings[0].points
        holes = [r.points for r in pav.rings[1:]]
        name = f"Pavement_surf{pav.surface}_{i:03d}"
        m = get(name)
        try:
            if slab_thickness > 0:
                m.add_extruded_polygon(outer, holes, -slab_thickness, 0.0)
            else:
                m.add_flat_polygon(outer, holes, 0.0)
        except Exception as e:
            # skip a malformed polygon rather than aborting the whole airport
            print(f"  [warn] skipped {name}: {e}")

    return groups
