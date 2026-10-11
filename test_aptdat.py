"""Regression tests. Run:  python -m unittest -v test_aptdat"""
import math
import os
import tempfile
import unittest

import aptdat_core as core
import aptdat_export as export

HERE = os.path.dirname(os.path.abspath(__file__))
SAMPLE = os.path.join(HERE, "sample_KNPA.apt.dat")


def _write(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".dat", delete=False, newline="")
    f.write(text)
    f.close()
    return f.name


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _crossings(ring):
    n, c = len(ring), 0
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            a, b, p, q = ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n]
            if _cross(p, q, a) * _cross(p, q, b) < 0 and _cross(a, b, p) * _cross(a, b, q) < 0:
                c += 1
    return c


def _volume(m):
    v = 0.0
    for a, b, c in m.faces:
        A, B, C = m.vertices[a], m.vertices[b], m.vertices[c]
        v += (A[0] * (B[1] * C[2] - B[2] * C[1]) - A[1] * (B[0] * C[2] - B[2] * C[0])
              + A[2] * (B[0] * C[1] - B[1] * C[0])) / 6
    return v


class ParserTests(unittest.TestCase):
    def test_windsock_is_not_an_airport(self):
        p = _write("I\n1100 test\n\n1 10 0 0 KAAA Alpha\n1302 iata_code AAA\n19 30.0 -87.0 1 WS\n"
                   "19 30.1 -87.1 1 Biruta\n1\t20 0 0 KBBB Bravo\n17 5 0 0 HEL1 [H] Heli\n99\n")
        rows = core.list_airports(p)
        self.assertEqual([r.ident for r in rows], ["KAAA", "KBBB", "HEL1"])
        self.assertEqual(rows[0].iata, "AAA")
        self.assertEqual(rows[2].kind, "heliport")

    def test_helipad_columns(self):
        p = _write("1 10 0 0 KAAA Alpha\n1302 datum_lat 47.5\n1302 datum_lon -122.3\n"
                   "102 H1 47.5 -122.3 90.00 20.00 10.00 2 0 0 0.25 0\n")
        d = core.parse_airport(p, "KAAA")
        h = d.helipads[0]
        self.assertEqual((h.name, h.length, h.width, h.heading_deg, h.surface), ("H1", 20.0, 10.0, 90.0, 2))
        self.assertAlmostEqual(h.center[0], 0, places=3)
        self.assertAlmostEqual(h.center[1], 0, places=3)

    def test_lookup_by_iata_and_comment_lines(self):
        p = _write("1 10 0 0 XEP87 Foo\n1302 iata_code FOO\n1302 datum_lat 10\n1302 datum_lon 10\n"
                   "100 30 1 0 0.25 0 0 0 09 10.0 10.0 0 0 0 0 0 0 27 10.0 10.01 0 0 0 0 0 0\n"
                   "# a comment\n")
        self.assertEqual(core.parse_airport(p, "foo").icao, "XEP87")
        self.assertIsNone(core.parse_airport(p, "NOPE"))

    def test_split_corner_makes_no_spikes(self):
        # WorldEditor "split" corner: three nodes at the same coordinate
        sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
        nodes = [core._Node(sq[0], None), core._Node(sq[1], None),
                 core._Node((10, 10), (10, 13)), core._Node((10, 10), None), core._Node((10, 10), (7, 10)),
                 core._Node(sq[3], None)]
        ring = core._clean_ring(core._flatten_chain(nodes, closed=True))
        self.assertTrue(all(-1e-6 <= x <= 10 + 1e-6 and -1e-6 <= y <= 10 + 1e-6 for x, y in ring),
                        "bezier spike left the square")
        self.assertEqual(_crossings(ring), 0)
        self.assertGreater(abs(core._signed_area(ring)), 90)

    def test_split_corner_wrapping_the_chain_end(self):
        nodes = [core._Node((0, 0), (-3, 0)), core._Node((10, 0), None), core._Node((10, 10), None),
                 core._Node((0, 10), None), core._Node((0, 0), None)]
        ring = core._clean_ring(core._flatten_chain(nodes, closed=True))
        self.assertEqual(_crossings(ring), 0)

    def test_antimeridian(self):
        pr = core.Projector(-17.0, 179.9999)
        x, _ = pr.to_xy(-17.0, -179.9999)
        self.assertLess(abs(x), 50)

    def test_projection_scale_matches_wgs84(self):
        pr = core.Projector(45.5, 0.0)
        self.assertAlmostEqual(pr.to_xy(46.0, 0)[1] - pr.to_xy(45.0, 0)[1], 111141.5, delta=2)
        self.assertAlmostEqual(pr.to_xy(45.5, 1.0)[0], 78158.06, delta=0.5)   # cross-checked with pyproj/WGS84


class SearchTests(unittest.TestCase):
    def setUp(self):
        E = core.AirportEntry
        self.idx = core.AirportIndex([
            E("KNPA", "Pensacola NAS", "airport", "KNPA", "NPA", "NPA", "Pensacola", "Florida", "USA"),
            E("KPNS", "Pensacola Intl", "airport", "KPNS", "PNS", "PNS", "Pensacola", "Florida", "USA"),
            E("KNP", "Napa Thing", "airport"),
            E("XEP87", "Kakolewo", "airport", "EPKK", "", "", "Poznan", "", "Poland"),
        ])

    def test_exact_code_wins(self):
        self.assertEqual(self.idx.search("knp")[0][0].ident, "KNP")
        self.assertEqual(self.idx.search("knpa")[0][0].ident, "KNPA")
        self.assertEqual(self.idx.search("pns")[0][0].ident, "KPNS")

    def test_name_and_multiword(self):
        res, total = self.idx.search("pensacola intl")
        self.assertEqual([e.ident for e in res], ["KPNS"])
        self.assertEqual(self.idx.search("pensacola")[1], 2)

    def test_icao_differs_from_ident_and_empty_query(self):
        self.assertEqual(self.idx.search("EPKK")[0][0].ident, "XEP87")
        self.assertEqual(self.idx.search("")[1], 4)
        self.assertEqual(self.idx.search("zzzz"), ([], 0))


class MeshTests(unittest.TestCase):
    def _check_clean(self, meshes):
        for name, m in meshes.items():
            used = {i for f in m.faces for i in f}
            self.assertEqual(len(used), len(m.vertices), f"{name}: loose vertices")
            self.assertTrue(all(len({a, b, c}) == 3 for a, b, c in m.faces), f"{name}: degenerate face")

    def test_sample_knpa_is_clean_and_complete(self):
        d = core.parse_airport(SAMPLE, "KNPA")
        meshes = core.build_meshes(d)
        self._check_clean(meshes)
        for i, pav in enumerate(d.pavements):
            m = meshes[f"Pavement_surf{pav.surface}_{i:03d}"]
            tri = sum(abs(core._signed_area([m.vertices[a][:2], m.vertices[b][:2], m.vertices[c][:2]]))
                      for a, b, c in m.faces)
            want = abs(core._signed_area(pav.rings[0].points)) - sum(abs(core._signed_area(r.points)) for r in pav.rings[1:])
            self.assertAlmostEqual(tri, want, delta=want * 1e-4)   # vertices are welded/rounded to 0.1 mm

    def test_slab_is_outward_facing_for_either_winding(self):
        for outer, hole in (([(0, 0), (20, 0), (20, 20), (0, 20)], [(5, 5), (5, 10), (10, 10), (10, 5)]),
                            ([(0, 0), (0, 20), (20, 20), (20, 0)], [(5, 5), (10, 5), (10, 10), (5, 10)])):
            m = core.Mesh()
            m.add_extruded_polygon(outer, [hole], -0.5, 0.0)
            m.finalize()
            self.assertAlmostEqual(_volume(m), (400 - 25) * 0.5, places=6)

    def test_flat_faces_point_up(self):
        m = core.Mesh()
        m.add_flat_polygon([(0, 0), (0, 5), (5, 5), (5, 0)], [], 0.0)
        m.finalize()
        for a, b, c in m.faces:
            A, B, C = m.vertices[a], m.vertices[b], m.vertices[c]
            self.assertGreater(_cross(A, B, C), 0)

    def test_obj_export_indices_valid(self):
        d = core.parse_airport(SAMPLE, "KNPA")
        meshes = core.build_meshes(d)
        path = os.path.join(tempfile.mkdtemp(), "k.obj")
        export.export_obj(meshes, path, d)
        nv, faces = 0, []
        for line in open(path):
            if line.startswith("v "):
                nv += 1
            elif line.startswith("f "):
                faces.append([int(t) for t in line.split()[1:]])
        self.assertEqual(nv, sum(len(m.vertices) for m in meshes.values()))
        self.assertTrue(all(1 <= i <= nv for f in faces for i in f))
        self.assertEqual({i for f in faces for i in f}, set(range(1, nv + 1)))


if __name__ == "__main__":
    unittest.main()
