# apt.dat → Blender Layout Tool

Pick an X-Plane `apt.dat` scenery file, find an airport, and get back either:
- an **.obj** file (import into Blender / any 3D app), or
- a **Blender Python script** (paste into Blender's Scripting tab and run it
  to build the mesh natively, organized into a named collection)

It extracts **runways**, **water runways**, **helipads** and **taxiway/apron
pavement polygons** (including curved fillets and cutout "islands"/holes),
projects them to flat local meters, and triangulates them into clean mesh
geometry. No markings, lights, signs, or ATC data - just the layout.

## v3 changes
**Airport search (GUI).** Type in the search box and the list narrows as you
type: ICAO, IATA/FAA code, airport name or city (all words must match; exact
code matches rank first). The best match is pre-selected, so
`type "KNPA" → Generate` just works. Big files are scanned on a background
thread with a progress bar, so the window never freezes.

**Broken meshes (loose vertices / missing faces) - fixed.** Root cause:
WorldEditor writes a corner between two curves as several nodes at the *same
coordinate* (`112 P` / `111 P` / `112 P`). These were drawn as zero-length
bezier segments = self-overlapping spikes, which the triangulator then dropped
triangles on and left orphaned vertices behind. Coincident nodes are now merged
into one corner (incoming handle from the first node, outgoing from the last),
rings are cleaned, and only vertices that a face uses are ever written. Tested on
real apt.dat data: before, 31 of 180 meshes were missing area (up to 147%) and
42 had loose vertices; after, 0 of 185.

**Other fixes**
- **Airport list was ~3x too long**: windsock rows (code `19`) were being treated
  as airport headers. Only `1`, `16`, `17` are headers.
- **Helipads (`102`)** were read with the wrong columns (length = longitude!).
- **Scale**: the old flat `111320 m/deg` made north-south distances ~0.4% too long
  at mid latitudes (an 8.9 m error on a 2.2 km runway). Now an ellipsoid-correct
  local projection (cm accuracy). Also survives the antimeridian.
- **Slabs** (`--thickness > 0`): side walls were inside-out depending on polygon
  winding; now always outward, and vertices are welded.
- Seaplane bases: water runways (`101`) are now exported.
- Whitespace/tab-separated files, `#` comment lines, malformed rows and missing
  `datum_lat/lon` no longer break parsing.
- GUI: typed paths work (press Enter), thickness is validated, generation runs on a
  background thread, and the default output folder is `~/AptDatToBlender_output`
  instead of the (possibly unwritable) working directory.
- `--icao` accepts ICAO, IATA or FAA codes.
- Warnings are reported in the log if a polygon in the source data is itself
  self-intersecting.

## Running it
```
pip install -r requirements.txt
python aptdat_app.py
```
Browse to your `apt.dat` → type to search → choose OBJ / Blender script / both → Generate.

**Command line**
```
python aptdat_app.py --input apt.dat --icao KNPA --outdir output --format both
python aptdat_app.py --list apt.dat                     # every airport in the file
python aptdat_app.py --list apt.dat --search pensacola  # search by code / name / city
```
Options: `--format obj|blender|both`, `--thickness 0.15` (pavement thickness in meters;
default `0` = flat plane).

**Blender OBJ import:** the OBJ is Z-up. In the importer set *Forward = Y, Up = Z*,
or the layout will stand on its edge. (The Blender script has no such issue.)

## Getting the .exe
- **No Windows machine:** push this folder to a GitHub repo (keep `.github`); the
  *Actions* tab builds `AptDatToBlender.exe` (and runs the tests first).
- **On Windows:** double-click `build_exe.bat` → `dist\AptDatToBlender.exe`.

## Tests
```
python -m unittest -v test_aptdat
```

## Files
- `aptdat_app.py` - GUI + CLI entry point
- `aptdat_core.py` - apt.dat parsing + geometry engine
- `aptdat_export.py` - OBJ / Blender-script writers
- `test_aptdat.py` - regression tests
- `sample_KNPA.apt.dat` - Pensacola NAS excerpt for testing
- `build_exe.bat`, `requirements.txt`, `.github/workflows/build-exe.yml` - exe build

## Known limitations
- Flat ground layout only (no buildings, markings, textures).
- Runways are the full rectangle between the two end coordinates; overrun/blast-pad
  areas beyond the ends and shoulders are not added.
- Pavement polygons overlap each other and runways at junctions (as in apt.dat),
  so coplanar overlaps can z-fight in the Blender viewport. Use a small thickness
  or offset layers if that bothers you.
- Only apt.dat 1000+ rows are read (older 810/850 airports produce nothing and say so).
