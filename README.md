# apt.dat → Blender Layout Tool

Upload an X-Plane `apt.dat` scenery file, pick an ICAO, and get back either:
- an **.obj** file (import into Blender / any 3D app), or
- a **Blender Python script** (paste into Blender's Scripting tab and run it
  to build the mesh natively, already organized into a named collection)

It extracts **runways** and **taxiway/apron pavement polygons** (including
curved fillets and cutout "islands"/holes), projects them to flat local
meters, and triangulates them into clean mesh geometry. That's it — no
markings, lights, signs, or ATC data, since you said you just want the
layout.

## v2 changes
- **Fixed jagged / self-crossing pavement edges and "missing" tarmac.** The
  bezier curve handles on taxiway fillets were applied backwards, so curves
  looped back on themselves and the triangulator produced pinched/missing
  pavement at hold pads, ramp edges and runway turn-ons. Curves are now
  smooth and correct.
- **Real .exe without a Windows PC:** `.github/workflows/build-exe.yml`
  builds `AptDatToBlender.exe` on GitHub's Windows runners (see below).

## ⚠️ About the .exe

I built and tested this on a Linux machine, which can't cross-compile a
genuine Windows `.exe` (that requires Windows itself, or Wine, neither of
which I have here). So instead of a fake/broken exe, I'm giving you:

1. The **full working Python app** (tested end-to-end on real KNPA data —
   see `sample_KNPA.apt.dat` and the sample outputs included here).
2. A **`build_exe.bat`** that turns it into a real `AptDatToBlender.exe` in
   about a minute, on your own Windows machine.

### Get the exe with no Windows machine (GitHub Actions)
1. Create a free GitHub repo and upload this whole folder (keep the
   `.github` folder).
2. Open the repo's **Actions** tab; "Build Windows exe" runs by itself.
3. When it's green (~2 min), open the run and download the
   **AptDatToBlender-windows-exe** artifact. That zip contains the .exe.

### Fastest path (no exe needed at all)
If you have Python installed, you can just run the GUI directly:
```
pip install -r requirements.txt
python aptdat_app.py
```
A window opens — browse to your apt.dat, pick the ICAO, click Generate.

### To get an actual .exe
1. Install Python from python.org if you don't have it (check "Add to PATH"
   during install).
2. Double-click `build_exe.bat` in this folder.
3. It installs the needed packages and runs PyInstaller.
4. Your exe appears at `dist\AptDatToBlender.exe` — a single file you can
   copy anywhere and double-click to run, no Python required after that.

## Using the app

**GUI:** Browse to your `apt.dat` → pick the airport from the dropdown
(it scans the whole file, no need to trim it yourself) → choose OBJ /
Blender script / both → Generate.

**Command line:**
```
python aptdat_app.py --input apt.dat --icao KNPA --outdir output --format both
python aptdat_app.py --list apt.dat        # see every ICAO in a file
```

Options:
- `--format obj|blender|both`
- `--thickness 0.15` — gives the pavement real thickness (meters). Default
  `0` = flat plane, which is usually what you want for tracing/texturing.

## Files in this folder
- `aptdat_app.py` — GUI + CLI entry point
- `aptdat_core.py` — apt.dat parsing + geometry engine
- `aptdat_export.py` — OBJ / Blender-script writers
- `sample_KNPA.apt.dat` — the Pensacola NAS excerpt you sent me, for testing
- `sample_output/` — KNPA_layout.obj, KNPA_layout_blender.py, and a preview
  PNG, already generated, so you can see exactly what to expect
- `build_exe.bat`, `requirements.txt` — for building the .exe

## Known limitations
- apt.dat has no building/terminal/texture data — this is a **flat ground
  layout** (runways + taxiway/apron shapes), not a dressed-up airport.
- Displaced thresholds/overrun areas aren't cut out of the runway rectangle
  (the full paved rectangle is used) — fine for a base layout, let me know
  if you want that added.
- Helipad polygons (row code `102`) are supported but there weren't any in
  the KNPA sample to test against — flag it if one comes out wrong.
