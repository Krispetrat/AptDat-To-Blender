"""
aptdat_app.py
-------------
apt.dat -> Blender/OBJ airport layout tool.

GUI mode (default, double-click the .exe / run with no arguments):
    Pick an apt.dat file, then just TYPE in the search box (ICAO, IATA/FAA code,
    airport name or city) and the list narrows as you type. Choose OBJ /
    Blender-script / both, tick what to include (taxi lines, lights, boundary,
    taxi network, signs and other placed objects ...), and generate.

CLI mode (for scripting / automation):
    python aptdat_app.py --input apt.dat --icao KNPA --outdir out --format both
    python aptdat_app.py --input apt.dat --icao KNPA --no-lights --no-network   # leave things out
    python aptdat_app.py --list apt.dat --search pensacola
"""

import os
import sys
import argparse
import dataclasses
import traceback

import aptdat_core as core
import aptdat_export as export


# --------------------------------------------------------------------------
# Shared "do the work" function used by both CLI and GUI
# --------------------------------------------------------------------------

def generate(input_path: str, icao: str, outdir: str, fmt: str, thickness: float, log=print,
             options=None, write_json: bool = True):
    log(f"Reading {input_path} ...")
    data = core.parse_airport(input_path, icao)
    if data is None:
        raise ValueError(f"Airport '{icao}' was not found in this apt.dat file "
                         f"(try --list FILE --search TEXT to look it up).")

    opts = (dataclasses.replace(options, slab_thickness=thickness) if options is not None
            else core.BuildOptions(slab_thickness=thickness))

    log(f"Found {data.icao} - {data.name}")
    log(f"  {len(data.runways)} runway(s), {len(data.pavements)} pavement polygon(s), "
        f"{len(data.helipads)} helipad(s)")
    n_runs = sum(len(f.runs) for f in data.linear_features)
    n_edge = sum(len(p.edge_runs) for p in data.pavements)
    log(f"  {len(data.linear_features)} linear feature(s) ({n_runs} styled run(s)), "
        f"{n_edge} pavement-edge line/light run(s), {len(data.boundaries)} boundary polygon(s)")
    log(f"  taxi network: {len(data.taxi_nodes)} node(s), {len(data.taxi_edges)} edge(s)")
    kinds = {}
    for m in data.markers:
        kinds[m.kind] = kinds.get(m.kind, 0) + 1
    if kinds:
        log("  placed objects: " + ", ".join(f"{n} {k}" for k, n in sorted(kinds.items())))
    log(f"  {len(data.frequencies)} frequenc(ies), {len(data.flows)} ATC flow(s)")
    if data.ignored:
        log("  [note] row codes not handled: " + ", ".join(f"{c} x{n}" for c, n in sorted(data.ignored.items())))
    for w in data.warnings:
        log(f"  [warn] {w}")

    meshes = core.build_meshes(data, options=opts, log=log)
    markers = core.all_markers(data, opts)
    if not meshes and not markers:
        raise ValueError(f"{data.icao} has no runway, helipad or pavement geometry to export "
                         f"(apt.dat formats older than 1000 are not supported).")
    total_v = sum(len(m.vertices) for m in meshes.values())
    total_f = sum(len(m.faces) for m in meshes.values())
    total_e = sum(len(m.edges) for m in meshes.values())
    log(f"  Built {len(meshes)} mesh groups ({total_v} verts, {total_f} tris, {total_e} wire edges) "
        f"+ {len(markers)} placed marker(s)")

    os.makedirs(outdir, exist_ok=True)
    outputs = []

    if fmt in ("obj", "both"):
        obj_path = os.path.join(outdir, f"{data.icao}_layout.obj")
        obj_groups = dict(meshes)
        obj_groups.update(core.build_marker_meshes(markers))     # an OBJ has no empties: markers become small pyramids
        export.export_obj(obj_groups, obj_path, data)
        outputs.append(obj_path)
        log(f"  Wrote {obj_path}")

    if fmt in ("blender", "both"):
        py_path = os.path.join(outdir, f"{data.icao}_layout_blender.py")
        export.export_blender_script(meshes, py_path, data, markers)
        outputs.append(py_path)
        log(f"  Wrote {py_path}")

    if write_json:
        json_path = os.path.join(outdir, f"{data.icao}_data.json")
        export.export_json(data, json_path)
        outputs.append(json_path)
        log(f"  Wrote {json_path}")

    log("Done.")
    return outputs


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def run_cli(argv=None):
    ap = argparse.ArgumentParser(description="Convert an X-Plane apt.dat airport layout to OBJ / Blender script.")
    ap.add_argument("--input", "-i", help="Path to apt.dat")
    ap.add_argument("--icao", "-c", help="Airport to extract: ICAO, IATA or FAA code, e.g. KNPA")
    ap.add_argument("--outdir", "-o", default="output", help="Output directory (default: ./output)")
    ap.add_argument("--format", "-f", choices=["obj", "blender", "both"], default="both",
                     help="Output format (default: both)")
    ap.add_argument("--thickness", "-t", type=float, default=0.0,
                     help="Slab thickness in meters. 0 = flat plane (default).")
    ap.add_argument("--no-lines", action="store_true", help="Leave out painted lines (taxi lines, hold lines, edge markings).")
    ap.add_argument("--no-lights", action="store_true", help="Leave out light strings.")
    ap.add_argument("--no-boundary", action="store_true", help="Leave out the airport boundary.")
    ap.add_argument("--no-network", action="store_true", help="Leave out the taxi route / ground-vehicle network.")
    ap.add_argument("--no-runway-extras", action="store_true",
                    help="Leave out runway shoulders, overruns / blast pads and displaced-threshold bars.")
    ap.add_argument("--no-markers", action="store_true",
                    help="Leave out placed objects (signs, windsocks, startups, PAPI/VASI, trucks, runway ends ...).")
    ap.add_argument("--no-json", action="store_true", help="Do not write the <ICAO>_data.json dump.")
    ap.add_argument("--light-spacing", type=float, default=15.0,
                    help="Metres between light dots on a light string (default 15).")
    ap.add_argument("--list", "-l", metavar="APTDAT", help="List the airports found in a file, then exit.")
    ap.add_argument("--search", "-s", metavar="TEXT", default="",
                     help="With --list: only show airports matching TEXT (code, name, city...).")
    ap.add_argument("--gui", action="store_true", help="Force GUI mode.")
    args = ap.parse_args(argv)

    if args.gui or (args.input is None and args.list is None):
        run_gui()
        return

    if args.list:
        entries = core.list_airports(args.list)
        results, total = core.AirportIndex(entries).search(args.search, limit=10**9)
        for e in results:
            print("\t".join([e.ident, e.iata or e.faa, e.name, ", ".join(x for x in (e.city, e.country) if x)]))
        print(f"# {total} of {len(entries)} airports", file=sys.stderr)
        return

    if not args.input or not args.icao:
        ap.error("--input and --icao are required (or use --gui / --list)")

    opts = core.BuildOptions(lines=not args.no_lines, lights=not args.no_lights, boundary=not args.no_boundary,
                             network=not args.no_network, runway_extras=not args.no_runway_extras,
                             markers=not args.no_markers, light_spacing=args.light_spacing)
    try:
        generate(args.input, args.icao, args.outdir, args.format, args.thickness,
                 options=opts, write_json=not args.no_json)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------

MAX_RESULTS = 300      # rows shown in the result list at once (the full match count is displayed)


def _default_outdir() -> str:
    # not the working directory: a double-clicked exe may start somewhere unwritable
    return os.path.join(os.path.expanduser("~"), "AptDatToBlender_output")


def run_gui():
    import queue
    import threading
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("apt.dat -> Blender Layout Tool")
    root.geometry("760x960")
    root.minsize(660, 780)

    msgs = queue.Queue()      # worker threads -> UI thread
    state = {"path": None, "index": None, "results": [], "selected": None, "busy": False, "debounce": None}

    frm = ttk.Frame(root, padding=12)
    frm.pack(fill="both", expand=True)
    frm.columnconfigure(0, weight=1)

    # --- 1. file picker ---
    ttk.Label(frm, text="1. apt.dat file:").grid(row=0, column=0, sticky="w")
    path_var = tk.StringVar()
    path_entry = ttk.Entry(frm, textvariable=path_var)
    path_entry.grid(row=1, column=0, columnspan=2, sticky="we", pady=(0, 8))
    browse_btn = ttk.Button(frm, text="Browse...", command=lambda: browse_file())
    browse_btn.grid(row=1, column=2, padx=(6, 0), pady=(0, 8))

    # --- 2. airport search ---
    ttk.Label(frm, text="2. Find airport - type an ICAO / IATA code or part of the name:").grid(
        row=2, column=0, columnspan=2, sticky="w")
    count_var = tk.StringVar(value="Load an apt.dat file first")
    ttk.Label(frm, textvariable=count_var, foreground="#666666").grid(row=2, column=2, sticky="e")

    search_var = tk.StringVar()
    search_entry = ttk.Entry(frm, textvariable=search_var, state="disabled")
    search_entry.grid(row=3, column=0, columnspan=2, sticky="we")
    ttk.Button(frm, text="Clear", command=lambda: clear_search()).grid(row=3, column=2, padx=(6, 0))

    list_frame = ttk.Frame(frm)
    list_frame.grid(row=4, column=0, columnspan=3, sticky="nsew", pady=(4, 2))
    list_frame.rowconfigure(0, weight=1)
    list_frame.columnconfigure(0, weight=1)
    listbox = tk.Listbox(list_frame, height=10, exportselection=False, activestyle="dotbox",
                         font=("Consolas", 10))
    list_scroll = ttk.Scrollbar(list_frame, orient="vertical", command=listbox.yview)
    listbox.configure(yscrollcommand=list_scroll.set)
    listbox.grid(row=0, column=0, sticky="nsew")
    list_scroll.grid(row=0, column=1, sticky="ns")

    selected_var = tk.StringVar(value="Selected: (none)")
    ttk.Label(frm, textvariable=selected_var, font=("TkDefaultFont", 9, "bold")).grid(
        row=5, column=0, columnspan=3, sticky="w", pady=(0, 8))

    # --- 3. format ---
    ttk.Label(frm, text="3. Output format:").grid(row=6, column=0, sticky="w")
    fmt_var = tk.StringVar(value="both")
    fmt_frame = ttk.Frame(frm)
    fmt_frame.grid(row=7, column=0, columnspan=2, sticky="w", pady=(0, 8))
    ttk.Radiobutton(fmt_frame, text="OBJ file", variable=fmt_var, value="obj").pack(side="left")
    ttk.Radiobutton(fmt_frame, text="Blender script (.py)", variable=fmt_var, value="blender").pack(side="left", padx=(12, 0))
    ttk.Radiobutton(fmt_frame, text="Both", variable=fmt_var, value="both").pack(side="left", padx=(12, 0))

    # --- 4. thickness ---
    ttk.Label(frm, text="4. Slab thickness (meters, 0 = flat plane):").grid(row=8, column=0, sticky="w")
    thickness_var = tk.StringVar(value="0")
    ttk.Entry(frm, textvariable=thickness_var, width=10).grid(row=9, column=0, sticky="w", pady=(0, 8))

    # --- 5. what to include ---
    inc_frame = ttk.LabelFrame(frm, text="5. Include", padding=(8, 4))
    inc_frame.grid(row=10, column=0, columnspan=3, sticky="we", pady=(0, 8))
    inc_vars = {}
    inc_items = [
        ("lines", "Painted lines (taxi lines, hold lines, edge markings)"),
        ("lights", "Light strings (centre line / edge lights)"),
        ("boundary", "Airport boundary"),
        ("network", "Taxi routes + ground-vehicle routes"),
        ("runway_extras", "Runway shoulders, overruns, displaced thresholds"),
        ("markers", "Signs, windsocks, startups, PAPI/VASI, trucks, runway ends"),
        ("json", "JSON dump (frequencies, ATC flows, metadata ...)"),
    ]
    for n, (key, label) in enumerate(inc_items):
        inc_vars[key] = tk.BooleanVar(value=True)
        ttk.Checkbutton(inc_frame, text=label, variable=inc_vars[key]).grid(
            row=n // 2, column=n % 2, sticky="w", padx=(0, 16))
    ttk.Label(inc_frame, text="Light spacing (m):").grid(row=4, column=0, sticky="e", pady=(4, 0))
    spacing_var = tk.StringVar(value="15")
    ttk.Entry(inc_frame, textvariable=spacing_var, width=8).grid(row=4, column=1, sticky="w", pady=(4, 0))

    # --- 6. output dir ---
    ttk.Label(frm, text="6. Output folder:").grid(row=11, column=0, sticky="w")
    outdir_var = tk.StringVar(value=_default_outdir())
    ttk.Entry(frm, textvariable=outdir_var).grid(row=12, column=0, columnspan=2, sticky="we", pady=(0, 8))
    ttk.Button(frm, text="Browse...", command=lambda: browse_outdir()).grid(row=12, column=2, padx=(6, 0), pady=(0, 8))

    # --- generate + progress ---
    gen_btn = ttk.Button(frm, text="Generate", command=lambda: do_generate())
    gen_btn.grid(row=13, column=0, sticky="w", pady=(2, 4))
    progress_var = tk.DoubleVar(value=0.0)
    progress = ttk.Progressbar(frm, variable=progress_var, maximum=100.0, mode="determinate")
    progress.grid(row=13, column=1, columnspan=2, sticky="we", padx=(10, 0))

    # --- log box ---
    log_box = tk.Text(frm, height=9, state="disabled", bg="#111", fg="#0f0", font=("Consolas", 9))
    log_box.grid(row=14, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
    frm.rowconfigure(4, weight=3)
    frm.rowconfigure(14, weight=2)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def log(msg):
        log_box.configure(state="normal")
        log_box.insert("end", str(msg) + "\n")
        log_box.see("end")
        log_box.configure(state="disabled")

    def post(kind, *payload):          # safe to call from worker threads
        msgs.put((kind, payload))

    def set_busy(busy, indeterminate=False):
        state["busy"] = busy
        btn_state = "disabled" if busy else "normal"
        browse_btn.configure(state=btn_state)
        gen_btn.configure(state=btn_state)
        progress.stop()
        progress_var.set(0.0)
        if busy and indeterminate:
            progress.configure(mode="indeterminate")
            progress.start(12)
        else:
            progress.configure(mode="determinate")

    def row_text(e):
        return f"{e.ident:<9}{(e.iata or ''):<5}{e.name}"

    def select_index(i):
        if i is None or i >= len(state["results"]):
            state["selected"] = None
            selected_var.set("Selected: (none)")
        else:
            e = state["results"][i]
            state["selected"] = e
            extra = f"  [{e.iata}]" if e.iata else ""
            selected_var.set(f"Selected: {e.ident} - {e.name}{extra}")

    def refresh_results():
        state["debounce"] = None
        index = state["index"]
        if index is None:
            return
        query = search_var.get()
        results, total = index.search(query, MAX_RESULTS)
        state["results"] = results
        listbox.delete(0, "end")
        for e in results:
            listbox.insert("end", row_text(e))
        if query.strip():
            more = f" (showing first {len(results)})" if total > len(results) else ""
            count_var.set(f"{total:,} match{'' if total == 1 else 'es'}{more}")
            if results:                       # best match is pre-selected, so type + Generate just works
                listbox.selection_set(0)
                listbox.activate(0)
                select_index(0)
            else:
                select_index(None)
        else:
            count_var.set(f"{len(index):,} airports - type to filter")
            select_index(None)

    def on_query_change(*_):
        if state["debounce"] is not None:
            root.after_cancel(state["debounce"])
        state["debounce"] = root.after(120, refresh_results)

    search_var.trace_add("write", on_query_change)

    def clear_search():
        search_var.set("")
        search_entry.focus_set()

    def on_list_select(_event=None):
        sel = listbox.curselection()
        if sel:
            select_index(sel[0])

    listbox.bind("<<ListboxSelect>>", on_list_select)

    def on_down(_event):
        if listbox.size():
            listbox.focus_set()
            if not listbox.curselection():
                listbox.selection_set(0)
                listbox.activate(0)
                select_index(0)
        return "break"

    def on_return(_event):
        if state["debounce"] is not None:      # apply the pending filter immediately
            root.after_cancel(state["debounce"])
            refresh_results()
        gen_btn.focus_set()

    search_entry.bind("<Down>", on_down)
    search_entry.bind("<Return>", on_return)
    search_entry.bind("<Escape>", lambda e: clear_search())

    # ------------------------------------------------------------------
    # loading the file (background thread - big apt.dat files take seconds)
    # ------------------------------------------------------------------
    def load_file(p):
        if state["busy"]:
            return
        p = p.strip().strip('"')
        if not os.path.isfile(p):
            messagebox.showerror("File not found", f"Can't find:\n{p}")
            return
        state["path"] = None
        state["index"] = None
        state["results"] = []
        listbox.delete(0, "end")
        select_index(None)
        search_entry.configure(state="disabled")
        count_var.set("Scanning airports...")
        log(f"Scanning {p} ...")
        set_busy(True)

        def work():
            try:
                entries = core.list_airports(p, progress=lambda f: post("progress", f))
                post("airports", p, entries)
            except Exception as e:
                post("error", "Error reading file", str(e), traceback.format_exc())

        threading.Thread(target=work, daemon=True).start()

    def browse_file():
        p = filedialog.askopenfilename(title="Select apt.dat",
                                       filetypes=[("apt.dat / text", "*.dat *.txt"), ("All files", "*.*")])
        if p:
            path_var.set(p)
            load_file(p)

    def browse_outdir():
        d = filedialog.askdirectory(title="Select output folder")
        if d:
            outdir_var.set(d)

    path_entry.bind("<Return>", lambda e: load_file(path_var.get()))

    # ------------------------------------------------------------------
    # generating (background thread)
    # ------------------------------------------------------------------
    def do_generate():
        if state["busy"]:
            return
        if not state["path"] or state["index"] is None:
            messagebox.showwarning("No file", "Choose an apt.dat file first.")
            return
        sel = state["selected"]
        if sel is None:
            messagebox.showwarning("No airport", "Pick an airport from the list "
                                   "(type an ICAO code or part of the name to find it).")
            return
        try:
            thickness = float(thickness_var.get().strip() or "0")
        except ValueError:
            messagebox.showerror("Invalid thickness", "Slab thickness must be a number in meters, e.g. 0 or 0.15.")
            return
        if thickness < 0:
            messagebox.showerror("Invalid thickness", "Slab thickness can't be negative.")
            return
        try:
            spacing = float(spacing_var.get().strip() or "15")
            if spacing <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid spacing", "Light spacing must be a positive number of meters, e.g. 15.")
            return
        opts = core.BuildOptions(
            lines=inc_vars["lines"].get(), lights=inc_vars["lights"].get(), boundary=inc_vars["boundary"].get(),
            network=inc_vars["network"].get(), runway_extras=inc_vars["runway_extras"].get(),
            markers=inc_vars["markers"].get(), light_spacing=spacing)
        write_json = inc_vars["json"].get()
        outdir = outdir_var.get().strip() or _default_outdir()
        path, fmt, ident = state["path"], fmt_var.get(), sel.ident
        set_busy(True, indeterminate=True)

        def work():
            try:
                outputs = generate(path, ident, outdir, fmt, thickness, log=lambda m: post("log", m),
                                   options=opts, write_json=write_json)
                post("done", outputs)
            except Exception as e:
                post("error", "Error", str(e), traceback.format_exc())

        threading.Thread(target=work, daemon=True).start()

    gen_btn.configure(command=do_generate)

    # ------------------------------------------------------------------
    # worker -> UI message pump
    # ------------------------------------------------------------------
    def poll():
        try:
            while True:
                kind, payload = msgs.get_nowait()
                if kind == "log":
                    log(payload[0])
                elif kind == "progress":
                    progress_var.set(payload[0] * 100.0)
                elif kind == "airports":
                    path, entries = payload
                    set_busy(False)
                    if not entries:
                        count_var.set("No airports found")
                        messagebox.showwarning("Nothing found", "No airports were found in that file. "
                                               "Is it an X-Plane apt.dat?")
                        continue
                    state["path"] = path
                    state["index"] = core.AirportIndex(entries)
                    log(f"Loaded {len(entries):,} airport(s) from {os.path.basename(path)}")
                    search_entry.configure(state="normal")
                    search_entry.focus_set()
                    refresh_results()
                elif kind == "done":
                    set_busy(False)
                    messagebox.showinfo("Done", "Generated:\n" + "\n".join(payload[0]))
                elif kind == "error":
                    set_busy(False)
                    title, message, tb = payload
                    log("ERROR: " + message)
                    log(tb)
                    count_var.set(count_var.get() if state["index"] else "Load an apt.dat file first")
                    messagebox.showerror(title, message)
        except queue.Empty:
            pass
        root.after(60, poll)

    root.after(60, poll)
    root.mainloop()


if __name__ == "__main__":
    run_cli()
