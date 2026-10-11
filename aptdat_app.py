"""
aptdat_app.py
-------------
apt.dat -> Blender/OBJ airport layout tool.

GUI mode (default, double-click the .exe / run with no arguments):
    A window lets you pick an apt.dat file, pick an ICAO from a dropdown,
    choose OBJ / Blender-script / both, and generate.

CLI mode (for scripting / automation):
    python aptdat_app.py --input apt.dat --icao KNPA --outdir out --format both
"""

import os
import sys
import argparse
import traceback

import aptdat_core as core
import aptdat_export as export


# --------------------------------------------------------------------------
# Shared "do the work" function used by both CLI and GUI
# --------------------------------------------------------------------------

def generate(input_path: str, icao: str, outdir: str, fmt: str, thickness: float, log=print):
    log(f"Reading {input_path} ...")
    data = core.parse_airport(input_path, icao)
    if data is None:
        raise ValueError(f"ICAO '{icao}' was not found in this apt.dat file.")

    log(f"Found {data.icao} - {data.name}")
    log(f"  {len(data.runways)} runway(s), {len(data.pavements)} pavement polygon(s), "
        f"{len(data.helipads)} helipad(s)")

    meshes = core.build_meshes(data, slab_thickness=thickness)
    total_v = sum(len(m.vertices) for m in meshes.values())
    total_f = sum(len(m.faces) for m in meshes.values())
    log(f"  Built {len(meshes)} mesh groups ({total_v} verts, {total_f} tris)")

    os.makedirs(outdir, exist_ok=True)
    outputs = []

    if fmt in ("obj", "both"):
        obj_path = os.path.join(outdir, f"{data.icao}_layout.obj")
        export.export_obj(meshes, obj_path, data)
        outputs.append(obj_path)
        log(f"  Wrote {obj_path}")

    if fmt in ("blender", "both"):
        py_path = os.path.join(outdir, f"{data.icao}_layout_blender.py")
        export.export_blender_script(meshes, py_path, data)
        outputs.append(py_path)
        log(f"  Wrote {py_path}")

    log("Done.")
    return outputs


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def run_cli(argv=None):
    ap = argparse.ArgumentParser(description="Convert an X-Plane apt.dat airport layout to OBJ / Blender script.")
    ap.add_argument("--input", "-i", help="Path to apt.dat")
    ap.add_argument("--icao", "-c", help="ICAO code to extract, e.g. KNPA")
    ap.add_argument("--outdir", "-o", default="output", help="Output directory (default: ./output)")
    ap.add_argument("--format", "-f", choices=["obj", "blender", "both"], default="both",
                     help="Output format (default: both)")
    ap.add_argument("--thickness", "-t", type=float, default=0.0,
                     help="Slab thickness in meters. 0 = flat plane (default).")
    ap.add_argument("--list", "-l", metavar="APTDAT", help="List all ICAO codes found in a file, then exit.")
    ap.add_argument("--gui", action="store_true", help="Force GUI mode.")
    args = ap.parse_args(argv)

    if args.gui or (args.input is None and args.list is None):
        run_gui()
        return

    if args.list:
        for icao, name in core.list_airports(args.list):
            print(f"{icao}\t{name}")
        return

    if not args.input or not args.icao:
        ap.error("--input and --icao are required (or use --gui / --list)")

    try:
        generate(args.input, args.icao, args.outdir, args.format, args.thickness)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------

def run_gui():
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("apt.dat -> Blender Layout Tool")
    root.geometry("620x520")

    state = {"path": None, "airports": []}

    frm = ttk.Frame(root, padding=12)
    frm.pack(fill="both", expand=True)

    # --- file picker ---
    ttk.Label(frm, text="1. apt.dat file:").grid(row=0, column=0, sticky="w")
    path_var = tk.StringVar()
    path_entry = ttk.Entry(frm, textvariable=path_var, width=55)
    path_entry.grid(row=1, column=0, columnspan=2, sticky="we", pady=(0, 8))

    def browse_file():
        p = filedialog.askopenfilename(title="Select apt.dat",
                                        filetypes=[("apt.dat / text", "*.dat *.txt"), ("All files", "*.*")])
        if not p:
            return
        path_var.set(p)
        state["path"] = p
        try:
            airports = core.list_airports(p)
        except Exception as e:
            messagebox.showerror("Error reading file", str(e))
            return
        state["airports"] = airports
        icao_values = [f"{icao}  -  {name}" for icao, name in airports]
        icao_combo["values"] = icao_values
        if icao_values:
            icao_combo.current(0)
        log(f"Loaded {len(airports)} airport(s) from {os.path.basename(p)}")

    ttk.Button(frm, text="Browse...", command=browse_file).grid(row=1, column=2, padx=(6, 0))

    # --- ICAO picker ---
    ttk.Label(frm, text="2. Airport (ICAO):").grid(row=2, column=0, sticky="w")
    icao_combo = ttk.Combobox(frm, width=53, state="readonly")
    icao_combo.grid(row=3, column=0, columnspan=2, sticky="we", pady=(0, 8))

    # --- format ---
    ttk.Label(frm, text="3. Output format:").grid(row=4, column=0, sticky="w")
    fmt_var = tk.StringVar(value="both")
    fmt_frame = ttk.Frame(frm)
    fmt_frame.grid(row=5, column=0, columnspan=2, sticky="w", pady=(0, 8))
    ttk.Radiobutton(fmt_frame, text="OBJ file", variable=fmt_var, value="obj").pack(side="left")
    ttk.Radiobutton(fmt_frame, text="Blender script (.py)", variable=fmt_var, value="blender").pack(side="left", padx=(12, 0))
    ttk.Radiobutton(fmt_frame, text="Both", variable=fmt_var, value="both").pack(side="left", padx=(12, 0))

    # --- thickness ---
    ttk.Label(frm, text="4. Slab thickness (meters, 0 = flat plane):").grid(row=6, column=0, sticky="w")
    thickness_var = tk.StringVar(value="0")
    ttk.Entry(frm, textvariable=thickness_var, width=10).grid(row=7, column=0, sticky="w", pady=(0, 8))

    # --- output dir ---
    ttk.Label(frm, text="5. Output folder:").grid(row=8, column=0, sticky="w")
    outdir_var = tk.StringVar(value=os.path.join(os.getcwd(), "output"))
    ttk.Entry(frm, textvariable=outdir_var, width=55).grid(row=9, column=0, columnspan=2, sticky="we", pady=(0, 8))

    def browse_outdir():
        d = filedialog.askdirectory(title="Select output folder")
        if d:
            outdir_var.set(d)

    ttk.Button(frm, text="Browse...", command=browse_outdir).grid(row=9, column=2, padx=(6, 0))

    # --- log box ---
    log_box = tk.Text(frm, height=14, width=70, state="disabled", bg="#111", fg="#0f0", font=("Consolas", 9))
    log_box.grid(row=11, column=0, columnspan=3, sticky="nsew", pady=(10, 0))
    frm.rowconfigure(11, weight=1)

    def log(msg):
        log_box.configure(state="normal")
        log_box.insert("end", str(msg) + "\n")
        log_box.see("end")
        log_box.configure(state="disabled")
        root.update_idletasks()

    # --- generate button ---
    def do_generate():
        if not state["path"]:
            messagebox.showwarning("No file", "Choose an apt.dat file first.")
            return
        if not icao_combo.get():
            messagebox.showwarning("No airport", "Choose an ICAO from the list.")
            return
        icao = icao_combo.get().split("  -  ")[0].strip()
        try:
            thickness = float(thickness_var.get())
        except ValueError:
            thickness = 0.0
        try:
            outputs = generate(state["path"], icao, outdir_var.get(), fmt_var.get(), thickness, log=log)
            messagebox.showinfo("Done", "Generated:\n" + "\n".join(outputs))
        except Exception as e:
            log("ERROR: " + str(e))
            log(traceback.format_exc())
            messagebox.showerror("Error", str(e))

    ttk.Button(frm, text="Generate", command=do_generate).grid(row=10, column=0, sticky="w", pady=(4, 4))

    frm.columnconfigure(0, weight=1)
    frm.columnconfigure(1, weight=1)

    root.mainloop()


if __name__ == "__main__":
    run_cli()
