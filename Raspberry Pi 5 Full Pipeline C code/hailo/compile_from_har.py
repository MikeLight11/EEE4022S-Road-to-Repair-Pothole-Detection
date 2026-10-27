#!/usr/bin/env python3
"""
compile_from_har.py
===================
Compiles a quantised model saved by compile_hef.sh (hef_build/<variant>.har)
into a HEF, without quantising again. Run with the Dataflow Compiler's Python:

    hef_build/venv/bin/python compile_from_har.py hef_build/best_seg_hailo_model_a16cls.har

Writes <variant>.zip next to the HAR, holding best_seg.hef and a metadata.yaml
like Ultralytics' export (the Pi reads hailo_arch from it for its logs).
"""

import os
import sys
import zipfile
from pathlib import Path

from hailo_sdk_client import ClientRunner

os.environ.setdefault("USER", "root")   # the compiler reads $USER, which Colab leaves unset

har = Path(sys.argv[1])
runner = ClientRunner(har=str(har))
# The allocator setting from compile_hef.sh, in case the HAR doesn't carry it (Hailo's yolov11n_seg.alls).
try:
    runner.load_model_script("allocator_param(automatic_reshapes=disabled)")
except Exception as e:  # already part of the saved script
    print(f"(allocator setting not re-applied: {e})")

print(f"compiling {har.name} for {runner.hw_arch} (this is the slow part)...", flush=True)
hef = runner.compile()

out = har.with_suffix(".zip")
meta = (f"task: segment\nimgsz:\n- 640\n- 640\nnames:\n  0: Manhole Cover\n  1: Pothole\n"
        f"hailo_arch: {runner.hw_arch}\nnms: false\nsource_har: {har.name}\n")
with zipfile.ZipFile(out, "w") as z:
    z.writestr("best_seg.hef", hef)
    z.writestr("metadata.yaml", meta)
print(f"Done: {out} ({len(hef) / 1e6:.1f} MB HEF)")
