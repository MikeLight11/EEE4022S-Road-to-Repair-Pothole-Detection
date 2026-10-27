#!/usr/bin/env python3
"""
eval_quant.py
=============
Compares quantised model variants in the Hailo emulator, before spending an
hour or more compiling each one. Run with the Dataflow Compiler's Python (the
venv compile_hef.sh builds):

    hef_build/venv/bin/python eval_quant.py hef_build/calib hef_build/*.har

For each HAR (saved by compile_hef.sh, e.g. with OPTIMIZE_ONLY=1), the grille
video frames from the calibration set are padded to 640x640 as on the Pi and
run through the emulator twice: as the float model (SDK_FP_OPTIMIZED) and as
the quantised one (SDK_QUANTIZED). For every anchor where the float model
scores a pothole in a confidence band, it reports what the quantised model
scores. The first HEF squashed the float 0.4-0.6 band to ~0.2 (on the Pi, in
bench/eval_hef.py); the variant that keeps that band nearest the float scores
is the one to compile.

These frames were also part of the calibration set, so absolute numbers are a
little optimistic; the comparison between variants is what counts.
"""

import glob
import sys
from pathlib import Path

import cv2
import numpy as np
from hailo_sdk_client import ClientRunner, InferenceContext

NC = 2           # Manhole Cover, Pothole
POTHOLE = 1
N_FRAMES = 150
BANDS = [(0.4, 0.5), (0.5, 0.6), (0.6, 0.7), (0.7, 1.01)]


def letterbox_640(path):
    """640x360 RGB frame centred on grey (114) padding, as img_yuv420_letterbox on the Pi."""
    im = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
    h, w = im.shape[:2]
    out = np.full((640, 640, 3), 114, np.uint8)
    top, left = round((640 - h) / 2 - 0.1), round((640 - w) / 2 - 0.1)
    out[top:top + h, left:left + w] = im
    return out


def pothole_scores(runner, data, context):
    """Pothole score of every anchor (all three strides), shape (frames, anchors)."""
    with runner.infer_context(context) as ctx:
        outs = runner.infer(ctx, data)
    if isinstance(outs, dict):
        outs = list(outs.values())
    if not isinstance(outs, (list, tuple)):
        outs = [outs]
    cls = sorted((o for o in outs if o.shape[-1] == NC), key=lambda o: o.shape[1])
    if len(cls) != 3:
        sys.exit(f"expected 3 class outputs with {NC} channels, got shapes {[o.shape for o in outs]}")
    return np.concatenate([c[..., POTHOLE].reshape(len(data), -1) for c in cls], axis=1)


def main():
    calib, hars = Path(sys.argv[1]), sys.argv[2:]
    frames = sorted(glob.glob(str(calib / "images" / "val" / "ir-cam-vid-*.jpg")))
    if not frames:
        sys.exit(f"no grille-video frames in {calib}/images/val")
    frames = frames[:: max(1, len(frames) // N_FRAMES)][:N_FRAMES]
    data = np.stack([letterbox_640(p) for p in frames]).astype(np.float32)   # 0-255: the HAR normalises
    print(f"{len(frames)} grille-video frames, padded to 640x640 as on the Pi\n")

    rows = []
    for har in hars:
        runner = ClientRunner(har=har)
        fp = pothole_scores(runner, data, InferenceContext.SDK_FP_OPTIMIZED).ravel()
        q = pothole_scores(runner, data, InferenceContext.SDK_QUANTIZED).ravel()
        name = Path(har).stem.replace("best_seg_hailo_model", "plain").replace("plain_", "")
        print(f"== {name}: quantised score where the float model scores")
        cells = []
        for lo, hi in BANDS:
            sel = (fp >= lo) & (fp < hi)
            if not sel.any():
                print(f"   {lo:.1f}-{min(hi, 1):.1f}: no anchors")
                cells.append(None)
                continue
            med, keep = float(np.median(q[sel])), float((q[sel] >= 0.4).mean() * 100)
            print(f"   {lo:.1f}-{min(hi, 1):.1f}: n={sel.sum():6d}  median {med:.3f} (float {np.median(fp[sel]):.3f})"
                  f"  >= 0.4: {keep:5.1f}%")
            cells.append(keep)
        rows.append((name, cells))
        print()

    print("Summary: % of anchors still >= 0.4 after quantisation, by float band")
    print(f"  {'variant':<18}" + "".join(f"{f'{lo:.1f}-{min(hi, 1):.1f}':>10}" for lo, hi in BANDS))
    for name, cells in rows:
        print(f"  {name:<18}" + "".join(f"{c:>9.1f}%" if c is not None else f"{'-':>10}" for c in cells))
    borderline = [(sum(c for c in cells[:2] if c is not None), name) for name, cells in rows]
    best = max(borderline)[1]
    print(f"\nKeeps the borderline 0.4-0.6 band best: {best}")


if __name__ == "__main__":
    main()
