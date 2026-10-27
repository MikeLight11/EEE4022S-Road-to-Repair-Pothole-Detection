#!/usr/bin/env python3
"""
make_calibration_set.py
=======================
Prepares the files needed to compile best_seg.pt into a Hailo-8 HEF.
Run on the Mac, from the "Pi C code" folder:

    python3 hailo/make_calibration_set.py

Writes into hailo/:
  calib.zip          INT8 calibration images (images/val/*.jpg, no labels).
                     Half are random training images, half are frames spread
                     evenly across the OV5647 grille videos, so the
                     quantisation ranges cover both what the model learned
                     from and the camera it runs on.
  test_frame.jpg     A full-res grille frame with a known pothole.
  test_frame.json    PyTorch (best_seg.pt) detections on that frame, as a
                     reference for checking the HEF's output on the Pi.

Images are resized to a longest side of 640 px; the exporter letterboxes
them to 640x640 exactly as at inference, so nothing is lost.
"""

import argparse
import json
import random
import shutil
import tempfile
import zipfile
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
SRC = Path.home() / "Desktop" / "Pothole Detection"
DATASET = SRC / "Pothole Detection v2 -Potholes and Manholes-.v1i.yolov11" / "train" / "images"
VIDEOS = [SRC / f"ir-cam-vid-{i}.mov" for i in (1, 2, 3, 4)]
TEST_VIDEO, TEST_FRAME = SRC / "ir-cam-vid-4.mov", 4707   # 1-based, as in the seg pipeline's records


def save_small(img, path, max_side=640):
    h, w = img.shape[:2]
    s = max_side / max(h, w)
    if s < 1:
        img = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(path), img, [cv2.IMWRITE_JPEG_QUALITY, 95])


def read_frame(video, index0):
    cap = cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_POS_FRAMES, index0)
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=1024, help="calibration images (default 1024, as in the report)")
    ap.add_argument("--weights", default=str(SRC / "best_seg.pt"))
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    random.seed(args.seed)

    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "images" / "val"
        out.mkdir(parents=True)

        n_video = args.n // 2
        n_data = args.n - n_video
        images = sorted(p for p in DATASET.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
        for i, p in enumerate(random.sample(images, n_data)):
            img = cv2.imread(str(p))
            if img is not None:
                save_small(img, out / f"data_{i:04d}.jpg")
        print(f"{n_data} training images")

        per_video = n_video // len(VIDEOS) + 1
        count = 0
        for v in VIDEOS:
            cap = cv2.VideoCapture(str(v))
            total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            # Evenly spaced; sequential reads with grab() are much faster than seeking .mov files.
            wanted = {round((k + 0.5) * total / per_video) for k in range(per_video)}
            idx = 0
            while count < n_video and cap.grab():
                if idx in wanted:
                    ok, frame = cap.retrieve()
                    if ok:
                        save_small(frame, out / f"{v.stem}_{idx:05d}.jpg")
                        count += 1
                idx += 1
            cap.release()
            print(f"{v.name}: {count} frames so far")

        zpath = HERE / "calib.zip"
        with zipfile.ZipFile(zpath, "w", zipfile.ZIP_STORED) as z:   # JPEGs don't compress further
            for p in sorted(out.iterdir()):
                z.write(p, f"images/val/{p.name}")
        print(f"wrote {zpath} ({len(list(out.iterdir()))} images, {zpath.stat().st_size / 1e6:.0f} MB)")

    frame = read_frame(TEST_VIDEO, TEST_FRAME - 1)
    if frame is None:
        raise SystemExit(f"could not read frame {TEST_FRAME} of {TEST_VIDEO}")
    cv2.imwrite(str(HERE / "test_frame.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

    from ultralytics import YOLO
    model = YOLO(args.weights)
    r = model.predict(str(HERE / "test_frame.jpg"), conf=0.1, verbose=False)[0]
    dets = [
        dict(cls=model.names[int(b.cls)], conf=round(float(b.conf), 3),
             xyxy=[round(v, 1) for v in b.xyxy[0].tolist()])
        for b in r.boxes
    ]
    (HERE / "test_frame.json").write_text(json.dumps(
        dict(source=f"{TEST_VIDEO.name} frame {TEST_FRAME}", weights=Path(args.weights).name,
             size=[frame.shape[1], frame.shape[0]], detections=dets), indent=1))
    print(f"wrote test_frame.jpg + test_frame.json ({len(dets)} PyTorch detections >= 0.1):")
    for d in dets:
        print(f"  {d['cls']:<14} {d['conf']:.3f}  {d['xyxy']}")
    shutil.copy(args.weights, HERE / "best_seg.pt")
    print(f"copied {Path(args.weights).name} -> hailo/best_seg.pt")


if __name__ == "__main__":
    main()
