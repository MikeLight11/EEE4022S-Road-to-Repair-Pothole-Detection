"""
Convert a Roboflow YOLO *detection* export into a YOLO *segmentation*
dataset, using SAM to turn each hand-labelled box into a polygon.

The original box labels are used as SAM prompts (not best.pt's
predictions), so detector mistakes don't leak into the new labels. The
train/valid/test split is kept as-is, so a seg model trained on the
output is directly comparable to the existing detection model.

Expected input (Roboflow "YOLOv8"/"YOLO11" export):

    dataset/
        data.yaml
        train/images/*.jpg   train/labels/*.txt
        valid/images/...     valid/labels/...
        test/images/...      test/labels/...

Existing polygon labels with at least --keep-min-points vertices are kept
as-is (hand-drawn outlines beat SAM). Boxes, and polygons with fewer points
(e.g. 4-point rotated boxes), are re-segmented with SAM using their
bounding rectangle as the prompt.

Output has the same layout, with polygon labels instead of boxes. If SAM
fails on a box (empty or tiny mask) the box itself is written as a
4-point polygon so no object is silently dropped; those are counted and
listed in fallbacks.txt so you can fix them by hand in Roboflow.

Usage:
    python3 convert_to_seg.py path/to/dataset path/to/dataset-seg
    python3 convert_to_seg.py in out --preview 40   # also save overlays to spot-check
"""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

SPLITS = ("train", "valid", "test")
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def read_labels(txt_path: Path):
    """Return list of (cls, points) with points an (N, 2) array in normalised
    coords: N == 2 for a box (xc, yc / w, h), N >= 3 for a polygon."""
    rows = []
    if not txt_path.exists():
        return rows
    for line in txt_path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        vals = np.array(parts[1:], dtype=np.float32)
        if len(vals) != 4 and (len(vals) < 6 or len(vals) % 2):
            raise ValueError(f"{txt_path}: can't parse label line {line!r}")
        rows.append((int(parts[0]), vals.reshape(-1, 2)))
    return rows


def label_to_xyxy(pts, w, h):
    """Pixel bounding rectangle of a box or polygon label."""
    if len(pts) == 2:
        (xc, yc), (bw, bh) = pts
        return ((xc - bw / 2) * w, (yc - bh / 2) * h, (xc + bw / 2) * w, (yc + bh / 2) * h)
    xs, ys = pts[:, 0] * w, pts[:, 1] * h
    return (xs.min(), ys.min(), xs.max(), ys.max())


def box_polygon(xyxy):
    x1, y1, x2, y2 = xyxy
    return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)


def mask_to_polygon(mask: np.ndarray, epsilon_px: float):
    """Largest external contour of a binary mask, lightly simplified."""
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 16:
        return None
    contour = cv2.approxPolyDP(contour, epsilon_px, True)
    if len(contour) < 3:
        return None
    return contour.reshape(-1, 2).astype(np.float32)


def draw_preview(img, boxes_xyxy, polygons, fell_back, names, out_path: Path):
    vis = img.copy()
    overlay = img.copy()
    for (cls, xyxy), poly, fb in zip(boxes_xyxy, polygons, fell_back):
        color = (0, 0, 255) if fb else (0, 200, 0)
        cv2.fillPoly(overlay, [poly.astype(np.int32)], color)
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 160, 0), 2)
        label = names.get(cls, str(cls)) + (" (FALLBACK)" if fb else "")
        cv2.putText(vis, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 160, 0), 2)
    vis = cv2.addWeighted(overlay, 0.35, vis, 0.65, 0)
    cv2.imwrite(str(out_path), vis)


def convert(src: Path, dst: Path, sam_weights: str, device, epsilon_px: float, preview: int,
            keep_min_points: int):
    from ultralytics import SAM

    data = yaml.safe_load((src / "data.yaml").read_text())
    names = data["names"]
    names = dict(enumerate(names)) if isinstance(names, list) else {int(k): v for k, v in names.items()}

    sam = SAM(sam_weights)
    dst.mkdir(parents=True, exist_ok=True)
    fallback_log = []
    stats = {"images": 0, "objects": 0, "kept": 0, "fallbacks": 0}
    previews_left = preview

    for split in SPLITS:
        img_dir = src / split / "images"
        if not img_dir.is_dir():
            continue
        out_img_dir = dst / split / "images"
        out_lbl_dir = dst / split / "labels"
        out_img_dir.mkdir(parents=True, exist_ok=True)
        out_lbl_dir.mkdir(parents=True, exist_ok=True)

        images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXTS)
        for i, img_path in enumerate(images, 1):
            print(f"\r[{split}] {i}/{len(images)}", end="", flush=True)
            shutil.copy2(img_path, out_img_dir / img_path.name)
            rows = read_labels(src / split / "labels" / (img_path.stem + ".txt"))
            out_txt = out_lbl_dir / (img_path.stem + ".txt")
            stats["images"] += 1
            if not rows:
                out_txt.write_text("")
                continue

            img = cv2.imread(str(img_path))
            h, w = img.shape[:2]
            boxes = [(cls, label_to_xyxy(pts, w, h)) for cls, pts in rows]
            keep = [len(pts) >= keep_min_points for _, pts in rows]

            to_sam = [xyxy for (_, xyxy), k in zip(boxes, keep) if not k]
            masks = []
            if to_sam:
                result = sam(img, bboxes=to_sam, device=device, verbose=False, save=False)[0]
                masks = list(result.masks.data.cpu().numpy()) if result.masks is not None else []

            lines, polygons, fell_back = [], [], []
            sam_idx = 0
            for j, ((cls, xyxy), (_, pts), k) in enumerate(zip(boxes, rows, keep)):
                fb = False
                if k:
                    poly = pts * [w, h]
                    stats["kept"] += 1
                else:
                    poly = None
                    if sam_idx < len(masks):
                        mask = masks[sam_idx]
                        if mask.shape != (h, w):
                            mask = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
                        poly = mask_to_polygon(mask > 0, epsilon_px)
                    sam_idx += 1
                    if poly is None:
                        fb = True
                        poly = pts * [w, h] if len(pts) >= 3 else box_polygon(xyxy)
                        stats["fallbacks"] += 1
                        fallback_log.append(f"{split}/images/{img_path.name}  object {j}  ({names.get(cls, cls)})")
                polygons.append(poly)
                fell_back.append(fb)
                norm = np.clip(poly / [w, h], 0.0, 1.0).reshape(-1)
                lines.append(f"{cls} " + " ".join(f"{v:.6f}" for v in norm))
                stats["objects"] += 1

            out_txt.write_text("\n".join(lines) + "\n")

            if previews_left > 0:
                prev_dir = dst / "previews"
                prev_dir.mkdir(exist_ok=True)
                draw_preview(img, boxes, polygons, fell_back, names, prev_dir / f"{split}_{img_path.name}")
                previews_left -= 1
        print()

    data_out = {k: v for k, v in data.items() if k not in ("train", "val", "test", "path")}
    data_out.update({
        "path": str(dst.resolve()),
        "train": "train/images",
        "val": "valid/images",
    })
    if (dst / "test" / "images").is_dir():
        data_out["test"] = "test/images"
    (dst / "data.yaml").write_text(yaml.safe_dump(data_out, sort_keys=False))
    (dst / "fallbacks.txt").write_text("\n".join(fallback_log) + ("\n" if fallback_log else ""))

    print(f"Done: {stats['images']} images, {stats['objects']} objects, "
          f"{stats['kept']} existing polygons kept, {stats['fallbacks']} fell back to box polygons (see fallbacks.txt)")
    if preview:
        print(f"Spot-check overlays in {dst / 'previews'} "
              "(green = SAM polygon, red = fallback, orange = original box)")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("src", type=Path, help="Roboflow YOLO detection export folder (contains data.yaml)")
    p.add_argument("dst", type=Path, help="Output folder for the segmentation dataset")
    p.add_argument("--sam", default="sam2.1_b.pt", help="SAM weights (auto-downloaded), e.g. sam2.1_b.pt, sam_b.pt")
    p.add_argument("--device", default=None, help="e.g. mps, cpu, 0 (default: auto)")
    p.add_argument("--epsilon", type=float, default=1.5,
                   help="Polygon simplification tolerance in pixels (higher = fewer points)")
    p.add_argument("--keep-min-points", type=int, default=5,
                   help="Keep existing polygon labels with at least this many points; re-segment smaller ones")
    p.add_argument("--preview", type=int, default=30, help="Number of overlay images to save for spot-checking")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.src.resolve() == args.dst.resolve():
        raise SystemExit("dst must be different from src (the original labels are the comparison baseline)")
    convert(args.src, args.dst, args.sam, args.device, args.epsilon, args.preview, args.keep_min_points)
