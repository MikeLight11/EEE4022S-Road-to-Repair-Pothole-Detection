"""
check_focal_lengths.py
========================
Checks the EXIF focal length of every photo in your calibration folder
(and/or test photo folder) and flags any that don't match the rest --
a mismatch usually means the phone silently switched lenses on that shot,
which would corrupt calibration if mixed in with the others.

Usage:
    python check_focal_lengths.py ./calib_photos
"""

import sys
import glob
from PIL import Image
from PIL.ExifTags import TAGS


def get_focal_length(path):
    img = Image.open(path)
    exif = img._getexif()
    if not exif:
        return None, None
    tags = {TAGS.get(k, k): v for k, v in exif.items()}
    return tags.get("FocalLength"), tags.get("LensModel")


def check_folder(folder):
    images = (glob.glob(f"{folder}/*.jpg") + glob.glob(f"{folder}/*.jpeg")
              + glob.glob(f"{folder}/*.JPG") + glob.glob(f"{folder}/*.JPEG"))

    if not images:
        print(f"No jpg files found in {folder}")
        return

    print(f"Checking {len(images)} photos in {folder}\n")
    results = []
    for path in sorted(images):
        f, lens = get_focal_length(path)
        results.append((path, f, lens))
        print(f"  {path:40s}  FocalLength={f!s:>8}  Lens={lens}")

    focal_lengths = [f for _, f, _ in results if f is not None]
    missing = [p for p, f, _ in results if f is None]

    print()
    if missing:
        print(f"WARNING: {len(missing)} photo(s) had no EXIF focal length "
              f"at all (metadata likely stripped in transfer):")
        for p in missing:
            print(f"    {p}")
        print()

    if focal_lengths:
        unique_vals = set(focal_lengths)
        if len(unique_vals) == 1:
            print(f"All photos with EXIF data use FocalLength={unique_vals.pop()} "
                  f"-- consistent, good.")
        else:
            print(f"WARNING: multiple different focal lengths found: {unique_vals}")
            print("Photos with a different focal length than the majority "
                  "likely used a different lens/zoom and should be removed "
                  "from the calibration set:\n")
            from collections import Counter
            counts = Counter(focal_lengths)
            majority_val = counts.most_common(1)[0][0]
            for path, f, lens in results:
                if f is not None and f != majority_val:
                    print(f"    {path}  (FocalLength={f}, expected {majority_val})")


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else "./calib_photos"
    check_folder(folder)
