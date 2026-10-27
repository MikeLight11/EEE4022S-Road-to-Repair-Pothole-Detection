"""One-off: shrink the City of Cape Town "Official Planning Suburbs" GeoJSON for the web map.

Usage (from the Webapp folder):
    pip install shapely          # pyproj too, only if the source isn't already WGS84
    python scripts/prepare_suburbs.py [input.geojson] [output.geojson] [--tolerance 0.00002]

Keeps only the suburb name and ID, simplifies the boundaries, reprojects to
EPSG:4326 if needed, rounds coordinates, and writes static/data/suburbs.geojson.
"""
import argparse
import json
import re
from pathlib import Path

from shapely.geometry import MultiPolygon, mapping, shape
from shapely.ops import transform
from shapely.validation import make_valid

BASE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = BASE_DIR / "Official_Planning_Suburbs.geojson"
DEFAULT_OUTPUT = BASE_DIR / "static" / "data" / "suburbs.geojson"
NAME_FIELD = "OFC_SBRB_NAME"
ID_FIELD = "SL_OFC_SBRB_KEY"
WGS84_NAMES = {"urn:ogc:def:crs:OGC:1.3:CRS84", "urn:ogc:def:crs:EPSG::4326", "EPSG:4326", "CRS84"}
PRECISION = 5  # ~1.1 m at Cape Town's latitude


def source_crs(collection):
    """Return the CRS name declared in the file, or None if it's already lon/lat WGS84."""
    name = (collection.get("crs") or {}).get("properties", {}).get("name")
    return None if name is None or name in WGS84_NAMES else name


def round_coords(coords):
    if isinstance(coords[0], (int, float)):
        return [round(value, PRECISION) for value in coords[:2]]
    return [round_coords(part) for part in coords]


def clean_name(name):
    return re.sub(r"\s+", " ", str(name or "")).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--tolerance", type=float, default=0.00002,
                        help="simplify tolerance in degrees (default 0.00002, roughly 2 m)")
    args = parser.parse_args()

    collection = json.loads(args.input.read_text(encoding="utf-8"))
    crs = source_crs(collection)
    reproject = None
    if crs:
        from pyproj import Transformer
        reproject = Transformer.from_crs(crs, "EPSG:4326", always_xy=True).transform
        print(f"Reprojecting from {crs} to EPSG:4326")

    features, warnings = [], []
    for feature in collection["features"]:
        props = feature.get("properties") or {}
        name, suburb_id = clean_name(props.get(NAME_FIELD)), props.get(ID_FIELD)
        if not feature.get("geometry"):
            warnings.append(f"skipped {name or suburb_id!r}: no geometry")
            continue
        if not name or suburb_id is None:
            warnings.append(f"missing name or ID: name={name!r} id={suburb_id!r}")
        if name != props.get(NAME_FIELD):
            warnings.append(f"cleaned whitespace in name {props.get(NAME_FIELD)!r}")

        geometry = shape(feature["geometry"])
        if reproject:
            geometry = transform(reproject, geometry)
        if not geometry.is_valid:
            warnings.append(f"repaired invalid geometry for {name!r}")
            geometry = make_valid(geometry)
            # make_valid can return a GeometryCollection with stray lines; keep only the polygons.
            if geometry.geom_type == "GeometryCollection":
                polys = [g for g in geometry.geoms if g.geom_type in ("Polygon", "MultiPolygon")]
                geometry = MultiPolygon([p for g in polys for p in getattr(g, "geoms", [g])])
        geometry = geometry.simplify(args.tolerance, preserve_topology=True)

        geojson_geometry = mapping(geometry)
        features.append({
            "type": "Feature",
            "properties": {"id": suburb_id, "name": name},
            "geometry": {"type": geojson_geometry["type"], "coordinates": round_coords(geojson_geometry["coordinates"])},
        })

    features.sort(key=lambda f: f["properties"]["name"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"type": "FeatureCollection", "features": features}, separators=(",", ":")),
                           encoding="utf-8")

    for warning in warnings:
        print(f"  warning: {warning}")
    before, after = args.input.stat().st_size, args.output.stat().st_size
    print(f"{len(features)} suburbs written to {args.output}")
    print(f"Size: {before / 1024 / 1024:.2f} MB -> {after / 1024 / 1024:.2f} MB ({after / before:.0%} of original)")


if __name__ == "__main__":
    main()
