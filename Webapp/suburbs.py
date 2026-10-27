"""Offline GPS -> suburb lookup for the City of Cape Town, using static/data/suburbs.geojson.

Pure Python (no shapely) so the web app needs no extra dependencies. Points outside every
suburb polygon fall back to the nearest suburb boundary within FALLBACK_RADIUS_M, which covers
the small gaps left by boundary simplification and roads that run along suburb borders.
"""
import json
import math
import random
from functools import lru_cache
from pathlib import Path

SUBURBS_PATH = Path(__file__).resolve().parent / "static" / "data" / "suburbs.geojson"
FALLBACK_RADIUS_M = 150
METRES_PER_DEG_LAT = 111_320
# Longitude degrees shrink with latitude; Cape Town sits around 34 degrees south.
METRES_PER_DEG_LON = METRES_PER_DEG_LAT * math.cos(math.radians(34))


class Suburb:
    def __init__(self, suburb_id, name, polygons):
        self.id = suburb_id
        self.name = name
        self.polygons = polygons  # list of polygons; each polygon is [outer_ring, *hole_rings] of (lon, lat)
        lons = [lon for polygon in polygons for lon, _ in polygon[0]]
        lats = [lat for polygon in polygons for _, lat in polygon[0]]
        self.bbox = (min(lons), min(lats), max(lons), max(lats))

    def in_bbox(self, lon, lat, margin=0.0):
        min_lon, min_lat, max_lon, max_lat = self.bbox
        return min_lon - margin <= lon <= max_lon + margin and min_lat - margin <= lat <= max_lat + margin

    def contains(self, lon, lat):
        return any(_in_ring(lon, lat, polygon[0]) and not any(_in_ring(lon, lat, hole) for hole in polygon[1:])
                   for polygon in self.polygons)

    def distance_m(self, lon, lat):
        return min(_ring_distance_m(lon, lat, ring) for polygon in self.polygons for ring in polygon)


def _in_ring(lon, lat, ring):
    """Ray-casting point-in-polygon test."""
    inside = False
    x1, y1 = ring[-1]
    for x2, y2 in ring:
        if (y1 > lat) != (y2 > lat) and lon < (x2 - x1) * (lat - y1) / (y2 - y1) + x1:
            inside = not inside
        x1, y1 = x2, y2
    return inside


def _ring_distance_m(lon, lat, ring):
    best = math.inf
    points = [((x - lon) * METRES_PER_DEG_LON, (y - lat) * METRES_PER_DEG_LAT) for x, y in ring]
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, -(x1 * dx + y1 * dy) / length_sq))
        best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return best


@lru_cache(maxsize=1)
def load_suburbs(path=SUBURBS_PATH):
    collection = json.loads(Path(path).read_text(encoding="utf-8"))
    suburbs = []
    for feature in collection["features"]:
        geometry = feature["geometry"]
        polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        polygons = [[[tuple(point[:2]) for point in ring] for ring in polygon] for polygon in polygons]
        suburbs.append(Suburb(feature["properties"]["id"], feature["properties"]["name"], polygons))
    return suburbs


def find_suburb(latitude, longitude):
    """Return the Suburb containing the point, the nearest one within FALLBACK_RADIUS_M, or None."""
    suburbs = load_suburbs()
    for suburb in suburbs:
        if suburb.in_bbox(longitude, latitude) and suburb.contains(longitude, latitude):
            return suburb
    margin = FALLBACK_RADIUS_M / METRES_PER_DEG_LON
    nearby = [(suburb.distance_m(longitude, latitude), suburb) for suburb in suburbs
              if suburb.in_bbox(longitude, latitude, margin)]
    distance, suburb = min(nearby, key=lambda item: item[0], default=(math.inf, None))
    return suburb if distance <= FALLBACK_RADIUS_M else None


def suburb_name(latitude, longitude):
    suburb = find_suburb(latitude, longitude)
    return suburb.name if suburb else None


def random_point(rng=random):
    """Return a random (latitude, longitude, Suburb) inside a Cape Town suburb.

    Picks a suburb first, then a point within it, so small urban suburbs are as likely as the
    huge rural farm areas that would dominate a uniform-by-area sample.
    """
    suburb = rng.choice(load_suburbs())
    min_lon, min_lat, max_lon, max_lat = suburb.bbox
    while True:
        lon, lat = rng.uniform(min_lon, max_lon), rng.uniform(min_lat, max_lat)
        if suburb.contains(lon, lat):
            return round(lat, 6), round(lon, 6), suburb
