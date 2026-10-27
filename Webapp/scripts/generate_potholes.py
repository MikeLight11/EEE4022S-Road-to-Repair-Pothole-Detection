"""Generate simulated pothole detections at random GPS points across Cape Town.

Usage (from the Webapp folder):
    python scripts/generate_potholes.py 50                  # insert 50 potholes into the local database
    python scripts/generate_potholes.py 50 --seed 1         # repeatable run
    python scripts/generate_potholes.py 20 --url http://127.0.0.1:5000   # post to a running server

Each pothole goes through the real POST /api/v1/detections endpoint (in-process via Flask's test
client, or over HTTP with --url), so the server assigns the suburb and severity exactly as it
would for a real detection.
"""
import argparse
import json
import math
import os
import random
import sys
import urllib.error
import urllib.request
import uuid
from collections import Counter
from datetime import datetime, timedelta, timezone
from io import BytesIO
from pathlib import Path

from dotenv import load_dotenv
from PIL import Image, ImageDraw, ImageFilter

BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))
load_dotenv(BASE_DIR / ".env")

from suburbs import random_point  # noqa: E402


def fake_pothole_image(rng):
    """A rough asphalt texture with a dark irregular blob, so the dashboard previews look plausible."""
    size = (320, 240)
    image = Image.effect_noise(size, 28).convert("RGB")
    image = Image.blend(image, Image.new("RGB", size, (92, 92, 96)), 0.6)
    draw = ImageDraw.Draw(image)
    cx, cy = rng.randint(110, 210), rng.randint(90, 150)
    points = []
    for i in range(14):
        radius = rng.uniform(40, 85)
        angle = i / 14 * 6.2832
        points.append((cx + radius * 1.3 * math.cos(angle), cy + radius * 0.8 * math.sin(angle)))
    draw.polygon(points, fill=(38, 36, 34))
    image = image.filter(ImageFilter.GaussianBlur(1.2))
    buffer = BytesIO()
    image.save(buffer, format="JPEG", quality=80)
    return buffer.getvalue()


def random_detection(rng, days):
    latitude, longitude, suburb = random_point(rng)
    width = rng.randint(150, 1000)
    captured_at = datetime.now(timezone.utc) - timedelta(seconds=rng.uniform(0, days * 86400))
    fields = {
        "detection_id": f"sim-{uuid.UUID(int=rng.getrandbits(128)).hex}",
        "object_name": "pothole",
        "width_mm": str(width),
        "length_mm": str(int(width * rng.uniform(0.7, 1.6))),
        "latitude": str(latitude),
        "longitude": str(longitude),
        "captured_at": captured_at.isoformat(timespec="seconds"),
    }
    return fields, suburb.name


class TestClientSender:
    def __init__(self):
        from app import app
        self.client = app.test_client()
        self.headers = {"Authorization": f"Bearer {app.config['DEVICE_API_KEY']}"}

    def send(self, fields, image):
        data = {**fields, "image": (BytesIO(image), "pothole.jpg", "image/jpeg")}
        response = self.client.post("/api/v1/detections", data=data, headers=self.headers,
                                    content_type="multipart/form-data")
        return response.status_code, response.get_json()


class HttpSender:
    def __init__(self, url, api_key):
        self.url = url.rstrip("/") + "/api/v1/detections"
        self.api_key = api_key

    def send(self, fields, image):
        boundary = uuid.uuid4().hex
        body = BytesIO()
        for name, value in fields.items():
            body.write(f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode())
        body.write(f'--{boundary}\r\nContent-Disposition: form-data; name="image"; filename="pothole.jpg"\r\n'
                   f"Content-Type: image/jpeg\r\n\r\n".encode() + image + f"\r\n--{boundary}--\r\n".encode())
        request = urllib.request.Request(self.url, data=body.getvalue(), method="POST", headers={
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": f"multipart/form-data; boundary={boundary}",
        })
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read() or b"{}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("count", type=int, help="number of potholes to generate")
    parser.add_argument("--seed", type=int, help="random seed for a repeatable set of potholes")
    parser.add_argument("--days", type=float, default=30, help="spread capture times over the last N days (default 30)")
    parser.add_argument("--url", help="post to a running server (e.g. http://127.0.0.1:5000) instead of the local database")
    parser.add_argument("--api-key", help="device API key for --url (default: DEVICE_API_KEY from .env)")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("count must be at least 1")

    rng = random.Random(args.seed)
    sender = HttpSender(args.url, args.api_key or os.getenv("DEVICE_API_KEY", "dev-device-key")) if args.url else TestClientSender()

    created, suburbs = 0, Counter()
    for index in range(1, args.count + 1):
        fields, expected_suburb = random_detection(rng, args.days)
        status, result = sender.send(fields, fake_pothole_image(rng))
        if status == 200 and result.get("duplicate"):
            print(f"[{index}/{args.count}] skipped: duplicate of pothole #{result['duplicate_of']}")
            continue
        if status != 201:
            print(f"[{index}/{args.count}] failed ({status}): {result.get('error', result)}")
            continue
        created += 1
        suburb = result.get("suburb") or "(no suburb)"
        suburbs[suburb] += 1
        print(f"[{index}/{args.count}] {suburb:<32} {fields['latitude']:>11}, {fields['longitude']:<10} "
              f"{result['severity']}")
        if result.get("suburb") != expected_suburb:
            print(f"    note: generator picked {expected_suburb}, server assigned {result.get('suburb')}")

    print(f"\nCreated {created} of {args.count} potholes across {len(suburbs)} suburbs.")
    for suburb, count in suburbs.most_common(10):
        print(f"  {count:>4}  {suburb}")


if __name__ == "__main__":
    main()
