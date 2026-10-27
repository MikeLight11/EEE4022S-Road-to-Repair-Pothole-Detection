import json
import math
import secrets
import urllib.error
import urllib.parse
import urllib.request
import uuid
import os
from datetime import datetime, timedelta, timezone
from io import BytesIO
from pathlib import Path

from flask import Flask, abort, jsonify, redirect, render_template, request, send_from_directory, session, url_for
from flask_sqlalchemy import SQLAlchemy
from PIL import Image
from sqlalchemy import case, func, inspect, select, text, update
from dotenv import load_dotenv

from suburbs import suburb_name


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
UPLOAD_DIR = DATA_DIR / "uploads"
ALLOWED_OBJECTS = {"pothole"}
REPAIR_STATUSES = ("unfilled", "filled", "not_pothole")
# What a crew can report a detection as when it turns out not to be a pothole.
NOT_POTHOLE_REASONS = {
    "patch": "Already patched or repaired",
    "crack": "Crack, not a pothole",
    "manhole": "Manhole or drain cover",
    "marking": "Shadow, stain or road marking",
    "water": "Water or puddle",
    "debris": "Debris or object on the road",
    "speed_bump": "Speed bump",
    "other": "Other",
}
MAX_JOB_SUBURBS = 200
# Duplicate detections: a new pothole within this distance of an existing unfilled one is dropped, unless the two
# were captured close together in time (two neighbouring potholes seen one after the other on the same pass).
DUPLICATE_RADIUS_M = 10
DUPLICATE_MIN_GAP = timedelta(minutes=5)
HISTORY_LIMIT = 300
# Potholes are repaired by cutting out a rectangle whose sides go up in 0.25 m steps, so materials are based on
# that cut (each side rounded up to the next step), not on the pothole's measured size.
REPAIR_CUT_STEP_MM = 250
SEARCH_RADII_M = (100, 250, 500, 1000)
SEARCH_RESULT_LIMIT = 50
# Address search uses OpenStreetMap's Nominatim, limited to the City of Cape Town (lon/lat box).
GEOCODER_URL = "https://nominatim.openstreetmap.org/search"
GEOCODER_VIEWBOX = "18.30,-33.47,19.00,-34.36"
GEOCODER_USER_AGENT = "PotholeMonitor/1.0 (UCT EEE4022S student project)"

load_dotenv(BASE_DIR / ".env")


def resolve_database_uri():
    database_url = os.getenv("DATABASE_URL", f"sqlite:///{DATA_DIR / 'potholes.db'}")
    if database_url.startswith("sqlite:///") and not database_url.startswith("sqlite:////"):
        db_path = Path(database_url.removeprefix("sqlite:///"))
        if not db_path.is_absolute():
            db_path = (BASE_DIR / db_path).resolve()
            return f"sqlite:///{db_path.as_posix()}"
    return database_url


def resolve_upload_folder():
    configured = Path(os.getenv("UPLOAD_FOLDER", str(UPLOAD_DIR)))
    if not configured.is_absolute():
        configured = (BASE_DIR / configured).resolve()
    return configured


UPLOAD_PATH = resolve_upload_folder()

app = Flask(__name__)
app.config.from_mapping(
    SECRET_KEY=os.getenv("SECRET_KEY", "dev-only-change-me"),
    SQLALCHEMY_DATABASE_URI=resolve_database_uri(),
    SQLALCHEMY_TRACK_MODIFICATIONS=False,
    TEMPLATES_AUTO_RELOAD=True,  # pick up template edits without restarting the server
    UPLOAD_FOLDER=str(UPLOAD_PATH),
    MAX_CONTENT_LENGTH=int(os.getenv("MAX_CONTENT_LENGTH", str(10 * 1024 * 1024))),
    DEVICE_API_KEY=os.getenv("DEVICE_API_KEY", "dev-device-key"),
    DASHBOARD_KEY=os.getenv("DASHBOARD_KEY", "123"),
)
db = SQLAlchemy(app)


class Job(db.Model):
    """A group of potholes exported together from the suburb selection map."""
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(200), nullable=False)
    status = db.Column(db.String(20), nullable=False, default="open", index=True)
    suburbs = db.Column(db.Text, nullable=False, default="[]")  # JSON list of the suburb names selected at export
    created_at = db.Column(db.DateTime(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc))
    # Random, unguessable token for the public job link (/j/<token>); None until a link is first created.
    share_token = db.Column(db.String(64), nullable=True, unique=True, index=True)

    def as_dict(self, pothole_count=0, area_m2=0.0, unfilled_count=0, last_marked_at=None):
        return {
            "id": self.id,
            "name": self.name,
            "status": self.status,
            "suburbs": json.loads(self.suburbs),
            "created_at": as_utc(self.created_at).isoformat(),
            "pothole_count": pothole_count,
            "area_m2": round(area_m2, 4),
            "unfilled_count": unfilled_count,
            "completed": unfilled_count == 0,  # derived: a job is complete once no potholes are left unfilled
            # When the last pothole was marked, i.e. when the job was finished (None while it's still open).
            "completed_at": as_utc(last_marked_at).isoformat() if unfilled_count == 0 and last_marked_at else None,
            "url": url_for("job_view", job_id=self.id),
            "share_url": url_for("shared_job_view", token=self.share_token, _external=True) if self.share_token else None,
        }


class Detection(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    detection_id = db.Column(db.String(120), unique=True, nullable=False, index=True)
    object_name = db.Column(db.String(30), nullable=False, index=True)
    width_mm = db.Column(db.Float, nullable=False)
    length_mm = db.Column(db.Float, nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    suburb = db.Column(db.String(120), nullable=True, index=True)  # None when outside Cape Town's suburbs
    job_id = db.Column(db.Integer, db.ForeignKey("job.id"), nullable=True, index=True)  # None while unassigned
    repair_status = db.Column(db.String(20), nullable=False, default="unfilled", server_default="unfilled", index=True)
    repair_reason = db.Column(db.String(40), nullable=True)  # NOT_POTHOLE_REASONS key when marked "not_pothole"
    repair_photo = db.Column(db.String(255), nullable=True)  # proof photo uploaded with the latest status change
    repair_updated_at = db.Column(db.DateTime(timezone=True), nullable=True)
    captured_at = db.Column(db.DateTime(timezone=True), nullable=False, index=True)
    received_at = db.Column(db.DateTime(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc))
    severity = db.Column(db.String(20), nullable=False, index=True)
    image_filename = db.Column(db.String(255), nullable=False)
    image_mimetype = db.Column(db.String(80), nullable=False)
    image_size = db.Column(db.Integer, nullable=False)

    def as_dict(self):
        return {
            "id": self.id,
            "detection_id": self.detection_id,
            "object_name": self.object_name,
            "width_mm": self.width_mm,
            "length_mm": self.length_mm,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "suburb": self.suburb,
            "job_id": self.job_id,
            "repair_status": self.repair_status,
            "repair_reason": self.repair_reason,
            "repair_reason_label": NOT_POTHOLE_REASONS.get(self.repair_reason),
            "repair_photo_url": f"/api/v1/media/{self.repair_photo}" if self.repair_photo else None,
            "repair_updated_at": as_utc(self.repair_updated_at).isoformat() if self.repair_updated_at else None,
            "captured_at": as_utc(self.captured_at).isoformat(),
            "received_at": as_utc(self.received_at).isoformat(),
            "severity": self.severity,
            "image_url": f"/api/v1/media/{self.image_filename}",
            # The rectangle actually cut out for the repair; all material estimates use this.
            "repair_cut_width_mm": repair_cut_mm(self.width_mm),
            "repair_cut_length_mm": repair_cut_mm(self.length_mm),
            "repair_area_m2": round(repair_area_m2(self.width_mm, self.length_mm), 4),
        }


class StatusChange(db.Model):
    """One repair status change on a pothole, kept as an audit trail (who, when, why, photo and where)."""
    id = db.Column(db.Integer, primary_key=True)
    detection_id = db.Column(db.Integer, db.ForeignKey("detection.id"), nullable=False, index=True)
    job_id = db.Column(db.Integer, db.ForeignKey("job.id"), nullable=True, index=True)
    old_status = db.Column(db.String(20), nullable=False)
    new_status = db.Column(db.String(20), nullable=False)
    reason = db.Column(db.String(40), nullable=True)
    photo = db.Column(db.String(255), nullable=True)
    source = db.Column(db.String(10), nullable=False)  # "crew" (job link) or "admin" (dashboard)
    # Where the crew's phone was when they submitted (None for admins, or when the phone gave no location).
    latitude = db.Column(db.Float, nullable=True)
    longitude = db.Column(db.Float, nullable=True)
    accuracy_m = db.Column(db.Float, nullable=True)
    distance_m = db.Column(db.Float, nullable=True)  # from the phone to the pothole
    created_at = db.Column(db.DateTime(timezone=True), nullable=False, default=lambda: datetime.now(timezone.utc), index=True)


def as_utc(value):
    """SQLite hands datetimes back without a timezone; everything is stored in UTC."""
    return value.replace(tzinfo=timezone.utc) if value.tzinfo is None else value


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres (haversine)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((phi2 - phi1) / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6_371_000 * math.asin(math.sqrt(a))


def find_duplicate(latitude, longitude, captured_at):
    """An existing unfilled pothole within DUPLICATE_RADIUS_M captured at least DUPLICATE_MIN_GAP apart, if any."""
    lat_margin = DUPLICATE_RADIUS_M / 111_320
    lon_margin = DUPLICATE_RADIUS_M / (111_320 * max(math.cos(math.radians(latitude)), 0.01))
    nearby = db.session.scalars(select(Detection).where(
        Detection.object_name == "pothole", Detection.repair_status == "unfilled",
        Detection.latitude.between(latitude - lat_margin, latitude + lat_margin),
        Detection.longitude.between(longitude - lon_margin, longitude + lon_margin),
    ))
    for existing in nearby:
        if (distance_m(latitude, longitude, existing.latitude, existing.longitude) <= DUPLICATE_RADIUS_M
                and abs(captured_at - as_utc(existing.captured_at)) >= DUPLICATE_MIN_GAP):
            return existing
    return None


def record_status_change(detection, old_status, source, photo=None, position=None):
    """Add a StatusChange row for a change already applied to `detection` (committed with the caller's session)."""
    change = StatusChange(detection_id=detection.id, job_id=detection.job_id, old_status=old_status,
                          new_status=detection.repair_status, reason=detection.repair_reason, photo=photo, source=source)
    if position:
        change.latitude, change.longitude, change.accuracy_m = position
        change.distance_m = round(distance_m(position[0], position[1], detection.latitude, detection.longitude), 1)
    db.session.add(change)


def crew_position(form):
    """(lat, lon, accuracy) sent by the crew's phone, or None if missing or invalid."""
    try:
        latitude, longitude = float(form["crew_latitude"]), float(form["crew_longitude"])
        accuracy = float(form["crew_accuracy"]) if form.get("crew_accuracy") else None
    except (KeyError, TypeError, ValueError):
        return None
    if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
        return None
    return latitude, longitude, accuracy


def repair_cut_mm(size_mm):
    """Round a measured side up to the next repair-cut step (a tiny tolerance keeps exact multiples as they are)."""
    return math.ceil(size_mm / REPAIR_CUT_STEP_MM - 1e-9) * REPAIR_CUT_STEP_MM


def repair_area_m2(width_mm, length_mm):
    return repair_cut_mm(width_mm) * repair_cut_mm(length_mm) / 1_000_000


def calculate_severity(width_mm, length_mm):
    area_m2 = (width_mm * length_mm) / 1_000_000
    if area_m2 >= 0.5:
        return "critical"
    if area_m2 >= 0.25:
        return "high"
    if area_m2 >= 0.1:
        return "medium"
    return "low"


def suburb_label(name):
    return " ".join(word.capitalize() for word in name.split())


def job_name(suburbs):
    labels = sorted(suburb_label(name) for name in suburbs)
    return labels[0] if len(labels) == 1 else f"{labels[0]} + {len(labels) - 1} more"


def verify_image(file_storage):
    raw = file_storage.read()
    file_storage.seek(0)
    if not raw:
        raise ValueError("Image is empty")
    try:
        with Image.open(BytesIO(raw)) as image:
            image.verify()
            mimetype = Image.MIME.get(image.format)
    except Exception as exc:
        raise ValueError("Image must be a valid JPEG or PNG") from exc
    if mimetype not in {"image/jpeg", "image/png"}:
        raise ValueError("Image must be a valid JPEG or PNG")
    return raw, mimetype


def save_image(raw, mimetype):
    """Write a verified image to the upload folder under a random name; returns (filename, path)."""
    filename = f"{uuid.uuid4().hex}.{mimetype.split('/')[-1].replace('jpeg', 'jpg')}"
    path = Path(app.config["UPLOAD_FOLDER"]) / filename
    path.write_bytes(raw)
    return filename, path


def authorized_device():
    header = request.headers.get("Authorization", "")
    return secrets.compare_digest(header, f"Bearer {app.config['DEVICE_API_KEY']}")


def bad_request(message, status=400):
    return jsonify({"error": message}), status


def dashboard_authenticated():
    return bool(session.get("dashboard_authenticated"))


@app.post("/api/v1/detections")
def create_detection():
    if not authorized_device():
        return bad_request("Authentication required", 401)
    image = request.files.get("image")
    required = ["detection_id", "object_name", "width_mm", "length_mm", "latitude", "longitude", "captured_at"]
    missing = [name for name in required if not request.form.get(name)]
    if missing or image is None:
        return bad_request(f"Missing fields: {', '.join(missing + ([] if image else ['image']))}")
    try:
        object_name = request.form["object_name"].lower()
        width_mm = float(request.form["width_mm"])
        length_mm = float(request.form["length_mm"])
        latitude = float(request.form["latitude"])
        longitude = float(request.form["longitude"])
        captured_at = datetime.fromisoformat(request.form["captured_at"].replace("Z", "+00:00"))
        if captured_at.tzinfo is None:
            raise ValueError
        if object_name not in ALLOWED_OBJECTS or not (0 < width_mm <= 10000 and 0 < length_mm <= 10000):
            raise ValueError
        if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
            raise ValueError
        raw, mimetype = verify_image(image)
    except (ValueError, TypeError):
        return bad_request("Invalid object, dimensions, coordinates, timestamp, or image")

    detection_id = request.form["detection_id"]
    existing = db.session.scalar(select(Detection).where(Detection.detection_id == detection_id))
    if existing:
        same = (existing.object_name == object_name and existing.width_mm == width_mm and existing.length_mm == length_mm
                and existing.latitude == latitude and existing.longitude == longitude)
        return (jsonify(existing.as_dict()), 200) if same else bad_request("detection_id already exists with different data", 409)

    captured_at = captured_at.astimezone(timezone.utc)
    duplicate = find_duplicate(latitude, longitude, captured_at)
    if duplicate:
        # Same pothole seen again on a later pass: acknowledge it (so the device doesn't retry) but store nothing.
        return jsonify({"duplicate": True, "duplicate_of": duplicate.id, "detection": duplicate.as_dict()}), 200

    filename, path = save_image(raw, mimetype)
    detection = Detection(
        detection_id=detection_id, object_name=object_name, width_mm=width_mm, length_mm=length_mm,
        latitude=latitude, longitude=longitude, suburb=suburb_name(latitude, longitude), captured_at=captured_at,
        severity=calculate_severity(width_mm, length_mm), image_filename=filename,
        image_mimetype=mimetype, image_size=len(raw),
    )
    try:
        db.session.add(detection)
        db.session.commit()
    except Exception:
        db.session.rollback()
        path.unlink(missing_ok=True)
        raise
    return jsonify(detection.as_dict()), 201


def new_share_token():
    return secrets.token_urlsafe(16)


def job_for_token(token):
    return db.session.scalar(select(Job).where(Job.share_token == token)) if token else None


@app.get("/api/v1/detections")
def list_detections():
    """Dashboard users can list every pothole; a share token lists only that job's potholes."""
    query = select(Detection).where(Detection.object_name == "pothole").order_by(Detection.captured_at.desc()).limit(5000)
    if request.args.get("share"):
        job = job_for_token(request.args["share"])
        if job is None:
            return bad_request("Job link not found", 404)
        return jsonify({"detections": [d.as_dict() for d in db.session.scalars(query.where(Detection.job_id == job.id))]})
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    if request.args.get("severity"):
        query = query.where(Detection.severity == request.args["severity"])
    if request.args.get("job_id", "").isdigit():
        query = query.where(Detection.job_id == int(request.args["job_id"]))
    elif request.args.get("assignment") == "unassigned":
        query = query.where(Detection.job_id.is_(None))
    elif request.args.get("assignment") == "assigned":
        query = query.where(Detection.job_id.is_not(None))
    return jsonify({"detections": [d.as_dict() for d in db.session.scalars(query)]})


EMPTY_JOB_TOTALS = (0, 0.0, 0, None)


def job_totals(job_ids):
    """Map job id -> (pothole count, repair cut area in m2, unfilled pothole count, latest repair update).

    The area is the 0.25 m-step repair cut, and leaves out detections marked "not a pothole" (nothing to fill).
    Summed in Python because the rounding up isn't available in SQLite.
    """
    totals = {}
    rows = db.session.execute(
        select(Detection.job_id, Detection.width_mm, Detection.length_mm, Detection.repair_status, Detection.repair_updated_at)
        .where(Detection.job_id.in_(job_ids))
    )
    for job_id, width_mm, length_mm, status, updated in rows:
        count, area, unfilled, last_marked = totals.get(job_id, EMPTY_JOB_TOTALS)
        if status != "not_pothole":
            area += repair_area_m2(width_mm, length_mm)
        if updated and (last_marked is None or updated > last_marked):
            last_marked = updated
        totals[job_id] = (count + 1, area, unfilled + (status == "unfilled"), last_marked)
    return totals


@app.get("/api/v1/jobs")
def list_jobs():
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    query = select(Job).order_by(Job.created_at.desc())
    if request.args.get("status"):
        query = query.where(Job.status == request.args["status"])
    jobs = db.session.scalars(query).all()
    totals = job_totals([job.id for job in jobs])
    return jsonify({"jobs": [job.as_dict(*totals.get(job.id, EMPTY_JOB_TOTALS)) for job in jobs]})


@app.post("/api/v1/jobs")
def create_job():
    """Create a job from every unassigned pothole in the given suburbs (map filters are deliberately ignored)."""
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    payload = request.get_json(silent=True) or {}
    suburbs = payload.get("suburbs")
    if (not isinstance(suburbs, list) or not suburbs or len(suburbs) > MAX_JOB_SUBURBS
            or not all(isinstance(name, str) and name.strip() for name in suburbs)):
        return bad_request(f"suburbs must be a list of 1-{MAX_JOB_SUBURBS} suburb names")
    suburbs = sorted({name.strip() for name in suburbs})

    job = Job(name=job_name(suburbs), suburbs=json.dumps(suburbs), share_token=new_share_token())
    db.session.add(job)
    db.session.flush()
    # A single UPDATE claims the potholes, so two exports can never put the same pothole in two jobs.
    claimed = db.session.execute(
        update(Detection)
        .where(Detection.object_name == "pothole", Detection.job_id.is_(None), Detection.repair_status == "unfilled",
               Detection.suburb.in_(suburbs))
        .values(job_id=job.id)
    ).rowcount
    if not claimed:
        db.session.rollback()
        return bad_request("No unassigned potholes in the selected suburbs", 409)
    db.session.commit()
    return jsonify(job.as_dict(*job_totals([job.id])[job.id])), 201


@app.delete("/api/v1/detections/<int:detection_id>")
def delete_detection(detection_id):
    if not session.get("dashboard_authenticated"):
        return bad_request("Authentication required", 401)
    detection = db.session.get(Detection, detection_id)
    if detection is None:
        return bad_request("Detection not found", 404)
    if detection.object_name != "pothole":
        return bad_request("Only potholes can be removed", 400)
    upload_folder = Path(app.config["UPLOAD_FOLDER"])
    changes = db.session.scalars(select(StatusChange).where(StatusChange.detection_id == detection.id)).all()
    names = {detection.image_filename, detection.repair_photo, *(change.photo for change in changes)}
    files = [upload_folder / name for name in names if name]
    for change in changes:
        db.session.delete(change)
    db.session.delete(detection)
    db.session.commit()
    for path in files:
        path.unlink(missing_ok=True)
    return jsonify({"deleted": detection_id})


@app.post("/api/v1/detections/<int:detection_id>/status")
def admin_update_repair_status(detection_id):
    """Dashboard admins mark a detection as not a pothole: a reason is required, a photo is not."""
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    detection = db.session.get(Detection, detection_id)
    if detection is None:
        return bad_request("Detection not found", 404)
    payload = request.get_json(silent=True) or {}
    if payload.get("status") != "not_pothole":
        return bad_request("status must be 'not_pothole'")
    if payload.get("reason") not in NOT_POTHOLE_REASONS:
        return bad_request("Choose what it is")
    old_status = detection.repair_status
    detection.repair_status = "not_pothole"
    detection.repair_reason = payload["reason"]
    detection.repair_photo = None  # a crew photo no longer matches the new status (it stays in the history)
    detection.repair_updated_at = datetime.now(timezone.utc)
    record_status_change(detection, old_status, "admin")
    db.session.commit()
    return jsonify(detection.as_dict())


@app.post("/api/v1/shared/<token>/detections/<int:detection_id>/status")
def update_repair_status(token, detection_id):
    """Job link holders mark a pothole filled, or not a pothole (with a reason); both need a proof photo."""
    job = job_for_token(token)
    if job is None:
        return bad_request("Job link not found", 404)
    detection = db.session.get(Detection, detection_id)
    if detection is None or detection.job_id != job.id:
        return bad_request("Pothole not found in this job", 404)
    if detection.repair_status != "unfilled":
        return bad_request("This pothole has already been marked", 409)
    status = request.form.get("status")
    reason = request.form.get("reason") or None
    if status not in ("filled", "not_pothole"):
        return bad_request("status must be 'filled' or 'not_pothole'")
    if status == "not_pothole" and reason not in NOT_POTHOLE_REASONS:
        return bad_request("Choose what it is")
    photo = request.files.get("photo")
    if photo is None:
        return bad_request("A photo is required")
    try:
        raw, mimetype = verify_image(photo)
    except ValueError as exc:
        return bad_request(str(exc))

    filename, path = save_image(raw, mimetype)
    old_status = detection.repair_status
    detection.repair_status = status
    detection.repair_reason = reason if status == "not_pothole" else None
    detection.repair_photo = filename
    detection.repair_updated_at = datetime.now(timezone.utc)
    record_status_change(detection, old_status, "crew", photo=filename, position=crew_position(request.form))
    try:
        db.session.commit()
    except Exception:
        db.session.rollback()
        path.unlink(missing_ok=True)
        raise
    return jsonify(detection.as_dict())


@app.get("/api/v1/healthz")
def healthz():
    return jsonify({"status": "ok"})


@app.get("/api/v1/media/<path:filename>")
def media(filename):
    return send_from_directory(app.config["UPLOAD_FOLDER"], filename)


@app.post("/api/v1/jobs/<int:job_id>/share")
def share_job(job_id):
    """Return the job's public link, creating it first for jobs made before links existed."""
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    job = db.session.get(Job, job_id)
    if job is None:
        return bad_request("Job not found", 404)
    if not job.share_token:
        job.share_token = new_share_token()
        db.session.commit()
    return jsonify({"share_url": url_for("shared_job_view", token=job.share_token, _external=True)})


@app.get("/")
def map_view():
    if not dashboard_authenticated():
        return redirect(url_for("login"))
    return render_template("map.html", mode="main", job=None, not_pothole_reasons=NOT_POTHOLE_REASONS)


@app.get("/jobs/<int:job_id>")
def job_view(job_id):
    if not dashboard_authenticated():
        return redirect(url_for("login"))
    job = db.session.get(Job, job_id) or abort(404)
    return render_template("map.html", mode="job", job=job.as_dict(*job_totals([job.id]).get(job.id, EMPTY_JOB_TOTALS)),
                           not_pothole_reasons=NOT_POTHOLE_REASONS)


@app.get("/j/<token>")
def shared_job_view(token):
    """Public job page for the people doing the repairs: no login, only this job's potholes."""
    job = job_for_token(token) or abort(404)
    details = job.as_dict(*job_totals([job.id]).get(job.id, EMPTY_JOB_TOTALS))
    details.pop("url")  # the admin job page link is no use to someone without a dashboard login
    return render_template("map.html", mode="shared", job=details, share_token=token,
                           not_pothole_reasons=NOT_POTHOLE_REASONS)


@app.get("/api/v1/detections/nearby")
def nearby_detections():
    """Potholes within `radius` metres of lat/lon (or of pothole `id`), nearest first, with their latest change."""
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    anchor = None
    if request.args.get("id", "").lstrip("#").isdigit():
        anchor = db.session.get(Detection, int(request.args["id"].lstrip("#")))
        if anchor is None or anchor.object_name != "pothole":
            return bad_request("No pothole with that ID", 404)
        latitude, longitude = anchor.latitude, anchor.longitude
    else:
        try:
            latitude, longitude = float(request.args["lat"]), float(request.args["lon"])
        except (KeyError, ValueError):
            return bad_request("Give a pothole id, or lat and lon")
    radius = request.args.get("radius", type=int)
    radius = radius if radius in SEARCH_RADII_M else 250

    lat_margin = radius / 111_320
    lon_margin = radius / (111_320 * max(math.cos(math.radians(latitude)), 0.01))
    candidates = db.session.scalars(select(Detection).where(
        Detection.object_name == "pothole",
        Detection.latitude.between(latitude - lat_margin, latitude + lat_margin),
        Detection.longitude.between(longitude - lon_margin, longitude + lon_margin),
    )).all()
    found = sorted(((distance_m(latitude, longitude, d.latitude, d.longitude), d) for d in candidates), key=lambda item: item[0])
    found = [(dist, d) for dist, d in found if dist <= radius][:SEARCH_RESULT_LIMIT]

    latest = {}  # detection id -> its most recent StatusChange
    if found:
        for change in db.session.scalars(select(StatusChange).where(StatusChange.detection_id.in_([d.id for _, d in found]))
                                         .order_by(StatusChange.created_at)):
            latest[change.detection_id] = change
    results = []
    for dist, detection in found:
        item = detection.as_dict()
        item["distance_m"] = round(dist, 1)
        change = latest.get(detection.id)
        item["last_change"] = {
            "source": change.source, "created_at": as_utc(change.created_at).isoformat(),
            "crew_distance_m": change.distance_m,
            "photo_url": f"/api/v1/media/{change.photo}" if change.photo else None,
        } if change else None
        results.append(item)
    return jsonify({"center": {"lat": latitude, "lon": longitude}, "radius_m": radius,
                    "anchor_id": anchor.id if anchor else None, "results": results})


@app.get("/api/v1/geocode")
def geocode():
    """Look up a Cape Town address with OpenStreetMap Nominatim (called server-side, so browsers only talk to us)."""
    if not dashboard_authenticated():
        return bad_request("Authentication required", 401)
    query = (request.args.get("q") or "").strip()
    if len(query) < 3:
        return bad_request("Type at least 3 characters")
    params = urllib.parse.urlencode({"q": query, "format": "jsonv2", "limit": 5, "countrycodes": "za",
                                     "viewbox": GEOCODER_VIEWBOX, "bounded": 1})
    lookup = urllib.request.Request(f"{GEOCODER_URL}?{params}", headers={"User-Agent": GEOCODER_USER_AGENT})
    try:
        with urllib.request.urlopen(lookup, timeout=8) as response:
            places = json.loads(response.read())
    except (urllib.error.URLError, TimeoutError, ValueError):
        return bad_request("Address search is unavailable right now. Try a pothole ID or click the map.", 502)
    return jsonify({"results": [{"label": place["display_name"], "lat": float(place["lat"]), "lon": float(place["lon"])}
                                for place in places]})


@app.get("/history")
def history_view():
    """Status history page: recent changes (newest first, optionally for one job) and a "find a pothole" search."""
    if not dashboard_authenticated():
        return redirect(url_for("login"))
    tab = "search" if request.args.get("tab") == "search" else "changes"
    job_filter = request.args.get("job", type=int)
    query = (select(StatusChange, Detection).join(Detection, StatusChange.detection_id == Detection.id)
             .order_by(StatusChange.created_at.desc()).limit(HISTORY_LIMIT))
    if job_filter:
        query = query.where(StatusChange.job_id == job_filter)
    labels = {"unfilled": "Unfilled", "filled": "Filled", "not_pothole": "Not a pothole"}
    entries = [{
        "created_at": as_utc(change.created_at),
        "job_id": change.job_id,
        "detection_id": detection.id,
        "suburb": suburb_label(detection.suburb) if detection.suburb else "Unknown suburb",
        "severity": detection.severity,
        "new_status": change.new_status, "new_label": labels.get(change.new_status, change.new_status),
        "reason": NOT_POTHOLE_REASONS.get(change.reason),
        "source": change.source,
        "before_url": f"/api/v1/media/{detection.image_filename}",
        "photo_url": f"/api/v1/media/{change.photo}" if change.photo else None,
        "distance_m": change.distance_m,
        "accuracy_m": change.accuracy_m,
        "maps_url": (f"https://www.google.com/maps/search/?api=1&query={change.latitude},{change.longitude}"
                     if change.latitude is not None else None),
    } for change, detection in db.session.execute(query)] if tab == "changes" else []
    jobs = db.session.scalars(select(Job).order_by(Job.id.desc())).all()
    return render_template("history.html", tab=tab, entries=entries, jobs=jobs, job_filter=job_filter,
                           limit=HISTORY_LIMIT, radii=SEARCH_RADII_M)


@app.route("/login", methods=["GET", "POST"])
def login():
    error = None
    if request.method == "POST":
        if secrets.compare_digest(request.form.get("access_key", ""), app.config["DASHBOARD_KEY"]):
            session["dashboard_authenticated"] = True
            return redirect(url_for("map_view"))
        error = "Invalid access key"
    return render_template("login.html", error=error)


@app.post("/logout")
def logout():
    session.clear()
    return redirect(url_for("login"))


DATA_DIR.mkdir(parents=True, exist_ok=True)
Path(app.config["UPLOAD_FOLDER"]).mkdir(parents=True, exist_ok=True)


def initialize_database():
    with app.app_context():
        db.create_all()
        # create_all() doesn't alter existing tables, so add newer columns to older databases.
        new_columns = {
            "detection": {
                "suburb": "VARCHAR(120)",
                "job_id": "INTEGER REFERENCES job (id)",
                "repair_status": "VARCHAR(20) NOT NULL DEFAULT 'unfilled'",
                "repair_reason": "VARCHAR(40)",
                "repair_photo": "VARCHAR(255)",
                "repair_updated_at": "DATETIME",
            },
            "job": {"share_token": "VARCHAR(64)"},
        }
        indexed_columns = {("detection", "suburb"), ("detection", "job_id"), ("detection", "repair_status"), ("job", "share_token")}
        unique_columns = {("job", "share_token")}
        inspector = inspect(db.engine)
        with db.engine.begin() as connection:
            for table, columns in new_columns.items():
                existing = {column["name"] for column in inspector.get_columns(table)}
                for column, ddl in columns.items():
                    if column not in existing:
                        connection.execute(text(f"ALTER TABLE {table} ADD COLUMN {column} {ddl}"))
                        if (table, column) in indexed_columns:
                            unique = "UNIQUE " if (table, column) in unique_columns else ""
                            connection.execute(text(f"CREATE {unique}INDEX IF NOT EXISTS ix_{table}_{column} ON {table} ({column})"))
        missing = db.session.scalars(select(Detection).where(Detection.suburb.is_(None))).all()
        for detection in missing:
            detection.suburb = suburb_name(detection.latitude, detection.longitude)
        if missing:
            db.session.commit()


initialize_database()


if __name__ == "__main__":
    app.run(debug=os.getenv("FLASK_DEBUG", "0") == "1")
