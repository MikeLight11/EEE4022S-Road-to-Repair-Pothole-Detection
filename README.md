# Road to Repair: Automatic Pothole Detection, Classification and Sizing

**EEE4022S Final Year Project, University of Cape Town, 2026**
Michael Lighton · Supervisor: Dr Stephen Paine · Department of Electrical Engineering

Webapp - https://potholevision.pythonanywhere.com/login (password = "pothole")

## Abstract

Potholes are a constant problem on South African roads. Municipalities mostly find out about them through public complaints and manual inspections, which are slow and say little about how large each pothole is, making it difficult to plan repairs and estimate the materials needed. This project set out to automate the whole process, from the moment a vehicle starts driving to the moment a pothole is repaired. It developed a complete pipeline made up of a low-cost, vehicle-mounted device that detects, measures and locates potholes using only a single camera, and a web application that turns each detection into a repair job.

The pipeline starts when the engine does. The device is powered from the vehicle's OBD2 port and turns itself on and off by monitoring the battery voltage, so the driver never has to interact with it. A Raspberry Pi 5 with a Hailo-8 accelerator processes video from a camera on the front grille. YOLO11 models, trained on public datasets, detect potholes and manhole covers, and a SORT-based tracker links detections across frames so that each pothole is normally reported only once. Inverse perspective mapping then converts the pothole's outline into a real-world length and width. Each detection is tagged with its GPS position and uploaded over 4G, and is queued on the device if coverage is lost. In the web application, administrators group potholes into repair jobs by suburb, and crews confirm each repair with a photograph. These photographs also provide labelled images from the device's own viewpoint, which feed back into retraining the models.

Detection and sizing were first tested offline on recorded road video at 20–30 km/h. The models had a recall of 0.68–0.86 and a precision of 0.35–0.56, and most false detections were caused by shadows and old repairs. Segmentation masks measured width far more accurately than bounding boxes. On normal road, the grille camera's mean error against the full pothole was 8.6% in width and 12.6% in length. Sizing depends strongly on the camera tilt, which must be known to within about 0.25° to keep length errors below 5%. The full pipeline was then run live on the Raspberry Pi 5 in an end-to-end drive. Precision rose to 0.83 at a recall of 0.60, with no false detections from shadows or old repairs, and the live sizes had mean errors of 9.5% in width and 9.8% in length. Potholes were placed a median of 6.9 m from their true position, and 29 of 30 reports reached the web application, including those queued during a 136 s gap in 4G coverage.

These results show that the complete pipeline works, taking a pothole from a live detection on a moving vehicle to a located, measured repair job on the web application. Because each stage is separate, each can be improved without changing the rest. A global shutter camera would give sharper images free of rolling-shutter distortion. The camera's IMU could track changes in tilt over time to make sizing more accurate. Retraining on the repair photographs would improve detection, and the longer the system is used, the more of these photographs it collects.

## Project details

The system has three parts, shown in [`system_overview.pdf`](system_overview.pdf).

- **Camera unit** on the front grille, with an IMU to flag sudden camera movement.
- **Processing unit** in the cabin, a Raspberry Pi 5 with a Hailo-8 accelerator that detects, tracks, measures and locates each pothole, powered from the OBD2 port with automatic on and off.
- **Web application** where potholes are mapped, grouped into repair jobs and confirmed by crews.

## Repository layout

| Folder | Contents |
|---|---|
| `Raspberry Pi 5 Full Pipeline C code` | Live detection and measurement pipeline on the Pi 5 and Hailo-8 |
| `Python Yolo Model (video input)` | Offline detection, tracking and sizing used for the recorded road tests |
| `Python Sizing Algorithm (photo input)` | Camera calibration and early sizing tests |
| `ESP32 Power Sensing Code` | Firmware for ignition detection and power control |
| `ESP32 Camera Motion Code` | Firmware for the camera unit's IMU |
| `Webapp` | Repair management web application |
| `Enclosures CAD Files & 3D Printing Files` | Grille mount, camera housing and enclosure designs |
| `Schematics` | Circuit schematics |
| `Ethics` | Ethics pre-screening outcome |
