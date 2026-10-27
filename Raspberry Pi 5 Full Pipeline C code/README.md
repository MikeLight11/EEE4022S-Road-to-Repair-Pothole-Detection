# Pothole detector – Raspberry Pi 5

On-vehicle software for the Road to Repair edge device (Pi 5 + Hailo-8 AI HAT+).
It's being built up one stage at a time:

| Step | What | Status |
|---|---|---|
| 1 | Camera capture + livestream over the Pi's own hotspot | done |
| 2a | Hailo-8 set up, `best_seg.pt` compiled to a HEF and checked on the Pi (0.831 vs PyTorch 0.859, 13.8 ms) | done |
| 2b | Detection on every camera frame, masks + boxes drawn on the stream (30 fps, 18.7 ms/frame) | done |
| 3 | Tracking (min 5 high-confidence detections), ROI, measurement, one record + still per pothole | done |
| 4a | GPS + 4G upload, tested with the SIM7600X 4G HAT (section 13) | **this** |
| 4b | ATGM336H GPS (10 Hz) + Air780EU 4G for the final build | |
| 5 | Power-control lines, IMU, installation calibration | |

## Language

Everything is C except `src/camera.cpp`. libcamera, the Pi's camera stack, only has a
C++ API, and on the Pi 5 the raw V4L2 device gives unprocessed Bayer data with no ISP.
So that file is a thin wrapper behind the C header `src/camera.h`. No Python is needed.

## Layout

```
src/camera.h/.cpp   libcamera capture: two ISP streams, main (full res) + lores (stream)
src/jpeg.h/.c       YUV420 -> JPEG (libjpeg-turbo; the Pi 5 has no hardware encoder)
src/stream.h/.c     HTTP server: /, /stream (MJPEG), /snapshot.jpg, /status
src/main.c          capture loop
src/hailo_model.h/.c   runs a HEF on the Hailo-8 (HailoRT C API), outputs as float tensors
src/yolo_seg.h/.c      decodes the raw YOLO11-seg head: DFL boxes, NMS, mask coefficients
src/image.h/.c         letterbox to 640x640, identical to Ultralytics'
src/hailo_probe.c      checks a HEF on the Pi: tensors, timing, test-frame detections
src/overlay.h/.c       masks, boxes and labels drawn on the stream (built-in 5x7 font)
src/sysstats.h/.c      RAM / CPU / temperature / clock / throttling from /proc and /sys
src/runlog.h/.c        per-boot CSV log, logs/boot_NNNN.csv (see step 10)
src/geometry.h/.c      measure_polygon on the road + the OpenCV parts it used (outline trace, minAreaRect)
src/tracker.h/.c       SORT + ByteTrack + DIoU tracker, best-mask selection (pothole_tracker.py)
src/potholes.h/.c      per frame: ROI filter, outlines, sizes, tracking, holding best-frame buffers
src/recorder.h/.c      background writer: detections/<id>.jpg + <id>.json (the upload queue), located by GPS
src/gps.h/.c           NMEA reader thread, fixes on the camera's clock, nearest-fix lookup + forward shift
src/uploader.h/.c      sends each record to the web app (libcurl multipart POST), files it in sent/
src/serial.h/.c        serial ports + AT commands
src/sim7600_test.c     bench checks for the SIM7600X HAT: ports, signal, GPS, data link, one upload
uplink.conf.example    GPS ports, upload URL, API key; copy to uplink.conf on the Pi
bench/replay*.{py,c}   checks the C tracker + geometry against the Python pipeline on a video
bench/review_detections.py  after a drive: annotated stills, gallery page and CSV of every pothole
pothole.conf.example   camera height / tilt / intrinsics; copy to pothole.conf on the Pi
src/calibrate.h/.c     stream page Calibrate button: captures 30 frames, runs calib/estimate_pose.py
calib/estimate_pose.py checkerboard -> camera height / tilt / roll (OpenCV; the one Python part on the Pi)
bench/analyse_logs.py  summarises and plots those logs on the Mac (4 GB / 13 TOPS question)
setup/setup_pi.sh   packages, build, systemd service, hotspot
hailo/              compiling best_seg.pt -> best_seg.hef (runs off the Pi, see step 7)
```

The ISP scales the livestream frame for free. JPEG encoding only runs while someone is
watching, and is capped at `--stream-fps`. The main stream (1920×1080) is captured
but unused for now; detection plugs in there.

## 1. Hardware

- **Camera cable:** the Pi 5 has the small 22-pin camera connectors. The OV5647's
  standard 15-pin cable won't fit, so you need a 22-pin to 15-pin ("standard to mini")
  cable. Contacts face the board on the Pi end.
- **With the AI HAT+ fitted**, feed the camera ribbon through the slot in the HAT before
  you seat it. Either CAM/DISP port works.
- **Power:** use the official 27 W (5 V 5 A) supply on the bench. Lower-rated supplies
  limit USB current and can throttle the Pi.

## 2. Flash the SD card

In Raspberry Pi Imager, choose **Raspberry Pi OS (64-bit) Lite** (no desktop needed).
Before flashing, check Raspberry Pi's AI HAT+ documentation for the OS release that
`hailo-all` supports. That way you won't have to reflash at step 2.

In the Imager settings (the gear icon / "Edit settings"):
- hostname `pothole`, plus a username and password
- enable SSH
- add your home Wi-Fi and set the country to ZA (for internet during setup)

## 3. First boot and camera check

```bash
ssh <user>@pothole.local
sudo apt update && sudo apt full-upgrade -y && sudo reboot

rpicam-hello --list-cameras      # should list ov5647
rpicam-still -o test.jpg         # take a still (copy it back with scp to look at it)
```

If no camera is listed, it's nearly always the ribbon: the wrong way round, not fully
seated, or the wrong cable.

## 4. Copy, build and install

From the Mac (copy to a path without spaces, because the service file can't handle them):

```bash
rsync -av --exclude build --exclude hailo/calib.zip --exclude bench/logs --exclude detections --exclude logs --exclude calib/run "/Users/michaellighton/Desktop/Pi C code/" admin@pothole-pi.local:~/pothole/
```

(`uplink.conf`, with the API key, is created on the Pi and isn't overwritten by this.)

On the Pi, preferably over **Ethernet**: the last step turns wlan0 into a hotspot,
which drops your home Wi-Fi:

```bash
cd ~/pothole
sudo HOTSPOT_SSID=pothole-pi HOTSPOT_PSK=potholecam ./setup/setup_pi.sh
```

This installs the build tools, `libcamera-dev` and `libturbojpeg0-dev`, then builds the
code, installs and starts `pothole.service`, and creates the hotspot.

## 5. Watch the stream

Join Wi-Fi **pothole-pi** (password `potholecam`) and open **http://10.42.0.1:8000**.
The status line shows the camera fps, skipped frames and viewers. You can still SSH in
over the hotspot with `ssh <user>@10.42.0.1`.

## 6. Hailo-8 AI HAT+ (on the Pi)

This needs internet. Use Ethernet, or `sudo nmcli con down pothole-hotspot` while you install.

```bash
sudo apt install -y dkms hailo-all
sudo reboot
hailortcli fw-control identify    # should show Device Architecture: HAILO8
hailortcli --version              # 4.23.0 -> the HEF must be compiled with DFC 3.33
```

If `identify` finds no device, check that the HAT's PCIe ribbon is fully seated at both ends.

## 7. Compile best_seg.pt -> best_seg.hef (once, off the Pi)

Hailo's Dataflow Compiler only runs on Linux x86_64, so it can't run on the Mac or the
Pi. Google Colab works. The compile uses Ultralytics' Hailo export: INT8, 640×640,
calibrated on 1024 images.

1. **Calibration set** (done already; re-run only if the model changes):
   `python3 hailo/make_calibration_set.py`. This writes `hailo/calib.zip`, which is
   half training images and half frames from the OV5647 grille videos. It also writes
   `hailo/test_frame.jpg/.json` and copies in `hailo/best_seg.pt`.
2. **Get the compiler:** in the [Hailo Developer Zone](https://hailo.ai/developer-zone/)
   (free account), go to Software Downloads → Dataflow Compiler and download the
   **3.33** wheel, the version that matches HailoRT 4.23.
3. **Colab:** make a Google Drive folder `hailo_compile` and put these files in it:
   `best_seg.pt`, `calib.zip`, the wheel, and `compile_hef.sh`, `eval_quant.py` and
   `compile_from_har.py` from `hailo/`. Then open `hailo/compile_hef_colab.ipynb` in
   Colab, select **Runtime → Change runtime type → T4 GPU**, and run all cells. It
   quantises a few versions of the model, keeps the one whose pothole scores are
   closest to `best_seg.pt`, and compiles only that one (about 2–2.5 h in all). When it
   finishes, `best_seg_hailo_model.zip` appears in the Drive folder.
   (Alternatively, on any Ubuntu x86_64 PC with an NVIDIA GPU:
   `./hailo/compile_hef.sh hailo/best_seg.pt hailo/calib.zip hailo_dataflow_compiler-3.33.0-*.whl`.)

## 8. Check the HEF on the Pi

```bash
# Mac
scp best_seg_hailo_model.zip admin@pothole-pi.local:~/pothole/
# Pi
cd ~/pothole && unzip -o best_seg_hailo_model.zip -d models
make probe
./build/hailo_probe models/best_seg.hef hailo/test_frame.jpg
```

Expected output:
- 10 outputs: box (×64), class (×2) and mask (×32) maps at 80/40/20, plus 160×160×32
  prototypes. Class ranges should lie within [0, 1], since the sigmoid runs on the chip.
- One **Pothole ≈ 0.86** near **[851, 674, 1184, 812]**. That's the PyTorch result in
  `hailo/test_frame.json`. INT8 shifts it slightly; a few hundredths is fine.
- The inference time, which sets the frame rate available to the pipeline.

## 9. Live detection on the stream

With `models/best_seg.hef` in place, `build/pothole` runs the model on every camera
frame and draws masks, boxes and confidences on the livestream. The top-left line
shows the detection fps, the inference time and the number of detections:

```bash
# Mac
rsync -av --exclude build --exclude hailo/calib.zip --exclude bench/logs --exclude detections --exclude logs --exclude calib/run "/Users/michaellighton/Desktop/Pi C code/" admin@pothole-pi.local:~/pothole/
# Pi
cd ~/pothole && make && sudo systemctl restart pothole
journalctl -u pothole -f          # "detect: models/best_seg.hef ..." then stats every 10 s
```

Then open http://10.42.0.1:8000. Each frame is processed as follows:
1. The ISP's 640×360 image is converted from YUV to RGB.
2. It's padded to 640×640 to make the model input.
3. The camera buffer is released, and inference runs on the Hailo-8.
4. Boxes and masks are decoded. Boxes are kept in 1920×1080 coordinates for the
   tracking and measurement to come.

Options: `--conf 0.25` (lowest confidence drawn), `--hef PATH`, `--no-detect`. If the HEF
is missing or won't load, the stream carries on without detection.

## 10. Benchmark: is a 4 GB Pi + Hailo-8L (13 TOPS) enough?

**Logging is always on.** Every second, `build/pothole` appends a line to
`logs/boot_NNNN.csv`, and each power-on starts a new file (a service restart within
the same boot appends to that boot's file). There's no RTC battery and no network time
in the car, so the wall clock can be wrong; `uptime_s` is the reliable time column.
Each line records:
- **fps:** camera, detection, and `capacity_fps` (1000 ÷ processing time per frame,
  i.e. what one-frame-at-a-time processing could reach with an unlimited camera)
- **Timings:** prep, inference and total per frame
- **RAM:** system RAM used (MemTotal − MemAvailable), the process's current and peak
  memory, CMA (camera and Hailo buffers) and swap
- **System:** CPU (total and the pothole process), SoC temperature, ARM clock and
  throttle flags

Analyse the logs on the Mac. This prints a summary and a 4 GB verdict, and saves a
figure next to each log:
```bash
rsync -av admin@pothole-pi.local:~/pothole/logs/ "/Users/michaellighton/Desktop/Pi C code/bench/logs/"
python3 "/Users/michaellighton/Desktop/Pi C code/bench/analyse_logs.py" "/Users/michaellighton/Desktop/Pi C code/bench/logs"/boot_*.csv
```
The verdict counts 4 GB as enough if peak system RAM stays below 80% of the roughly
4048 MB a 4 GB Pi 5 gives Linux, and swap is never used.

**RAM: tested directly.** Add `total_mem=4096` under `[all]` in
`/boot/firmware/config.txt` and reboot. `free -m` then shows about 4 GB, and the Pi
behaves like a 4 GB board; the SoC and memory speed are the same, so only capacity
differs. Drive or run it like that, then remove the line and reboot.

**Maximum detection fps (above the camera's 30).** `--uncapped` doesn't wait for the
camera. When no new frame is ready it reprocesses the latest one, so `det_fps` shows the
real maximum while `camera_fps` stays at 30. Use it for benchmarking only, since it
keeps the CPU and Hailo busy:
```bash
sudo systemctl stop pothole
./build/pothole --uncapped                                    # Hailo-8 HEF, logs as usual
./build/pothole --uncapped --hef models/hailo8l/best_seg.hef  # Hailo-8L HEF
```

**13 TOPS: inferred, not simulated.** Build the Hailo-8L HEF in Colab with
`!HAILO_ARCH=hailo8l bash compile_hef.sh best_seg.pt calib.zip "$WHEEL"`, then on the Pi
run `unzip -o best_seg_hailo_model_hailo8l.zip -d models/hailo8l`. Run it on the Hailo-8
(normal and `--uncapped`), and compare the HEFs on their own with no camera:
`hailortcli benchmark models/best_seg.hef` and
`hailortcli benchmark models/hailo8l/best_seg.hef` (stop the service first). Support this
with Hailo's published Model Zoo figures for `yolov11n_seg` at 640×640 (v2.20):
Hailo-8 132 / 287 fps and Hailo-8L 145 / 317 fps, at batch 1 / 8.

Suggested runs, at least 5 minutes each with a phone on the stream:

| Run | RAM | HEF | Mode |
|---|---|---|---|
| A | 8 GB | Hailo-8 | normal, then `--uncapped` |
| B | 4 GB (`total_mem`) | Hailo-8 | normal |
| C | 4 GB (`total_mem`) | Hailo-8L | normal, then `--uncapped` (closest to the cheap build) |

## 11. Tracking, measurement and pothole records

`build/pothole` now does the whole on-vehicle pipeline from the report, with the same
steps and settings as `detect_and_measure_seg.py`:
1. **ROI:** detections are kept only if their whole box is inside the 11 m region of
   interest (`--max-distance`).
2. **Mask and outline:** each detection's mask is built as Ultralytics does
   (`process_mask` with upsampling, then `crop_mask`). Its outer outline is traced and
   mapped to 1920×1080. The outline counts as cut off if it comes within 10 px of the
   image border.
3. **Sizing:** `measure_polygon` projects every outline point onto the road with the
   OV5647 calibration from `calibration.py`. Width and length are the extents; area and
   the minimum-area rectangle are also recorded.
4. **Tracking:** SORT with Kalman filters, DIoU + Hungarian matching gated at −0.2, and
   ByteTrack's low-confidence pass. Detections ≥ 0.4 start tracks and those ≥ 0.1 extend
   them. A track closes after 0.1 s unmatched, and is reported only with at least **5**
   high-confidence detections (`--min-hits`).
5. **Best frame:** the largest measurable mask that isn't cut off.

**Records** go to `detections/` (this is the upload queue for step 4). The ID is the
first 8 hex digits of the boot ID plus the track number:
- `<id>.jpg`: the full-resolution (1920×1080) best frame, with this pothole's mask outline
  drawn on it (thick yellow, no fill, so the road inside stays visible)
- `<id>.json`: sizes, confidence, frames, the outline in pixels and on the road, the
  calibration, and the monotonic timestamp of the best frame (for GPS matching later)

Each file is written to a temporary name and then renamed, JSON last, so a power cut
can't leave a half-written record.

**Stills without stalling the camera.** The best frame changes nearly every frame as
the car approaches, and copying a 3 MB frame out of uncached camera memory takes about
12 ms. So each track holds the camera buffer of its current best frame instead (which
is why the default is `--buffers 8`). A background thread copies it and encodes the
JPEG once the pothole is finalised.

**On the stream:**
- a dashed yellow line marks the ROI's far edge
- **green**: a confirmed pothole, labelled `#id W×L M` with the best size so far
- **red**: a detection at or above `start_conf` (a hit), counting towards `min_hits`
- **orange** (thin box): seen but below `start_conf`, so it can extend a track but
  never confirm one. A marker that stays red or orange never reached `min_hits`.
- **grey**: outside the ROI

`start_conf` and `min_hits` can be set in `pothole.conf` (or with `--start-conf` and
`--min-hits`). The report used 0.4 / 5, but the INT8 HEF scores borderline potholes
lower than PyTorch did. On the Hailo's replay of `ir-cam-vid-4`, **0.30 / 4** recovered
26 of the 34 clearer potholes, against 15 at 0.4 / 5; `bench/eval_hef.py` measures this
for any HEF.
- the top line shows the number of potholes saved

**Checked against the Python pipeline on the Mac:**
- The outline tracer matches `cv2.findContours` on 300 random masks.
- Mask → outline → `measure_polygon` on the test frame matches Ultralytics plus
  `seg_geometry.py` to 5 decimal places.
- To compare the tracker on a whole video:
  ```bash
  python3 bench/replay_python.py ~/Desktop/"Pothole Detection"/ir-cam-vid-4.mov /tmp/replay
  cc -O2 -Isrc bench/replay.c src/geometry.c src/tracker.c -lm -o /tmp/replay/replay
  /tmp/replay/replay /tmp/replay/ir-cam-vid-4_dets.txt > /tmp/replay/c.jsonl
  python3 bench/compare_replay.py /tmp/replay/ir-cam-vid-4_seg_potholes.json /tmp/replay/c.jsonl
  ```

## 12. Road test (no GPS / 4G yet)

On the road, every pothole is stored on the Pi in `~/pothole/detections/` as
`<id>.jpg` + `<id>.json`. Nothing is deleted until the 4G upload exists, so bring the
data home and review it on the Mac.

**Before driving**
1. **Calibrate the mount.** Sizes depend strongly on it: about 19% length error per
   degree of tilt. On the Pi, first create the config file: `cp -n pothole.conf.example pothole.conf`.
   Then lay the A3 checkerboard (8×6 inner corners, 38.5 mm squares) flat on the road,
   fully in view, and press **Calibrate** under the stream:
   1. It captures 30 frames in about 5 s.
   2. It runs `calib/estimate_pose.py`, the report's method (frames over 1 px
      reprojection error rejected, median of the rest). This takes about 1–2 minutes.
   3. It shows height, tilt, roll and the lens-to-board distance, with an image of the
      detected corners and the horizon line.

   Check the numbers with a tape measure, then press **Apply and restart**. That writes
   `h_mm` and `tilt_deg` to `pothole.conf`, and the service comes back with them; the log
   prints the calibration in use. If roll is more than about 1°, level the mount and
   calibrate again. To do it by hand instead, measure the lens height `h` and the distance
   `d` along the road to the point under the yellow crosshair, set `h_mm` and
   `tilt_deg = atan(h/d)` in `pothole.conf`, and restart.
2. **Optional: set the clock.** With no RTC battery or network, the Pi's clock can be
   wrong. Records still sort by boot and monotonic time, but you can set the real time
   over SSH: `sudo date -s "2026-10-25 09:30"`.
3. **Check the stream** at http://10.42.0.1:8000:
   - 30 fps
   - the dashed ROI line (11 m) sits sensibly up the road
   - the crosshair is on the road
4. **Tape-measure a few potholes** on the route, as in your earlier tests, for ground
   truth.

**Power off safely.** There's no power controller yet (step 5). Over the hotspot:
`ssh admin@10.42.0.1`, then `sudo systemctl stop pothole`. That saves any pothole
still in view and flushes the log. Then run `sudo poweroff`, wait for the green LED to
stop, and unplug. If the power is simply pulled, everything already written is safe;
you'd lose only a pothole still in view and the last ~10 s of the log.

**At home**
```bash
D="/Users/michaellighton/Desktop/Pi C code/bench/drives/$(date +%F)"
mkdir -p "$D"
rsync -av admin@pothole-pi.local:~/pothole/detections/ "$D/"
rsync -av admin@pothole-pi.local:~/pothole/logs/ "$D/logs/"
python3 "/Users/michaellighton/Desktop/Pi C code/bench/review_detections.py" "$D"
python3 "/Users/michaellighton/Desktop/Pi C code/bench/analyse_logs.py" "$D"/logs/boot_*.csv
open "$D/review/index.html"
```
`review/` holds each still with its outline (red if cut off by the frame edge), box and
size drawn on it, an `index.html` gallery in drive order, and `summary.csv` for the
report. Each pothole is roughly 0.5 MB. Once a drive is safely copied, clear the Pi with
`rm ~/pothole/detections/*`.

## 13. GPS + 4G upload (SIM7600X 4G HAT, for testing)

Each finished pothole is now located and sent to the web app as well as saved:
1. **Locate.** The record takes the GPS fix nearest in time to its best frame. Fixes are
   stamped on the camera's clock as they arrive, minus `gps_delay_s`. The position is
   then moved ahead along the course by the pothole's forward distance on the road plus
   `gps_offset_m` (the report's Eq. gps_shift). Below 5 km/h it isn't moved. This goes
   in the record as `"gps": {lat, lon, utc, fix_lat, fix_lon, speed_kmh, sats, ...}`.
2. **Send.** A background thread posts every record in `detections/` to
   `upload_url`. It's the same multipart request as the Air780EU test, with the same
   nine fields, and `captured_at` is the fix's UTC time. If the server answers 200 or
   201, the record moves to `detections/sent/`. Records that can never be accepted go
   to `detections/not_uploaded/`, with the reason logged: no GPS fix, a manhole, no
   size, or a 4xx reply. Anything else (no signal, 5xx, a wrong key) stays in the queue
   and is retried with back-off up to 5 minutes, so potholes found out of coverage go
   up later.
3. **On the stream:** the top line shows `GPS <sats>` (`--` with no fix) and `UP <sent>`.
   `/status` and the CSV log also record the fix, its rate and the upload counts.

**Why libcurl and not the modem's AT socket commands:** the HAT runs in RNDIS mode, where
it is simply a network interface (`usb0`), so Linux does the DNS, HTTPS and retries. That
avoids the Air780EU firmware problems (no working AT+HTTP and no DNS). The same code
uploads over Wi-Fi or Ethernet, and over the Air780EU later via PPP or its USB network
mode.

### Hardware
- **Stacking:** fit the HAT on the AI HAT+'s stacking header, with 10 mm+ standoffs
  so its underside (SIM holder) can't touch the Hailo module or the PCIe ribbon.
  It uses the UART pins (unused here, since data goes over USB) and BCM 6 for PWRKEY.
  Neither clashes with the AI HAT+.
- **USB cable:** connect the HAT's **USB** port (the module's own, not "USB TO
  UART") to a Pi USB port. GPS and data both go over it.
- **Antennas:** LTE on MAIN, and the active GNSS antenna on GNSS, placed with sky view.
  Indoors it rarely gets a fix.
- **Power:** the module draws up to ~2 A bursts when transmitting. Use the 27 W supply
  and watch `throttled` in the log.
- **Heat:** the HAT covers the Hailo-8 and the Pi's cooler, so watch `temp_c` in the log.

### Bring-up (once, on the Pi, with internet over Ethernet)
```bash
cd ~/pothole
sudo apt install -y libcurl4-openssl-dev && sudo usermod -aG dialout $USER   # then log in again
sudo systemctl disable --now ModemManager      # it would grab the AT ports (if installed)
make && cp -n uplink.conf.example uplink.conf && nano uplink.conf    # set api_key
./build/sim7600_test ports          # the module's ttyUSB ports; set gps_dev / at_dev if they differ
./build/sim7600_test power          # only if no ports appear (module off)
./build/sim7600_test info           # SIM READY, signal (CSQ >= 10), registered (CEREG ,1/,5)
./build/sim7600_test rndis <APN>    # one-off: APN + RNDIS mode; the module reboots as usb0
./build/sim7600_test ports          # port numbers can change after the switch
./build/sim7600_test gps 180        # antenna outside: time to first fix, sats, rate (Hz)
./build/sim7600_test gps-rate 10    # optional: try 10 Hz; check with 'gps 60'
```
The APN depends on the SIM: e.g. `internet` (Vodacom, MTN, Cell C) or `telkom`. Check
with the network operator.

**Check the data link over 4G alone:** unplug Ethernet and turn the hotspot off
(`sudo nmcli con down pothole-hotspot`), or just check which interface is used. Then
`./build/sim7600_test info` should show `default route usb0` and `server ... HTTP 200`,
along with the local address used. If `usb0` gets no address, run
`./build/sim7600_test dial`. The hotspot doesn't take the default route, so it can stay
on while driving.

**End-to-end without driving:** `./build/sim7600_test send-test` waits for a fix, then
sends a made-up 300 × 500 mm pothole at your position with `hailo/test_frame.jpg`. Its
`detection_id` is `test-<time>`. Check that it appears on the web app's map, then delete
it there. `./build/sim7600_test send detections/<id>.json` sends one real record the
same way, and prints the HTTP exchange.

### Road test
`sudo systemctl restart pothole`. Then `journalctl -u pothole -f` should show
`gps: first fix`, then `pothole <id>: ... located` and `upload: <id>: sent pothole ...`
for each pothole. Afterwards, compare `fix_lat/fix_lon` (the car) with `lat/lon` (the
pothole) in `detections/sent/*.json`, and check the map against the potholes you
tape-measured. The SIM7600's GNSS runs at 1 Hz unless `gps-rate 10` worked, so expect
locations within about 8–11 m at 60 km/h (report, Table gps_rate). The ATGM336H at 10 Hz
brings that to about 3 m.

## Day-to-day

```bash
journalctl -u pothole -f                  # logs (fps / jpeg time every 10 s)
sudo systemctl stop pothole               # free the camera before running by hand
make && ./build/pothole --help            # build and run manually
./build/pothole --rotate180 --stream-fps 30
```

Get internet back for `apt` (the hotspot holds wlan0):
```bash
sudo nmcli con down pothole-hotspot       # falls back to your home Wi-Fi
sudo nmcli con up pothole-hotspot         # back to hotspot
```

Options: `--width/--height/--fps` (main stream), `--stream-width/--stream-height/--stream-fps`,
`--quality`, `--port`, `--rotate180`.
