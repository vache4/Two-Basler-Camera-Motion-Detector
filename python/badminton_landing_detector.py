#!/usr/bin/env python3
"""
Dual Basler USB camera viewer with motion-triggered coordinate capture.

Camera A ("motion" camera): runs motion detection on every acquired frame,
                            with display window refreshed with DISPLAY_INT
Camera B ("coordinate" camera): also displayed with DISPLAY_INT

When motion is detected on Camera A:
    1. The Camera-A frame that triggered it, plus Camera B's latest frame,
       are saved to disk.
    2. Motion detection is disarmed for REARM_DELAY_S seconds, then
       automatically re-arms.

Independently, every displayed frame from Camera B is searched for a
badminton shuttlecock seen from behind: an outer white circular skirt, and
- separately, since it is not concentric with the skirt - a smaller black
base visible inside it. This is purely for on-screen preview. Both centroid
coordinates (and radii) are only written to COORD_CSV_PATH when a motion
event fires - i.e. one CSV row per motion event, using that same Camera-B
frame the still image was saved from. A row is written even if no
shuttlecock was found for that event, so every event has exactly one row.

Each motion event is assigned an incrementing event number, recorded in the
CSV row and embedded in the filenames of the still images saved for that
event, so a CSV row and its corresponding PNGs can always be matched up.

Requirements:
    pip install pypylon opencv-python numpy sounddevice
"""

import os
import csv
import time
import threading
from collections import namedtuple
from datetime import datetime

import cv2
import numpy as np
import sounddevice as sd
from pypylon import pylon

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CAMERA_A_SERIAL = "REPLACE_WITH_CAMERA_A_SERIAL" # motion-detection camera
CAMERA_B_SERIAL = "REPLACE_WITH_CAMERA_B_SERIAL" # coordinate camera
MM_PER_PX = 550 / 400        # camB scaling in mm per pixel
#GAIN = 20                    # gain for both cameras
CAM_A_HEIGHT = 100        # image height (max 540) for the motion-detection camera
CAM_B_HEIGHT = 400        # image height (max 540) for the coordinate camera
# MOTION_MIN_AREA_FRAC
# WHITE_BRIGHTNESS_THRESH
# CIRCULARITY_MIN
# BLACK_BRIGHTNESS_THRESH
# BLACK_CIRCULARITY_MIN

CAMERA_A_FPS = 100     # acquisition rate for the motion-detection camera
CAMERA_B_FPS = 100     # acquisition rate for the coordinate camera
display_fps = 5.0      # both preview windows are refreshed at this rate
DISPLAY_INT = 1.0 / display_fps
CAM_A_EXPOSURE = 100.0 # exposure for the motion-detection camera (us)
CAM_B_EXPOSURE = 100.0 # exposure for the coordinate camera (us)

# --- motion detection -------------------------------------------------------
MOTION_THRESHOLD = 25        # per-pixel grayscale delta considered "changed"
MOTION_MIN_AREA_FRAC = 0.01  # fraction of frame that must change to trigger
REARM_DELAY_S = 5.0          # cooldown before motion detection re-arms

# --- white circle detection (shuttlecock skirt, seen from behind) ----------
WHITE_BRIGHTNESS_THRESH = 150    # pixels above this are considered white
CIRCULARITY_MIN = 0.70           # 1.0 = perfect circle, lower = more permissive
skirt_r_px = 65 / 2 / MM_PER_PX  # skirt radius in pixels (r=38-43mm on image, ranged below)
MIN_RADIUS_PX = 0.8 * skirt_r_px # ignore blobs smaller/larger than these two
MAX_RADIUS_PX = 1.4 * skirt_r_px # (shuttle is larger when closer)

# --- black circle detection (shuttlecock base, seen inside the white skirt) -
# The base is NOT assumed to be concentric with the white skirt, so it is
# searched for independently within the white circle rather than just being
# reported as "the middle" of it.
BLACK_BRIGHTNESS_THRESH = 100    # pixels darker than this are considered black
BLACK_CIRCULARITY_MIN = 0.50     # 1.0 = perfect circle, lower = more permissive
black_r_px = 15 / 2 / MM_PER_PX  # black circle radius in pixels (r=9-10mm on image, ranged below)
BLACK_MIN_RADIUS_PX = 0.6 * black_r_px
BLACK_MAX_RADIUS_PX = 1.7 * black_r_px
BLACK_SEARCH_RADIUS_FRAC = 0.90  # 1.0 = entire outer circle; 0.9 avoids its very outer edge

#MAX_HOLE_AREA_PX = MIN_RADIUS_PX ** 2   # fill interior holes up to this many pixels in area

# --- live candidate-mask overlay --------------------------------------------
WHITE_CANDIDATE_ALPHA = 0.40   # semi-transparent yellow
BLACK_CANDIDATE_ALPHA = 0.70   # semi-transparent red

# --- output paths ------------------------------------------------------------
_run_ts = datetime.now().strftime("%Y-%m-%d %H.%M.%S")
SAVE_DIR = f"captures3/{_run_ts}"
COORD_CSV_PATH = f"captures3/{_run_ts}_coords.csv"

# --- capture confirmation beep ----------------------------------------------
BEEP_FREQUENCY_HZ = 1000
BEEP_DURATION_S = 0.15
BEEP_VOLUME = 0.5   # 0.0 - 1.0
BEEP_SAMPLE_RATE = 44100


# ---------------------------------------------------------------------------
# Camera acquisition thread
# ---------------------------------------------------------------------------
class CameraStream:
    """Continuously grabs frames from a Basler camera in a background thread.
    Keeps two buffers:
      - latest_frame: updated on every single grabbed frame (at full rate)
      - display_frame: updated at DISPLAY_INT interval
    An optional on_frame callback is invoked with every full-rate frame,
    which is how the motion detector gets access to Camera A's 100 fps
    stream even though the display only shows with DISPLAY_INT.
    """

    def __init__(self, device, fps, exposure, height, name, on_frame=None):
        self.name = name
        self.on_frame = on_frame

        self.cam = pylon.InstantCamera(device)
        self.cam.Open()

        self.cam.ExposureAuto.SetValue("Off")
        self.cam.ExposureTime.SetValue(exposure)
        self.cam.GainAuto.SetValue("Once")
        #self.cam.GainAuto.SetValue("Off")
        #self.cam.Gain.SetValue(GAIN)
        self.cam.PixelFormat.SetValue("Mono8")
        self.cam.BinningVertical.SetValue(1)
        self.cam.BinningHorizontal.SetValue(1)
        #self._configure(fps)
        self.cam.AcquisitionFrameRateEnable.SetValue(True)
        self.cam.AcquisitionFrameRate.SetValue(fps)

        self.height = height
        sensor_height = self.cam.Height.Max
        self.offset = int((sensor_height - self.height) / 2)
        # Reset OffsetY to 0 first: if a previous session left OffsetY high
        # enough that OffsetY + new Height would exceed the sensor's max,
        # setting Height first throws a GenICam out-of-range error.
        self.cam.OffsetY.SetValue(0)
        self.cam.Height.SetValue(self.height)
        self.cam.OffsetY.SetValue(self.offset)

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_Mono8
        #self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        self.converter.OutputBitAlignment.Value = pylon.OutputBitAlignment_MsbAligned

        self.lock = threading.Lock()
        self.latest_frame = None
        self.display_frame = None
        self._last_display_update = 0.0

        self.running = False
        self.thread = None

    # def _configure(self, fps):
    #     try:
    #         self.cam.AcquisitionFrameRateEnable.SetValue(True)
    #         self.cam.AcquisitionFrameRate.SetValue(fps)
    #     except Exception as e:
    #         print(f"[{self.name}] warning: could not set frame rate to {fps}: {e}")

    def start(self):
        self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.running = True
        self.thread = threading.Thread(target=self._grab_loop, daemon=True)
        self.thread.start()

    def _grab_loop(self):
        while self.running and self.cam.IsGrabbing():
            try:
                grab = self.cam.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
            except pylon.TimeoutException:
                continue

            try:
                if grab.GrabSucceeded():
                    image = self.converter.Convert(grab)
                    frame = image.GetArray().copy()
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                    now = time.monotonic()

                    with self.lock:
                        self.latest_frame = frame
                        if now - self._last_display_update >= DISPLAY_INT:
                            self.display_frame = frame
                            self._last_display_update = now

                    if self.on_frame is not None:
                        self.on_frame(frame)
            finally:
                grab.Release()

    def get_latest(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def get_display(self):
        with self.lock:
            return None if self.display_frame is None else self.display_frame.copy()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.cam.IsGrabbing():
            self.cam.StopGrabbing()
        if self.cam.IsOpen():
            self.cam.Close()


# ---------------------------------------------------------------------------
# Motion detector
# ---------------------------------------------------------------------------
class MotionDetector:
    """Frame-differencing motion detector with an arm/disarm cooldown.

    Each trigger's on_motion callback runs on its own background thread (not
    the camera grab-loop thread), since it does file I/O and shuttlecock
    detection that would otherwise stall Camera A's frame acquisition.
    """

    def __init__(self, threshold, min_area_frac, rearm_delay, on_motion):
        self.threshold = threshold
        self.min_area_frac = min_area_frac
        self.rearm_delay = rearm_delay
        self.on_motion = on_motion

        self._prev_gray = None
        self._armed = True
        self._lock = threading.Lock()

        # Track event-processing threads so shutdown can wait for them
        # before closing the CSV file.
        self._callback_lock = threading.Lock()
        self._callback_threads = []

    def process(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray = cv2.GaussianBlur(gray, (21, 21), 0)

        trigger = False
        with self._lock:
            if self._prev_gray is None or not self._armed:
                self._prev_gray = gray
                return

            diff = cv2.absdiff(self._prev_gray, gray)
            self._prev_gray = gray

            _, thresh = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
            changed_frac = np.count_nonzero(thresh) / thresh.size

            if changed_frac >= self.min_area_frac:
                self._armed = False
                trigger = True

        if not trigger:
            return

        threading.Thread(target=self._rearm_after_delay, daemon=True).start()

        # Pass the actual Camera-A frame that caused the trigger, rather than
        # letting the callback re-fetch cam_a's *current* latest frame later.
        trigger_frame = frame.copy()
        worker = threading.Thread(target=self._run_callback, args=(trigger_frame,), daemon=True)

        with self._callback_lock:
            self._callback_threads = [t for t in self._callback_threads if t.is_alive()]
            self._callback_threads.append(worker)

        worker.start()

    def _run_callback(self, trigger_frame):
        try:
            self.on_motion(trigger_frame)
        except Exception as e:
            print(f"[motion] event-processing error: {e}")

    def _rearm_after_delay(self):
        time.sleep(self.rearm_delay)
        with self._lock:
            self._armed = True
        print("[motion] re-armed")

    def wait_for_callbacks(self):
        """Block until all currently running event-processing callbacks finish."""
        while True:
            with self._callback_lock:
                alive = [t for t in self._callback_threads if t.is_alive()]
                self._callback_threads = alive
            if not alive:
                return
            for t in alive:
                t.join()

# ---------------------------------------------------------------------------
# Shuttlecock detection
# ---------------------------------------------------------------------------
def detect_white_circle(frame):
    """Find the most white circular blob in frame (shuttlecock's skirt).
    Returns (circle, white_candidate_mask), where circle = (x, y, radius) in
    pixel coordinates (frame's own coordinate system, origin top-left), or
    None if nothing qualified.
    """
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (3, 3), 2)
    blurred = gray

    _, thresh = cv2.threshold(blurred, WHITE_BRIGHTNESS_THRESH, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_circularity = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        if area < 10 or perimeter == 0:
            continue

        (x, y), radius = cv2.minEnclosingCircle(c)
        if not (MIN_RADIUS_PX <= radius <= MAX_RADIUS_PX):
            continue

        circularity = area / np.pi / radius ** 2
        #circularity = 4 * np.pi * area / (perimeter ** 2)  # 1.0 = perfect circle
        if circularity < CIRCULARITY_MIN:
            continue

        if circularity > best_circularity:
            best_circularity = circularity
            best = (x, y, radius)

    return best, thresh


def _detect_black_circle_in_white(frame, white_circle):
    """Find a dark circular blob inside the previously detected white circle.
    white_circle = (xw, yw, rw). The search area is a circular mask around
    the white circle's center (not a tight crop), and any candidate is
    additionally required to be fully contained within the white circle's
    own radius - so the black circle is free to sit anywhere inside the
    skirt rather than being assumed concentric with it. Among qualifying
    candidates the one with the highest circularity * area (large and round)
    wins.
    Returns (circle, black_candidate_mask), where circle = (x, y, radius) in
    full-frame coordinates, or None if nothing qualified.
    """
    if white_circle is None:
        return None, None

    xw, yw, rw = white_circle

    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.GaussianBlur(gray, (3, 3), 1)
    blurred = gray

    # restrict the search to a circular region around the white circle
    mask = np.zeros_like(gray, dtype=np.uint8)
    search_radius = int(rw * BLACK_SEARCH_RADIUS_FRAC)
    cv2.circle(mask, (int(xw), int(yw)), search_radius, 255, -1)

    # THRESH_BINARY_INV: pixels darker than the threshold become white (255)
    _, dark = cv2.threshold(blurred, BLACK_BRIGHTNESS_THRESH, 255, cv2.THRESH_BINARY_INV)
    dark = cv2.bitwise_and(dark, mask)

    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = -1.0
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        if area < 10 or perimeter == 0:
            continue

        (xb, yb), rb = cv2.minEnclosingCircle(c)
        if not (BLACK_MIN_RADIUS_PX <= rb <= BLACK_MAX_RADIUS_PX):
            continue

        circularity = area / np.pi / rb ** 2
        #circularity = 4 * np.pi * area / (perimeter ** 2)  # 1.0 = perfect circle
        if circularity < BLACK_CIRCULARITY_MIN:
            continue

        # the black circle must actually sit inside the white circle
        center_distance = np.hypot(xb - xw, yb - yw)
        if center_distance + rb > rw:
            continue

        score = circularity * area  # prefer large, round candidates
        if score > best_score:
            best_score = score
            best = (xb, yb, rb)

    return best, dark

# Named tuple with detection result.
# All 3 black values (rightmost) are None by default, e.g.
# shcock=ShuttlecockDetection(120,250,22) gives shcock=(120,250,22,None,None,None)
ShuttlecockDetection = namedtuple(
    "ShuttlecockDetection",
    ["white_x", "white_y", "white_radius", "black_x", "black_y", "black_radius", "center_distance"],
    defaults=[None, None, None, None],
)


def detect_and_annotate(frame_b):
    """Detect shuttlecock and return (shcock, annotated_bgr_image)."""

    if frame_b is None:
        return None, None

    frame_b = cv2.normalize(frame_b, None, 0, 255, cv2.NORM_MINMAX)

    display = (cv2.cvtColor(frame_b, cv2.COLOR_GRAY2BGR) if frame_b.ndim == 2 else frame_b.copy())

    # Show ALL thresholded white candidates,
    white, white_mask = detect_white_circle(frame_b)
    overlay_binary_mask(display, white_mask, (0, 255, 0), WHITE_CANDIDATE_ALPHA)  # BGR Y:(0,255,255)

    if white is None:
        return None, display
    wx, wy, wr = white

    black, black_mask = _detect_black_circle_in_white(frame_b, white)
    overlay_binary_mask(display, black_mask, (0, 50, 255), BLACK_CANDIDATE_ALPHA)  # BGR

    black_x = black_y = black_radius = center_distance = None
    if black is not None:
        black_x, black_y, black_radius = black
        center_distance = float(np.hypot(black_x - wx, black_y - wy))

    shcock = ShuttlecockDetection(wx, wy, wr, black_x, black_y, black_radius, center_distance)

    # Draw circles on top of the candidate shading.
    cv2.circle(display, (int(wx), int(wy)), int(wr), (255, 255, 0), 2)
    cv2.circle(display, (int(wx), int(wy)), 3, (255, 255, 0), -1)

    if black is not None:
        cv2.circle(display, (int(black_x), int(black_y)), int(black_radius), (255, 0, 255), 2)

    #cv2.setMouseCallback(display, show_xy)

    return shcock, display

# ---------------------------------------------------------------------------
# File logger
# ---------------------------------------------------------------------------
class CoordinateLogger:
    """Appends detected shuttlecock coordinates to a CSV file, one row per
    motion event (even if no shuttlecock was found for that event - the
    coordinate columns are just left blank). Black-circle columns are left
    blank if a white circle was found but no black circle was."""

    def __init__(self, path):
        self._lock = threading.Lock()

        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        is_new = not os.path.exists(path) or os.path.getsize(path) == 0
        self._file = open(path, "a", newline="")
        self._writer = csv.writer(self._file)

        if is_new:
            self._writer.writerow([
                "event", "timestamp",
                "white_x_mm", "white_y_mm", "white_radius_mm",
                "black_x_mm", "black_y_mm", "black_radius_mm",
                "center_distance_mm",
            ])
            self._file.flush()

    def log(self, event_id, shcock):
        ts = datetime.now().strftime("%Y-%m-%d %H.%M.%S.%f")[:-3]

        def fmt(v):
            return "" if v is None else f"{v * MM_PER_PX:.2f}"

        if shcock is None:
            formatted_row = [event_id, ts, "", "", "", "", "", "", ""]
        else:
            formatted_row = [
                event_id, ts,
                fmt(shcock.white_x), fmt(shcock.white_y), fmt(shcock.white_radius),
                fmt(shcock.black_x), fmt(shcock.black_y), fmt(shcock.black_radius),
                fmt(shcock.center_distance),
            ]

        with self._lock:
            self._writer.writerow(formatted_row)
            self._file.flush()

        print(formatted_row)

    def close(self):
        with self._lock:
            if not self._file.closed:
                self._file.close()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def find_cameras():
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    found_serials = {d.GetSerialNumber() for d in devices}
    print(f"Found {len(devices)} Basler camera(s): {sorted(found_serials)}")

    if CAMERA_A_SERIAL == CAMERA_B_SERIAL:
        raise RuntimeError("CAMERA_A_SERIAL and CAMERA_B_SERIAL must be different cameras")
    if CAMERA_A_SERIAL not in found_serials:
        raise RuntimeError(f"Camera A not found, serial={CAMERA_A_SERIAL}")
    if CAMERA_B_SERIAL not in found_serials:
        raise RuntimeError(f"Camera B not found, serial={CAMERA_B_SERIAL}")

    dia = pylon.DeviceInfo()
    dia.SetSerialNumber(CAMERA_A_SERIAL)
    dib = pylon.DeviceInfo()
    dib.SetSerialNumber(CAMERA_B_SERIAL)

    dev_a = tl_factory.CreateFirstDevice(dia)
    dev_b = tl_factory.CreateFirstDevice(dib)
    return dev_a, dev_b


def beep():
    """Play a short sine-wave tone through the system's default audio
    output - guaranteed to make a sound regardless of OS or terminal
    settings (unlike winsound, which is Windows-only, or the terminal bell
    character, which many terminals mute or intercept).
    Runs in a background thread so it can't stall the camera grab loop
    while the ~150 ms tone plays.
    """
    def _play():
        try:
            t = np.linspace(0, BEEP_DURATION_S, int(BEEP_SAMPLE_RATE * BEEP_DURATION_S), endpoint=False)
            tone = BEEP_VOLUME * np.sin(2 * np.pi * BEEP_FREQUENCY_HZ * t)
            sd.play(tone.astype(np.float32), BEEP_SAMPLE_RATE)
            sd.wait()
        except Exception as e:
            print(f"[beep] warning: {e}")

    threading.Thread(target=_play, daemon=True).start()


def save_pair_frames(frame_a, frame_b, event_id):
    """Save the exact frame pair captured for this event."""
    os.makedirs(SAVE_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y-%m-%d %H.%M.%S.%f")[:-3]

    if frame_a is not None:
        cv2.imwrite(os.path.join(SAVE_DIR, f"a{event_id:03d}.png"), frame_a)
    if frame_b is not None:
        cv2.imwrite(os.path.join(SAVE_DIR, f"b{event_id:03d}.png"), frame_b)

    print(f"[motion] event #{event_id} - saved still pair ({ts})")
    beep()


def overlay_binary_mask(display, mask, color, alpha):
    """Overlay a binary mask on a BGR image without dimming unmasked pixels."""
    if mask is None or alpha <= 0:
        return

    selected = mask > 0
    if not np.any(selected):
        return

    color_array = np.asarray(color, dtype=np.float32)
    pixels = display[selected].astype(np.float32)
    pixels = pixels * (1.0 - alpha) + color_array * alpha
    display[selected] = np.clip(pixels, 0, 255).astype(np.uint8)


class LatestFrameBox:
    """Thread-safe single-slot mailbox for handing a frame from a worker
    thread to the main thread, which owns all cv2.imshow calls.
    Producers call set() from any thread; only the main thread calls
    get_if_updated() and imshow()s the result.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._updated = False

    def set(self, frame):
        with self._lock:
            self._frame = frame
            self._updated = True

    def get_if_updated(self):
        with self._lock:
            if not self._updated:
                return None
            self._updated = False
            return self._frame


""" Mouse callback that shows pixel values (x, y and brightness) at mouse position
def show_xy(event, x, y, flags, param):

    nonlocal ix, iy, text2, img_w_xy

    # if we move the mouse to a place, we want to update the x and y values shown
    if event == cv2.EVENT_MOUSEMOVE:
        ix, iy = x, y
        text2 = "(" + str(ix) + ", " +  str(iy) + "): " + str(matrix[img_num][iy][ix])
        img_w_frames = cv2.putText(copy.deepcopy(img), text, (tx1,ty1), font, font_size, font_color, font_thickness, cv2.LINE_AA)
        img_w_xy = cv2.putText(copy.deepcopy(img_w_frames), text2, (tx2,ty2), font, font_size, font_color, font_thickness, cv2.LINE_AA)
        #cv2.imshow("image", img_w_xy)
"""

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    dev_a, dev_b = find_cameras()

    cam_a = CameraStream(dev_a, CAMERA_A_FPS, CAM_A_EXPOSURE, CAM_A_HEIGHT, "CamA(motion)")
    cam_b = CameraStream(dev_b, CAMERA_B_FPS, CAM_B_EXPOSURE, CAM_B_HEIGHT, "CamB(coordinate)")

    #coord_logger = CoordinateLogger(COORD_CSV_PATH)
    coord_logger = None
    logger_lock = threading.Lock()

    event_id = 0
    event_lock = threading.Lock()
    last_event_a = None
    last_event_b = None
    event_until = 0.0

    # Motion-event frames are imshow()'d from the main thread - the worker thread drops them here.
    event_frame_a = LatestFrameBox()
    event_frame_b = LatestFrameBox()

    # Executed when motion is detected
    def handle_motion(trigger_frame_a):
        nonlocal event_id
        nonlocal coord_logger

        with event_lock:
            event_id += 1  # event counter
            current_event = event_id

        # Save image pair from cameras.
        # Camera A's frame is the exact one that triggered the motion event;
        # Camera B's is its latest available frame at the same moment.
        frame_a = trigger_frame_a
        frame_b = cam_b.get_latest()
        save_pair_frames(frame_a, frame_b, current_event)

        if frame_a is not None:
            event_frame_a.set(frame_a)

        shcock = None
        if frame_b is not None:
            event_frame_b.set(frame_b)
            shcock, _ = detect_and_annotate(frame_b)  # Detect shuttlecock coordinates and annotate Frame B.

        #coord_logger.log(current_event, shcock)  # Log to CSV file with one row per motion detection.
        # Create CSV only on first event
        with logger_lock:
            if coord_logger is None:
                coord_logger = CoordinateLogger(COORD_CSV_PATH)
        coord_logger.log(current_event,shcock)

    detector = MotionDetector(
        threshold=MOTION_THRESHOLD,
        min_area_frac=MOTION_MIN_AREA_FRAC,
        rearm_delay=REARM_DELAY_S,
        on_motion=handle_motion,
    )
    cam_a.on_frame = detector.process  # wire motion detection to Camera A's full-rate stream

    cam_a.start()
    cam_b.start()

    time.sleep(0.5)
    print(f"Auto gain: "f"A={cam_a.cam.Gain.GetValue():.0f}, "f"B={cam_b.cam.Gain.GetValue():.0f}")

    try:
        while True:
            t0 = time.monotonic()

            # Get latest display frames
            disp_a = cam_a.get_display()
            disp_b = cam_b.get_display()

            # Check whether a new motion-event image has arrived
            trigger_a = event_frame_a.get_if_updated()
            trigger_b = event_frame_b.get_if_updated()
            # Hold trigger images for REARM_DELAY_S time
            if trigger_a is not None:
                last_event_a = trigger_a
                event_until = time.monotonic() + REARM_DELAY_S
            if trigger_b is not None:
                last_event_b = trigger_b
                event_until = time.monotonic() + REARM_DELAY_S

            #now = time.monotonic()

            if t0 < event_until:                                       ## Triggered :
                if last_event_a is not None:
                    cv2.imshow("Camera A", last_event_a)        # Window 1
                if last_event_b is not None:
                    cv2.imshow("Camera B", last_event_b)        # Window 2
                    _, display_b = detect_and_annotate(last_event_b)
                    cv2.imshow("Camera B detection", display_b) # Window 3

            else:                                                      ## Live :
                if disp_a is not None:
                    cv2.imshow("Camera A", disp_a)              # Window 1
                if disp_b is not None:
                    cv2.imshow("Camera B", disp_b)              # Window 2
                    _, display_b = detect_and_annotate(disp_b)
                    cv2.imshow("Camera B detection", display_b) # Window 3

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 'q' or Esc
                break

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, DISPLAY_INT - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        cam_a.stop()
        cam_b.stop()

        # A motion callback may still be saving/detecting when Q/Esc is
        # pressed - finish those before closing the CSV file.
        detector.wait_for_callbacks()

        if coord_logger is not None:
            coord_logger.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
