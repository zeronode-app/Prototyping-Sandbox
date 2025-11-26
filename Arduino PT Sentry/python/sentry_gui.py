"""
Pan-tilt sentry prototype.

Features:
- OpenCV-based motion detection with bounding box overlay.
- Auto mode: locks on the first moving object, tracks while visible, turns laser on while locked.
- Manual mode: arrow buttons/keys nudge servos; corresponding edge of the frame lights up green.
- Tkinter GUI showing live video, lock status, mode, and bounding boxes.

Dependencies (Linux):
  pip install opencv-python pillow pyserial
Optional for person-only tracking:
  pip install ultralytics torch torchvision

Hardware defaults must match arduino/pt_sentry.ino:
  Serial @ 115200, commands: PAN, TILT, LASER, HOME
"""

import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import serial
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

try:
    from ultralytics import YOLO as YoloModel
    YOLO_AVAILABLE = True
except ImportError:
    YoloModel = None
    YOLO_AVAILABLE = False


# ---------- Configuration ----------
SERIAL_PORT = "/dev/ttyACM0"  # Update if your Uno appears differently
BAUD_RATE = 115200
SERVO_STEP = 4               # Degrees per manual nudge
PAN_LIMITS = (0, 180)
TILT_LIMITS = (0, 180)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
LOCK_TIMEOUT_FRAMES = 12
# Multi-track parameters (security-cam style)
TRACK_MAX_DIST = 140         # Max centroid distance to associate detections
TRACK_CONF_GAIN = 2          # Confidence boost when matched
TRACK_CONF_DECAY = 1         # Confidence loss when unmatched
TRACK_MIN_CONF_LOCK = 4      # Minimum confidence to be considered lockable
TRACK_SWITCH_MARGIN = 2      # New track must beat active by this margin
TRACK_SWITCH_PATIENCE = 6    # Frames a better track must persist before switching
TRACK_MISS_TOLERANCE = 15    # Drop track after this many misses
TRACK_MIN_AGE = 2            # Frames before a track can become active
USE_YOLO = True              # If ultralytics is installed, use YOLO for person filtering
YOLO_MODEL_PATH = "yolov8n.pt"
YOLO_CONF = 0.35
YOLO_INTERVAL = 3            # Run YOLO every N frames to save CPU
BASE_MIN_CONTOUR_AREA = 600  # Default medium size
COLOR_PRESETS = {
    "None": None,
    "Red": ((0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)),  # two ranges
    "Green": ((35, 80, 60), (85, 255, 255)),
    "Blue": ((95, 80, 60), (130, 255, 255)),
    "Yellow": ((20, 100, 100), (35, 255, 255)),
}
SIZE_PRESETS = {
    "Small": 300,
    "Medium": BASE_MIN_CONTOUR_AREA,
    "Large": 1500,
}
CONFIG_PATH = Path(__file__).resolve().parent / "sentry_config.json"


def clamp(value: int, bounds: tuple[int, int]) -> int:
    return max(bounds[0], min(bounds[1], value))


class YoloDetector:
    """Optional YOLO person detector with frame skipping for speed."""

    def __init__(self):
        self.model = None
        self.frame_idx = 0
        self.cache: list[tuple[int, int, int, int, tuple[int, int]]] | None = None
        if USE_YOLO and YOLO_AVAILABLE:
            try:
                self.model = YoloModel(YOLO_MODEL_PATH)
                print("YOLO loaded; using person-only detections.")
            except Exception as exc:
                print(f"YOLO load failed ({exc}); falling back to motion only.")
                self.model = None
        elif USE_YOLO:
            print("YOLO not installed; falling back to motion only.")

    def detect(self, frame: np.ndarray):
        if self.model is None:
            return None

        self.frame_idx += 1
        if self.cache is not None and self.frame_idx % YOLO_INTERVAL != 0:
            return self.cache

        try:
            results = self.model(frame, verbose=False)[0]
        except Exception as exc:
            print(f"YOLO inference error: {exc}")
            return self.cache

        boxes = []
        for xyxy, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            if int(cls) != 0:  # 0 == person in COCO
                continue
            if float(conf) < YOLO_CONF:
                continue
            x1, y1, x2, y2 = map(int, xyxy.tolist())
            w = x2 - x1
            h = y2 - y1
            c = (x1 + w // 2, y1 + h // 2)
            boxes.append((x1, y1, w, h, c))

        self.cache = boxes
        return boxes


class ServoController:
    """Simple serial protocol wrapper for pan/tilt + laser."""

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self._serial = None
        self.pan = 90
        self.tilt = 90
        self._connect()

    def _connect(self):
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(2)  # Allow Uno to reset
            self.home()
        except serial.SerialException:
            self._serial = None
            print("Serial not available, running in dry-run mode.")

    def _send(self, line: str):
        if self._serial is None:
            print(f"[DRY] {line.strip()}")
            return
        self._serial.write((line.strip() + "\n").encode("utf-8"))

    def set_pan(self, angle: int):
        self.pan = clamp(angle, PAN_LIMITS)
        self._send(f"PAN {self.pan}")

    def set_tilt(self, angle: int):
        self.tilt = clamp(angle, TILT_LIMITS)
        self._send(f"TILT {self.tilt}")

    def set_laser(self, on: bool):
        self._send(f"LASER {1 if on else 0}")

    def home(self):
        self.pan = 90
        self.tilt = 90
        self._send("HOME")

    def close(self):
        if self._serial:
            self._serial.close()


@dataclass
class Detection:
    bbox: tuple[int, int, int, int] | None
    centroid: tuple[int, int] | None
    locked: bool
    tracks: list[tuple[int, int, int, int]]  # all current boxes


class MotionTracker:
    """Background-subtraction motion detector with multi-track persistence."""

    def __init__(self, min_area: int = BASE_MIN_CONTOUR_AREA, color_range=None):
        self.bg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=32)
        self.tracks: list[dict] = []
        self.active_id: int | None = None
        self.switch_counter = 0
        self._next_id = 1
        self.min_area = min_area
        self.color_range = color_range

    def reset(self):
        self.tracks = []
        self.active_id = None
        self.switch_counter = 0
        self._next_id = 1

    def set_min_area(self, area: int):
        self.min_area = max(50, area)

    def set_color_range(self, hsv_range):
        self.color_range = hsv_range

    def process(self, frame: np.ndarray, detections_override: list[tuple[int, int, int, int, tuple[int, int]]] | None = None) -> Detection:
        detections: list[tuple[int, int, int, int, tuple[int, int]]] = []
        if detections_override is not None:
            detections = detections_override
        else:
            working = frame
            if self.color_range:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                if len(self.color_range) == 4:
                    lower1, upper1, lower2, upper2 = self.color_range
                    mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
                    mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
                    color_mask = cv2.bitwise_or(mask1, mask2)
                else:
                    lower, upper = self.color_range
                    color_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                working = cv2.bitwise_and(frame, frame, mask=color_mask)

            fg = self.bg.apply(working)
            _, mask = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                if cv2.contourArea(c) <= self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                centroid = (x + w // 2, y + h // 2)
                detections.append((x, y, w, h, centroid))

        self._associate_tracks(detections)
        active = self._select_active()

        bbox = active["bbox"] if active else None
        centroid = active["centroid"] if active else None
        locked = active is not None
        track_boxes = [t["bbox"] for t in self.tracks]
        return Detection(bbox, centroid, locked, track_boxes)

    def _associate_tracks(self, detections):
        # Greedy nearest-neighbor matching by centroid distance.
        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))

        def distance(c1, c2):
            dx = c1[0] - c2[0]
            dy = c1[1] - c2[1]
            return (dx * dx + dy * dy) ** 0.5

        matches = []
        for t_idx in list(unmatched_tracks):
            t_centroid = self.tracks[t_idx]["centroid"]
            best_d = None
            best_d_idx = None
            for d_idx in unmatched_detections:
                d_centroid = detections[d_idx][4]
                d = distance(t_centroid, d_centroid)
                if d <= TRACK_MAX_DIST and (best_d is None or d < best_d):
                    best_d = d
                    best_d_idx = d_idx
            if best_d_idx is not None:
                matches.append((t_idx, best_d_idx))
                unmatched_tracks.discard(t_idx)
                unmatched_detections.discard(best_d_idx)

        # Update matched tracks
        for t_idx, d_idx in matches:
            x, y, w, h, c = detections[d_idx]
            track = self.tracks[t_idx]
            track["bbox"] = (x, y, w, h)
            track["centroid"] = c
            track["age"] += 1
            track["missed"] = 0
            track["conf"] = min(track["conf"] + TRACK_CONF_GAIN, 100)

        # Decay unmatched tracks
        survivors = []
        for idx in unmatched_tracks:
            track = self.tracks[idx]
            track["missed"] += 1
            track["conf"] = max(0, track["conf"] - TRACK_CONF_DECAY)
            track["age"] += 1
            if track["missed"] <= TRACK_MISS_TOLERANCE and track["conf"] > 0:
                survivors.append(track)
        self.tracks = [self.tracks[i] for i in range(len(self.tracks)) if i not in unmatched_tracks] + survivors

        # Add new tracks for unmatched detections
        for d_idx in unmatched_detections:
            x, y, w, h, c = detections[d_idx]
            self.tracks.append(
                {
                    "id": self._next_id,
                    "bbox": (x, y, w, h),
                    "centroid": c,
                    "age": 1,
                    "missed": 0,
                    "conf": TRACK_CONF_GAIN,
                }
            )
            self._next_id += 1

        # Cull stale tracks
        self.tracks = [t for t in self.tracks if t["missed"] <= TRACK_MISS_TOLERANCE and t["conf"] > 0]

    def _select_active(self):
        # Choose track with highest confidence; switch only if it beats active consistently.
        if not self.tracks:
            self.active_id = None
            self.switch_counter = 0
            return None

        self.tracks.sort(key=lambda t: t["conf"], reverse=True)
        best = self.tracks[0]
        active = next((t for t in self.tracks if t["id"] == self.active_id), None)

        if active is None or active["conf"] < TRACK_MIN_CONF_LOCK or active["missed"] > LOCK_TIMEOUT_FRAMES:
            if best["conf"] >= TRACK_MIN_CONF_LOCK and best["age"] >= TRACK_MIN_AGE:
                self.active_id = best["id"]
                self.switch_counter = 0
                return best
            self.active_id = None
            self.switch_counter = 0
            return None

        if best["id"] != active["id"] and best["conf"] >= active["conf"] + TRACK_SWITCH_MARGIN and best["age"] >= TRACK_MIN_AGE:
            self.switch_counter += 1
            if self.switch_counter >= TRACK_SWITCH_PATIENCE:
                self.active_id = best["id"]
                self.switch_counter = 0
                return best
        else:
            self.switch_counter = 0

        return active


class SentryApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Arduino PT Sentry")
        self.root.configure(bg="#0f172a")

        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background="#0f172a")
        style.configure("TLabel", background="#0f172a", foreground="#e2e8f0", font=("Helvetica", 11))
        style.configure("Header.TLabel", background="#0f172a", foreground="#e2e8f0", font=("Helvetica", 14, "bold"))
        style.configure("Card.TFrame", background="#0b1224", relief="raised", borderwidth=1)
        style.configure("Status.TLabel", background="#111827", foreground="#e2e8f0", font=("Helvetica", 11),
                        padding=8, anchor="center", relief="groove")
        style.configure("TButton", font=("Helvetica", 10, "bold"), padding=8)
        style.configure("Mode.TButton", background="#1f2937", foreground="#e2e8f0",
                        font=("Helvetica", 10, "bold"), padding=(12, 8), relief="flat", borderwidth=0)
        style.configure("ModeActive.TButton", background="#2563eb", foreground="#e2e8f0",
                        font=("Helvetica", 10, "bold"), padding=(12, 8), relief="flat", borderwidth=0)
        style.configure("Dropdown.TCombobox",
                        fieldbackground="#1f2937",
                        background="#1f2937",
                        foreground="#e2e8f0",
                        selectbackground="#1f2937",
                        selectforeground="#e2e8f0",
                        borderwidth=0,
                        arrowcolor="#e2e8f0")
        style.map("TButton",
                  background=[("active", "#1d4ed8"), ("!active", "#1e293b")],
                  foreground=[("active", "#e2e8f0"), ("!active", "#e2e8f0")])
        style.map("Mode.TButton",
                  background=[("active", "#1e40af"), ("!active", "#1f2937")],
                  foreground=[("active", "#e2e8f0"), ("!active", "#e2e8f0")])
        style.map("ModeActive.TButton",
                  background=[("active", "#1d4ed8"), ("!active", "#2563eb")],
                  foreground=[("active", "#e2e8f0"), ("!active", "#e2e8f0")])

        self.mode = tk.StringVar(value="auto")
        self.status = tk.StringVar(value="Auto: waiting for motion")
        self.color_choice = tk.StringVar(value="None")
        self.size_choice = tk.StringVar(value="Medium")
        self.laser_on = tk.BooleanVar(value=False)

        header = ttk.Label(self.root, text="Arduino PT Sentry", style="Header.TLabel")
        header.pack(pady=(10, 6))

        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=10, pady=8)
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)

        video_card = ttk.Frame(main, style="Card.TFrame", padding=6)
        video_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        video_card.columnconfigure(0, weight=1)
        video_card.rowconfigure(0, weight=1)
        self.video_label = ttk.Label(video_card, anchor="center")
        self.video_label.grid(row=0, column=0, sticky="nsew")

        control_panel = ttk.Frame(main)
        control_panel.grid(row=0, column=1, sticky="nsew")
        control_panel.columnconfigure(0, weight=1)

        top_row = ttk.Frame(control_panel)
        top_row.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        top_row.columnconfigure(0, weight=1)
        help_btn = ttk.Button(top_row, text="?", width=3, command=self.show_help)
        help_btn.grid(row=0, column=1, padx=(4, 0), sticky="e")

        mode_bar = ttk.Frame(control_panel)
        mode_bar.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        mode_bar.columnconfigure((0, 1, 2), weight=1)

        btn_auto = ttk.Button(mode_bar, text="Auto", style="Mode.TButton",
                              command=lambda: self.on_mode_change("auto"))
        btn_manual = ttk.Button(mode_bar, text="Manual", style="Mode.TButton",
                                command=lambda: self.on_mode_change("manual"))
        btn_person = ttk.Button(mode_bar, text="Person (YOLO)", style="Mode.TButton",
                                command=lambda: self.on_mode_change("person"))
        btn_auto.grid(row=0, column=0, padx=(0, 4), sticky="ew")
        btn_manual.grid(row=0, column=1, padx=4, sticky="ew")
        btn_person.grid(row=0, column=2, padx=(4, 0), sticky="ew")
        self.mode_buttons = {"auto": btn_auto, "manual": btn_manual, "person": btn_person}

        action_row = ttk.Frame(control_panel)
        action_row.grid(row=2, column=0, sticky="ew", pady=(0, 12))
        action_row.columnconfigure((0, 1), weight=1)
        self.laser_button = ttk.Button(action_row, text="Laser: Off", command=self.toggle_laser)
        self.laser_button.grid(row=0, column=0, padx=4, sticky="w")
        ttk.Button(action_row, text="Home", command=self.home).grid(row=0, column=1, padx=4, sticky="e")

        manual = ttk.Frame(control_panel)
        manual.grid(row=3, column=0, pady=4)
        manual.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(manual, text="Up", command=lambda: self.nudge("up")).grid(row=0, column=1, pady=3, sticky="ew")
        ttk.Button(manual, text="Left", command=lambda: self.nudge("left")).grid(row=1, column=0, padx=3, sticky="ew")
        ttk.Button(manual, text="Right", command=lambda: self.nudge("right")).grid(row=1, column=2, padx=3, sticky="ew")
        ttk.Button(manual, text="Down", command=lambda: self.nudge("down")).grid(row=2, column=1, pady=3, sticky="ew")

        color_row = ttk.Frame(control_panel)
        color_row.grid(row=4, column=0, pady=(10, 4), sticky="ew")
        ttk.Label(color_row, text="Auto color target:", foreground="#cbd5e1").grid(row=0, column=0, sticky="w")
        color_row.columnconfigure(1, weight=1)
        color_menu = ttk.Combobox(color_row, textvariable=self.color_choice, values=list(COLOR_PRESETS.keys()),
                                  state="readonly", style="Dropdown.TCombobox")
        color_menu.grid(row=0, column=1, padx=6, sticky="ew")
        color_menu.bind("<<ComboboxSelected>>", lambda _e: self.on_color_change())

        size_row = ttk.Frame(control_panel)
        size_row.grid(row=5, column=0, pady=(4, 8), sticky="ew")
        size_row.columnconfigure(1, weight=1)
        ttk.Label(size_row, text="Motion size:", foreground="#cbd5e1").grid(row=0, column=0, sticky="w")
        size_menu = ttk.Combobox(size_row, textvariable=self.size_choice, values=list(SIZE_PRESETS.keys()),
                                 state="readonly", style="Dropdown.TCombobox")
        size_menu.grid(row=0, column=1, padx=6, sticky="ew")
        size_menu.bind("<<ComboboxSelected>>", lambda _e: self.on_size_change())

        self.status_label = ttk.Label(control_panel, textvariable=self.status, style="Status.TLabel")
        self.status_label.grid(row=6, column=0, sticky="ew", pady=(12, 0))

        self.root.bind("<Up>", lambda e: self.nudge("up"))
        self.root.bind("<Down>", lambda e: self.nudge("down"))
        self.root.bind("<Left>", lambda e: self.nudge("left"))
        self.root.bind("<Right>", lambda e: self.nudge("right"))

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.yolo = YoloDetector()
        self.tracker = MotionTracker(min_area=SIZE_PRESETS[self.size_choice.get()])
        self.servo = ServoController(SERIAL_PORT, BAUD_RATE)

        self.highlight_until: dict[str, float] = {}

        self._running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.on_color_change()
        self.on_size_change()
        self.load_persisted_mode()
        self.update_mode_styles()
        self.update_frame()

    def on_mode_change(self, selected: str | None = None):
        current = selected if selected else self.mode.get()
        self.mode.set(current)
        if current == "auto":
            self.set_auto(update_mode=False)
        elif current == "manual":
            self.set_manual(update_mode=False)
        elif current == "person":
            self.set_person(update_mode=False)
        self.persist_mode()
        self.update_mode_styles()

    def set_auto(self, update_mode: bool = True):
        if update_mode:
            self.mode.set("auto")
        self.status.set("Auto: waiting for motion")
        self.tracker.reset()
        self.servo.set_laser(False)
        if update_mode:
            self.persist_mode()
        self.laser_on.set(False)
        if hasattr(self, "laser_button"):
            self.laser_button.configure(text="Laser: Off")

    def set_manual(self, update_mode: bool = True):
        if update_mode:
            self.mode.set("manual")
        self.status.set("Manual: use arrows/buttons")
        self.tracker.reset()
        self.servo.set_laser(False)
        if update_mode:
            self.persist_mode()
        self.update_mode_styles()
        self.laser_on.set(False)
        if hasattr(self, "laser_button"):
            self.laser_button.configure(text="Laser: Off")

    def set_person(self, update_mode: bool = True):
        if update_mode:
            self.mode.set("person")
        if USE_YOLO and YOLO_AVAILABLE and self.yolo.model:
            self.status.set("Person: waiting for person detection")
        else:
            self.status.set("Person mode requested but YOLO unavailable; using motion fallback")
        self.tracker.reset()
        self.servo.set_laser(False)
        if update_mode:
            self.persist_mode()
        self.update_mode_styles()
        self.laser_on.set(False)
        if hasattr(self, "laser_button"):
            self.laser_button.configure(text="Laser: Off")

    def home(self):
        self.servo.home()

    def nudge(self, direction: str):
        if self.mode.get() != "manual":
            self.set_manual()

        if direction == "up":
            self.servo.set_tilt(self.servo.tilt - SERVO_STEP)
        elif direction == "down":
            self.servo.set_tilt(self.servo.tilt + SERVO_STEP)
        elif direction == "left":
            self.servo.set_pan(self.servo.pan + SERVO_STEP)
        elif direction == "right":
            self.servo.set_pan(self.servo.pan - SERVO_STEP)

        self.highlight_until[direction] = time.time() + 0.3

    def toggle_laser(self):
        if self.mode.get() != "manual":
            self.set_manual()
        new_state = not self.laser_on.get()
        self.laser_on.set(new_state)
        self.servo.set_laser(new_state)
        self.laser_button.configure(text=f"Laser: {'On' if new_state else 'Off'}")

    def on_color_change(self):
        choice = self.color_choice.get()
        rng = COLOR_PRESETS.get(choice, None)
        self.tracker.set_color_range(rng)

    def on_size_change(self):
        choice = self.size_choice.get()
        area = SIZE_PRESETS.get(choice, BASE_MIN_CONTOUR_AREA)
        self.tracker.set_min_area(area)

    def show_help(self):
        # Custom help window to control width and show color hints cleanly.
        if hasattr(self, "_help_win") and self._help_win and tk.Toplevel.winfo_exists(self._help_win):
            self._help_win.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("Help")
        win.configure(bg="#0f172a")
        win.geometry("600x520")
        win.transient(self.root)
        self._help_win = win

        colors = {
            "bg": "#0f172a",
            "text": "#e2e8f0",
            "muted": "#cbd5e1",
            "green": "#00c853",
            "orange": "#f59e0b",
            "gray": "#808080",
            "blue": "#3b82f6",
        }

        container = tk.Frame(win, bg=colors["bg"])
        container.pack(fill="both", expand=True, padx=14, pady=12)

        def add_section(title):
            tk.Label(container, text=title, fg=colors["text"], bg=colors["bg"],
                     font=("Helvetica", 12, "bold"), anchor="w", justify="left").pack(fill="x", pady=(8, 4))

        def add_line(text, fg=None, bullet=True):
            prefix = "\u2022 " if bullet else ""
            tk.Label(container, text=prefix + text, fg=fg or colors["text"], bg=colors["bg"],
                     font=("Helvetica", 11), anchor="w", justify="left", wraplength=560).pack(fill="x", pady=1)

        add_section("Modes")
        add_line("Auto: motion tracking; optional color filter and size preset.")
        add_line("Person: YOLO person-only (falls back to motion if YOLO missing).")
        add_line("Manual: arrow buttons/keys move servos; laser is manual only.")

        add_section("Bounding boxes")
        add_line("Active tracked target: green box and centroid dot with label.", fg=colors["green"])
        add_line("Acquiring/temporary target: blue/orange box while locking.", fg=colors["blue"])
        add_line("Other detected objects: thin gray outline (not tracked focus).", fg=colors["gray"])

        add_section("Manual cues")
        add_line("Pressing arrows in manual mode highlights the corresponding frame edge green.", fg=colors["green"])
        add_line("Laser button toggles the laser when in manual mode.", fg=colors["text"])

        add_section("Controls")
        add_line("Color dropdown limits auto detection to the selected hue (None disables).", fg=colors["muted"])
        add_line("Motion size sets minimum blob area (small/medium/large).", fg=colors["muted"])
        add_line("Home recenters servos.", fg=colors["muted"])

    def update_frame(self):
        if not self._running:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.status.set("Camera not available")
            self.root.after(100, self.update_frame)
            return

        if self.mode.get() == "auto":
            detection = self.tracker.process(frame)
        elif self.mode.get() == "person":
            yolo_dets = self.yolo.detect(frame)
            if yolo_dets is None:
                self.status.set("YOLO unavailable; using motion")
                detection = self.tracker.process(frame)
            else:
                detection = self.tracker.process(frame, detections_override=yolo_dets)
        else:
            detection = Detection(None, None, False, [])

        reticle_center = None

        if self.mode.get() in ("auto", "person"):
            label = None
            if detection.locked:
                if self.mode.get() == "person":
                    label = "Tracking Person"
                else:
                    label = "Tracking motion"
                    if self.color_choice.get() != "None":
                        label += f" ({self.color_choice.get()})"
                reticle_center = detection.centroid
            frame = self.draw_detection(frame, detection, label, reticle_center)
            self.auto_control(detection)
        else:
            frame = self.draw_manual_highlights(frame)
            reticle_center = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
            frame = self.draw_reticle(frame, reticle_center)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        self.video_label.configure(image=imgtk)
        self.video_label.image = imgtk

        self.root.after(10, self.update_frame)

    def auto_control(self, detection: Detection):
        if detection.locked and detection.centroid:
            cx, cy = detection.centroid
            dx = (cx - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
            dy = (cy - FRAME_HEIGHT / 2) / (FRAME_HEIGHT / 2)

            new_pan = self.servo.pan - int(dx * SERVO_STEP)
            new_tilt = self.servo.tilt + int(dy * SERVO_STEP)
            self.servo.set_pan(new_pan)
            self.servo.set_tilt(new_tilt)
            self.servo.set_laser(True)
            self.status.set("Locked: tracking target")
        else:
            self.servo.set_laser(False)
            self.status.set("Auto: waiting for motion")

    def draw_detection(self, frame: np.ndarray, detection: Detection, label: str | None, reticle_center) -> np.ndarray:
        # Draw secondary tracks lightly.
        for box in detection.tracks:
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 80, 80), 1)

        if detection.bbox:
            x, y, w, h = detection.bbox
            color = (0, 200, 0) if detection.locked else (0, 128, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            if detection.centroid:
                cv2.circle(frame, detection.centroid, 4, (0, 255, 0), -1)
            if label:
                cv2.putText(frame, label, (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        if reticle_center:
            frame = self.draw_reticle(frame, reticle_center)
        return frame

    def draw_manual_highlights(self, frame: np.ndarray) -> np.ndarray:
        now = time.time()
        active = {k: v for k, v in self.highlight_until.items() if v > now}
        self.highlight_until = active
        thickness = 10
        color = (0, 200, 0)
        h, w, _ = frame.shape

        if "up" in active:
            cv2.rectangle(frame, (0, 0), (w, thickness), color, -1)
        if "down" in active:
            cv2.rectangle(frame, (0, h - thickness), (w, h), color, -1)
        if "left" in active:
            cv2.rectangle(frame, (0, 0), (thickness, h), color, -1)
        if "right" in active:
            cv2.rectangle(frame, (w - thickness, 0), (w, h), color, -1)

        return frame

    def draw_reticle(self, frame: np.ndarray, center: tuple[int, int]) -> np.ndarray:
        cx, cy = int(center[0]), int(center[1])
        color = (20, 40, 255)  # red-ish
        # Outer circle
        cv2.circle(frame, (cx, cy), 45, color, 2, lineType=cv2.LINE_AA)
        # Inner circle
        cv2.circle(frame, (cx, cy), 22, color, 2, lineType=cv2.LINE_AA)
        # Crosshair lines
        cv2.line(frame, (cx - 50, cy), (cx - 15, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx + 15, cy), (cx + 50, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 50), (cx, cy - 15), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + 15), (cx, cy + 50), color, 2, cv2.LINE_AA)
        # Center plus
        cv2.line(frame, (cx - 6, cy), (cx + 6, cy), color, 2, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - 6), (cx, cy + 6), color, 2, cv2.LINE_AA)
        return frame

    def on_close(self):
        self._running = False
        self.cap.release()
        self.servo.set_laser(False)
        self.servo.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

    # -------- Persistence helpers --------
    def update_mode_styles(self):
        current = self.mode.get()
        for key, btn in self.mode_buttons.items():
            btn.configure(style="ModeActive.TButton" if key == current else "Mode.TButton")

    def persist_mode(self):
        try:
            CONFIG_PATH.write_text(self.mode.get(), encoding="utf-8")
        except Exception as exc:
            print(f"Could not save mode: {exc}")

    def load_persisted_mode(self):
        try:
            if CONFIG_PATH.exists():
                saved = CONFIG_PATH.read_text(encoding="utf-8").strip()
                if saved in ("auto", "manual", "person"):
                    self.mode.set(saved)
        except Exception as exc:
            print(f"Could not load saved mode: {exc}")
        # Apply the mode without re-saving
        self.on_mode_change()


if __name__ == "__main__":
    SentryApp().run()
