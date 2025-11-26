# Arduino PT Sentry

Pan/tilt sentry driven by an Arduino Uno and a Linux host. A Python GUI handles motion detection, optional YOLO-based person tracking, manual control, and laser signaling; the Arduino sketch drives two servos and a laser.

## Features
- Modes: **Auto** (motion tracking with optional color filter and size sensitivity), **Person** (YOLO person-only), **Manual** (arrow control, manual laser toggle).
- Bounding boxes with annotations (“Tracking motion” or “Tracking Person”), multi-target tracking with stable lock behavior.
- Reticle overlay at tracked target (auto/person) and centered crosshair in manual mode.
- Right-side professional control panel with mode bar, Home, manual arrows, color and size selectors, laser toggle (manual only), and in-app Help.
- Persists last selected mode across runs; optional YOLO if installed.

## Hardware
- Arduino Uno
- 2× servo motors (pan: D9, tilt: D10)
- Laser diode on D7 via transistor/MOSFET driver
- USB webcam
- 8 V external pack + regulator for servos; common ground with Arduino

## Software layout
- `arduino/pt_sentry.ino` — Servo + laser controller; serial commands: `PAN`, `TILT`, `LASER`, `HOME` at 115200 baud.
- `python/sentry_gui.py` — GUI, motion tracking, optional YOLO person detection, serial bridge.
- `PROJECT_SCOPE.md` — Requirements and notes.
- `requirements.txt` — Python dependencies (YOLO optional).
- See `PROJECT_SCOPE.md` for detailed functional requirements, pins, serial protocol, and safety/constraints.
- See `CHANGELOG.md` for the sequence of changes.
- KiCad wiring netlist: `wiring/kicad_netlist.txt`.
- Importable KiCad netlist: `wiring/kicad_net.net` (assign footprints/symbols as needed).

## Setup (Linux)
```bash
cd "Arduino PT Sentry"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
# If you skip YOLO, remove ultralytics/torch/torchvision from requirements before install.
```

Install Tk if missing:
- Ubuntu/Debian: `sudo apt-get install -y python3-tk`
- Fedora: `sudo dnf install -y python3-tkinter`

System deps (Fedora):
```bash
sudo dnf groupinstall -y "Development Tools"
sudo dnf install -y python3-devel opencv opencv-devel
```
System deps (Ubuntu/Debian):
```bash
sudo apt-get update
sudo apt-get install -y build-essential python3-dev libopencv-dev python3-opencv
```

Flash the Arduino:
1) Open `arduino/pt_sentry.ino` in Arduino IDE
2) Select board/port, upload

## Run the GUI
```bash
cd "Arduino PT Sentry"
source .venv/bin/activate
python python/sentry_gui.py
```
- Set `SERIAL_PORT` in `sentry_gui.py` if needed.
- Person mode requires YOLO (`ultralytics`/`torch`) and a `yolov8n.pt` model (downloaded automatically if not present).

## Usage
- Auto: motion tracking drives servos; choose a color filter (None/Red/Green/Blue/Yellow) and motion size (Small/Medium/Large); reticle follows the tracked target.
- Person: YOLO person-only tracking; falls back to motion if YOLO unavailable.
- Manual: arrow buttons or arrow keys; laser toggle button controls the laser; Home returns to center; reticle is centered.
- Status bar shows current state; mode selection persists across runs; “?” opens Help explaining modes, colors, and cues.

## Notes
- Do not power servos from the Uno 5 V pin; use a regulator and common ground.
- Keep laser eye-safe and consider a physical enable switch.
- Tune `SERVO_STEP`, limits, and size/color thresholds for your mount and environment.
