# Arduino Pan-Tilt Sentry Scope

Goal: Build a Linux-hosted motion-tracking pan/tilt sentry using an Arduino Uno, USB webcam, two servos, and a small laser indicator triggered on target lock.

## Functional requirements
- Motion detection from webcam frames; show live video with bounding box over detected moving target.
- Lock on the first detected motion, continue tracking until target exits the frame; turn on laser when locked.
- GUI view that displays video feed, target bounding box, and mode indicator; pro-styled right-side control panel.
- Modes:
  - **Auto**: motion detection + tracking drives servos; optional color filtering and adjustable motion blob size (small/medium/large).
  - **Person**: YOLO-based person-only tracking (falls back to motion if YOLO unavailable).
  - **Manual**: arrow buttons move servos; matching edge of the frame highlights green while pressed; manual laser toggle button.
- Bounding boxes annotated (“Tracking motion” with color hint; “Tracking Person” in person mode).
- Overlay reticle (crosshair) at tracked target in auto/person; centered reticle in manual mode.
- In-app Help dialog describing modes, cues, colors, and controls.
- Laser toggles when a target is locked in auto/person; manual laser toggle in manual mode.
- Runs on a Python script communicating with Arduino over serial.

## Non-goals (current)
- Object classification beyond person/motion (except optional YOLO person class)
- Networking/remote control UX
- Autonomous firing or power-actuated outputs (laser only)
- Audio alerts or recording pipeline

## Hardware
- Arduino Uno
- 2x servo motors (pan, tilt)
- USB webcam
- Laser diode + transistor or MOSFET driver on a digital pin
- 8 V external battery pack powering servos via suitable regulator; common ground with Arduino

## Interfaces and pins (defaults used in code)
- Pan servo: D9
- Tilt servo: D10
- Laser control: D7 (active HIGH)
- Serial: 115200 baud, newline-terminated commands

## Command protocol (Python → Arduino)
- `PAN <degrees>`: set pan angle 0–180
- `TILT <degrees>`: set tilt angle 0–180
- `LASER <0|1>`: turn laser off/on
- `HOME`: move servos to midpoint (90°, 90°)

## Software pieces
- `arduino/pt_sentry.ino`: servo + laser controller; listens to serial commands.
- `python/sentry_gui.py`: OpenCV + Tkinter app for motion detection, tracking, GUI, and serial control.

## Safety/notes
- Use an external regulator for servos; do not power servos from the Uno 5 V pin.
- Keep laser power eye-safe; add a physical enable switch if possible.
- Tune motion thresholds and servo limits to your mount’s geometry.
