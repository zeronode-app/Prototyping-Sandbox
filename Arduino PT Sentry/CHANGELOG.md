# Changelog

## v0.1.0 - Initial session
1. Added `PROJECT_SCOPE.md` capturing goals, requirements, pins, protocol, and safety notes.
2. Implemented Arduino sketch `arduino/pt_sentry.ino` for pan/tilt servos and laser control via serial commands.
3. Created Python GUI `python/sentry_gui.py` with motion detection, tracking, and manual control.
4. Upgraded tracking to multi-track, security-camera style with lock persistence and switch hysteresis.
5. Added optional YOLO person-only detection mode with fallback to motion.
6. Restyled GUI with dark/pro layout and segmented mode selector.
7. Persisted last-selected mode across runs and ensured active mode highlights on startup.
8. Moved controls into a right-side panel next to the video feed.
9. Added manual laser toggle (manual mode), bounding box annotations (“Tracking motion/person”), color filter selector, and motion size presets.
10. Expanded `.gitignore` for venvs, caches, YOLO artifacts, model/config files.
11. Added `README.md` and `requirements.txt`; documented setup, usage, and dependencies.
12. Documented distro-specific setup notes for Ubuntu/Debian and Fedora.
13. Clarified scope vs. README roles and added non-goals to `PROJECT_SCOPE.md`.

## v0.1.1 - UI/UX polish
1. Added reusable Tkinter styling template under `style-guide/` and updated top-level README pointer.
2. Introduced in-app Help dialog (top-right “?”) with colored cues for tracked/locked/other detections and control explanations.
3. Streamlined control panel layout and styling; switched selectors to themed comboboxes.
4. Added red crosshair reticle overlay: follows tracked target in auto/person, centers in manual.
5. Added KiCad-friendly wiring netlist at `wiring/kicad_netlist.txt`.
6. Added importable KiCad netlist file `wiring/kicad_net.net` (assign symbols/footprints in KiCad).
