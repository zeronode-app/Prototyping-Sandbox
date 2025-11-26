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
