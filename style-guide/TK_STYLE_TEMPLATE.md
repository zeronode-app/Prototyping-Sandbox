# UI Styling Template (Tkinter)

Design tokens (from Arduino PT Sentry):
- Background: `#0f172a`
- Card: `#0b1224`
- Status bar: `#111827`
- Accent (active): `#2563eb`
- Accent (hover): `#1d4ed8`
- Neutral button: `#1f2937`
- Text primary: `#e2e8f0`
- Text muted: `#cbd5e1`
- Mono font: `Helvetica` (bold for headers/buttons)

Layout pattern:
- Left: primary canvas/video area inside a card.
- Right: vertical control panel with segmented mode selector, action buttons, directional pad, dropdowns, and status bar.
- Use padding 6–10 px; prefer grid with weights so controls scale.

Component guidelines:
- Mode buttons: segmented group; inactive uses neutral color, active uses accent.
- Buttons: bold font, rounded via padding, flat relief.
- Labels: muted text for form labels; primary text for headers/status.
- Cards: slight raised relief with darker background than root.
- Status bar: full-width in control panel, distinct background, centered text.

Usage:
1) Import and call `apply_theme(root)` from `style-guide/tk_theme.py`.
2) Place main content in a two-column grid: video card in column 0, control panel in column 1.
3) Reuse `Mode.TButton` / `ModeActive.TButton` for segmented modes; `Status.TLabel` for status bar; `Card.TFrame` for framed containers.
