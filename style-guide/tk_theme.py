"""Reusable Tkinter styling helper based on the Arduino PT Sentry UI.

Usage:
    from style_guide.tk_theme import apply_theme
    root = tk.Tk()
    style = apply_theme(root)
    # build your UI...
"""

from tkinter import ttk


THEME_COLORS = {
    "bg": "#0f172a",
    "card": "#0b1224",
    "status": "#111827",
    "accent": "#2563eb",
    "accent_hover": "#1d4ed8",
    "accent_active": "#1e40af",
    "neutral": "#1f2937",
    "text": "#e2e8f0",
    "muted": "#cbd5e1",
}


def apply_theme(root):
    style = ttk.Style(root)
    style.theme_use("clam")

    style.configure("TFrame", background=THEME_COLORS["bg"])
    style.configure("Card.TFrame", background=THEME_COLORS["card"], relief="raised", borderwidth=1)
    style.configure("TLabel", background=THEME_COLORS["bg"], foreground=THEME_COLORS["text"], font=("Helvetica", 11))
    style.configure("Header.TLabel", background=THEME_COLORS["bg"], foreground=THEME_COLORS["text"], font=("Helvetica", 14, "bold"))
    style.configure(
        "Status.TLabel",
        background=THEME_COLORS["status"],
        foreground=THEME_COLORS["text"],
        font=("Helvetica", 11),
        padding=8,
        anchor="center",
        relief="groove",
    )

    style.configure("TButton", font=("Helvetica", 10, "bold"), padding=8)
    style.configure(
        "Mode.TButton",
        background=THEME_COLORS["neutral"],
        foreground=THEME_COLORS["text"],
        font=("Helvetica", 10, "bold"),
        padding=(12, 8),
        relief="flat",
        borderwidth=0,
    )
    style.configure(
        "ModeActive.TButton",
        background=THEME_COLORS["accent"],
        foreground=THEME_COLORS["text"],
        font=("Helvetica", 10, "bold"),
        padding=(12, 8),
        relief="flat",
        borderwidth=0,
    )

    style.map(
        "TButton",
        background=[("active", THEME_COLORS["accent_hover"]), ("!active", "#1e293b")],
        foreground=[("active", THEME_COLORS["text"]), ("!active", THEME_COLORS["text"])],
    )
    style.map(
        "Mode.TButton",
        background=[("active", THEME_COLORS["accent_active"]), ("!active", THEME_COLORS["neutral"])],
        foreground=[("active", THEME_COLORS["text"]), ("!active", THEME_COLORS["text"])],
    )
    style.map(
        "ModeActive.TButton",
        background=[("active", THEME_COLORS["accent_hover"]), ("!active", THEME_COLORS["accent"])],
        foreground=[("active", THEME_COLORS["text"]), ("!active", THEME_COLORS["text"])],
    )

    root.configure(bg=THEME_COLORS["bg"])
    return style
