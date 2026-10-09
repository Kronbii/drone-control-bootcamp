"""Plot style and figure export shared by all notebooks.

The palette follows the course's visual identity (dark surface, sage accent) with data
colours stepped for a dark background and checked for colour-vision-deficiency
separation: sage-green, magenta, blue, amber -- at most four series per axes, always
with a legend, and setpoints/references drawn in muted ink (dashed) rather than in a
series colour.

Figures are written to ``figures/`` only when ``BOOTCAMP_SAVE_FIGURES=1`` (set by
``make figures``), so students running notebooks never overwrite the committed ones.
"""

from __future__ import annotations

import os
from pathlib import Path

import matplotlib as mpl

BG = "#0b0d0c"
SURFACE = "#121614"
INK = "#eef1ec"
INK_2 = "#b3b8b2"  # ink at ~74 % on the surface
INK_3 = "#7f857f"  # ink at ~52 %
HAIR = "#2a302c"
SAGE_UI = "#b7d3a8"  # identity accent (text/annotations, not data)
SERIES = ["#5fa850", "#c75bb4", "#3987e5", "#c98500"]
REF = dict(color=INK_3, ls="--", lw=1.2)


def use_style() -> None:
    mpl.rcParams.update(
        {
            "figure.facecolor": BG,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": BG,
            "axes.edgecolor": HAIR,
            "axes.labelcolor": INK_2,
            "axes.titlecolor": INK,
            "axes.titleweight": "bold",
            "axes.titlesize": 11,
            "axes.labelsize": 9.5,
            "axes.prop_cycle": mpl.cycler(color=SERIES),
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "grid.color": HAIR,
            "grid.linewidth": 0.6,
            "xtick.color": INK_3,
            "ytick.color": INK_3,
            "xtick.labelcolor": INK_2,
            "ytick.labelcolor": INK_2,
            "xtick.labelsize": 8.5,
            "ytick.labelsize": 8.5,
            "text.color": INK,
            "legend.frameon": False,
            "legend.fontsize": 8.5,
            "legend.labelcolor": INK_2,
            "lines.linewidth": 1.8,
            "font.family": ["DejaVu Sans"],
            "figure.dpi": 100,
            "savefig.dpi": 160,
            "savefig.bbox": "tight",
        }
    )


def repo_root() -> Path:
    """The repository root (the directory holding ``pyproject.toml``)."""
    here = Path.cwd().resolve()
    for p in (here, *here.parents):
        if (p / "pyproject.toml").exists() and (p / "src" / "bootcamp").exists():
            return p
    return Path(__file__).resolve().parents[2]


def save(fig, name: str) -> Path | None:
    """Write ``figures/<name>.png`` when ``BOOTCAMP_SAVE_FIGURES=1``."""
    if os.environ.get("BOOTCAMP_SAVE_FIGURES") != "1":
        return None
    out = repo_root() / "figures" / f"{name}.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out)
    return out


def step_axes(ax, title: str | None = None, xlabel="time (s)", ylabel=None):
    if title:
        ax.set_title(title, loc="left")
    ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    return ax


use_style()
