"""Portfolio plates: the hero figure's panels, one per image, sized for the portfolio's frames.

A square plate (the seesaw rig) and 4:3 plates for the three panels, drawn by the same code as
``tools/hero_figure.py``. Usage::

    python tools/portfolio_plates.py <outdir>
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hero_figure import panel_cascade, panel_fault, panel_pid  # noqa: E402


def plate(panel, figsize, name, out: Path):
    fig, ax = plt.subplots(figsize=figsize)
    panel(ax)
    fig.tight_layout()
    path = out / f"{name}.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(path)


def main(argv: list[str]) -> int:
    out = Path(argv[1]) if len(argv) > 1 else Path("figures/portfolio")
    out.mkdir(parents=True, exist_ok=True)
    plate(panel_cascade, (5.4, 5.4), "plate", out)
    plate(panel_pid, (6.4, 4.8), "pid-rc", out)
    plate(panel_cascade, (6.4, 4.8), "rig-cascade", out)
    plate(panel_fault, (6.4, 4.8), "rotor-loss", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
