"""Reading the CSV logs that the lab sketches print over serial.

Every sketch prints comment lines starting with ``#`` (banner, settings, warnings), one
header line of comma-separated column names, then numeric rows. ``tools/serial_logger.py``
saves that stream to a file; :func:`read_log` turns it into numpy arrays and quietly
drops rows that were cut in half when the logger started or stopped.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def parse_lines(lines) -> dict[str, np.ndarray]:
    header: list[str] | None = None
    rows: list[list[float]] = []
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if header is None:
            if all(_is_name(p) for p in parts):
                header = parts
            continue
        if len(parts) != len(header):
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:
            if all(_is_name(p) for p in parts):  # a sketch reset prints the header again
                header = parts
                rows = []
            continue
    if header is None:
        raise ValueError("no header line found (expected comma-separated column names)")
    data = np.array(rows, dtype=float).reshape(-1, len(header))
    return {name: data[:, i] for i, name in enumerate(header)}


def _is_name(s: str) -> bool:
    return bool(s) and (s[0].isalpha() or s[0] == "_")


def read_log(path: str | Path) -> dict[str, np.ndarray]:
    with open(path, encoding="utf-8", errors="replace") as fh:
        return parse_lines(fh)


def write_log(path: str | Path, columns: dict[str, np.ndarray], comment: str = "") -> None:
    """Write arrays in the same format the sketches print (used for example logs)."""
    names = list(columns)
    data = np.column_stack([np.asarray(columns[n], dtype=float) for n in names])
    with open(path, "w", encoding="utf-8") as fh:
        for c in comment.splitlines():
            fh.write(f"# {c}\n")
        fh.write(",".join(names) + "\n")
        for row in data:
            fh.write(",".join(f"{v:.6g}" for v in row) + "\n")
