"""Build the student and solution notebooks from one source per session.

Sources live in ``notebooks/src/sNN_*.py`` in the "percent" format, so they can be
linted, diffed and reviewed like code::

    # %% [markdown]
    # # Title
    # Some *markdown*.

    # %%
    import numpy as np

    # %% [exercise 1.2]
    def my_function(x):
        ### BEGIN SOLUTION
        return 2 * x
        ### END SOLUTION

    # %% [check 1.2]
    check_cases("1.2", my_function, [((1,), 2)])

* ``[exercise X.Y]`` cells hold the code students complete. In the student version
  every ``### BEGIN SOLUTION ... ### END SOLUTION`` block becomes a TODO that raises
  ``NotImplementedError``; in the solution version the markers are simply removed.
* ``[check X.Y]`` cells run the auto-checks. They are tagged ``check`` so the tests can
  verify that, in the student version, *only* these cells fail.
* In markdown, ``<!-- BEGIN SOLUTION --> ... <!-- END SOLUTION -->`` (a written answer)
  becomes "*Your answer here*" for students.

Usage::

    python tools/build_notebooks.py           # write notebooks/ and solutions/notebooks/
    python tools/build_notebooks.py --check   # fail if the committed notebooks are stale
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "notebooks" / "src"
STUDENT_DIR = ROOT / "notebooks"
SOLUTION_DIR = ROOT / "solutions" / "notebooks"

HEADER = re.compile(r"^# %%(?:\s*\[(markdown|exercise|check)(?:\s+([\w.]+))?\])?\s*$")
BEGIN, END = "### BEGIN SOLUTION", "### END SOLUTION"
MD_BEGIN, MD_END = "<!-- BEGIN SOLUTION -->", "<!-- END SOLUTION -->"


@dataclass
class Cell:
    kind: str  # markdown | code | exercise | check
    ex: str | None
    lines: list[str] = field(default_factory=list)


def parse(text: str) -> list[Cell]:
    cells: list[Cell] = []
    cur: Cell | None = None
    for line in text.splitlines():
        m = HEADER.match(line)
        if m:
            cur = Cell(m.group(1) or "code", m.group(2))
            cells.append(cur)
            continue
        if cur is None:
            if line.strip():
                raise ValueError(f"text before the first cell marker: {line!r}")
            continue
        cur.lines.append(line)
    for c in cells:
        while c.lines and not c.lines[-1].strip():
            c.lines.pop()
        while c.lines and not c.lines[0].strip():
            c.lines.pop(0)
        if c.kind in ("exercise", "check") and not c.ex:
            raise ValueError(f"{c.kind} cell without an id")
    return cells


def _markdown(lines: list[str]) -> list[str]:
    out = []
    for ln in lines:
        if ln == "#":
            out.append("")
        elif ln.startswith("# "):
            out.append(ln[2:])
        else:
            raise ValueError(f"markdown line must start with '# ': {ln!r}")
    return out


def _strip_code(lines: list[str], ex: str | None, student: bool) -> list[str]:
    out, inside = [], False
    for ln in lines:
        s = ln.strip()
        if s == BEGIN:
            if inside:
                raise ValueError("nested BEGIN SOLUTION")
            inside = True
            if student:
                ind = ln[: len(ln) - len(ln.lstrip())]
                out.append(f"{ind}# TODO (exercise {ex}): your code here")
                out.append(f'{ind}raise NotImplementedError("exercise {ex}")')
            continue
        if s == END:
            if not inside:
                raise ValueError("END SOLUTION without BEGIN")
            inside = False
            continue
        if not (inside and student):
            out.append(ln)
    if inside:
        raise ValueError("unterminated BEGIN SOLUTION")
    return out


def _strip_markdown(lines: list[str], student: bool) -> list[str]:
    out, inside = [], False
    for ln in lines:
        s = ln.strip()
        if s == MD_BEGIN:
            inside = True
            if student:
                out.append("*Your answer here.*")
            continue
        if s == MD_END:
            inside = False
            continue
        if not (inside and student):
            out.append(ln)
    return out


def build(src: Path, student: bool) -> dict:
    cells = parse(src.read_text(encoding="utf-8"))
    nb_cells = []
    for i, c in enumerate(cells):
        cid = f"{src.stem.replace('_', '-')}-{i:03d}"
        if c.kind == "markdown":
            body = _strip_markdown(_markdown(c.lines), student)
            nb_cells.append({"cell_type": "markdown", "id": cid, "metadata": {}, "source": body})
            continue
        body = _strip_code(c.lines, c.ex, student) if c.kind == "exercise" else c.lines
        meta: dict = {}
        if c.kind in ("exercise", "check"):
            meta = {"tags": [c.kind], "exercise": c.ex}
        nb_cells.append(
            {
                "cell_type": "code",
                "execution_count": None,
                "id": cid,
                "metadata": meta,
                "outputs": [],
                "source": body,
            }
        )
    for c in nb_cells:  # nbformat stores source as a list of lines with newlines
        src_lines = c["source"]
        c["source"] = [ln + "\n" for ln in src_lines[:-1]] + src_lines[-1:]
    return {
        "cells": nb_cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
            "bootcamp": {"version": "solution" if not student else "student", "source": src.name},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }


def dumps(nb: dict) -> str:
    return json.dumps(nb, indent=1, ensure_ascii=False) + "\n"


def targets() -> list[tuple[Path, Path, Path]]:
    out = []
    for src in sorted(SRC.glob("s*.py")):
        name = src.stem + ".ipynb"
        out.append((src, STUDENT_DIR / name, SOLUTION_DIR / name))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="only verify the notebooks are current")
    a = ap.parse_args(argv)
    stale = []
    for src, stu, sol in targets():
        for path, student in ((stu, True), (sol, False)):
            text = dumps(build(src, student))
            if a.check:
                if not path.exists() or path.read_text(encoding="utf-8") != text:
                    stale.append(path.relative_to(ROOT))
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8")
                print(f"wrote {path.relative_to(ROOT)}")
    if stale:
        print("stale notebooks (run python tools/build_notebooks.py):", *stale, sep="\n  ")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
