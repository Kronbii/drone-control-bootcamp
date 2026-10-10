"""Cost tables for cohort/kit-and-costs.md, computed from cohort/kit.csv.

    python tools/kit_cost.py           # print the tables
    python tools/kit_cost.py --write   # rewrite the generated block in kit-and-costs.md
    python tools/kit_cost.py --check   # fail if that block is stale (tests/CI)

Prices are planning ranges (see the notes in kit-and-costs.md), not quotes.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "cohort" / "kit.csv"
DOC = ROOT / "cohort" / "kit-and-costs.md"
BEGIN, END = "<!-- BEGIN:kit-tables -->", "<!-- END:kit-tables -->"
TEAMS = 6  # 18 students in teams of 3
LEAN_RIGS = 3  # the lean option: two teams share each rig


def load() -> list[dict]:
    with open(CSV, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["qty"] = float(r["qty"])
        r["lo"] = float(r["unit_low_usd"]) * r["qty"]
        r["hi"] = float(r["unit_high_usd"]) * r["qty"]
    return rows


RIG_IDS = {
    "motor",
    "esc",
    "props",
    "psu",
    "estop",
    "fuse",
    "wire",
    "imu",
    "pot",
    "arm_switch",
    "beam",
    "axle",
    "bearings",
    "wood",
    "fasteners",
    "printing",
    "cage",
    "clamps",
}


def totals(rows):
    team = [r for r in rows if r["scope"] == "team"]
    shared = [r for r in rows if r["scope"] == "shared"]
    rig = [r for r in team if r["id"] in RIG_IDS]
    bench = [r for r in team if r["id"] not in RIG_IDS]

    def s(rs, k):
        return sum(r[k] for r in rs)

    return {
        "bench": (s(bench, "lo"), s(bench, "hi")),
        "rig": (s(rig, "lo"), s(rig, "hi")),
        "team": (s(team, "lo"), s(team, "hi")),
        "shared": (s(shared, "lo"), s(shared, "hi")),
        "cohort": (
            TEAMS * s(team, "lo") + s(shared, "lo"),
            TEAMS * s(team, "hi") + s(shared, "hi"),
        ),
        "lean": (
            TEAMS * s(bench, "lo") + LEAN_RIGS * s(rig, "lo") + s(shared, "lo"),
            TEAMS * s(bench, "hi") + LEAN_RIGS * s(rig, "hi") + s(shared, "hi"),
        ),
    }


def _fmt(x: float) -> str:
    return f"{x:.1f}" if x < 10 and x != int(x) else f"{x:,.0f}"


def money(lo: float, hi: float) -> str:
    return f"{_fmt(lo)}-{_fmt(hi)}"


def row(r: dict) -> str:
    cost = money(r["lo"], r["hi"])
    return f"| {r['item']} | {r['spec']} | {r['qty']:g} | {cost} | {r['notes']} |"


def total_row(label: str, pair: tuple[float, float], bold: bool = False) -> str:
    lo, hi = f"{pair[0]:,.0f}", f"{pair[1]:,.0f}"
    if bold:
        label, lo, hi = f"**{label}**", f"**{lo}**", f"**{hi}**"
    return f"| {label} | {lo} | {hi} |"


def render(rows) -> str:
    out = [
        "**Per team** (one bench kit and one rig):",
        "",
        "| item | specification | qty | USD (low-high) | notes |",
        "|---|---|---|---|---|",
    ]
    for r in rows:
        if r["scope"] == "team":
            out.append(row(r))
    t = totals(rows)
    out += [
        "",
        "**Shared per cohort:**",
        "",
        "| item | specification | qty | USD (low-high) | notes |",
        "|---|---|---|---|---|",
    ]
    for r in rows:
        if r["scope"] == "shared":
            out.append(row(r))
    out += [
        "",
        "**Totals** (USD, before shipping, customs and the contingency below):",
        "",
        "| | low | high |",
        "|---|---|---|",
        total_row("bench kit per team (labs 1-2)", t["bench"]),
        total_row("rig per team (labs 3-7)", t["rig"]),
        total_row("one team, total", t["team"], bold=True),
        total_row("shared items per cohort", t["shared"]),
        total_row(f"cohort of {TEAMS} teams, one rig each", t["cohort"], bold=True),
        total_row(f"lean cohort: {TEAMS} bench kits, {LEAN_RIGS} shared rigs", t["lean"]),
    ]
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--write", action="store_true")
    g.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    block = render(load())
    if not (a.write or a.check):
        print(block)
        return 0
    doc = DOC.read_text(encoding="utf-8")
    i, j = doc.index(BEGIN) + len(BEGIN), doc.index(END)
    new = doc[:i] + "\n" + block + "\n" + doc[j:]
    if a.check:
        if new != doc:
            print("cohort/kit-and-costs.md is stale: run python tools/kit_cost.py --write")
            return 1
        return 0
    DOC.write_text(new, encoding="utf-8")
    print("updated cohort/kit-and-costs.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
