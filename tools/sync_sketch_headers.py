"""Copy the shared headers in arduino/common/ next to every sketch that includes them.

Arduino sketches must be self-contained folders (the IDE opens a folder, nothing else),
so the few shared headers are *copied* into each sketch rather than installed as a
library. This script keeps the copies identical to the master versions.

    python tools/sync_sketch_headers.py           # update the copies
    python tools/sync_sketch_headers.py --check   # fail if a copy differs (CI)
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / "arduino" / "common"
INCLUDE = re.compile(r'^\s*#include\s+"([^"]+)"', re.M)


def sketches() -> list[Path]:
    return sorted(
        p for p in (ROOT / "arduino").iterdir() if p.is_dir() and (p / f"{p.name}.ino").exists()
    )


def needed_headers(sketch: Path) -> set[str]:
    todo = INCLUDE.findall((sketch / f"{sketch.name}.ino").read_text())
    seen: set[str] = set()
    while todo:
        h = todo.pop()
        if h in seen or not (COMMON / h).exists():
            continue
        seen.add(h)
        todo += INCLUDE.findall((COMMON / h).read_text())
    return seen


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    bad = []
    for sk in sketches():
        for h in sorted(needed_headers(sk)):
            master = (COMMON / h).read_text()
            copy = sk / h
            if a.check:
                if not copy.exists() or copy.read_text() != master:
                    bad.append(str(copy.relative_to(ROOT)))
            elif not copy.exists() or copy.read_text() != master:
                copy.write_text(master)
                print(f"updated {copy.relative_to(ROOT)}")
    if bad:
        print("out-of-date header copies (run tools/sync_sketch_headers.py):", *bad, sep="\n  ")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
