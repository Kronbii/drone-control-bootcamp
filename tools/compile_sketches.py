"""Compile every lab sketch with arduino-cli.

    python tools/compile_sketches.py                         # easyPID from the sketchbook
    python tools/compile_sketches.py --library ../easyPID    # or from a local checkout
    python tools/compile_sketches.py --fqbn arduino:avr:nano

Install the toolchain once (no sudo, everything in the user's directories):
    arduino-cli core update-index && arduino-cli core install arduino:avr
    arduino-cli lib install easyPID@1.1.0     # or use --library with a checkout
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOARDS = ("arduino:avr:uno", "arduino:avr:nano")


def sketches() -> list[Path]:
    return sorted(
        p for p in (ROOT / "arduino").iterdir() if p.is_dir() and (p / f"{p.name}.ino").exists()
    )


def compile_one(sketch: Path, fqbn: str, library: str | None, cli: str) -> tuple[bool, str]:
    with tempfile.TemporaryDirectory() as out:
        cmd = [cli, "compile", "-b", fqbn, "--warnings", "all", "--output-dir", out]
        if library:
            cmd += ["--library", library]
        cmd.append(str(sketch))
        res = subprocess.run(cmd, capture_output=True, text=True)
    text = res.stdout + res.stderr
    warnings = [ln for ln in text.splitlines() if "warning:" in ln and str(sketch) in ln]
    ok = res.returncode == 0 and not warnings
    return ok, text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--fqbn", action="append", help="board(s); default: Uno and Nano")
    ap.add_argument("--library", help="path to an easyPID checkout (otherwise the installed one)")
    ap.add_argument("--cli", default=shutil.which("arduino-cli") or "arduino-cli")
    a = ap.parse_args(argv)
    boards = a.fqbn or list(DEFAULT_BOARDS)
    failed = 0
    for sk in sketches():
        for b in boards:
            ok, text = compile_one(sk, b, a.library, a.cli)
            size = next((ln for ln in text.splitlines() if ln.startswith("Sketch uses")), "")
            print(f"{'ok  ' if ok else 'FAIL'} {sk.name:22s} {b:18s} {size[:60]}")
            if not ok:
                failed += 1
                print(text)
    print(f"{len(sketches())} sketches x {len(boards)} boards, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
