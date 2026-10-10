"""Save what a lab sketch prints over serial to a CSV file -- and pass your commands on.

    python tools/serial_logger.py --port /dev/ttyUSB0 logs/lab3_run1.csv     # until Ctrl+C
    python tools/serial_logger.py --port COM5 --seconds 40 logs/lab1_pi.csv
    python tools/serial_logger.py --port loop:// --seconds 1 --send hello logs/test.csv

Everything the board prints is written as-is (comment lines starting with '#', the
header, the data rows), so `bootcamp.logs.read_log` can read the file afterwards. Lines
starting with '#' (ARMED, DISARMED, TEST DONE, ...) are also shown in the terminal.

**Type commands into this terminal** (for example `seq`, `kick`, `test`, `kp 2`): every line
you type is sent to the board, exactly as in the Serial Monitor, while logging continues.

Opening the port **resets** most Arduino boards: the sketch restarts, the rig is disarmed
and anything set over serial (gains, `zero`) is back to the values in the sketch. So: start
the logger first, then arm (switch OFF -> ON), then type the command. Put your calibrated
`potCenter` and final gains into the sketch so that a reset does not lose them.

`--send CMD --send-after S` sends a command S seconds after opening (for unattended runs
of sketches that need no arming, such as the RC labs).

Needs pyserial: pip install pyserial  (or pip install -e ".[serial]").
Close the Arduino Serial Monitor/Plotter first: only one program can own the port.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path


def _forward_stdin(ser, stop: threading.Event) -> None:
    """Send every line typed in the terminal to the board."""
    for line in sys.stdin:
        if stop.is_set():
            break
        ser.write((line.rstrip("\r\n") + "\n").encode())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("output", type=Path, help="CSV file to write (folders are created)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0, /dev/ttyACM0, COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = Ctrl+C)")
    ap.add_argument("--send", action="append", default=[], help="command to send automatically")
    ap.add_argument("--send-after", type=float, default=2.5, help="seconds after opening (--send)")
    a = ap.parse_args(argv)
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial is not installed: pip install pyserial", file=sys.stderr)
        return 2

    a.output.parent.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    with (
        serial.serial_for_url(a.port, a.baud, timeout=0.2) as ser,
        open(a.output, "w", encoding="utf-8") as out,
    ):
        if sys.stdin is not None and sys.stdin.isatty():
            threading.Thread(target=_forward_stdin, args=(ser, stop), daemon=True).start()
            print("logging; type commands and press Enter to send them; Ctrl+C to stop")
        t0 = time.monotonic()
        pending = list(a.send)
        lines = 0
        try:
            while not a.seconds or time.monotonic() - t0 < a.seconds:
                if pending and time.monotonic() - t0 >= a.send_after:
                    ser.write((pending.pop(0) + "\n").encode())
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                out.write(text + "\n")
                lines += 1
                if text.startswith("#"):
                    print(text)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
    print(f"wrote {lines} lines to {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
