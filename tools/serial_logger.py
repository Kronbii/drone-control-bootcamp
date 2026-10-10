"""Save what a lab sketch prints over serial to a CSV file.

    python tools/serial_logger.py --port /dev/ttyUSB0 --seconds 40 logs/lab1_pi.csv
    python tools/serial_logger.py --port COM5 logs/capstone.csv        # until Ctrl+C
    python tools/serial_logger.py --port /dev/ttyACM0 --send "test" logs/team_capstone.csv

Everything the board prints is written as-is (comment lines starting with '#', the
header, the data rows), so `bootcamp.logs.read_log` can read the file afterwards.
`--send` writes one command line to the board after opening the port (for example
"seq" or "test"), so the log starts exactly with the experiment.

Needs pyserial: pip install pyserial  (or pip install -e ".[serial]").
Close the Arduino Serial Monitor/Plotter first: only one program can own the port.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("output", type=Path, help="CSV file to write (folders are created)")
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0, /dev/ttyACM0, COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = Ctrl+C)")
    ap.add_argument("--send", action="append", default=[], help="command(s) to send after opening")
    a = ap.parse_args(argv)
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial is not installed: pip install pyserial", file=sys.stderr)
        return 2

    a.output.parent.mkdir(parents=True, exist_ok=True)
    with (
        serial.Serial(a.port, a.baud, timeout=0.2) as ser,
        open(a.output, "w", encoding="utf-8") as out,
    ):
        time.sleep(2.0)  # opening the port resets most Arduinos; wait for the banner
        for c in a.send:
            ser.write((c + "\n").encode())
        t0 = time.monotonic()
        lines = 0
        try:
            while not a.seconds or time.monotonic() - t0 < a.seconds:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                out.write(text + "\n")
                lines += 1
                if text.startswith("#"):
                    print(text)  # show messages such as ARMED / DISARMED / TEST DONE
        except KeyboardInterrupt:
            pass
    print(f"wrote {lines} lines to {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
