# Setup

Everything runs on a normal laptop (Linux, macOS or Windows) with Python 3.11+ and the
Arduino IDE or `arduino-cli`. No administrator rights are needed beyond installing those.

## 1. The course repository and Python

```bash
git clone <this repository> drone-control-bootcamp
cd drone-control-bootcamp
python3 -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -e ".[notebooks,serial]"  # the bootcamp package, JupyterLab, pyserial
```

(`make setup` does the same plus the development tools, on Linux/macOS.)

Open the notebooks with `jupyter lab notebooks/`. Each starts with a cell of imports: if
it runs, you are set. If Jupyter cannot find `bootcamp`, the notebook is using another
Python: `python -m ipykernel install --user --name bootcamp` (inside the venv) and pick
the "bootcamp" kernel.

## 2. rotorfault (session 6, parts of sessions 5 and 7)

`rotorfault` is Rami Kronbi's rotor-fault simulation study, used here as a library. Install
it from its repository:

```bash
pip install "rotorfault @ git+https://github.com/Kronbii/rotor-fault-recovery"
# or, from a local checkout:
pip install -e ../rotor-fault-recovery
```

**Fallback without internet or before the repository is public:** your instructor
brings a wheel file on a USB stick. Install it with
`pip install rotorfault-0.1.0-py3-none-any.whl`. (Instructors: build it from a checkout with
`pip wheel ./rotor-fault-recovery --no-deps -w dist`.) It needs only numpy, scipy and
matplotlib, which the course package already installed.

Check: `rotorfault simulate --traj hover --loe 1.0 --duration 4 --plot logs/loss.png`.

Notebooks 1-4 do not need rotorfault. Notebook 5 uses it for one comparison and skips it
if missing; notebook 6 requires it.

## 3. Arduino

1. Install the Arduino IDE 2 (or `arduino-cli`).
2. Install the AVR core: *Tools → Board → Boards Manager → Arduino AVR Boards*
   (`arduino-cli core install arduino:avr`).
3. Install **easyPID 1.1.0**: *Library Manager → search "easyPID"*
   (`arduino-cli lib install easyPID@1.1.0`).
4. Open a sketch folder from `arduino/` (each folder is self-contained: the shared headers
   are copied into it), select the board (Arduino Uno or Nano) and the port, upload.

Nano clones: if upload fails, choose *Processor → ATmega328P (Old Bootloader)*. Clones with
a CH340 USB chip may need its driver on Windows and older macOS. On Linux, add yourself to
the `dialout` group once (`sudo usermod -aG dialout $USER`, then log out and in).

All sketches print at **115200 baud**.

## 4. Logging serial data

```bash
python tools/serial_logger.py --port /dev/ttyUSB0 logs/run1.csv           # until Ctrl+C
python tools/serial_logger.py --port COM5 --seconds 30 logs/run2.csv
```

While it logs, **type commands into the logger's terminal** (`seq`, `kick`, `kp 2`, ...):
they are sent to the board as from the Serial Monitor. Opening the port **resets** most
Arduino boards: the sketch restarts with the values written in it, and a rig is disarmed.
Start the logger first, then arm and type your command; keep calibrations (`potCenter`) and
final gains in the sketch itself. Close the Serial Monitor/Plotter first (one program per port). Read a log in a notebook
with `bootcamp.logs.read_log("logs/run1.csv")`. The `logs/` folder is ignored by git.

## 5. For instructors and contributors

```bash
make setup        # venv with dev tools (ruff, pytest, nbclient) and rotorfault
make check        # lint + all tests, including executing every notebook
make sketches EASYPID=../easyPID   # compile all sketches for Uno and Nano
make notebooks    # rebuild notebooks/ and solutions/notebooks/ from notebooks/src/
make figures      # regenerate figures/ from the solution notebooks
make slides       # rebuild slides/pdf/ (needs Node.js and Chrome)
```

Edit notebooks in `notebooks/src/*.py` (percent format), never the `.ipynb` files: they
are generated, and the tests fail if they are stale. Edit shared Arduino headers in
`arduino/common/` and run `python tools/sync_sketch_headers.py`.
