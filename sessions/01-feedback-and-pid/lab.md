# Lab 1 · easyPID on an Arduino: a simulated plant, then a real one

*About 45 minutes, teams of 2-3. Low voltage only (5 V from USB): no special safety
rules beyond normal bench practice. Electrolytic capacitors are polarised: the stripe
(minus) goes to GND.*

**You need:** Arduino Uno or Nano + USB cable, breadboard, 10 kΩ resistor, 100 µF
electrolytic capacitor (≥ 10 V), 3 jumper wires; a laptop with the Arduino IDE (or
`arduino-cli`) and easyPID 1.1.0; optionally Python with `pyserial` for logging.

**You hand in:** one plot of your measured closed-loop response next to the notebook's
prediction (section 6 of notebook 1), and the three numbers asked for in part B.

## Part A · No wiring: a simulated plant (15 min)

1. Open `arduino/s01_sim_plant/s01_sim_plant.ino`, select your board and port, upload.
2. Open **Tools → Serial Plotter** at **115200 baud**. You see setpoint `sp`, output `y`,
   controller output `u` and the three PID terms. The board simulates
   $\tau\dot y + y = Ku$ with $\tau = 1$ s, $K = 1.5$, and the sketch starts with P only, $K_p = 1$.
3. Type commands in the box at the bottom (newline ending):
   - `kp 1` → note where `y` settles. Compare with $\frac{KK_p}{1+KK_p}\,r = 0.6 \cdot 50 = 30$.
   - `kp 4` → closer, faster. Predict the new value first.
   - `ki 1` → the offset disappears. Watch the `i` trace grow.
   - `sp 80`, `sp 20` → step the setpoint.
   - `dist 20` → add a constant input disturbance. P only: what happens? With `ki 1`?
   - `kp 20` → why does `u` hit 100 and stay there for a while?
4. Write down: final value with `kp 1` and `kp 4` (P only), and whether they match the formula.

## Part B · A real plant: the RC circuit (25 min)

### Wiring

```
   D5 ----[ 10 kΩ ]----+---- A0
                       |
                    [+ 100 µF]      stripe (-) to GND
                       |
   GND ----------------+
```

The capacitor voltage obeys $RC\,\dot v + v = u$ with $RC = 10^4 \cdot 10^{-4} = 1$ s and
gain 1 V/V, where $u$ is the *average* voltage of the PWM pin. Exactly the plant of the
notebook.

### Steps

1. Upload `arduino/s01_rc_plant/s01_rc_plant.ino`, open the Serial Plotter (115200).
2. **Open loop.** Type `open`, then `u 2.5`. The voltage rises to 2.5 V. Using the plotter
   (or a log, below), estimate the time to reach 63 % of the final value: that is your
   **measured τ**. Your capacitor has a tolerance (often ±20 %), so it need not be 1.00 s.
3. **P control.** Type `u 0`, wait for 0 V, then `pid`, `kp 1`, `ki 0`, `sp 2.5`.
   Measure the final value. Predicted: $\frac{1}{2}\cdot 2.5 = 1.25$ V. Then `kp 4`
   (predicted 2.0 V).
4. **PI control.** `kp 2`, `ki 1`. Then let the sketch step the setpoint for you:
   `steps 1.5 3.5 8` (8 s at 1.5 V, 8 s at 3.5 V, repeating).
5. **Log it.** Close the Serial Plotter (only one program can own the port) and run
   ```bash
   python tools/serial_logger.py --port /dev/ttyUSB0 --seconds 40 logs/lab1_pi.csv
   ```
   (Windows: `--port COM5`; macOS: `/dev/cu.usbserial-...`.) Load it in the notebook with
   `bootcamp.logs.read_log("logs/lab1_pi.csv")` and plot `y` against the prediction of
   notebook section 6. If you have no logger, a screenshot of the Serial Plotter will do.
6. **Try `kp 4 ki 3`.** Overshoot appears. Why does the first part of the rise look like
   an open-loop charge towards 5 V rather than a fast exponential? (Hint: look at `u`.)

**Record:** measured τ; final value with `kp 1` (P only); overshoot with `kp 4 ki 3`.

## Part C · If you finish early (5 min)

- Add a second RC stage (A0 → 10 kΩ → A1 → 100 µF → GND), type `tap 1` to measure on A1,
  and repeat step 4 with `kp 2 ki 1`. Overshoot appears even though each stage alone is
  first order: two lags make a second-order plant. This is the plant of lab 2.
- Look at easyPID's `examples/BasicPID` and find the three lines that correspond to
  part A.

## Checklist before you leave

- [ ] measured τ, P-only final value, PI overshoot written down
- [ ] one plot of measured vs predicted response
- [ ] components back in the team box; capacitor discharged (unplug USB and wait 10 s)
