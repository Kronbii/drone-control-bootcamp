# Session 6 · When things break: fault detection and fault-tolerant control

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; not yet taught.
Mostly simulation; the optional rig part is props-on ([safety rules](../../hardware/rig/safety.md)).*

## After this session you can

- model a rotor fault as a loss of effectiveness and derive the hover limit $\eta_{min} = 1/(T/W)$;
- build residuals and fault signatures, and isolate a faulty rotor by projection;
- implement a CUSUM test and reason about detection delay against false alarms;
- re-allocate thrust with an estimated effectiveness;
- run the rotorfault pipeline (estimator, CUSUM bank, supervisor, yaw-relaxed controller),
  read its results, and explain why a quad that loses a rotor must spin to survive;
- read a research study's limitations section and name what would invalidate its results.

## Before the evening (30 min)

Install `rotorfault` ([docs/setup.md](../../docs/setup.md#2-rotorfault)) and run
`rotorfault simulate --traj hover --loe 1.0 --plot logs/loss.png` once. Skim the abstract
and section 6 (limitations) of the rotorfault README.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Quiz 5 review; a chipped prop, a dead ESC | [slides](../../slides/06-when-things-break.md) |
| 0:10 | Loss of effectiveness; the hover limit; passive behaviour | [notes §1-2](notes.md), notebook §1 |
| 0:30 | Residuals, signatures, isolation; CUSUM | notes §3-4, notebook §2-3 |
| 1:10 | Break | |
| 1:20 | Re-allocation; the rotorfault pipeline; complete loss and yaw-relaxed control | notes §5-6, notebook §4-6 |
| 1:55 | **Lab**: fault experiments in rotorfault (+ optional fault injection on the rig) | [lab.md](lab.md) |
| 2:35 | Reading the limitations | notes §7 |
| 2:45 | Quiz 6; capstone tracks and teams | [quiz.md](quiz.md), [capstone](../07-capstone/README.md) |

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s06_faults.ipynb`](../../notebooks/s06_faults.ipynb) (6 auto-checked exercises) |
| Simulator | `rotorfault` (Rami Kronbi's rotor-fault simulation study) |
| Lab | [lab.md](lab.md); optional rig part with the `fault` command of `s03_rig_angle_pid` / `s04_rig_cascaded` |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/06-when-things-break.md](../../slides/06-when-things-break.md) |
