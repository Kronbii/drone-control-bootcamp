"""figures/hero_arc.png: the course arc in three panels, from the same code as the notebooks.

(a) session 1: P and PI on the lab-1 RC circuit (easyPID port);
(b) session 4: single loop vs cascaded rate/angle control on the rig model, tap at 4 s;
(c) session 6: complete loss of rotor 1 on the rotorfault 5-inch quad, passive vs the
    fault-tolerant pipeline.

    python tools/hero_figure.py
"""

from __future__ import annotations

import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from bootcamp import attitude as att  # noqa: E402
from bootcamp import easypid as ep  # noqa: E402
from bootcamp import fault, rig  # noqa: E402
from bootcamp.loop import simulate_pid  # noqa: E402
from bootcamp.plants import rc_ladder  # noqa: E402
from bootcamp.plotting import INK_2, REF, SAGE_UI, SERIES, save  # noqa: E402


def panel_pid(ax):
    rc = rc_ladder(10e3, 100e-6, 1)
    for (g, name), c in zip(
        (
            ((2.0, 0, 0), "P: offset"),
            ((2.0, 2.0, 0), "PI: no offset"),
            ((2.0, 8.0, 0), "PI, too much Ki"),
        ),
        SERIES,
    ):
        r = simulate_pid(rc, ep.PIDController(*g, 0.0, 5.0), 2.5, 6.0, 0.01)
        ax.plot(r.t, r.y, color=c, label=name)
    ax.axhline(2.5, **REF, label="setpoint")
    ax.set(xlabel="time (s)", ylabel="capacitor voltage (V)")
    ax.set_title("1 · PI on an RC circuit (easyPID)", loc="left")
    ax.legend(fontsize=8)


def panel_cascade(ax):
    sens = rig.SensorModel(gyro_bias_dps=0.2)

    def tap(t):
        return 0.3 if 4.0 <= t < 4.05 else 0.0

    def single():
        pid = ep.PIDController(2.0, 1.5, 0.8, -200, 200)
        pid.setDerivativeFilter(ep.FILTER_EMA, 0.6)
        pid.begin()
        return lambda t, sp, m, dt: pid.update(sp, m["pot"], dt * 1000)

    def cascade():
        outer = ep.PIDController(4.0, 0.0, 0.0, -150, 150)
        inner = ep.PIDController(3.0, 1.0, 0.05, -200, 200)
        inner.setDerivativeFilter(ep.FILTER_EMA, 0.7)
        outer.begin()
        inner.begin()
        f = att.ComplementaryFilter(0.0)

        def ctrl(t, sp, m, dt):
            f.alpha = att.alpha_from_time_constant(0.5, dt)
            est = f.update(m["gyro"], math.degrees(math.atan2(m["accel_x"], m["accel_z"])), dt)
            return inner.update(outer.update(sp, est, dt * 1000), m["gyro"], dt * 1000)

        return ctrl

    for ctrl, c, name in (
        (single(), SERIES[1], "single loop"),
        (cascade(), SERIES[0], "rate + angle cascade"),
    ):
        log = rig.simulate_rig(
            ctrl,
            7.0,
            setpoint=lambda t: 10.0 if t >= 1.0 else 0.0,
            sensors=sens,
            seed=1,
            disturbance=tap,
        )
        ax.plot(log.t, log.theta, color=c, label=name)
    ax.plot(log.t, log.setpoint, **REF, label="setpoint")
    ax.annotate("tap", (4.35, 15.5), color=INK_2, fontsize=8)
    ax.set(xlabel="time (s)", ylabel="beam angle (deg)")
    ax.set_title("4 · The seesaw rig: cascaded control", loc="left")
    ax.legend(fontsize=8)


def panel_fault(ax):
    for b, c, name in (
        ("passive", SERIES[1], "passive"),
        ("estimated", SERIES[0], "detect + yaw-relaxed"),
    ):
        r, s = fault.run_case(1.0, rotor=1, baseline=b, duration=6.0, onset=2.0, seed=3)
        ax.plot(
            r.t,
            100 * np.linalg.norm(r.position - r.p_ref, axis=1),
            color=c,
            label=name + (" (lost)" if s.lost else ""),
        )
    ax.axvline(2.0, **REF, label="rotor 1 lost")
    ax.set_ylim(0, 120)
    ax.set(xlabel="time (s)", ylabel="position error (cm)")
    ax.set_title("6 · Losing a rotor (rotorfault sim)", loc="left")
    ax.legend(fontsize=8)


def main():
    os.environ["BOOTCAMP_SAVE_FIGURES"] = "1"
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.0))
    panel_pid(axs[0])
    panel_cascade(axs[1])
    panel_fault(axs[2])
    fig.suptitle(
        "Teach it: PID, then attitude, then fault tolerance",
        x=0.01,
        ha="left",
        fontsize=13,
        fontweight="bold",
        color=SAGE_UI,
    )
    fig.tight_layout()
    print(save(fig, "hero_arc"))


if __name__ == "__main__":
    main()
