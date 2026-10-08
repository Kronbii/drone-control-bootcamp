"""Auto-checks for the notebook exercises.

Every exercise in a notebook is followed by a *check cell* that calls one or more of
these functions. A check prints ``[PASS] ...`` or raises :class:`CheckFailed` with a
hint, so a failing exercise is impossible to miss and the message says what to look at.

In a student notebook an unfinished exercise raises ``NotImplementedError``; the check
turns that into a friendly message too.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable

import numpy as np


class CheckFailed(AssertionError):
    """Raised by a check that did not pass."""


def _fail(ex: str, msg: str, hint: str | None) -> None:
    text = f"[FAIL] exercise {ex}: {msg}"
    if hint:
        text += f"\n       hint: {hint}"
    raise CheckFailed(text)


def passed(ex: str, msg: str = "") -> None:
    print(f"[PASS] exercise {ex}" + (f": {msg}" if msg else ""))


def run(ex: str, fn: Callable, *args, **kwargs):
    """Call the student's function; turn an unfinished TODO into a clear failure."""
    try:
        return fn(*args, **kwargs)
    except NotImplementedError:
        _fail(ex, "not implemented yet (the TODO still raises NotImplementedError)", None)
    except CheckFailed:
        raise
    except Exception as e:  # show the student's own error, but as a check failure
        _fail(ex, f"your code raised {type(e).__name__}: {e}", None)


def check_close(
    ex: str,
    actual,
    expected,
    rtol: float = 1e-6,
    atol: float = 1e-9,
    what: str = "value",
    hint: str | None = None,
    quiet: bool = False,
) -> None:
    """``actual`` must equal ``expected`` within ``atol + rtol |expected|`` everywhere."""
    if actual is None:
        _fail(ex, f"{what} is None -- did you forget to return it?", hint)
    a = np.asarray(actual, dtype=float)
    e = np.asarray(expected, dtype=float)
    if a.shape != e.shape:
        _fail(ex, f"{what} has shape {a.shape}, expected {e.shape}", hint)
    if not np.all(np.isfinite(a)):
        _fail(ex, f"{what} contains nan or inf", hint)
    err = np.abs(a - e)
    tol = atol + rtol * np.abs(e)
    if np.any(err > tol):
        k = int(np.argmax(err - tol))
        got, want = a.ravel()[k], e.ravel()[k]
        _fail(ex, f"{what} = {got:.6g}, expected {want:.6g}", hint)
    if not quiet:
        passed(ex, f"{what} correct")


def check_true(ex: str, condition: bool, msg: str, hint: str | None = None) -> None:
    if not bool(condition):
        _fail(ex, msg, hint)
    passed(ex, msg)


def check_less(
    ex: str, value: float, limit: float, what: str, unit: str = "", hint: str | None = None
) -> None:
    v = float("nan") if value is None else float(value)
    if not (math.isfinite(v) and v < limit):
        _fail(ex, f"{what} = {v:.4g}{unit}, must be below {limit:g}{unit}", hint)
    passed(ex, f"{what} = {v:.4g}{unit} < {limit:g}{unit}")


def check_between(
    ex: str, value: float, lo: float, hi: float, what: str, unit: str = "", hint=None
) -> None:
    v = float("nan") if value is None else float(value)
    if not (math.isfinite(v) and lo <= v <= hi):
        _fail(ex, f"{what} = {v:.4g}{unit}, must be in [{lo:g}, {hi:g}]{unit}", hint)
    passed(ex, f"{what} = {v:.4g}{unit} in [{lo:g}, {hi:g}]{unit}")


def check_cases(
    ex: str,
    fn: Callable,
    cases: Iterable[tuple[tuple, object]],
    rtol: float = 1e-6,
    atol: float = 1e-9,
    hint: str | None = None,
) -> None:
    """``fn(*args)`` must match ``expected`` for every ``(args, expected)`` case."""
    n = 0
    for args, expected in cases:
        got = run(ex, fn, *args)
        if got is None:
            _fail(ex, f"{fn.__name__}{args} returned None -- missing return?", hint)
        a = np.asarray(got, dtype=float)
        e = np.asarray(expected, dtype=float)
        if a.shape != e.shape or not np.allclose(a, e, rtol=rtol, atol=atol):
            _fail(ex, f"{fn.__name__}{_short(args)} = {_short(got)}, expected {_short(e)}", hint)
        n += 1
    passed(ex, f"{fn.__name__} correct on {n} test cases")


def _short(x) -> str:
    try:
        s = np.array2string(np.asarray(x, dtype=float), precision=4, threshold=6)
    except (TypeError, ValueError):
        s = repr(x)
    return s if len(s) < 120 else s[:117] + "..."
