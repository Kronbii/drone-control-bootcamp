"""A line-by-line Python port of the easyPID Arduino library (version 1.1.0).

Why a port: the notebooks simulate *the same controller* that students later flash
on an Arduino. Method names, argument order, units and defaults are identical to the
C++ library, so a tuning found in a notebook transfers to a sketch unchanged::

    # Python (notebook)                       // C++ (Arduino sketch)
    pid = PIDController(2.0, 0.5, 0.1, 0, 255)  PIDController pid(2.0, 0.5, 0.1, 0, 255);
    pid.begin()                                 pid.begin();
    u = pid.update(sp, y, 10)   # dt in ms      float u = pid.update(sp, y, 10);

The method names are camelCase on purpose: they mirror the C++ API.

Behaviour that matters and is reproduced exactly (see easyPID's ``src/easyPID.cpp``):

* ``integral += error * dt`` (rectangular), ``derivative = (e - e_prev) / dt`` on the
  *error*, and no derivative on the first update after ``begin()``/``reset()``;
* optional EMA derivative filter ``d_f = alpha d_f + (1 - alpha) d`` (alpha clamped
  to [0, 0.999]);
* output clamped to ``[outMin, outMax]``; anti-windup ``CLAMP`` (default) undoes this
  step's integration only when it pushes further into saturation, ``BACKCALC`` feeds
  the saturation excess back through ``1/ki``;
* ``REVERSE`` negates the control error; ``getError()`` is always
  ``setpoint - measurement``;
* timing: the three-argument ``update`` takes ``dtMs`` in **milliseconds**; the
  two-argument form reads a clock (here a :class:`VirtualClock`, on the board
  ``millis()``) and returns the previous output when no whole millisecond has passed.

Arithmetic is done in Python doubles whereas an AVR uses 32-bit floats; over the few
thousand steps of a lab the difference is far below sensor noise. The test-suite
compiles the real C++ sources on the host and checks this port against them.
"""

from __future__ import annotations

from enum import IntEnum

EASYPID_VERSION = "1.1.0"
EASYPID_MIN_KI = 1e-6
_U32 = 0xFFFFFFFF


class AntiWindupMode(IntEnum):
    ANTIWINDUP_NONE = 0
    ANTIWINDUP_CLAMP = 1
    ANTIWINDUP_BACKCALC = 2


class DerivativeFilterMode(IntEnum):
    FILTER_NONE = 0
    FILTER_EMA = 1


class ControlDirection(IntEnum):
    DIRECT = 0
    REVERSE = 1


class TuningRule(IntEnum):
    TUNING_ZIEGLER_NICHOLS = 0
    TUNING_TYREUS_LUYBEN = 1
    TUNING_PESSEN = 2
    TUNING_NO_OVERSHOOT = 3


class TunerState(IntEnum):
    TUNER_IDLE = 0
    TUNER_RELAY_STEP = 1
    TUNER_ANALYZING = 2
    TUNER_COMPLETE = 3
    TUNER_FAILED = 4


# The same unprefixed / prefixed constant names as the C++ headers.
ANTIWINDUP_NONE = AntiWindupMode.ANTIWINDUP_NONE
ANTIWINDUP_CLAMP = AntiWindupMode.ANTIWINDUP_CLAMP
ANTIWINDUP_BACKCALC = AntiWindupMode.ANTIWINDUP_BACKCALC
FILTER_NONE = DerivativeFilterMode.FILTER_NONE
FILTER_EMA = DerivativeFilterMode.FILTER_EMA
DIRECT = ControlDirection.DIRECT
REVERSE = ControlDirection.REVERSE
TUNING_ZIEGLER_NICHOLS = TuningRule.TUNING_ZIEGLER_NICHOLS
TUNING_TYREUS_LUYBEN = TuningRule.TUNING_TYREUS_LUYBEN
TUNING_PESSEN = TuningRule.TUNING_PESSEN
TUNING_NO_OVERSHOOT = TuningRule.TUNING_NO_OVERSHOOT
TUNER_IDLE = TunerState.TUNER_IDLE
TUNER_RELAY_STEP = TunerState.TUNER_RELAY_STEP
TUNER_ANALYZING = TunerState.TUNER_ANALYZING
TUNER_COMPLETE = TunerState.TUNER_COMPLETE
TUNER_FAILED = TunerState.TUNER_FAILED


class VirtualClock:
    """Stand-in for Arduino's ``millis()``: a 32-bit millisecond counter you advance."""

    def __init__(self, start_ms: int = 0):
        self._ms = int(start_ms)

    def millis(self) -> int:
        return self._ms & _U32

    def advance(self, ms: int) -> None:
        self._ms += int(ms)

    def set(self, ms: int) -> None:
        self._ms = int(ms)


#: Clock used by controllers created without an explicit ``clock=``.
DEFAULT_CLOCK = VirtualClock()


class PIDController:
    """Port of ``PIDController`` from easyPID 1.1.0 (see the module docstring)."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        outMin: float,
        outMax: float,
        clock: VirtualClock | None = None,
    ):
        self.clock = clock or DEFAULT_CLOCK
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        self.outMin_ = float(outMin)
        self.outMax_ = float(outMax)

        self.error_ = 0.0
        self.controlError_ = 0.0
        self.previousError_ = 0.0
        self.firstUpdate_ = True
        self.integral_ = 0.0
        self.derivative_ = 0.0
        self.derivativeFiltered_ = 0.0

        self.pTerm_ = 0.0
        self.iTerm_ = 0.0
        self.dTerm_ = 0.0
        self.output_ = 0.0

        self.antiWindupMode_ = ANTIWINDUP_CLAMP
        self.filterMode_ = FILTER_NONE
        self.filterAlpha_ = 0.8
        self.direction_ = DIRECT

        self.sampleTime_ = 100
        self.lastTime_ = 0

        self.integralMin_ = float(outMin)
        self.integralMax_ = float(outMax)
        self.integralLimitsSet_ = False

        self.storedSetpoint_ = 0.0
        self.storedMeasurement_ = 0.0
        self.initialized_ = False

    # ------------------------------------------------------------------ lifecycle
    def begin(self) -> None:
        self.lastTime_ = self.clock.millis()
        self.initialized_ = True
        self.reset()

    def update(self, setpoint: float, measurement: float, dtMs: float | None = None) -> float:
        """``update(sp, y)`` uses the clock; ``update(sp, y, dtMs)`` takes dt in ms."""
        if not self.initialized_:
            self.begin()
        if dtMs is None:
            now = self.clock.millis()
            elapsed = (now - self.lastTime_) & _U32  # unsigned, rollover-safe
            if elapsed == 0:
                return self.output_
            self.lastTime_ = now
            dt = elapsed / 1000.0
        else:
            if dtMs <= 0.0:
                return self.output_
            dt = dtMs / 1000.0
            self.lastTime_ = self.clock.millis()
        return self._computePID(float(setpoint), float(measurement), dt)

    def setSetpoint(self, setpoint: float) -> None:
        self.storedSetpoint_ = float(setpoint)

    def setMeasurement(self, measurement: float) -> None:
        self.storedMeasurement_ = float(measurement)

    def compute(self) -> float:
        return self.update(self.storedSetpoint_, self.storedMeasurement_)

    # ------------------------------------------------------------------ the maths
    def _computePID(self, setpoint: float, measurement: float, dt: float) -> float:
        self.error_ = setpoint - measurement
        self.controlError_ = -self.error_ if self.direction_ == REVERSE else self.error_

        self.pTerm_ = self.kp_ * self.controlError_

        integralBefore = self.integral_
        self.integral_ += self.controlError_ * dt
        if self.integralLimitsSet_:
            if self.integral_ > self.integralMax_:
                self.integral_ = self.integralMax_
            elif self.integral_ < self.integralMin_:
                self.integral_ = self.integralMin_
        self.iTerm_ = self.ki_ * self.integral_

        if self.firstUpdate_:
            self.derivative_ = 0.0
            self.derivativeFiltered_ = 0.0
            self.firstUpdate_ = False
        else:
            self.derivative_ = (self.controlError_ - self.previousError_) / dt

        derivativeToUse = self.derivative_
        if self.filterMode_ == FILTER_EMA:
            self.derivativeFiltered_ = (
                self.filterAlpha_ * self.derivativeFiltered_
                + (1.0 - self.filterAlpha_) * self.derivative_
            )
            derivativeToUse = self.derivativeFiltered_
        self.dTerm_ = self.kd_ * derivativeToUse

        rawOutput = self.pTerm_ + self.iTerm_ + self.dTerm_
        clampedOutput = rawOutput
        if clampedOutput > self.outMax_:
            clampedOutput = self.outMax_
        elif clampedOutput < self.outMin_:
            clampedOutput = self.outMin_

        self._applyAntiWindup(rawOutput, clampedOutput, dt, integralBefore)
        self.iTerm_ = self.ki_ * self.integral_

        self.previousError_ = self.controlError_
        self.output_ = clampedOutput
        return self.output_

    def _applyAntiWindup(
        self, rawOutput: float, clampedOutput: float, dt: float, integralBefore: float
    ) -> None:
        if self.antiWindupMode_ == ANTIWINDUP_NONE:
            return
        saturated = rawOutput != clampedOutput
        if self.antiWindupMode_ == ANTIWINDUP_CLAMP:
            pushingIntoSaturation = (rawOutput > self.outMax_ and self.controlError_ > 0.0) or (
                rawOutput < self.outMin_ and self.controlError_ < 0.0
            )
            if pushingIntoSaturation:
                self.integral_ = integralBefore
        elif self.antiWindupMode_ == ANTIWINDUP_BACKCALC:
            haveIntegralGain = self.ki_ > EASYPID_MIN_KI or self.ki_ < -EASYPID_MIN_KI
            if saturated and haveIntegralGain:
                backCalcGain = 1.0 / self.ki_
                error_back = (clampedOutput - rawOutput) * backCalcGain
                self.integral_ += error_back * dt

    # ------------------------------------------------------------------ configuration
    def setTunings(self, kp: float, ki: float, kd: float) -> None:
        self.kp_, self.ki_, self.kd_ = float(kp), float(ki), float(kd)

    def setOutputLimits(self, min: float, max: float) -> None:
        if min >= max:
            return
        self.outMin_, self.outMax_ = float(min), float(max)

    def setIntegralLimits(self, min: float, max: float) -> None:
        if min >= max:
            return
        self.integralMin_, self.integralMax_ = float(min), float(max)
        self.integralLimitsSet_ = True

    def setAntiWindup(self, mode: AntiWindupMode) -> None:
        self.antiWindupMode_ = AntiWindupMode(mode)

    def setDerivativeFilter(self, mode: DerivativeFilterMode, alpha: float = 0.8) -> None:
        self.filterMode_ = DerivativeFilterMode(mode)
        self.filterAlpha_ = min(max(float(alpha), 0.0), 0.999)

    def setSampleTime(self, ms: int) -> None:
        if ms > 0:
            self.sampleTime_ = int(ms)

    def setDirection(self, direction: ControlDirection) -> None:
        direction = ControlDirection(direction)
        if direction == self.direction_:
            return
        self.direction_ = direction
        self.integral_ = -self.integral_
        self.previousError_ = -self.previousError_
        self.derivativeFiltered_ = -self.derivativeFiltered_
        self.controlError_ = -self.controlError_

    def reset(self) -> None:
        self.error_ = 0.0
        self.controlError_ = 0.0
        self.previousError_ = 0.0
        self.firstUpdate_ = True
        self.integral_ = 0.0
        self.derivative_ = 0.0
        self.derivativeFiltered_ = 0.0
        self.pTerm_ = 0.0
        self.iTerm_ = 0.0
        self.dTerm_ = 0.0
        self.output_ = 0.0
        self.lastTime_ = self.clock.millis()

    # ------------------------------------------------------------------ introspection
    def getError(self) -> float:
        return self.error_

    def getPterm(self) -> float:
        return self.pTerm_

    def getIterm(self) -> float:
        return self.iTerm_

    def getDterm(self) -> float:
        return self.dTerm_

    def getOutput(self) -> float:
        return self.output_


# ======================================================================================
# Relay autotuner (PIDTuner.h / PIDTuner.cpp)
# ======================================================================================
MIN_CYCLES_FOR_TUNING = 3
MAX_WAIT_TIME_MS = 60000
MAX_TUNING_TIME_MS = 900000
EASYPID_PI = 3.14159265359


def tuning_rule_gains(Ku: float, Pu: float, rule: TuningRule) -> tuple[float, float, float]:
    """The four rules of ``PIDTuner::applyTuningRule`` (kp, ki, kd from Ku and Pu)."""
    if rule == TUNING_TYREUS_LUYBEN:
        kp = Ku / 2.2
        return kp, kp / (2.2 * Pu), kp * Pu / 6.3
    if rule == TUNING_PESSEN:
        return 0.7 * Ku, 1.75 * Ku / Pu, 0.105 * Ku * Pu
    if rule == TUNING_NO_OVERSHOOT:
        return 0.2 * Ku, 0.4 * Ku / Pu, 0.066 * Ku * Pu
    return 0.6 * Ku, 1.2 * Ku / Pu, 0.075 * Ku * Pu  # Ziegler-Nichols (and default)


class PIDTuner:
    """Port of ``PIDTuner`` (relay / limit-cycle autotuner, Astrom-Hagglund)."""

    def __init__(self, pid: PIDController):
        self.pid_ = pid
        self.state_ = TUNER_IDLE
        self.resultsValid_ = False
        self.ultimateGain_ = 0.0
        self.ultimatePeriod_ = 0.0
        self.setpoint_ = 0.0
        self.relayAmplitude_ = 0.0
        self.noiseBand_ = 0.0
        self.relayHigh_ = False
        self.relayHighPrev_ = False
        self.outputHigh_ = 0.0
        self.outputLow_ = 0.0
        self.outputBias_ = 0.0
        self.cycleMax_ = 0.0
        self.cycleMin_ = 0.0
        self.haveCycleStart_ = False
        self.cycleStartTime_ = 0
        self.tuningStartTime_ = 0
        self.lastTransitionTime_ = 0
        self.cyclesDetected_ = 0
        self.cyclesNeeded_ = 0
        self.periodSum_ = 0.0
        self.amplitudeSum_ = 0.0

    @property
    def clock(self) -> VirtualClock:
        return self.pid_.clock

    def start(
        self,
        setpoint: float,
        relayAmplitude: float,
        noiseBand: float = 0.5,
        outputBias: float = 0.0,
    ) -> bool:
        if self.state_ in (TUNER_RELAY_STEP, TUNER_ANALYZING):
            return False
        if relayAmplitude <= 0.0 or noiseBand < 0.0:
            return False
        self.setpoint_ = float(setpoint)
        self.relayAmplitude_ = float(relayAmplitude)
        self.noiseBand_ = float(noiseBand)
        self.outputBias_ = float(outputBias)
        self.outputHigh_ = self.outputBias_ + self.relayAmplitude_
        self.outputLow_ = self.outputBias_ - self.relayAmplitude_
        self.relayHigh_ = False
        self.relayHighPrev_ = False
        self.cycleMax_ = self.setpoint_
        self.cycleMin_ = self.setpoint_
        self.haveCycleStart_ = False
        self.cycleStartTime_ = self.clock.millis()
        self.tuningStartTime_ = self.cycleStartTime_
        self.lastTransitionTime_ = self.cycleStartTime_
        self.cyclesDetected_ = 0
        self.cyclesNeeded_ = MIN_CYCLES_FOR_TUNING + 2
        self.periodSum_ = 0.0
        self.amplitudeSum_ = 0.0
        self.resultsValid_ = False
        self.ultimateGain_ = 0.0
        self.ultimatePeriod_ = 0.0
        self.pid_.reset()
        self.state_ = TUNER_RELAY_STEP
        return True

    def update(self, measurement: float) -> float:
        if self.state_ != TUNER_RELAY_STEP:
            return 0.0
        now = self.clock.millis()
        if ((now - self.lastTransitionTime_) & _U32) > MAX_WAIT_TIME_MS or (
            (now - self.tuningStartTime_) & _U32
        ) > MAX_TUNING_TIME_MS:
            self.state_ = TUNER_ANALYZING
            self._calculateResults()
            self.state_ = TUNER_COMPLETE if self.resultsValid_ else TUNER_FAILED
            return 0.0
        if measurement > self.setpoint_ + self.noiseBand_:
            self.relayHigh_ = False
        elif measurement < self.setpoint_ - self.noiseBand_:
            self.relayHigh_ = True
        output = self.outputHigh_ if self.relayHigh_ else self.outputLow_
        self._trackLimitCycle(float(measurement), now)
        if self.cyclesDetected_ >= self.cyclesNeeded_:
            self.state_ = TUNER_ANALYZING
            self._calculateResults()
            self.state_ = TUNER_COMPLETE if self.resultsValid_ else TUNER_FAILED
        return output

    def _trackLimitCycle(self, measurement: float, now: int) -> None:
        if measurement > self.cycleMax_:
            self.cycleMax_ = measurement
        if measurement < self.cycleMin_:
            self.cycleMin_ = measurement
        risingEdge = self.relayHigh_ and not self.relayHighPrev_
        if self.relayHigh_ != self.relayHighPrev_:
            self.lastTransitionTime_ = now
        self.relayHighPrev_ = self.relayHigh_
        if not risingEdge:
            return
        if self.haveCycleStart_:
            self.amplitudeSum_ += (self.cycleMax_ - self.cycleMin_) * 0.5
            self.periodSum_ += float((now - self.cycleStartTime_) & _U32)
            self.cyclesDetected_ += 1
        else:
            self.haveCycleStart_ = True
        self.cycleStartTime_ = now
        self.cycleMax_ = measurement
        self.cycleMin_ = measurement

    def _calculateResults(self) -> None:
        if self.cyclesDetected_ < MIN_CYCLES_FOR_TUNING:
            self.resultsValid_ = False
            return
        avgPeriod = self.periodSum_ / self.cyclesDetected_
        avgAmplitude = self.amplitudeSum_ / self.cyclesDetected_
        self.ultimatePeriod_ = avgPeriod / 1000.0
        h = self.noiseBand_
        effective = avgAmplitude * avgAmplitude - h * h
        if avgAmplitude > 0.0 and effective > 0.0:
            self.ultimateGain_ = (4.0 * self.relayAmplitude_) / (EASYPID_PI * effective**0.5)
            self.resultsValid_ = True
        else:
            self.resultsValid_ = False

    def isComplete(self) -> bool:
        return self.state_ == TUNER_COMPLETE and self.resultsValid_

    def getTunings(self, rule: TuningRule = TUNING_ZIEGLER_NICHOLS):
        """Returns ``(ok, kp, ki, kd)`` (C++ returns the gains through references)."""
        if not self.resultsValid_ or self.ultimatePeriod_ <= 0.0:
            return False, 0.0, 0.0, 0.0
        kp, ki, kd = tuning_rule_gains(self.ultimateGain_, self.ultimatePeriod_, rule)
        return True, kp, ki, kd

    def applyTunings(self, rule: TuningRule = TUNING_ZIEGLER_NICHOLS) -> bool:
        ok, kp, ki, kd = self.getTunings(rule)
        if not ok:
            return False
        self.pid_.setTunings(kp, ki, kd)
        self.pid_.reset()
        return True

    def getUltimateGain(self) -> float:
        return self.ultimateGain_

    def getUltimatePeriod(self) -> float:
        return self.ultimatePeriod_

    def cancel(self) -> None:
        self.state_ = TUNER_IDLE
        self.resultsValid_ = False

    def getState(self) -> TunerState:
        return self.state_

    def getProgress(self) -> float:
        if self.state_ in (TUNER_COMPLETE, TUNER_ANALYZING):
            return 1.0
        if self.state_ != TUNER_RELAY_STEP or self.cyclesNeeded_ <= 0:
            return 0.0
        return min(self.cyclesDetected_ / self.cyclesNeeded_, 1.0)
