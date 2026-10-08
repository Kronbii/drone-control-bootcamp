// Drives the real easyPID C++ sources from a command script on stdin and prints every
// result, so tests/test_easypid_port.py can compare the Python port line by line.
//
//   new kp ki kd min max | begin | reset | aw M | filt M alpha | dir D
//   olim lo hi | ilim lo hi | tun kp ki kd | st ms | adv ms
//   upd sp y dtms  -> "o p i d e"      auto sp y  -> "o p i d e"
//   tstart sp amp nb bias -> "ok"     tupd y -> "o state progress"
//   tres rule -> "ok kp ki kd Ku Pu"
#include <stdio.h>
#include <string.h>

#include "PIDTuner.h"
#include "easyPID.h"

unsigned long g_harnessMillis = 0;

static void terms(PIDController& c, float o) {
    printf("%.9g %.9g %.9g %.9g %.9g\n", o, c.getPterm(), c.getIterm(), c.getDterm(), c.getError());
}

int main() {
    setvbuf(stdout, NULL, _IOLBF, 0);
    PIDController* pid = 0;
    PIDTuner* tuner = 0;
    char cmd[32];
    while (scanf("%31s", cmd) == 1) {
        float a, b, c, d, e;
        if (!strcmp(cmd, "new")) {
            if (scanf("%f %f %f %f %f", &a, &b, &c, &d, &e) != 5) return 2;
            delete tuner;
            delete pid;
            pid = new PIDController(a, b, c, d, e);
            tuner = new PIDTuner(*pid);
        } else if (!strcmp(cmd, "begin")) {
            pid->begin();
        } else if (!strcmp(cmd, "reset")) {
            pid->reset();
        } else if (!strcmp(cmd, "aw")) {
            int m;
            if (scanf("%d", &m) != 1) return 2;
            pid->setAntiWindup((AntiWindupMode)m);
        } else if (!strcmp(cmd, "filt")) {
            int m;
            if (scanf("%d %f", &m, &a) != 2) return 2;
            pid->setDerivativeFilter((DerivativeFilterMode)m, a);
        } else if (!strcmp(cmd, "dir")) {
            int m;
            if (scanf("%d", &m) != 1) return 2;
            pid->setDirection((ControlDirection)m);
        } else if (!strcmp(cmd, "olim")) {
            if (scanf("%f %f", &a, &b) != 2) return 2;
            pid->setOutputLimits(a, b);
        } else if (!strcmp(cmd, "ilim")) {
            if (scanf("%f %f", &a, &b) != 2) return 2;
            pid->setIntegralLimits(a, b);
        } else if (!strcmp(cmd, "tun")) {
            if (scanf("%f %f %f", &a, &b, &c) != 3) return 2;
            pid->setTunings(a, b, c);
        } else if (!strcmp(cmd, "st")) {
            unsigned long ms;
            if (scanf("%lu", &ms) != 1) return 2;
            pid->setSampleTime(ms);
        } else if (!strcmp(cmd, "adv")) {
            unsigned long ms;
            if (scanf("%lu", &ms) != 1) return 2;
            g_harnessMillis += ms;
        } else if (!strcmp(cmd, "upd")) {
            if (scanf("%f %f %f", &a, &b, &c) != 3) return 2;
            terms(*pid, pid->update(a, b, c));
        } else if (!strcmp(cmd, "auto")) {
            if (scanf("%f %f", &a, &b) != 2) return 2;
            terms(*pid, pid->update(a, b));
        } else if (!strcmp(cmd, "tstart")) {
            if (scanf("%f %f %f %f", &a, &b, &c, &d) != 4) return 2;
            printf("%d\n", tuner->start(a, b, c, d) ? 1 : 0);
        } else if (!strcmp(cmd, "tupd")) {
            if (scanf("%f", &a) != 1) return 2;
            float o = tuner->update(a);
            printf("%.9g %d %.9g\n", o, (int)tuner->getState(), tuner->getProgress());
        } else if (!strcmp(cmd, "tres")) {
            int r;
            if (scanf("%d", &r) != 1) return 2;
            float kp = 0, ki = 0, kd = 0;
            bool ok = tuner->getTunings(kp, ki, kd, (TuningRule)r);
            printf("%d %.9g %.9g %.9g %.9g %.9g\n", ok ? 1 : 0, kp, ki, kd,
                   tuner->getUltimateGain(), tuner->getUltimatePeriod());
        } else {
            fprintf(stderr, "unknown command %s\n", cmd);
            return 3;
        }
    }
    delete tuner;
    delete pid;
    return 0;
}
