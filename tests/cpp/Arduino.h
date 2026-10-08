// Host-side stand-in for Arduino.h, just enough to compile easyPID's sources natively.
// millis() reads a virtual clock that the harness advances explicitly.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define PI 3.1415926535897932384626433832795

extern unsigned long g_harnessMillis;
inline unsigned long millis() { return g_harnessMillis; }
