# Pilot plan

**Status: this curriculum has not been taught yet.** Everything in it -- timings, rig
parameters, the difficulty of the exercises, the specification limits, the costs -- is a
design estimate. The software has been tested (notebooks execute, solutions pass their
auto-checks, sketches compile, the easyPID port matches the C++ library, the rig and quad
models pass sanity tests); no student has used it and no rig has been built from these
documents. This plan turns the first cohort (or a smaller pilot before it) into
measurements.

## Stage 0 · Instructor run-through (before any student)

| what | how | pass if |
|---|---|---|
| Build one rig from the docs only | an instructor who did not write them follows build.md and wiring.md | built in ≤ 4 h; every unclear step logged and fixed |
| Commission and fly it | lab 3 part A-D, lab 4, `s07_capstone` test | all interlocks work; the test completes three times without a trip |
| Measure the model parameters | thrust curve, mass, swing period, motor lag (audio or slow-motion video) | parameters within a factor of 2 of `RigParams`; otherwise update the defaults and the spec |
| Re-validate the capstone spec | default and tuned gains on the real rig | tuned gains pass with margin, the start gains fail at least one line (the spec discriminates) |
| Time each notebook | an instructor not involved in writing them | each fits its slot in the session plan |

## Stage 1 · Pilot evenings (optional, recommended)

Run sessions 1 and 3 with 4-6 volunteers (two teams), ideally students from the target group.

## Stage 2 · First cohort: what to measure

| question | measurement | collected |
|---|---|---|
| Do the timings hold? | actual minutes per block vs the instructor guide | instructor log, every evening |
| Are the exercises the right size? | fraction of auto-checks passing by the end of the evening (students tick a list) | per session |
| Do the labs work? | lab hand-ins complete / incomplete; hardware faults and their cause | per session |
| Do students learn the core ideas? | quiz 1 questions repeated in quiz 7; per-question scores | sessions 1 and 7 |
| Is it safe? | incidents and near misses (none expected; every one is written up) | every rig session |
| Does the hardware survive? | broken props, ESCs, motors, printed parts per session | per session |
| What did it cost? | actual spend vs kit-and-costs.md | after procurement |
| What do students think? | [feedback form](feedback-form.md), per session and at the end | per session, week 7 |

## Decisions the pilot data feeds

- Any block more than 20 % over its time two weeks running → cut or move material
  (candidates: Kalman preview in session 4, part D of lab 2, section 8 of notebook 5).
- Fewer than half the teams finishing a lab → simplify the lab or move a part to the
  open-lab evening.
- A safety near miss → the layer that failed is redesigned before the next rig session.
- Rig parameters far from the model → update `RigParams`, regenerate the notebook-3/4
  figures and the example capstone log, re-check the spec limits.
- Feedback that a session is too fast/slow for 2nd-year students → adjust prerequisites or
  add a primer.

## Reporting

After the cohort, a short public note in this repository (`docs/pilot-results.md`, to be
written then) with the measured numbers above, what changed, and what did not work.
Until then, nothing in this repository claims outcomes for students.
