# Test-day protocol

*For the acceptance tests in session 7. One test station per rig, one instructor per two
stations, the full [safety rules](../../hardware/rig/safety.md) apply.*

## Track R: at the test station (about 6 minutes per team)

1. Instructor checks: cage closed, guards and props intact, e-stop tested this evening,
   `US_CAP` unchanged (the instructor compiles the sketch from the team's file and checks
   `rig_safety.h` is the course version: `python tools/sync_sketch_headers.py --check`).
2. Team uploads `s07_capstone` with its gains as defaults, or sets them over serial
   (write the commands down: they go into the report).
3. Start the logger (opening the port restarts the sketch, so the gains must be the
   sketch's defaults or be typed again into the logger):
   ```bash
   python tools/serial_logger.py --port <port> logs/team<N>_run1.csv
   ```
4. Checklist read aloud. Arm (switch OFF → ON), wait for the 2 s spin-up, type `test` into
   the logger's terminal.
5. Hands off; the test runs 33 s and prints `# TEST DONE`. Disarm, stop the logger (Ctrl+C).
6. Repeat for runs 2 and 3. If the rig disarms during a run (a trip at the stops), that run
   counts as failed on every line after the trip; the team may make **one** gain change and
   restart the series once.
7. Copy the three logs to the team folder; the scoring uses notebook 7 with
   `TEAM_LOG` pointing at each run in turn. The worst run is the result.

## Track S: freeze and score

1. Commit (or zip) the code at 1:50; no edits after that.
2. Run the scoring cell of notebook 7 on seeds 0-4 with `gyro_noise=0.03`; report the worst seed.
3. Save the plots for the demo.

## Demo (3 minutes per team)

- one plot that shows your controller doing its job;
- one number from the spec you are proud of, and one you are not;
- one thing that surprised you about the hardware or the model.

No slides needed: show the notebook.

## After the evening

The report (two pages, template in this folder) is due seven days later, to the instructor
by email or as a pull request to the cohort repository. Late reports are accepted up to the
certificate deadline in the cohort plan with no penalty other than delayed feedback.
