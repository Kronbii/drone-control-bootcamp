# Cohort plan for Space²

How to run one cohort of the bootcamp: who it is for, how to recruit, what to prepare,
the calendar, how students are assessed and what earns the certificate. Written before the
first cohort; the [pilot plan](pilot-plan.md) describes how these choices get tested.

## Format

| | |
|---|---|
| Sessions | 7 evenings, one per week, about 3 hours each (21 contact hours) |
| Extra evenings | 1 rig-build evening before session 3 (volunteers and the cohort), 1 optional open-lab evening between sessions 6 and 7 |
| Cohort size | 18 students, 6 teams of 3 (set by the number of rigs and supervisors, not by the room) |
| Staff | 2 instructors for every session, plus 1 helper for the props-on sessions 3, 4, 6 and 7 |
| Language | Course material in English; teaching in whatever mix of English, Arabic and French works for the cohort |
| Cost to Space² | [kit-and-costs.md](kit-and-costs.md): roughly USD 700-1,240 for 6 rigs plus shared items, before shipping and contingency; most of it is reusable |

## Who it is for

2nd-4th year students of mechatronics, mechanical, electrical, computer or aerospace
engineering (or physics) who want to understand how a drone stays in the air and would
like a hardware project with a controls core for their CV.

**Prerequisites** (checked by the self-assessment in the application):

- calculus with first-order linear ODEs; vectors and matrices (multiplication, inverse);
- basic programming in Python or C/C++ (loops, functions, arrays);
- an Arduino-class board used at least once, or the willingness to do a one-hour primer
  before session 1 (the Arduino "Blink" and "AnalogReadSerial" examples);
- a laptop that can run Python 3.11+ and the Arduino IDE;
- availability for all seven evenings (missing session 3 blocks rig work).

No prior control course is needed; students who have taken one will find sessions 3-6 new.

## Recruitment

1. **Announce 6-8 weeks before session 1**: Space² channels, university engineering clubs
   and student branches, and lecturers of control/mechatronics courses. The announcement
   says plainly: seven evenings, hardware with spinning propellers under safety rules, a
   capstone, a certificate of completion from Space² (not academic credit).
2. **Application form** (10 minutes): university, major, year; programming experience; a
   five-question prerequisite self-check (below); one paragraph "why this, and what would you
   build after it"; confirmation of availability and of a laptop.
3. **Selection**, in this order: availability for all evenings; prerequisite self-check;
   then a mix of universities, majors and years so that each team has complementary skills;
   aim for a gender-balanced cohort and say so in the announcement. Keep a waiting list.
4. **Acceptance 2 weeks before session 1**, with the setup instructions
   ([docs/setup.md](../docs/setup.md)) and a request to install everything before the first
   evening. Ask accepted students to confirm within 3 days; then offer places to the list.

**Prerequisite self-check** (in the form; honest answers, not graded):

1. Solve $\dot y = -2y + 4$, $y(0) = 0$.
2. Multiply a 2x2 matrix by a vector.
3. Write a loop that sums a list of numbers in any language.
4. What does `analogRead()` return on an Arduino Uno?
5. Have you used Jupyter, Git, a soldering iron? (yes/no each)

## Timeline

| week | what | who |
|---|---|---|
| -8 | Confirm dates, room, budget; order motors, ESCs, props, supplies, e-stops (long lead times) | organisers |
| -7 | Announce; open applications | organisers |
| -5 | Close applications; buy local parts (wood, tube, fasteners, mesh); print parts | organisers, volunteers |
| -4 | Build and commission **one** rig end to end (props-off, props-on, `s07_capstone` test); fix the design notes | instructors |
| -3 | Select and accept; build the remaining rigs | organisers, volunteers |
| -2 | Dry run of session 1 and 3 with volunteers; check every notebook on the room network (`make test`) | instructors |
| -1 | Install support evening (optional, online); team boxes packed | instructors |
| 1-6 | Sessions 1-6 (build evening between 2 and 3; open lab between 6 and 7) | |
| 7 | Session 7: capstone tests and demos | |
| 8 | Write-ups due; marking (two instructors per write-up) | instructors |
| 9 | Certificates; retrospective with the feedback forms; update the course | organisers |

## Assessment

| component | weight | notes |
|---|---|---|
| Lab hand-ins (labs 1-6) | 30 % | one per team per lab: the numbers and plot asked for; marked complete / incomplete, with feedback |
| Quiz 7 (final concept check) | 10 % | individual; quizzes 1-6 are formative |
| Capstone write-up | 40 % | team; rubric below |
| Capstone demo and test conduct | 10 % | team; includes following the test-day protocol |
| Engineering practice and safety | 10 % | individual; observed across the rig sessions: roles, checklist, logging, one change at a time |

### Assessment rubric

Applied to the capstone write-up and demo by two instructors independently.

| criterion | excellent (4) | good (3) | developing (2) | insufficient (1) |
|---|---|---|---|---|
| **Results** | every spec line computed from logs, worst of three runs, all numbers traceable | all lines computed, minor gaps in traceability | some lines estimated by eye or missing | numbers without logs |
| **Evidence and analysis** | plots chosen to prove each claim; failures explained with data | clear plots; most failures explained | plots present but not tied to claims | no usable evidence |
| **Model vs hardware** | quantitative comparison with the simulation, a tested explanation of the gap | qualitative comparison with a plausible explanation | comparison mentioned, no explanation | absent |
| **Engineering practice** | systematic tuning log, one change at a time, safety rules followed throughout | mostly systematic; rules followed | ad hoc tuning; a rule reminder needed | unsafe practice or no record |
| **Communication** | two clear pages and a 3-minute demo that a newcomer follows | clear with minor issues | hard to follow in places | unclear or incomplete |

A team that fails a spec line but measures and explains it well can score "excellent".

## Certificate criteria

**Certificate of completion** (issued by Space², stating that the course was completed;
it is not academic credit):

- attended at least 6 of the 7 sessions, including session 3 (safety briefing and
  commissioning) and session 7;
- the team submitted at least 5 of the 6 lab hand-ins;
- quiz 7: at least 6 of 10;
- the capstone write-up submitted, with no rubric row at "insufficient";
- no breach of the safety rules after a warning.

**With distinction**: in addition, the capstone scores "good" or better on every rubric
row and at least "excellent" on two, and the team's controller passes the specification or
the write-up explains each failure with data.

The certificate lists the topics (PID and its practical implementation with easyPID,
attitude estimation and cascaded control on a hardware rig, quadrotor control in
simulation, fault detection and fault-tolerant control) and the capstone track.

## Data and photos

Logs and write-ups stay with the team and the instructors unless a team agrees to share
them. Photos or videos of students only with their consent; none go into the public
repository.
