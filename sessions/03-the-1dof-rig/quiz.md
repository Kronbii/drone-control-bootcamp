# Quiz 3 · The seesaw rig

*10 minutes, individual. Answers: `instructor/quiz-keys.md`.*

1. Write the equation of motion of the rig and say what each of $J$, $l$, $h$ and $b$ is.

2. The motor-2 end rests on a tared kitchen scale and motor 1 spins. The scale reads 180 g.
   The arms are equal. What is the thrust of motor 1 in newtons? Why does the beam's own
   weight not matter?

3. A thrust fit gives $c_1 = 1.6$ N, $c_2 = 6.4$ N. With $l = 0.25$ m and a base of 1350 µs,
   what torque does a differential command of $u = 50$ µs produce?

4. The rig swings freely with a period of 4.2 s. $J = 0.0126$ kg m², $m = 0.29$ kg. Is the
   centre of gravity above or below the pivot, and by how much?

5. Why does the rig go unstable with proportional control alone at a fairly small gain,
   and what does the D term add?

6. Explain why the mixer shifts both motor commands when one would exceed 1600 µs, instead
   of clipping that one.

7. Name four of the software interlocks in `rig_safety.h` and the failure each one guards against.

8. Why is the e-stop in the 12 V power line rather than an input to the Arduino?
