# Quiz 2 · Tuning and real-world PID

*10 minutes, individual. Answers: `instructor/quiz-keys.md`.*

1. For $\tau = 0.5$ s and $T_s = 0.1$ s, compute the ZOH coefficient $a = e^{-T_s/\tau}$. If $K = 2$, what is $b$?

2. A loop with a 3 Hz bandwidth is sampled at 30 Hz. How much phase does the
   zero-order hold cost at 3 Hz?

3. You see this in a log: the setpoint dropped at t = 10 s, but the controller output stayed
   at its maximum until t = 16 s, and the I term was ten times the output range. Name the
   problem and the easyPID setting that prevents it (and its default).

4. easyPID's EMA derivative filter runs at 200 Hz with `alpha = 0.9`. What is its cut-off
   frequency? Which `alpha` gives the same cut-off at 100 Hz?

5. Why does a setpoint step produce a spike in easyPID's D term, and give one way to avoid it.

6. A step test gives $t_{28} = 1.2$ s, $t_{63} = 3.0$ s, $\Delta y = 4$ for $\Delta u = 2$. Give $K$, $\tau$, $L$.

7. A relay of amplitude $d = 2$ produces an oscillation with half peak-to-peak amplitude
   $a = 0.1$ and no hysteresis. What is $K_u$? With $P_u = 2.6$ s, what are the Ziegler-Nichols gains?

8. In simulation, easyPID's tuner on the RC ladder reported $K_u = 42$ on its first run and
   $K_u = 28$ on a second run started straight afterwards (exact value 29). Explain the difference.
