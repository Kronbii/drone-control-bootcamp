# Quiz 1 · Feedback and PID

*10 minutes, individual, closed book. Answers: `instructor/quiz-keys.md`.*

1. A plant has $\tau\dot y + y = Ku$ with $K = 2$ and $\tau = 0.5$ s. After a step $u = 1$
   from rest, what is $y$ at $t = 0.5$ s, and what does it settle to?

2. The same plant is controlled with $u = K_p(r - y)$, $K_p = 4.5$, $r = 10$.
   (a) Final value of $y$? (b) Closed-loop time constant?

3. True or false, with one sentence each:
   (a) Increasing $K_p$ always reduces the steady-state error of a P controller on a stable first-order plant.
   (b) Integral action removes the offset caused by a constant disturbance.
   (c) Derivative action is the best way to speed up a first-order plant.

4. In easyPID, `pid.update(sp, y, 20)` means the time step is:
   (a) 20 s (b) 20 ms (c) 20 µs (d) 20 samples

5. Why does easyPID set the derivative term to zero on the first call after `begin()` or `reset()`?

6. A second-order plant has $\omega_n = 2$ rad/s and $\zeta = 0.2$, $K = 1$. With PD control and
   $K_p = 3$, which $K_d$ gives a closed-loop damping ratio of 0.7?
   (Use $\omega_{cl} = \omega_n\sqrt{1+KK_p}$, $\zeta_{cl} = (2\zeta\omega_n + K\omega_n^2K_d)/(2\omega_{cl})$.)

7. In lab 1, with $K_p = 20$ on the RC circuit, the first part of the step response looks like
   an open-loop charge towards 5 V, much slower than the closed-loop time constant
   $\tau/(1+K_p)$ predicts. Explain.

8. Name one thing feedback gives you that open-loop control cannot, and one new problem it creates.
