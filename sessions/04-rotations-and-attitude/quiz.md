# Quiz 4 · Rotations and attitude

*10 minutes, individual. Answers: `instructor/quiz-keys.md`.*

1. Write $R_z(90°)$ and compute $R_z(90°)\,(1, 0, 0)^T$. What does the result mean physically?

2. Is $R_z(\psi)R_y(\theta) = R_y(\theta)R_z(\psi)$ in general? Give a one-line argument or a counterexample.

3. What happens to Z-Y-X Euler angles at a pitch of 90°, and why do flight controllers
   keep attitude as a quaternion or a rotation matrix?

4. The quaternion for a 90° rotation about z is $q = (\cos 45°, 0, 0, \sin 45°)$. Write $q^*$ and
   say what rotation it represents.

5. An accelerometer at rest on a beam reads $a_x = 0.5\,g$, $a_z = 0.866\,g$. What is the tilt?
   Would the reading be correct while the beam swings? Why or why not?

6. A complementary filter runs at 200 Hz with $\tau = 0.5$ s. What is $\alpha$? If the gyro has
   an uncorrected bias of 2 deg/s, roughly what steady error results?

7. Name one thing a Kalman filter with state $[\theta, b]$ does that the complementary filter does not.

8. In a cascaded controller, which loop uses the gyro, which loop is tuned first, and why
   does the cascade reject a tap on the beam better than a single angle loop?
