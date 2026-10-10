# Quiz 7 · Final concept check

*15 minutes, individual, covering the whole bootcamp. Answers: `instructor/quiz-keys.md`.
Part of the certificate criteria (at least 6 of 10).*

1. A P controller on a first-order plant with static gain 2 uses $K_p = 4$. What fraction of
   the setpoint does the output reach?

2. Why does integral action need anti-windup when the actuator saturates? Which easyPID
   mode is the default, and what exactly does it do?

3. You pass `pid.update(sp, y, 0.01)` to easyPID, meaning a 10 ms step. What goes wrong?

4. Why did proportional control alone go unstable on the seesaw rig, and what fixed it?

5. Give one reason the accelerometer alone is a poor attitude sensor on a flying drone,
   and one reason the gyro alone is.

6. In a cascaded attitude controller, which loop is faster and which is tuned first?

7. For an X quadrotor, which of thrust, roll/pitch torque and yaw torque is sacrificed first
   when the motors saturate?

8. A quad has thrust-to-weight 3. Can it hover with its heading held if one rotor is at 30 %
   effectiveness? Show why.

9. In one sentence each: what does a CUSUM test accumulate, and what sets its threshold
   in the rotorfault study?

10. Name one result of this bootcamp's simulations that you would not trust on real
    hardware without a measurement, and the measurement you would make.
