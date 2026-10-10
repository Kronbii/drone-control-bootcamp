# Quiz 5 · Quadrotor dynamics and control

*10 minutes, individual. Answers: `instructor/quiz-keys.md`.*

1. Why must a quadrotor tilt to accelerate horizontally?

2. Write the first two rows of the allocation matrix $B$ for an X quad with rotor
   positions FR $(a, -a)$, FL $(a, a)$, RL $(-a, a)$, RR $(-a, -a)$.

3. A quad of mass 0.64 kg has $k_f = 8.96\times10^{-7}$ N s². Compute the hover rotor speed.
   If each rotor can give 5.6 N, what is the thrust-to-weight ratio?

4. When the motors saturate, which of thrust, roll/pitch torque and yaw torque is
   sacrificed first, and why that one?

5. The yaw angle is 0. The velocity loop asks for $a_x = 2$ m/s², $a_y = 0$. Which pitch
   angle does the small-angle map command? Which sign would roll have for $a_y = +2$ m/s²
   (body y points left)?

6. List the four loops of the cascade from outside in, and explain "time-scale separation"
   in one sentence.

7. The vehicle oscillates in attitude only when the motor time constant is 8x larger than
   the controller assumes. Which loop runs out of phase margin first, and what would you
   change in its tuning?

8. The minimal model agrees with the rotorfault model for a roll command but not for a
   yaw command. Which physical effect is missing, and why does it matter more for yaw?
