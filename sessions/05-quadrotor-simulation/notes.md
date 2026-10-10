# Session 5 notes · Quadrotor dynamics and control

The vehicle throughout is the nominal 5-inch quad of the rotorfault study: 0.638 kg,
inertia diag(1.36, 2.12, 3.13)·10⁻³ kg m², arm 0.11 m, 5.6 N maximum thrust per rotor
(thrust-to-weight 3.6), motor time constant 25 ms. All of these are the study's
assumptions, not measured values.

## 1. Rigid-body dynamics

World frame z up, body frame FLU, attitude $R$ (body → world), body rates $\omega$:

$$\dot p = v, \qquad m\dot v = R\begin{bmatrix}0\\0\\T\end{bmatrix} - R D R^Tv - mg\,e_3,$$

$$\dot q = \tfrac12\,q\otimes(0,\omega), \qquad J\dot\omega = \tau - \omega\times J\omega - K_\omega\omega .$$

The thrust always points along body z: to move sideways the vehicle must tilt. The term
$\omega\times J\omega$ (gyroscopic coupling) is small near hover and large when spinning (session 6).

**Rotors.** Rotor $i$ spins at $\Omega_i$ and produces thrust $f_i = k_f\Omega_i^2$ and a reaction
torque $s_i\kappa f_i$ about body z, with $s_i = \pm1$ by spin direction and $\kappa \approx$ 0.01-0.02 m
for small propellers. The ESC and motor make $\Omega_i$ follow its command with a lag
$\tau_m$. The minimal simulator leaves out the reaction torque of *accelerating* rotors
($s_iJ_r\dot\Omega_i$) and rotor gyroscopics; notebook 5 shows what that changes.

## 2. The allocation (mixing) matrix

Rotors 1-4 (front-right, front-left, rear-left, rear-right) at positions $(x_i, y_i)$:

$$\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix} = B\,f, \qquad
B = \begin{bmatrix}1&1&1&1\\ y_1&y_2&y_3&y_4\\ -x_1&-x_2&-x_3&-x_4\\ \kappa s_1&\kappa s_2&\kappa s_3&\kappa s_4\end{bmatrix}.$$

For the X layout with $a = l/\sqrt2$ and $s = (+1, -1, +1, -1)$:

$$B = \begin{bmatrix}1&1&1&1\\ -a&a&a&-a\\ -a&-a&a&a\\ \kappa&-\kappa&\kappa&-\kappa\end{bmatrix}.$$

$B$ is square and invertible: four motors, four controlled quantities, **no redundancy**
(the root of session 6). In hover $\tau = 0$ and $T = mg$, so $f_i = mg/4$ and
$\Omega_h = \sqrt{mg/(4k_f)}$ (1321 rad/s, 28 % of maximum thrust for the study's quad).

## 3. Saturation and priorities

$f = B^{-1}[T, \tau]$ may ask for more than $[f_{min}, f_{max}]$. Order of sacrifice:

1. keep **roll/pitch** torque (attitude keeps the vehicle in the air); scale it only if its
   spread across rotors exceeds the whole thrust range;
2. **shift collective thrust** by the smallest amount that fits (air mode);
3. scale **yaw** torque by the largest $\alpha\in[0,1]$ such that every rotor stays in range:
   $\alpha = \min\big(1, \min_{d_i>0}\frac{f_{max}-f_{0,i}}{d_i}, \min_{d_i<0}\frac{f_{min}-f_{0,i}}{d_i}\big)$.

Yaw goes first because quad yaw authority is small ($\kappa = 1.5$ cm against a 7.8 cm lever
for roll/pitch) and an unwanted slow yaw is harmless. `bootcamp.quad.allocate` implements
these steps in closed form; rotorfault's `PriorityAllocator` does the same (and accepts
an effectiveness vector, session 6).

## 4. From acceleration to attitude

The outer loops output a desired acceleration $a$. With thrust $\approx mg$ and small angles,
$Re_3 \approx (\theta\cos\psi + \phi\sin\psi,\ \theta\sin\psi - \phi\cos\psi,\ 1)$, so

$$\theta_d = \frac{a_x\cos\psi + a_y\sin\psi}{g}, \qquad \phi_d = \frac{a_x\sin\psi - a_y\cos\psi}{g}, \qquad T = \frac{m(g + a_z)}{\cos\phi\cos\theta}.$$

Large manoeuvres use the exact geometric version (desired body z axis $= a/|a|$), as the
rotorfault study's SE(3) controller does. A tilt limit (35° here) caps horizontal acceleration.

## 5. The cascade and time-scale separation

| loop | input → output | bandwidth (order of) |
|---|---|---|
| position | position error → velocity setpoint (P, limited to 3 m/s) | ~1-2 rad/s |
| velocity | velocity error → acceleration (PI) | ~3-4 rad/s |
| attitude | angle error → rate setpoint (P) | ~8 rad/s |
| rate | rate error → angular acceleration (PID), × J → torque | ~25 rad/s |
| motors | command → thrust | 40 rad/s ($1/\tau_m$) |

Each loop is several times faster than the one around it, so each outer loop can treat
the inner one as "does what I ask, quickly". The innermost loop must stay slower than the
actuators: in notebook 5 the default rate loop tolerates a motor lag of 6x nominal,
oscillates at 8x and crashes at 12x.

**Tuning order**: rate → attitude → velocity → position, always inside out, always with a
specification and a log. In the notebook, removing the horizontal velocity integrator
(it integrates velocity error, i.e. position error) cut a 1 m step's overshoot from 5.5 %
to 1.9 % -- the kind of insight a spec forces on you.

## 6. Models and their limits

`bootcamp.quad` matches rotorfault's dynamics to 10⁻⁵ on a pure roll command but
under-predicts the initial yaw acceleration, because it omits the rotor reaction torque
$s_iJ_r\dot\Omega_i$, which for a yaw command is comparable to the drag torque. Neither model
includes vibration, battery sag, ground effect, blade flapping or a real wind field. A
simulation answers "what would happen if the model were right"; the rig, and eventually
a real vehicle, answer the rest.

## Further reading

- R. Mahony, V. Kumar, P. Corke, "Multirotor aerial vehicles", *IEEE RAM*, 19(3), 2012.
- T. Lee, M. Leok, N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)", CDC 2010.
- The rotorfault README, sections 2.1-2.2 (vehicle model and allocation).
