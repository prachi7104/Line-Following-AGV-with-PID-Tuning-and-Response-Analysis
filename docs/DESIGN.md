# Design notes — the decisions you need to defend

Every number below comes from `common/config.py`. Units are SI: metres, seconds, radians.

## 1. What the controller outputs (`u`) and how it reaches the wheels

**`u` is a commanded yaw rate in rad/s** (how fast the robot turns), not a wheel speed and not a
steering angle. Positive `u` turns the robot left (counter-clockwise seen from above).

The robot drives at a constant forward speed `v = 0.30 m/s`. A differential drive turns by
spinning its wheels at different speeds. For a unicycle command `(v, u)` the wheel angular
speeds are

```
omega_right = (v + u * W / 2) / r      [rad/s]
omega_left  = (v - u * W / 2) / r      [rad/s]
```

with wheel radius `r = 0.035 m` and track width `W = 0.16 m`. `u` is clamped to
`|u| <= omega_max = 3.0 rad/s`. At the limit the inner wheel still turns forwards
(`0.30 - 3.0 * 0.08 = 0.06 m/s`), so the robot never pivots or reverses. The wheel speeds are
sent to PyBullet with `VELOCITY_CONTROL` and a torque cap of `5 N m`, so the motors need a short
time to reach the commanded speed. That lag is one of the places where the simulation differs
from the ideal model.

**Sign chain.** `e > 0` means the robot is left of the line. To correct it, the robot must turn
right, which is `u < 0`. So the control law is `u = -PID(e)`.

## 2. From five fractional readings to a signed error in metres

The sensor bar sits `d = 0.10 m` ahead of the wheel axle. It has 5 sensors with centres at
`c = [-0.06, -0.03, 0, +0.03, +0.06] m` (robot frame, left positive). Each sensor looks at a
3 cm wide patch of floor, modelled by 10 downward rays spread across that patch. Each ray is cast
with `rayTestBatch`. A ray "sees the line" only when the body it hits is the line body (its
`objectUniqueId` equals the line id). Hitting the floor or anything else does not count.

**Reading of sensor i**: `r_i = (rays of sensor i that hit the line) / 10`, a number in [0, 1].
This behaves like an analog IR reflectance sensor, which reports how much of its patch is dark.

**Where the line is** (in the robot frame, left positive):

```
y_line = sum(c_i * r_i) / sum(r_i)            [m]
```

**Signed error**: `e_sensed = -y_line`. If the line is to the robot's right (`y_line < 0`), the
robot is left of the line and `e_sensed > 0`, which matches the sign convention. The units are
metres because each `c_i` is in metres and each `r_i` is a dimensionless weight.

**Why this is exact, not just an approximation.** The line is 3 cm wide, the same as the spacing
and the patch width. Suppose the line centre is at `c_k + delta`, with `0 <= delta < 3 cm`. Then
sensor `k` is covered by the fraction `(s - delta)/s` and sensor `k+1` by `delta/s`, so

```
y_line = c_k (1 - delta/s) + (c_k + s) delta/s = c_k + delta
```

The weighted centroid returns the true line position. It is limited only by the ray pitch
(3 mm), not by the 3 cm sensor spacing. With on/off sensors, the error could only take about
5 values, so the ±2.5 mm settling band could never be measured.

**Coverage.** The bar sees the line while it is within ±(0.06 + 0.015) = ±7.5 cm. A 5 cm start
offset plus half the line width (1.5 cm) is 6.5 cm, so the line is inside the array from the
first step.

**Line lost.** If no ray hits the line, `sum(r_i) = 0`, and the sensor returns `None` rather than
0. The controller keeps the last valid error and the run logs `line_detected = 0`. Treating a
miss as zero error would tell the robot it is perfectly on the line exactly when it is furthest
away.

**`e_sensed` is not `e_true`.** `e_true` is the axle's distance from the line, calculated from
the robot's known pose and the track geometry. The bar is ahead of the axle, so for a small
heading error `theta` it reads about `e + d * theta`. This difference is the look-ahead effect
and is what gives the loop its damping (Section 3). Metrics use `e_true`, because that is where
the robot actually is.

## 3. The linearised model, and the stability check done before coding

Near a straight line, for small heading error `theta` (radians, positive = pointing left of the
line direction):

```
de/dt     = v * sin(theta) ≈ v * theta
dtheta/dt = u
e_s       = e + d * theta                 (what the bar measures)
```

Taking Laplace transforms: `E_s(s)/U(s) = (v + d s) / s^2`. With `u = -C(s) e_s` and
`C(s) = Kp + Ki/s + Kd s`, the closed-loop poles are the roots of
`s^2 · den(C) + num(C) · (v + d s) = 0`.

Assumptions: small heading error, straight line, the robot turns at exactly the commanded rate
(no wheel lag, no slip), no saturation, continuous time, sensor quantisation ignored.

**Check run on 16 Nov 2026** (`v = 0.30`, starting gains, `numpy.roots`):

| Controller | d = 0 (sensor at axle) | d = 0.10 m (our robot) |
|---|---|---|
| P (20, 0, 0) | **marginal**: poles ±2.449j, sustained oscillation | **stable**: −1.00 ± 2.24j, ζ = 0.408 |
| PI (20, 5, 0) | **unstable**: 0.124 ± 2.46j | **stable**: −0.88 ± 2.30j (ζ = 0.356), −0.247 |
| PID (20, 5, 2), ideal D (theory only) | stable but barely: ζ = 0.071 | **stable**: −0.96 ± 2.00j (ζ = 0.431), −0.253 |
| PID (20, 5, 2), filtered D τf = 0.05 s (**implemented**) | ζ = 0.072 | **stable**: −0.99 ± 2.00j (ζ = 0.443), −0.254, extra fast pole at −23.8 |

The P result matches the hand formula: `s^2 + Kp d s + Kp v = 0`, so `ωn = sqrt(Kp v) = 2.449`
rad/s and `ζ = (d/2) sqrt(Kp/v) = 0.408`.

**What this predicts, to be checked against the simulation:**

1. The look-ahead `d` supplies the damping. Without it, P oscillates forever and PI grows.
2. With `d = 0.10`, all three controllers are stable. P should overshoot by roughly 25 %
   (from ζ ≈ 0.41).
3. Integral action adds a slow real pole near −0.25 s⁻¹, almost on top of the controller zero at
   `-Ki/Kp = -0.25`. They nearly cancel, so on the straight track PI/PID should look much like
   P, with a small slow tail. The integrator should matter on the curve, where P must hold a
   steady offset to produce the turn rate `v/R`.
4. The D term adds little damping here (ζ 0.356 → 0.431 with an ideal D; 0.443 with the
   implemented filtered D), because the look-ahead already provides most of it.
5. With a slow pole at −0.25 s⁻¹ (time constant ≈ 4 s), the integrator will only partly remove
   the curve offset during the ~7.9 s arc. This is checked in Phase 2 tuning. If `Ki` is raised,
   the new gains are re-checked with the same pole calculation before they are frozen.

These are predictions from an idealised model, not claims about the simulated robot. The results
document reports where the simulation agrees and where it doesn't.
