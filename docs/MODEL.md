# The linearised lateral model, and how to read the Bode plot

Code: `analysis/lateral_model.py` (transfer functions, poles, time response) and
`analysis/bode_analysis.py` (margins, Bode plot, model-vs-simulation plot, `results/model_margins.md`).
Numbers: `common/config.py` (`v = 0.30 m/s`, `d = 0.10 m`, starting gains P (20, 0, 0),
PI (20, 5, 0), PID (20, 5, 2), derivative filter `τf = 0.05 s`).

Sign convention (same as the code): `e > 0` means the robot is left of the line, `ω > 0` turns
left, and the control law is `ω = −C(s)·e_s`.

## 1. Assumptions (stated before any maths)

1. **Small heading error**: `sin θ ≈ θ`, so the equations are linear.
2. **Straight line**: the reference is a straight line. On the curve the model only says what
   happens to deviations from the arc, not the steady offset P needs to turn.
3. **The robot turns at exactly the commanded rate, instantly**: `θ̇ = ω`. No wheel-motor lag.
4. **No wheel slip**: the robot moves along its heading at constant speed `v`.
5. **No saturation**: `|ω|` is never clipped (the time-domain helper can clip it, see section 6).
6. **Continuous time**: the controller acts continuously, not every 1/240 s.
7. **Perfect sensing**: no sensor noise, no quantisation; the bar reads `e + d·θ` exactly.

## 2. State equations

Let `e` be the axle's lateral distance from the line and `θ` the heading error (radians,
positive = pointing left of the line direction).

```
de/dt     = v · sin θ  ≈  v · θ          (moving forward at angle θ drifts sideways)
dθ/dt     = ω                             (assumption 3)
e_s       = e + d · θ                     (the bar is d ahead of the axle; for small θ it
                                           sees the axle error plus d·θ)
```

## 3. Laplace transform → plant

Take Laplace transforms with zero initial conditions:

```
s Θ(s) = Ω(s)                 ⇒  Θ(s) = Ω(s)/s
s E(s) = v Θ(s)               ⇒  E(s) = v Ω(s)/s²
E_s(s) = E(s) + d Θ(s) = (v/s² + d/s) Ω(s)
```

so the plant seen by the controller is

```
G(s) = E_s(s)/Ω(s) = (v + d s) / s²
```

Two integrators (`s²`): turning rate → heading → lateral position. One zero at `s = −v/d`
(= −3 rad/s), created entirely by the look-ahead. With `d = 0` the zero disappears.

## 4. The loop

Controller `C(s) = Kp + Ki/s + Kd s`, optionally with the derivative filtered as
`Kd s/(τf s + 1)`; optional actuator lag `1/(τ s + 1)` on `ω` (default τ = 0, i.e. assumption 3).

```
L(s) = C(s) · (v + d s)/s²  [ · 1/(τ s + 1) ]
```

The closed-loop poles are the roots of `1 + L(s) = 0`, i.e. `den(L) + num(L) = 0`.

**A trap to avoid.** Written over a common denominator, `C(s) = (Kd s² + Kp s + Ki)/s`. If you
use that form for P (`Ki = Kd = 0`) you get `Kp s / s`; the `s` does not cancel numerically, a
fake pole at the origin appears, and a perfectly stable P loop looks marginal. The code only
introduces the `1/s` when `Ki ≠ 0` (and the filter factor only when `Kd ≠ 0`), so no common factor
ever appears (`analysis.lateral_model.controller_tf`).

## 5. Characteristic polynomials

**P** (`C = Kp`): `s² + Kp d s + Kp v = 0`. Compare with the standard form
`s² + 2ζωn s + ωn² = 0`:

```
ωn = sqrt(Kp v)                         = sqrt(20 · 0.30)           = 2.449 rad/s
ζ  = Kp d / (2 ωn) = (d/2) sqrt(Kp/v)   = 0.05 · sqrt(20/0.30)      = 0.408
poles = −ζωn ± j ωn sqrt(1 − ζ²)        = −1.000 ± 2.236j
```

All the damping comes from `d`: with `d = 0`, `ζ = 0`, the poles are `±2.449j` and the P loop
oscillates forever (marginal). ζ = 0.408 predicts about 25 % overshoot
(`exp(−πζ/sqrt(1−ζ²))`), noting that the plant zero changes the exact figure somewhat.

**PI** (`C = (Kp s + Ki)/s`):

```
s³ + Kp d s² + (Kp v + Ki d) s + Ki v = 0
```

Routh–Hurwitz for a cubic `s³ + a2 s² + a1 s + a0`: stable iff all coefficients are positive and
`a2 a1 > a0`. With `d = 0`, `a2 = 0`, so PI is **always unstable** without look-ahead (poles
`0.124 ± 2.459j`). With `d = 0.10`: `a2 a1 = 2 · 6.5 = 13 > a0 = 1.5`, stable.

**PID** (`C = (Kd s² + Kp s + Ki)/s`):

```
(1 + Kd d) s³ + (Kp d + Kd v) s² + (Kp v + Ki d) s + Ki v = 0
```

Here `Kd` adds the `Kd v` term to `a2`, so PID is stable even at `d = 0`, but only barely
(ζ = 0.071). That is the **ideal-D** PID, a theoretical reference only. The **implemented**
controller filters its D term: C(s) = Kp + Ki/s + Kd s/(τf s + 1) with τf = 0.05 s, and
`controller/pid.py` runs the backward-Euler discretisation of exactly this filter. The filter adds
one pole far to the left (−23.8 at d = 0.10) and raises the damping slightly (ζ 0.431 → 0.443). The
Bode plot, the margins table and the model-vs-simulation figure all use the implemented (filtered)
form for PID. The ideal form appears only as a dashed, labelled reference.

**Results** (`results/model_margins.md`, reproduces docs/DESIGN.md section 3):

| Controller | d = 0 | d = 0.10 m |
|---|---|---|
| P (20, 0, 0) | marginal, ±2.449j | stable, −1.000 ± 2.236j, ζ = 0.408 |
| PI (20, 5, 0) | unstable, 0.124 ± 2.459j | stable, −0.876 ± 2.302j (ζ = 0.356), −0.247 |
| PID (20, 5, 2), filtered D τf = 0.05 s — **implemented** | stable, ζ = 0.072 | stable, −0.989 ± 2.000j (ζ = 0.443), −0.254, −23.8 |
| PID (20, 5, 2), ideal D — theory only | stable, ζ = 0.071 | stable, −0.957 ± 2.004j (ζ = 0.431), −0.253 |

The real pole near −0.25 sits almost on the controller zero at `−Ki/Kp = −0.25`, so the two
nearly cancel: on a straight line PI and PID should look like P plus a small slow tail.

## 6. Time response used for the model-vs-simulation plot

`simulate_offset_response` integrates the same model in time with `scipy.integrate.solve_ivp`,
states `[e, θ, z]` with `z = ∫ e_s dt`, starting from `e(0) = e0`, `θ(0) = 0`, `z(0) = 0`
(the 5 cm offset test). The D term needs `de_s/dt = v θ + d ω`, which itself contains `ω`, so it is
solved algebraically:

```
ω = −(Kp e_s + Ki z + Kd (v θ + d ω))   ⇒   ω = −(Kp e_s + Ki z + Kd v θ)/(1 + Kd d)
```

That algebraic form is the **ideal-D** case (`tau_f = 0`), a theory reference only. The
**implemented** PID filters its derivative (`tau_f = 0.05 s`), and the model-vs-simulation figure
uses that form for PID. A fourth state `f`, the low-passed derivative of `e_s`, is added. The
derivative is filtered, so ω no longer appears on both sides of its own equation (no algebraic loop):

```
τf df/dt = de_s/dt − f,   de_s/dt = v θ + d ω,   f(0) = 0
ω = −(Kp e_s + Ki z + Kd f)
```

`f(0) = 0` matches the controller, whose `reset(e0)` zeroes the filter and sets `e_prev = e0`, so it
starts with no derivative kick. P and PI have no D term and use the three-state model.

When `omega_max` is given, `ω` is clipped to `±omega_max` (the one non-linearity allowed in this
helper, used for the comparison with the simulation). The decay rate of the computed `e(t)`
matches `−Re(pole)` for both forms (checked in `tests/test_model.py`).

## 7. What the Bode plot shows and how to read it

`results/figures/bode.png` plots `|L(jω)|` in dB and the phase of `L(jω)` in degrees against
frequency for P, PI and PID as implemented (solid; PID with its filtered D), plus the ideal-D
PID as a dashed theory reference, at `d = 0.10 m`.

- **Gain crossover `ω_gc`**: where `|L| = 1` (0 dB). Roughly the closed-loop bandwidth; here
  about 2.9–3.0 rad/s for all three, because `Kp` and `d` dominate there.
- **Phase margin** = `180° + phase(L(jω_gc))`. How much extra phase lag (for example from a
  delay or a slow motor) the loop tolerates before it oscillates. P: 43.9°, PI: 39.0°
  (the integrator costs phase), PID: 56.5° at 3.01 rad/s as implemented, 56.0° at 2.93 rad/s
  with an ideal D (the D term adds phase).
- **Phase crossover `ω_pc`**: where the phase is −180°. **Gain margin** = `−20 log10 |L(jω_pc)|`,
  how far the gain can change before the loop becomes unstable.
  - P: the phase is `−180° + atan(dω/v)`, always above −180°, so the gain margin is infinite: in
    this ideal model any `Kp > 0` is stable when `d > 0`.
  - PI and PID: the phase starts at −270° (three integrators) and rises through −180°. These
    loops are *conditionally stable*: the gain margin is **negative** (−18.8 dB for PI, −20.9 dB
    for PID), which means the gain could be **reduced** by that much (about 8.7× for PI) before
    the loop goes unstable. Check for PI: the Routh condition `a2 a1 > a0` with every gain
    scaled by `k` gives `13 k² > 1.5 k`, i.e. `k > 0.115` = −18.8 dB. Raising the gain is safe in
    this model.
- Look at the phase lead from the plant zero at `v/d = 3 rad/s`: without it (d = 0) the P phase is
  exactly −180° at every frequency, the phase margin is 0 and the gain margin is undefined.
  That is the frequency-domain picture of "the look-ahead supplies the damping".
- Always check the poles too. For PI at d = 0 the phase only approaches −180° from below, so
  the algorithm reports "no phase crossing" (infinite gain margin) even though the loop is
  unstable; the negative phase margin (−5.8°) and the pole at `+0.124` tell the truth.

Margins are computed numerically (`analysis.bode_analysis.margins`): crossings are found on a
log-spaced frequency grid (1e−4 to 1e4 rad/s) and refined with `scipy.optimize.brentq`; the phase
is built as a continuous sum of the angles of each pole and zero (an unwrapped phase), so a loop
that starts at −270° is not mistaken for +90°. Check case: `L = 1/(s(s+1))` gives PM = 51.8° and
infinite GM (tested).

## 8. Why this is NOT a claim about the PyBullet robot

The model is a prediction, made before running the simulation. The simulated robot differs in
ways the model leaves out:

- **Wheel-motor lag**: wheel speeds are commanded with `VELOCITY_CONTROL`; the robot does not
  reach the commanded turn rate instantly. Lag adds phase loss near crossover and reduces the
  real phase margin. (The code can include a first-order lag `τ` to explore this; it is off by
  default because τ has not been measured.)
- **Torque limit**: the motors are capped at `5 N·m`, so large speed changes take time.
- **Saturation**: `|ω| ≤ 3 rad/s`. A 5 cm offset asks for `Kp·e = 1 rad/s` with P, which is
  inside the limit, but D and transients can hit it, and a saturated loop is not linear.
- **Discrete control at 240 Hz**: the controller updates every 1/240 s. That adds about half a
  sample of delay, small at 3 rad/s but not zero.
- **Sensor quantisation**: the line position is measured from 10 rays per sensor, a 3 mm pitch.
  Near zero error this can cause a small limit cycle the linear model cannot show, which is why
  a run that never settles is reported as "not meaningful" rather than forced.
- **Large angles and the curve**: on the arc, and at the start of a big correction, `θ` is not
  small.

So `results/figures/model_vs_sim.png` overlays the model's `e(t)` on the simulated `e_true`, and
`docs/RESULTS.md` is where the agreement and the disagreement are reported. The model explains the
trends (look-ahead gives damping, PI without look-ahead diverges, the integrator adds a slow
tail); it does not certify the simulated robot's numbers.

## 9. Viva quick answers

- *Why is the plant `(v + d s)/s²`?* Two integrations (turn rate → heading → lateral position)
  and a sensor that also sees `d·θ`, which adds a zero at `−v/d`.
- *Why does P work at all?* Only because `d > 0`: ζ = `(d/2) sqrt(Kp/v)`. With the sensor at the
  axle, P is a frictionless spring: it oscillates forever.
- *Why can't PI work without look-ahead?* The `s²` coefficient of `s³ + Kp d s² + …` is zero
  when `d = 0`, so Routh fails.
- *What does a negative gain margin mean here?* Conditional stability: lowering the gain too much
  destabilises PI/PID, raising it does not (in the ideal model).
- *What would change the answer?* Motor lag, saturation, sampling delay and quantisation, all
  present in the simulation and absent here.
