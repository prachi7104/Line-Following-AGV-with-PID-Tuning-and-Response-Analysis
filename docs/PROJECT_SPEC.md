# Project Spec — Line-Following AGV with PID Tuning and Response Analysis

CSAI4014 AISR, Batch B3, Group G05, Project P03. Platform: PyBullet + Python.
This file freezes the technical decisions agreed on 27 Oct 2026. Personal implementation by
Prachi Agarwalla; the group submission lives in a separate repository.

## Problem (from the project allocation sheet)

Simulate an automated guided vehicle that tracks a floor line using simulated ground sensors;
compare P, PI and PID, plot step response (rise time, overshoot, settling time) and a Bode plot
of the loop. Syllabus: Open/Closed-Loop Control; Time and Frequency Domain; Stability and
Performance. Industry use: factory-floor AGVs, hospital tug robots.

## Frozen decisions

| Item | Decision |
|---|---|
| Robot | Differential-drive AGV, two driven wheels + passive casters |
| Track | Straight track; straight lead-in + constant-curvature arc |
| Line | Separate thin collision box body (1–2 mm high), robot–line contact disabled |
| Sensors | 5 downward reflectance sensors on a bar a distance `d` ahead of the wheel axle; each sensor modelled by several sub-rays (`rayTestBatch`), a sub-ray "sees" the line when its `objectUniqueId` equals the line body id |
| Coverage | Sensor bar wider than ±5 cm so the line is inside the array at a 5 cm offset |
| Line width | At least the sensor spacing, so the line can never sit between sensors |
| Controllers | P, PI, PID, selectable, same interface, output limits |
| Sign convention | `e > 0` ⇔ robot is LEFT of the line; `u > 0` turns LEFT; control law `u = -PID(e)` |
| Derivative | Previous error initialised to the starting error each run (no derivative kick) |
| Line lost | Hold last valid error, log `line_detected = 0`; never treat a miss as zero error |
| Main test | 5 cm lateral offset, aligned heading, zero initial I and D state — "response to a 5 cm lateral-reference step, represented by the equivalent initial-offset condition" |
| Curve test | All three controllers on the curved track |
| Metrics source | Ground-truth error `e_true` for metrics; sensed error `e_sensed` for controller behaviour |
| Bode | From a derived linearised model including `d` and `v`, assumptions stated |
| Optional | Look-ahead (`d`) sweep, only after the required experiments work |
| Out of scope | Payload tests, auto-tuning, RL, extra track designs |

## Metric definitions (fixed before running)

- Rise time: time for |e| to fall from 90 % to 10 % of the initial 5 cm error.
- Overshoot: largest crossing beyond the line, as % of the initial offset (0 if none).
- Settling time: first time `e_true` enters and stays within ±2.5 mm until the run ends.
- IAE: ∫|e_true| dt, trapezoidal over logged samples.
- Curve: mean |e|, max |e|, steady-state e (mean over the last part of the arc).
- A metric that is not meaningful for a run is reported as "not meaningful", never forced.

## CSV schema (one row per simulation step)

`t,x,y,yaw,e_true,e_sensed,u,controller,line_detected`

Personal-repo addition (appended, does not reorder the agreed columns): `segment`
(`straight` or `arc`), needed to compute curve metrics on the arc only.

## Novelty claims (report only what results show)

1. Quantitative comparison of P, PI and PID on one consistent simulated AGV.
2. Response analysis on ground-truth lateral error with predefined transient metrics.
3. Model-to-simulation comparison including sensor look-ahead distance.
