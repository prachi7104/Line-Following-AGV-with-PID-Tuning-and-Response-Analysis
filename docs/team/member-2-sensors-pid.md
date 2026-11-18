# Member 2 — Sensors and PID Controller Contribution Record

- **Member:** Prachi Agarwalla (team lead)
- **GitHub username:** prachi7104
- **Assigned module:** Member 2 — Sensors and PID controller
- **Branch:** `module/sensors-pid`, based on `main` at `b158af1`
- **Group:** Project P03 · Group G05 · Batch B3 (CSAI4014)

## 1. Scope

The sensor bar (`controller/sensors.py`) and the controller (`controller/pid.py`) come from the
baseline imported in `b158af1`. They are not part of this contribution, and nothing in them was
changed. This contribution adds tools that measure how they behave, plus tests built on those tools:

| File | Status |
|---|---|
| `controller/diagnostics.py` | New. Read-only sensor and controller diagnostics |
| `tests/test_sensor_controller_module.py` | New. 29 tests |
| `docs/team/member-2-sensors-pid.md` | New. This record |

No changes were made to `common/config.py`, the sensor geometry, the robot, the gains
(P 20/0/0, PI 20/5/0, PID 20/5/2), the PID algorithm, the physics, the experiment runner, or
any file under `results/`.

## 2. What the module under test does

- **Five-sensor bar, ray casting.** Five sensors 30 mm apart, `d = 0.10 m` ahead of the axle.
  Each sensor casts 10 downward sub-rays across its 30 mm footprint, all in one
  `rayTestBatch` call.
- **Fractional reflectance.** A sensor's reading is the fraction of its sub-rays that hit the
  line body, a value in [0, 1].
- **Weighted-centroid error.** `e_sensed = -Σ c_i r_i / Σ r_i`. The sign follows
  `e > 0 ⇔ robot left of the line`. If no ray sees the line, the error is `None`, never 0.
- **PID.** Parallel form, `u = -clamp(kp·e + ki·I + kd·d_f, ±3 rad/s)`. The derivative is
  low-pass filtered (`τ_f = 0.05 s`, `α = dt/(τ_f + dt) = 1/13`). Anti-windup uses conditional
  integration: the integral is skipped on a step where the output would saturate and the error
  pushes it further into saturation.
- **Line loss.** The controller holds the last valid error, or the reset value if the line has
  never been seen, and keeps steering with it.

## 3. Diagnostics added (`controller/diagnostics.py`)

| Function | What it gives |
|---|---|
| `ideal_line_readings(pose)` / `readings_for_offset(e)` / `sensed_error(e)` | The readings and error that a perfect straight line would produce. Uses the same `ray_endpoints` geometry as `LineSensor`, but needs no PyBullet |
| `sensor_characteristic()` | Sweeps the true offset and returns the static transfer curve, plus its quantisation step, dead band, linear range and detection limit |
| `step_with_diagnostics(ctrl, e)` / `trace_controller(mode, errors)` | Steps a real `LineFollowController` and reports the P, I and D terms, the unclamped output, saturation, and whether anti-windup froze the integral this step |
| `run_trial_with_diagnostics(...)` | The `run_trial` loop with one diagnostic record per step. Its DataFrame is identical to `run_trial`'s |
| `line_loss_events(detected)` / `summarize_run(df)` | Line-loss stretches and reacquisition, time of first detection, max \|u\| and saturated share, for any run in the `common.schema` format |

`python -m controller.diagnostics` prints the sensor characteristic and a summary of each
published run. It only reads files.

### Measured sensor characteristic (default configuration)

| Quantity | Value | Where it comes from |
|---|---|---|
| Quantisation step | 3.000 mm | spacing / sub-rays = 30 mm / 10 |
| Dead band | ±1.5 mm (1.495 mm on the 10 µm sweep grid) | half a step. Matches docs/RESULTS.md §4 |
| Linear range | ±61.5 mm, error ≤ 1.5 mm inside it | the line stays fully under the outer sub-rays |
| Detection limit | ±88.5 mm | outermost sub-ray (73.5 mm) + half the line width (15 mm) |

## 4. Test scenarios (`tests/test_sensor_controller_module.py`, 29 tests)

I read the existing coverage in `tests/test_sensors.py`, `tests/test_pid.py` and
`tests/test_integration.py` first and did not repeat it. The new tests cover:

1. **Analytic model vs PyBullet ray casting.** A sweep of 41 offsets across ±95 mm at three
   headings (0, +0.15 and −0.30 rad). The readings arrays must match exactly. Offsets where a
   sub-ray falls within 0.2 mm of a line edge are skipped, because there the two models can
   legitimately disagree; at least 30 offsets must be checked for each heading.
2. **Detection limit in PyBullet.** 1 mm inside the limit the line is seen; 1 mm outside it
   reads `None` with all readings zero, on both sides.
3. **Static characteristic.** The 3 mm step and the 1.5 mm dead band; the error stays within
   half a step across the linear range; the transfer curve is monotonic and odd; the detection
   limit matches the geometric value.
4. **Diagnostics don't change the controller.** For P, PI and PID, the outputs `u`, `e_used`
   and `line_detected` over a sequence that includes line loss and saturation are identical
   (`==`) with and without instrumentation.
5. **Term decomposition.** `u = -clamp(p + i + d)`. The I term is 0 for P, and the D term is 0
   for P and PI.
6. **Derivative filter over 60 steps.** The D term follows `kd·Δe/dt·α(1-α)^k` exactly, and
   after one time constant it is `(12/13)^12 = 0.383` of its first value, close to 1/e.
7. **Anti-windup with the frozen gains** (PI, PID). With a sustained 0.2 m error, every step is
   saturated with the integral frozen. When the error drops, integration resumes at once. For
   PID, the derivative kick saturates the output the other way, and the integral correctly keeps
   integrating.
8. **Line-loss segmentation** of the `line_detected` column.
9. **`run_trial_with_diagnostics` equals `run_trial`** (`DataFrame.equals`).
10. **Line lost at the start, then reacquired** (P, PI, PID, starting 0.15 m left of the line,
    beyond the detection limit). There is exactly one loss stretch, at the start. While lost,
    the held error is 0.15 m (not 0) and the command is −3 rad/s (hard right). For PI and PID,
    anti-windup freezes the integral. The line is found between 0.1 s and 0.5 s and never lost
    again, the run reaches the track end, and the final error is inside ±2.5 mm.
11. **Published runs** (all six CSVs). The line is detected on 100 % of steps with no loss
    events, |u| never reaches the limit, and max |u| < 1.1 rad/s. This checks the docs/RESULTS.md
    statements that the line is never lost and anti-windup never acts.

## 5. Verification

Environment: Windows 11, micromamba env `agv`, the locked reference environment
(`environment.lock.yml`): Python 3.11.18, PyBullet 3.25 from conda-forge (reports itself as
3.2.5), numpy 2.4.6 with OpenBLAS, scipy 1.17.1, pandas 3.1.0, matplotlib 3.11.2, pytest 9.1.1.

| Check | Command | Result |
|---|---|---|
| New tests | `python -m pytest -q -rs tests/test_sensor_controller_module.py` | 29 passed, 0 skipped |
| Focused (sensors, PID, new) | `python -m pytest -q -rs tests/test_sensors.py tests/test_pid.py tests/test_sensor_controller_module.py` | 50 passed, 0 skipped |
| Full suite | `python -m pytest -q -rs` | 136 passed, 0 skipped (107 baseline + 29 new) |
| Baseline suite on `main` (`b158af1`), fresh clone | `python -m pytest -q -rs` | 107 passed, 0 skipped |
| Published CSVs regenerated | `python -m experiments.run_experiments --experiment all`, run in a disposable fresh clone of `main` | all six files byte-identical to the reference workspace's `results/runs/*.csv` |
| Instrumented runs vs published | `run_trial_with_diagnostics` for all six runs, compared with `read_run` of the published CSV | all six exactly equal (every column, every row); 0 saturated and 0 frozen-integral steps |

The `results/` files on `main` were not regenerated or touched. The regeneration ran only in the
disposable clone.

## 6. Known limitations

- The analytic sensor model assumes a straight line along the world x axis. It doesn't model
  arc curvature under the bar, and near a sub-ray/line-edge tie it can differ from PyBullet. The
  tests avoid those offsets on purpose.
- `run_trial_with_diagnostics` repeats the `run_trial` loop (headless only, no GUI or
  real-time). A test asserts that the two give identical output, so if the two loops drift
  apart, the suite fails.
- The diagnostics read the PID's internal state (`integral`, `d_filtered`), so they depend on
  `controller/pid.py`'s attribute names. `integral_frozen` is reported as False when the error is
  exactly 0, because skipping and integrating can't be told apart then.
- Line loss is exercised only at the start of a run. With the frozen tracks and gains, the line
  is never lost mid-run, and no new tracks or disturbances were added to force it.
- All results above were run on Windows with the conda-forge PyBullet build only.

## 7. Proposed root README contribution row

```markdown
| Prachi Agarwalla | Sensors and PID controller | Added controller/diagnostics.py (sensor transfer characteristic, PID term/saturation/anti-windup tracing, line-loss and reacquisition summaries), tests/test_sensor_controller_module.py (29 tests) and docs/team/member-2-sensors-pid.md | 29 new / 136 full tests passed (0 skipped); six runs regenerated byte-identical to the published CSVs |
```
