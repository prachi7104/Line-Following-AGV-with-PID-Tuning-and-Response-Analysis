# Line-Following AGV with PID Tuning — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A PyBullet differential-drive AGV that follows a floor line with selectable P / PI / PID control, reproducible experiments logged to CSV, transient and curve metrics, and a Bode analysis from a derived linearised model.

**Architecture:** Five small packages with one job each: `common/` (config + CSV schema), `simulation/` (world, robot, track geometry), `controller/` (ray sensors, PID), `analysis/` (metrics, plots, model, Bode), `experiments/` (runner, GUI demo). Geometry, PID, sensor maths, metrics and the model are pure NumPy so they are unit-tested without PyBullet; only thin wrappers touch the physics client.

**Tech Stack:** Python 3.11, PyBullet 3.25 (conda-forge, via micromamba env `agv`), NumPy, SciPy, pandas, matplotlib, pytest.

**Spec:** `docs/PROJECT_SPEC.md`

## Global Constraints

- Sign convention: `e > 0` ⇔ robot LEFT of the line (looking along travel); `u > 0` turns LEFT; control law `u = -PID(e)`.
- CSV columns, in order: `t,x,y,yaw,e_true,e_sensed,u,controller,line_detected,segment`.
- Every tunable number lives in `common/config.py`; no magic numbers elsewhere.
- Initial values: `dt = 1/240 s`, `v = 0.30 m/s`, `d = 0.10 m`, `wheel_radius = 0.035 m`, `track_width = 0.16 m`, `omega_max = 3.0 rad/s`, `wheel_force = 5.0 N·m`.
- Sensors: 5 sensors, spacing `0.03 m` (centres −0.06…+0.06), 10 sub-rays per sensor across its 3 cm footprint; rays cast from `z = 0.012` to `z = −0.005`; chassis bottom ≥ `0.02 m` so rays never start inside the robot.
- Line: width `0.03 m`, height `0.0015 m`, ONE body (compound shape), robot–line collisions disabled.
- Tests: `initial offset = 0.05 m`, settling band `±0.0025 m`; curve: lead-in `1.0 m`, radius `0.75 m`, left turn of `π` rad; straight track `4.0 m`.
- Starting gains (Kp, Ki, Kd): P `(20, 0, 0)`, PI `(20, 5, 0)`, PID `(20, 5, 2)`; derivative low-pass `tau = 0.05 s`. Retuned once in Phase 2, then frozen.
- Out of scope: payload tests, auto-tuning, RL, extra tracks.

## Review Focus

1. Sign error anywhere in the chain (sensor → error → u → wheels): a robot started +5 cm left must turn right (yaw < 0) within the first 0.5 s. Pinned by `test_integration.py::test_offset_left_turns_right` (Task 6).
2. Line lost (off the edge, or between sub-rays): error held, `line_detected = 0`, never 0.0 error. Pinned by `test_sensors.py::test_no_hit_holds_last_error` (Task 3).
3. Rays hitting the robot or floor instead of the line: only the line id counts. Pinned by `test_sim.py::test_rays_see_line_under_robot` (Task 2).
4. A run that never settles (limit cycle, quantisation): metric returns `None`, reported as "not meaningful". Pinned by `test_metrics.py::test_never_settles_returns_none` (Task 4).
5. P controller's steady curve offset pushing the line out of the array: PID and P must finish the arc with ≥ 99 % detected samples. Pinned by `test_integration.py::test_curve_keeps_line` (Task 6).

---

## Phase 0 — Foundation (serial, done by the lead session)

### Task 1: Environment, skeleton, config, schema

**Files:**
- Create: `environment.yml`, `requirements.txt` (pip, for Linux/macOS), `pyproject.toml` (pytest `pythonpath = ["."]`, `testpaths = ["tests"]`), `.gitignore`, `README.md` (setup + run commands), `common/__init__.py`, `common/config.py`, `common/schema.py`, empty `__init__.py` in `simulation/ controller/ analysis/ experiments/`
- Test: `tests/test_schema.py`

**Interfaces:**
- Produces: `common.config.SimConfig` (frozen dataclass, all values from Global Constraints), `common.config.DEFAULT: SimConfig`, `common.config.GAINS: dict[str, tuple[float, float, float]]` keyed `"P"`, `"PI"`, `"PID"`, `SimConfig.sensor_centres() -> np.ndarray` (shape (5,), left positive).
- Produces: `common.schema.COLUMNS: list[str]`, `common.schema.write_run(path, rows: list[dict]) -> None`, `common.schema.read_run(path) -> pandas.DataFrame` (raises `ValueError` on missing/extra/misordered columns).

- [x] Write `tests/test_schema.py`: `test_columns_exact_order` (COLUMNS equals the Global Constraints list), `test_roundtrip` (write 3 rows, read back equal), `test_read_rejects_bad_header` (`pytest.raises(ValueError)`), `test_sensor_centres` (`== [-0.06,-0.03,0,0.03,0.06]`, coverage > 0.05).
- [x] Run `pytest tests/test_schema.py -v` → FAIL (import error).
- [x] Implement config and schema.
- [x] Run → PASS. Check `python -c "import pybullet"` inside env `agv`.
- [x] Commit `chore: project skeleton, shared config and CSV schema` and push `main`.

---

## Phase 1 — Four modules in parallel (one subagent each, disjoint files)

Subagents do not commit; the lead reviews each module and commits it separately. Each consumes only Task 1.

### Task 2: Simulation — world, robot, track (module owner: Ishita's part)

**Files:**
- Create: `simulation/track.py`, `simulation/robot.py`, `simulation/world.py`, `simulation/assets/agv.urdf`
- Test: `tests/test_track.py` (pure), `tests/test_sim.py` (`pytest.importorskip("pybullet")`, DIRECT mode)

**Interfaces:**
- Produces `simulation.track`: `StraightTrack(length=4.0)`, `CurveTrack(lead=1.0, radius=0.75, angle=math.pi)`, both with `name: str`, `lateral_error(x, y) -> float`, `segment(x, y) -> str` (`"straight"|"arc"`), `start_pose(offset: float) -> tuple[float, float, float]`, `end_reached(x, y) -> bool`, `polyline(step: float = 0.02) -> np.ndarray` (N, 2). Line starts at origin heading +x; curve centre `(lead, radius)`; arc progress `φ = atan2(x - lead, radius - y)` in `[0, angle]`; on the arc `e = radius - |p - centre|` (inside of a left turn is left ⇒ positive).
- Produces `simulation.world.Simulation(track, cfg=DEFAULT, gui=False, start_offset=0.0)` with `.client: int`, `.line_id: int`, `.robot: AGV`, `.step() -> None`, `.close() -> None`; spawns line as one compound body (`createCollisionShapeArray` boxes along `polyline`), disables robot–line collision pairs, GUI camera follows the robot.
- Produces `simulation.robot.AGV`: `pose() -> tuple[float, float, float]` (axle midpoint x, y, yaw), `set_command(v: float, omega: float) -> None` (unicycle → wheel speeds `(v ∓ ω·W/2)/r`, `VELOCITY_CONTROL`, force `wheel_force`), `reset(pose) -> None`. URDF: base frame at axle midpoint, +x forward, 2 continuous wheel joints at y = ±W/2, front and rear spherical casters with zero lateral friction.

- [x] Tests for track: `test_straight_sign` (`lateral_error(1.0, 0.05) == 0.05`, `(1.0, -0.05) == -0.05`), `test_curve_sign_inside_positive` (point 5 cm inside the arc → +0.05), `test_curve_segments` (`segment(0.5,0) == "straight"`, `segment(1.75,0.75) == "arc"`), `test_curve_end` (`end_reached(1.0, 1.5)` true near φ = π), `test_start_pose_offset` (`start_pose(0.05) == (0, 0.05, 0)`), `test_polyline_on_line` (all polyline points have |lateral_error| < 1e-6).
- [x] Tests for sim: `test_spawn_pose` (pose ≈ `(0, 0.05, 0)` after 10 steps), `test_drives_straight` (`set_command(0.3, 0)` for 2 s → x ≈ 0.6 ± 0.06, |yaw| < 0.05), `test_turns_left` (`omega = +1` → yaw increases), `test_rays_see_line_under_robot` (a `rayTestBatch` straight down from z 0.012 at the robot's sensor point returns `line_id`, and at 0.2 m beside the line returns not `line_id`), `test_no_bump` (driving over the line keeps |roll|, |pitch| < 0.02).
- [x] Run → FAIL; implement; run → PASS.

### Task 3: Sensors and PID controller (module owner: Harsh's part)

**Files:**
- Create: `controller/sensors.py`, `controller/pid.py`
- Test: `tests/test_sensors.py`, `tests/test_pid.py`

**Interfaces:**
- Produces `controller.sensors`: `subray_offsets(cfg) -> tuple[np.ndarray, np.ndarray]` (lateral offsets of 50 sub-rays, sensor index of each); `ray_endpoints(pose, cfg) -> tuple[np.ndarray, np.ndarray]` (world starts/ends at forward distance `d`); `readings_from_hits(hit_ids, line_id, cfg) -> np.ndarray` (5 values in [0, 1] = fraction of each sensor's sub-rays hitting `line_id`); `error_from_readings(readings, cfg) -> float | None` (`-Σcᵢrᵢ/Σrᵢ`, `None` if Σrᵢ = 0); `LineSensor(client, line_id, cfg).read(pose) -> SensorReading` (`error: float | None`, `readings: np.ndarray`) using one `rayTestBatch`.
- Produces `controller.pid`: `PID(kp, ki, kd, dt, limit, d_tau)` with `reset(e0: float) -> None` (integral 0, prev error = e0, filtered derivative 0) and `update(e: float) -> float` (saturated to ±limit; conditional-integration anti-windup); `LineFollowController(mode: str, cfg=DEFAULT)` with `reset(e0) -> None` and `step(e_sensed: float | None) -> ControlOutput` (`u: float` = `-PID(e)`, `e_used: float`, `line_detected: bool`; holds last valid error when `None`); `make_controller(mode, cfg) -> LineFollowController` raising `ValueError` on unknown mode.

- [x] Tests for sensors: `test_line_centred_zero_error` (only centre sensor full → 0.0), `test_line_to_right_means_left_error` (line under the −0.03 sensor → +0.03), `test_partial_interpolates` (half on −0.03, half on 0 → +0.015), `test_no_hit_returns_none`, `test_endpoints_ahead_by_d` (pose (0,0,0) → all ray x == 0.10, z from 0.012 to −0.005), `test_endpoints_rotate` (yaw π/2 → ray y == 0.10), `test_linesensor_with_box` (own DIRECT scene: plane + one line box at y = 0 → `read((−0.10, 0.03, 0)).error ≈ +0.03 ± 0.003`).
- [x] Tests for PID: `test_p_only` (kp 2, e 0.1 → 0.2), `test_integral_accumulates` (ki 1, dt 0.1, e 1 for 10 steps → 1.0), `test_no_derivative_kick` (`reset(0.05)`, first `update(0.05)` → derivative term 0), `test_saturation` (|out| ≤ limit), `test_antiwindup` (after 100 saturated steps the output leaves saturation within 5 steps of the error reversing), `test_controller_sign` (e +0.05 → u < 0), `test_no_hit_holds_last_error` (step(0.02) then step(None) → `e_used == 0.02`, `line_detected is False`), `test_unknown_mode`.
- [x] Run → FAIL; implement; run → PASS.

### Task 4: Metrics, dummy data and plots (module owner: Prachi's part)

**Files:**
- Create: `analysis/metrics.py`, `analysis/dummy_data.py`, `analysis/plot_results.py`
- Test: `tests/test_metrics.py`

**Interfaces:**
- Produces `analysis.metrics`: `step_metrics(t, e, e0, band) -> dict` with keys `rise_time`, `overshoot_pct`, `settling_time`, `iae` (float or `None`); `curve_metrics(t, e, segment, tail_fraction=0.4) -> dict` with keys `mean_abs`, `max_abs`, `steady_state`, `iae` computed on `segment == "arc"` rows, steady state = mean e over the last `tail_fraction` of arc time; `summarize(run_paths: dict[str, Path], kind: str) -> pandas.DataFrame` (one row per controller); `to_markdown(df) -> str` writing `None` as "not meaningful".
- Produces `analysis.dummy_data.make_dummy_run(mode, t_end=10.0, dt=1/240, kind="straight") -> pandas.DataFrame` in schema order (damped sinusoids, different damping per mode, PI/PID curve tails → 0, P tail → constant offset).
- Produces `analysis.plot_results`: `plot_error_vs_time(runs: dict[str, DataFrame], title, path, band=None) -> Path`, `plot_paths(runs, track, path) -> Path`; CLI `python -m analysis.plot_results [--dummy]` writes `results/figures/*.png` and `results/summary_*.md`.

- [x] Tests: `test_metrics_known_response` (e = 0.05·e^(−t)·cos(2t) sampled at 1 kHz: overshoot ≈ 100·e^(−π/2)·… computed numerically from the sample, rise time from analytic crossings ± 2 ms, IAE vs `scipy.integrate.quad` ± 1 %), `test_no_overshoot_is_zero` (pure exponential → 0.0), `test_never_settles_returns_none` (sustained 0.01 oscillation → `settling_time is None`), `test_never_reaches_10pct_rise_none`, `test_curve_steady_state` (arc rows with constant 0.02 tail → steady_state ≈ 0.02, straight rows ignored), `test_markdown_not_meaningful`.
- [x] Run → FAIL; implement; run `python -m analysis.plot_results --dummy` → figures written; tests PASS.

### Task 5: Linearised model and Bode analysis (module owner: Nitya's part)

**Files:**
- Create: `analysis/lateral_model.py`, `analysis/bode_analysis.py`, `docs/MODEL.md` (derivation + assumptions)
- Test: `tests/test_model.py`

**Interfaces:**
- Model (state it in `docs/MODEL.md`): `ė = v·θ`, `θ̇ = ω`, sensed `e_s = e + d·θ` ⇒ `E_s(s)/Ω(s) = (v + d·s)/s²`; controller `ω = −C(s)·e_s`, `C(s) = Kp + Ki/s + Kd·s`; optional actuator lag `1/(τs + 1)`. Assumptions: small heading error, straight line, unicycle with commanded turn rate, no slip, no saturation, continuous-time, sensor noise and quantisation ignored.
- Produces `analysis.lateral_model`: `loop_tf(kp, ki, kd, v, d, tau=0.0) -> tuple[np.ndarray, np.ndarray]` (num, den of L(s)); `closed_loop_poles(kp, ki, kd, v, d, tau=0.0) -> np.ndarray`; `is_stable(...) -> bool`; `simulate_offset_response(kp, ki, kd, v, d, e0, t, omega_max=None) -> np.ndarray` (`solve_ivp` on states `[e, θ, ∫e_s]`, D term resolved algebraically: `ω = −(Kp·e_s + Ki·z + Kd·v·θ)/(1 + Kd·d)`).
- Produces `analysis.bode_analysis`: `margins(num, den) -> dict` (`gain_margin_db`, `phase_margin_deg`, `w_gc`, `w_pc`; `math.inf` when no crossing); `plot_bode(gains, cfg, path) -> Path` (P, PI, PID on one magnitude + phase figure, margins in legend); `plot_model_vs_sim(sim_runs: dict[str, DataFrame], gains, cfg, path) -> Path`; CLI `python -m analysis.bode_analysis` writes `results/figures/bode.png` and `results/model_margins.md`.

- [x] Tests: `test_p_poles_match_formula` (poles of `s² + Kp·d·s + Kp·v` for Kp 20, v 0.3, d 0.1), `test_p_zero_lookahead_marginal` (d = 0 → poles on imaginary axis, `is_stable` False), `test_pi_zero_lookahead_unstable` (d = 0, Ki > 0 → unstable), `test_pid_default_stable` (starting gains, d 0.1 → stable), `test_margins_known_tf` (L = 1/(s(s+1)) → phase margin ≈ 51.8°, gain margin inf), `test_offset_response_decays` (P, d 0.1 → |e(10 s)| < 0.005, e(0) == 0.05), `test_offset_response_matches_poles` (dominant decay rate from fit within 10 % of −Re(pole)).
- [x] Run → FAIL; implement; run → PASS; `python -m analysis.bode_analysis` writes the figure.

---

## Phase 2 — Integration and tuning (lead session)

### Task 6: Experiment runner, GUI demo, gain tuning

**Files:**
- Create: `experiments/run_experiments.py`, `experiments/demo.py`
- Modify: `common/config.py` (frozen final gains, with a comment citing the model ζ / sim evidence)
- Test: `tests/test_integration.py` (`importorskip("pybullet")`)

**Interfaces:**
- Consumes: Tasks 1–5 as declared.
- Produces: `run_trial(track, mode, cfg=DEFAULT, start_offset=0.0, max_time=12.0, gui=False) -> pandas.DataFrame` (one schema row per step, `controller = mode`); CLI `python -m experiments.run_experiments --experiment {straight,curve,lookahead,all}` writing `results/runs/{experiment}_{mode}.csv`; `python -m experiments.demo --track {straight,curve} --controller {P,PI,PID}` running real-time in the GUI for the video.

- [x] Tests: `test_offset_left_turns_right` (straight, P, offset 0.05 → yaw < 0 at t = 0.5 s), `test_p_reduces_error` (|e_true| < 0.01 by t = 8 s), `test_csv_schema` (output columns == COLUMNS), `test_curve_keeps_line` (P and PID on curve → `line_detected` mean ≥ 0.99, run ends by `end_reached`).
- [x] Run → FAIL; implement; run → PASS.
- [x] Tune once: run all three on straight + curve; adjust gains only if a controller is unstable, loses the line, or the comparison is degenerate (e.g. PI identical to P). Record the rationale in `config.py`. Freeze.
- [x] Commit `feat: experiment runner, GUI demo and frozen gains`.

---

## Phase 3 — Results (parallel: runs in background while docs are written)

### Task 7: Experiments, figures, model-vs-sim, results write-up

**Files:**
- Create: `results/runs/*.csv`, `results/figures/*.png`, `results/summary_straight.md`, `results/summary_curve.md`, `results/model_margins.md`, `docs/RESULTS.md`
- Modify: `README.md` (how to reproduce, figure gallery, repo map)

- [x] Run `python -m experiments.run_experiments --experiment all` (straight + curve, all three controllers).
- [x] Run `python -m analysis.plot_results` and `python -m analysis.bode_analysis` → figures: error vs time (straight), error vs time (curve), path overlay, Bode, model vs sim.
- [x] Write `docs/RESULTS.md`: tables, which controller wins under which test, where model and simulation agree/disagree and why (saturation, wheel dynamics, quantisation, discrete time). No claim beyond the numbers.
- [ ] NOT DONE (optional, left out at closeout to freeze the project): Optional (only if Tasks 1–7 pass): `--experiment lookahead` for `d ∈ {0.05, 0.10, 0.15}` with P, and the matching model curves; add a section to `docs/RESULTS.md`.
- [x] Commit `results: experiment data, figures and analysis`.

---

## Phase 4 — Verification and hand-off

### Task 8: Clean-run check and final review

- [x] Fresh clone into a temp folder → create env from `environment.yml` → `pytest -q` all pass → reproduce commands from README produce the same summary tables.
- [x] One reviewer subagent checks the whole branch against `docs/PROJECT_SPEC.md` and the Review Focus list.
- [x] Fix findings, commit, push `main`.

---

## Closeout (16 Nov 2026) — project frozen

All required tasks are done. Evidence is in `docs/VERIFICATION.md`.

| Phase | Commit(s) | Evidence |
|---|---|---|
| 0 Foundation | `487c7c5` | schema tests; PyBullet 3.25 from conda-forge; OpenBLAS pinned (MKL crashed NumPy on Windows) |
| 1 Modules | `0bacfb5` sensors/PID, `690eda2` metrics/plots, `aab6b5a` simulation, `cbd6599` model/Bode | module tests pass; model reproduces the DESIGN.md stability table |
| 2 Integration and tuning | `b7d1992` | all 6 runs complete; gains met the retune criteria unchanged and were frozen (rationale in `common/config.py`); line run-out added past the finish |
| 3 Results | `e6578a5` | 6 deterministic runs, figures, `docs/RESULTS.md` |
| 4 Verification | `4bce3cb`, closeout commit | fresh-clone rebuild reproduced every result; independent review fixed; closeout corrections (below) |

Closeout corrections, from the external review:

1. **Filtered PID made consistent.** The Bode plot, the margins table and the model-vs-simulation
   figure use the implemented C(s) = Kp + Ki/s + Kd s/(τf s + 1). The ideal-D PID is kept only as
   a labelled theory reference.
2. **Curve `steady_state` defined precisely.** It is the signed mean of e_true over the final 40 % of
   the arc: a tail-window measurement, not a converged equilibrium. The data and definition are
   unchanged.
3. **End-to-end tests cover all 6 controller/track runs.** Tests: 75 → 107, none skipped.
4. **Reproducibility recorded.** `environment.lock.yml` holds the exact tested package versions,
   the README has Windows commands, and `docs/VERIFICATION.md` records the evidence.

Not done (by decision):

- The optional look-ahead sweep (Task 7).

Known limitations, documented in `docs/RESULTS.md`:

- **Sensor resolution.** The sensor reads in 3 mm steps with a ±1.5 mm dead band. This cannot be
  fixed without changing the frozen sensor spec.
- **Short arc.** The PI/PID integrator does not finish converging within the 7.9 s arc.

Test-first note: the plan's "run → FAIL" steps are not separately visible in git history, because
each module's tests and implementation were committed together.
