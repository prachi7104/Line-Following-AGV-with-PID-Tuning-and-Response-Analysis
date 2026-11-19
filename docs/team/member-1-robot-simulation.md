# Member 1 — Robot Design and Simulation Environment

- **Member:** Harsh Kumar
- **GitHub:** harsh6512
- **Module:** Member 1 — Robot design and simulation environment
- **Branch:** `module/robot-simulation`
- **Pull request:** [#1](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/1), head `c91b642`, merged as `faf1dc4`
- **Group:** Project P03 · Group G05 · Batch B3 (CSAI4014)

## 1. Scope and authorship

The simulation code (`simulation/assets/agv.urdf`, `simulation/robot.py`, `simulation/world.py`,
`simulation/track.py`), its baseline tests (`tests/test_sim.py`, `tests/test_track.py`), the
configuration (`common/config.py`), the controller, analysis, experiments and the published
results under `results/` come from the baseline on `main`. This module owns their review,
verification and technical explanation. They are **not** newly authored code, and none of them was
modified.

Files added by this contribution:

| File | Status | Purpose |
|---|---|---|
| `tests/test_robot_module.py` | New | 5 tests: pure yaw dynamics ($v=0, u=\pm 1.5\text{ rad/s}$), isolation of two PyBullet worlds stepped at the same time, and rejection of mismatched URDF geometry |
| `docs/team/member-1-robot-simulation.md` | New | This record |

No changes were made to `common/config.py`, the frozen robot geometry
($W=0.16\text{ m}, r=0.035\text{ m}, d=0.10\text{ m}$), the controller gains
($P=(20,0,0), PI=(20,5,0), PID=(20,5,2)$), the simulation time step ($\Delta t = 1/240\text{ s}$)
or any published result under `results/`.

## 2. Technical overview

### 2.1 Robot configuration and kinematics

The AGV is a differential-drive robot defined in `simulation/assets/agv.urdf` and wrapped by
`simulation.robot.AGV`:

- **Chassis and mass distribution:** rectangular chassis ($0.22 \times 0.14 \times 0.04\text{ m}$)
  with a `base_link` mass of $1.5\text{ kg}$. The total URDF robot mass is $1.74\text{ kg}$,
  including both wheels and both casters. The base-link centre of mass is $0.015\text{ m}$ forward
  of the wheel axle midpoint. This gives a stable tripod load path across the two driven wheels and
  the front passive caster, with $\approx 80\%$ of the weight on the traction wheels to prevent slip.
- **Support points and casters:** two low-friction spherical casters ($r = 0.0125\text{ m}$) at
  $x = \pm 0.07\text{ m}$, with zero friction. The front caster rests on the floor (centre
  $0.0225\text{ m}$ below the axle). The rear caster sits $0.5\text{ mm}$ higher as an anti-tip
  guard and engages only at $\approx 0.007\text{ rad}$ nose-up pitch.
- **Coordinate frames:** the URDF `base_link` frame is at the midpoint of the driven wheel axle
  ($+x$ forward, $+y$ left, $+z$ up). PyBullet tracks the body's centre-of-mass frame.
  `AGV.pose()` and `AGV.reset()` apply `invertTransform` and `multiplyTransforms` between the COM
  and the axle midpoint, so all lateral errors and metrics are referenced to the axle centre.
- **Unicycle kinematics and motor actuation:** for forward speed $v$ and commanded yaw rate $u$
  (positive turns left):
  $$\omega_R = \frac{v + u \cdot W / 2}{r}, \quad \omega_L = \frac{v - u \cdot W / 2}{r}$$
  With $W = 0.16\text{ m}$ and $r = 0.035\text{ m}$, straight cruise ($v = 0.30\text{ m/s}, u = 0$)
  gives
  $$\omega_R = \omega_L = \frac{0.30}{0.035} \approx 8.5714\text{ rad/s}$$
  Commands are applied with `setJointMotorControlArray` in `VELOCITY_CONTROL` with a
  $5.0\text{ N}\cdot\text{m}$ torque cap.

### 2.2 Simulation environment and tracks

- **Physics world (`simulation.world.Simulation`):** fixed time step $\Delta t = 1/240\text{ s}$
  ($240\text{ Hz}$) and gravity $g = -9.81\text{ m/s}^2$. Runs headless (`DIRECT`, batch runs) or
  with a window (`GUI`, follow camera).
- **Line collision suppression:** the floor line is a static multibody built from the track's
  polyline segments. `setCollisionFilterPair` disables collisions between every robot link and every
  line segment, so the robot never touches the raised tape ($1.5\text{ mm}$). Ray casting
  (`rayTestBatch`) still detects the line.
- **Tracks:**
  - `StraightTrack`: a straight $4.0\text{ m}$ line along $+x$.
  - `CurveTrack`: a $1.0\text{ m}$ straight lead-in, then a $180^\circ$ ($\pi\text{ rad}$) left arc
    of radius $R = 0.75\text{ m}$.

## 3. Implementation and tests

`tests/test_robot_module.py` has 5 tests:

1. `test_pure_yaw_commands`, parametrised for CCW $+1.5\text{ rad/s}$ and CW $-1.5\text{ rad/s}$:
   - pure rotation with $v = 0$;
   - checks opposite wheel speeds ($\omega_L = -\omega_R$), rotation $> 0.4\text{ rad}$ in the
     commanded direction, and translation of the axle midpoint $< 0.02\text{ m}$.
2. `test_simultaneous_direct_worlds_isolation`:
   - connects two PyBullet DIRECT clients at once (`sim1` and `sim2`);
   - commands `sim1` forward ($v=0.30$) and `sim2` at rest ($v=0.0$);
   - steps both simulations in the same loop for $0.8\text{ s}$ of physics time;
   - checks that `sim1` advances ($x_1 > 0.20\text{ m}$) while `sim2` stays at the origin
     ($|x_2| < 0.005\text{ m}, |y_2| < 0.005\text{ m}$);
   - closes `sim1` and checks that `sim2` keeps stepping and advances ($x_2 > 0.20\text{ m}$).
3. `test_geometry_mismatch_rejected`, parametrised for a mismatched wheel radius and track width:
   - checks that `AGV` initialisation raises `ValueError`, disconnecting PyBullet clients in a
     `finally` block.

## 4. Verification

### 4.1 Environments

- **Reference locked environment (`environment.lock.yml`):** Python `3.11.18`; PyBullet `3.25`
  (conda-forge, reports itself as `3.2.5`); NumPy `2.4.6` (OpenBLAS `0.3.34`); SciPy `1.17.1`;
  Pandas `3.1.0`; Matplotlib `3.11.2`; PyTest `9.1.1`.
- **Harsh's Linux environment (`requirements.txt`, virtual environment):** Linux x86_64 (kernel
  7.0.0-38-generic); X11 (`DISPLAY=:0`, AMD Radeon Graphics, OpenGL 4.6 Core Profile); Python
  `3.11.15` (CPython via `uv`); PyBullet `3.2.7` (PyPI, Linux); NumPy `2.4.6`; SciPy `1.17.1`;
  Pandas `3.1.1`; Matplotlib `3.11.2`; PyTest `9.2.0`.

### 4.2 Harsh's runs (Linux)

| Check | Command | Result |
|---|---|---|
| Member 1 tests | `python -m pytest -q -rs tests/test_robot_module.py` | 5 passed in 0.57 s |
| Focused simulation tests | `python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_robot_module.py` | 24 passed in 1.10 s (7 `test_sim` + 12 `test_track` + 5 `test_robot_module`, 0 skipped) |
| Baseline suite (8 inherited modules) | `python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_integration.py tests/test_metrics.py tests/test_model.py tests/test_pid.py tests/test_schema.py tests/test_sensors.py` | 107 passed in 15.84 s (0 skipped, 0 failed) |
| Baseline + Member 1 | the same command plus `tests/test_robot_module.py` | 112 passed in 16.50 s (0 skipped, 0 failed) |
| Integrated suite | `python -m pytest -q -rs` | 185 collected: 179 passed, 6 exact CSV comparisons failed |

The six failures differ from the published CSVs in the final decimal digits, between the Linux
PyPI PyBullet 3.2.7 results and the published Windows conda-forge PyBullet 3.25 results. See
[README limitations](../../README.md#limitations) and the [Member 2 record](member-2-sensors-pid.md).

**GUI smoke tests**, run by Harsh in real time on a Linux X11 display:

1. `python -m experiments.demo --track straight --controller P`
   - `rise_time`: `0.6000 s`
   - `overshoot_pct`: `24.6636 %` (close to the linear model's $\approx 25\%$ from $\zeta \approx 0.41$)
   - `settling_time`: `3.2833 s` (±2.5 mm settling band)
   - `iae`: `0.0542 m·s`
2. `python -m experiments.demo --track curve --controller PID`
   - `mean_abs`: `0.0037 m` ($3.7\text{ mm}$)
   - `max_abs`: `0.0125 m` ($12.5\text{ mm}$)
   - `steady_state`: `0.0020 m` ($2.0\text{ mm}$). Integral action reduced the curve offset
     compared with the P controller's $13.8\text{ mm}$.
   - `iae`: `0.0291 m·s`

### 4.3 Integration verification (team lead, locked Windows environment)

PR head `c91b642` on base `4d95505`. The headless runs passed all 5 Member 1 tests, all 24 focused
simulation tests and all 185 tests in the full suite, with 0 skipped and 0 failed. All 6 published
CSV reproduction cases also passed in a targeted run. The final integrated results (225 passed,
6 of 6 runs reproduced) are in [docs/VERIFICATION.md](../VERIFICATION.md) §6.

## 5. Limitations

- **Platform-dependent PyBullet builds:** results differ in the final decimal places between Linux
  PyPI PyBullet (`3.2.7`) and Windows conda-forge PyBullet (`3.25`, reported as `3.2.5`). This does
  not affect stability, the qualitative response or control performance.
- **Rigid-body physics, not hardware:** the URDF simulation assumes ideal rigid bodies and
  tire-ground contact. It does not model tire deformation, non-linear motor saturation below
  $5.0\text{ N}\cdot\text{m}$ or battery voltage droop.
- **Sensor ray discretisation:** the sensed error moves in $3\text{ mm}$ steps (10 sub-rays per
  sensor), which leaves a dead band of about $1.5\text{ mm}$ around the line.

## 6. README entry

See [Merged module contributions](../../README.md#merged-module-contributions).
