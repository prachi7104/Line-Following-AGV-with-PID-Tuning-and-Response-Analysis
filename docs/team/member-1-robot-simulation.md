# Member 1 — Robot Design and Simulation Environment Contribution Record

- **Member Name:** Harsh Kumar
- **GitHub Username:** harsh6512
- **Assigned Module:** Member 1 — Robot design and simulation environment
- **Branch:** `module/robot-simulation`, updated from latest `main`
- **Group:** Project P03 · Group G05 · Batch B3 (CSAI4014)

---

## 1. Scope and Authorship

### 1.1 Inherited Baseline Scope vs Assigned Module Ownership
The foundational simulation codebase (`simulation/assets/agv.urdf`, `simulation/robot.py`, `simulation/world.py`, `simulation/track.py`), baseline test suites (`tests/test_sim.py`, `tests/test_track.py`), configuration (`common/config.py`), controller, analysis, experiments, and published results under `results/` are inherited from the verified baseline published on `main`. They constitute assigned module ownership for thorough code review, mathematical verification, and technical explanation, **not** individual newly authored code. None of these baseline files were modified.

### 1.2 Newly Authored Contribution Scope
This contribution strictly authorises and adds new verification diagnostics, tests, and documentation:

| File | Status | Scope & Purpose |
|---|---|---|
| `tests/test_robot_module.py` | New | 5 automated tests verifying pure yaw dynamics ($v=0, u=\pm 1.5\text{ rad/s}$), simultaneous dual-world PyBullet stepping isolation, and URDF geometry consistency checks |
| `docs/team/member-1-robot-simulation.md` | New | Complete technical contribution record, kinematics derivation, simulation parameters, verification evidence, limitations, and proposed root README row |

No changes were made to `common/config.py`, frozen robot geometry ($W=0.16\text{ m}, r=0.035\text{ m}, d=0.10\text{ m}$), controller gains ($P=(20,0,0), PI=(20,5,0), PID=(20,5,2)$), simulation time-step ($\Delta t = 1/240\text{ s}$), or any published result files under `results/`.

---

## 2. Module Overview & Technical Architecture

### 2.1 Physical Robot Configuration & Kinematics
The AGV is modelled as a differential-drive mobile robot defined in `simulation/assets/agv.urdf` and wrapped by `simulation.robot.AGV`:
- **Chassis & Mass Distribution:** Rectangular chassis ($0.22 \times 0.14 \times 0.04\text{ m}$) with a `base_link` mass of $1.5\text{ kg}$. The total URDF robot mass is $1.74\text{ kg}$, including both wheels and both casters. The base-link centre of mass is offset $0.015\text{ m}$ forward of the wheel axle midpoint. This establishes a stable tripod load path across the two driven wheels and the front passive caster, ensuring $\approx 80\%$ of vehicle weight rests on the traction wheels to prevent slip.
- **Support Points & Casters:** Two low-friction spherical casters ($r = 0.0125\text{ m}$) are placed at $x = \pm 0.07\text{ m}$ with zero friction. The front caster rests on the floor (centre $0.0225\text{ m}$ below the axle), while the rear caster is offset $0.5\text{ mm}$ higher as an anti-tip guard (engaging only at $\approx 0.007\text{ rad}$ nose-up pitch).
- **Coordinate Frames:** The URDF `base_link` frame is anchored at the midpoint of the driven wheel axle ($+x$ forward, $+y$ left, $+z$ upward). PyBullet tracks the body's inertial centre-of-mass frame. `AGV.pose()` and `AGV.reset()` apply `invertTransform` and `multiplyTransforms` between the COM and axle midpoint so all lateral tracking errors and metrics remain referenced to the wheel axle center.
- **Unicycle Kinematics & Motor Actuation:** For forward velocity $v$ and commanded yaw rate $u$ (positive turning left):
  $$\omega_R = \frac{v + u \cdot W / 2}{r}, \quad \omega_L = \frac{v - u \cdot W / 2}{r}$$
  With track width $W = 0.16\text{ m}$ and wheel radius $r = 0.035\text{ m}$:
  - At straight cruise ($v = 0.30\text{ m/s}, u = 0$):
    $$\omega_R = \omega_L = \frac{0.30}{0.035} \approx 8.5714\text{ rad/s}$$
  - Commands are applied via `setJointMotorControlArray` in `VELOCITY_CONTROL` with a torque cap of $5.0\text{ N}\cdot\text{m}$.

### 2.2 Simulation Environment & Track Implementation
- **Physics World (`simulation.world.Simulation`):** Configured with fixed time-step $\Delta t = 1/240\text{ s}$ ($240\text{ Hz}$) and standard gravity $g = -9.81\text{ m/s}^2$. Supports both headless batch execution (`DIRECT` mode) and visual demonstration (`GUI` mode with follow-camera tracking).
- **Line Collision Suppression:** The floor line is spawned as a static multibody from track polyline segments. To prevent mechanical contact between the robot and the raised tape ($1.5\text{ mm}$ height), `setCollisionFilterPair` explicitly disables collisions between every robot link and every line segment. Reflectance ray casting (`rayTestBatch`) continues to intersect and detect line geometry unobstructed.
- **Track Geometry:**
  - `StraightTrack`: Straight $4.0\text{ m}$ line along $+x$.
  - `CurveTrack`: $1.0\text{ m}$ straight lead-in followed by a $180^\circ$ ($\pi\text{ rad}$) semicircle arc of radius $R = 0.75\text{ m}$ turning left.

---

## 3. Environment & Dependency Specifications

### 3.1 Reference Environment vs Runtime Linux Environment
- **Reference Locked Environment (`environment.lock.yml`):**
  - Python: `3.11.18`
  - PyBullet: `3.25` (conda-forge, reports itself as `3.2.5`)
  - NumPy: `2.4.6` (OpenBLAS `0.3.34`)
  - SciPy: `1.17.1`
  - Pandas: `3.1.0`
  - Matplotlib: `3.11.2`
  - PyTest: `9.1.1`
- **Tested Linux Runtime Environment (`requirements.txt` via virtual environment):**
  - OS: Linux x86_64 (Kernel 7.0.0-38-generic)
  - Display Server: X11 (`DISPLAY=:0`, AMD Radeon Graphics OpenGL 4.6 Core Profile)
  - Python: `3.11.15` (CPython via `uv`)
  - PyBullet: `3.2.7` (PyPI release for Linux)
  - NumPy: `2.4.6`
  - SciPy: `1.17.1`
  - Pandas: `3.1.1`
  - Matplotlib: `3.11.2`
  - PyTest: `9.2.0`

---

## 4. Test Verification & Results

### 4.1 Automated Verification Suite (`tests/test_robot_module.py`)
Implemented 5 distinct automated tests:
1. `test_pure_yaw_commands` (parametrized for CCW $+1.5\text{ rad/s}$ and CW $-1.5\text{ rad/s}$):
   - Pure rotation with $v = 0$.
   - Verifies opposite wheel angular velocities ($\omega_L = -\omega_R$), rotation $> 0.4\text{ rad}$ in the commanded sign direction, and minimal linear translation of the axle midpoint ($< 0.02\text{ m}$).
2. `test_simultaneous_direct_worlds_isolation`:
   - Connects two simultaneous PyBullet DIRECT clients (`sim1` and `sim2`).
   - Commands `sim1` forward ($v=0.30$) while commanding `sim2` at rest ($v=0.0$).
   - Concurrently steps both simulations in the same loop over $0.8\text{ s}$ of physics time.
   - Verifies `sim1` advances ($x_1 > 0.20\text{ m}$) while `sim2` remains undisturbed at origin ($|x_2| < 0.005\text{ m}, |y_2| < 0.005\text{ m}$).
   - Closes `sim1` cleanly and confirms `sim2` continues stepping and advances forward ($x_2 > 0.20\text{ m}$).
3. `test_geometry_mismatch_rejected` (parametrized for mismatched wheel radius and track width):
   - Verifies that `AGV` initialization fails loudly with `ValueError` on mismatched wheel dimensions, cleanly disconnecting PyBullet clients in a `finally` block.

```bash
python -m pytest -q -rs tests/test_robot_module.py
# Harsh's Linux run: 5 passed in 0.57s
```

### 4.2 Focused Simulation Tests
```bash
python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_robot_module.py
# Harsh's Linux run: 24 passed in 1.10s (7 test_sim + 12 test_track + 5 test_robot_module, 0 skipped)
```

### 4.3 Baseline and Full Suite Verification
- **Baseline Test Suite (Inherited 8 modules):**
  ```bash
  python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_integration.py tests/test_metrics.py tests/test_model.py tests/test_pid.py tests/test_schema.py tests/test_sensors.py
  # Harsh's Linux run: 107 passed in 15.84s (0 skipped, 0 failed)
  ```
- **Baseline + Member 1 Module:**
  ```bash
  python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_integration.py tests/test_metrics.py tests/test_model.py tests/test_pid.py tests/test_schema.py tests/test_sensors.py tests/test_robot_module.py
  # Harsh's Linux run: 112 passed in 16.50s (0 skipped, 0 failed)
  ```
- **Harsh’s integrated Linux run:** 185 tests collected, 179 passed and 6 exact CSV comparisons failed. The differences were in the final decimal digits between the Linux PyPI PyBullet 3.2.7 results and the published Windows conda-forge PyBullet 3.25 results. See [README limitations](../../README.md#limitations) and the [Member 2 record](member-2-sensors-pid.md).
- **Integrated Windows reference run:** PR head `c91b642` on base `4d95505`. The headless runs passed all 5 Member 1 tests, all 24 focused simulation tests and all 185 tests in the full suite, with zero skipped and zero failed. All 6 published CSV reproduction cases also passed in a targeted run.

### 4.4 GUI Smoke Tests
Harsh ran these GUI smoke tests in real time on an active Linux X11 display.
1. **Straight Track with P-Controller:**
   ```bash
   python -m experiments.demo --track straight --controller P
   ```
   - `rise_time`: `0.6000 s`
   - `overshoot_pct`: `24.6636 %` (agrees closely with linear model prediction of $\approx 25\%$ from $\zeta \approx 0.41$)
   - `settling_time`: `3.2833 s` (within $\pm 2.5\text{ mm}$ settling band)
   - `iae`: `0.0542 m·s`
2. **Curve Track with PID-Controller:**
   ```bash
   python -m experiments.demo --track curve --controller PID
   ```
   - `mean_abs`: `0.0037 m` ($3.7\text{ mm}$)
   - `max_abs`: `0.0125 m` ($12.5\text{ mm}$)
   - `steady_state`: `0.0020 m` ($2.0\text{ mm}$). Integral action reduced the measured curve offset compared with the P controller’s $13.8\text{ mm}$.
   - `iae`: `0.0291 m·s`

---

## 5. Limitations

- **Platform-Dependent PyBullet Builds:** Minor floating-point divergence in the final decimal places occurs between Linux PyPI PyBullet (`3.2.7`) and Windows conda-forge PyBullet (`3.25`, reported as `3.2.5` by Python). This does not affect simulation stability, qualitative response, or control performance.
- **Rigid-Body Physics vs Physical Hardware:** The URDF simulation assumes ideal rigid bodies and tire-ground contact without flexible tire deformation, non-linear motor saturation below $5.0\text{ N}\cdot\text{m}$, or battery voltage droop.
- **Sensor Ray Discretisation:** Sensed error resolution is limited to $3\text{ mm}$ increments due to the 10 sub-rays per sensor patch, introducing a small $\approx 1.5\text{ mm}$ dead-band around the line.

---

## 6. README contribution record

See the [merged module contributions](../../README.md#merged-module-contributions) for the integration summary.
