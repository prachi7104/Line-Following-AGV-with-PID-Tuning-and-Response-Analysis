# Member 1 — Robot Design and Simulation Environment Contribution Record

- **Member Name:** Harsh Kumar
- **GitHub Username:** harsh6512
- **Assigned Module:** Member 1 — Robot design and simulation environment
- **Branch:** `module/robot-simulation`
- **Group:** Project P03 · Group G05 · Batch B3 (CSAI4014)

---

## 1. Module Overview & Technical Architecture

### 1.1 Physical Robot Configuration & Kinematics
The AGV is modelled as a differential-drive mobile robot defined in `simulation/assets/agv.urdf` and wrapped by `simulation.robot.AGV`:
- **Chassis & Mass Distribution:** The rectangular chassis ($0.22 \times 0.14 \times 0.04\text{ m}$) has total base mass $1.5\text{ kg}$. The centre of mass is placed $0.015\text{ m}$ forward of the wheel axle midpoint. This forms a stable tripod load path on the two driven wheels and the front passive caster, ensuring approximately $80\%$ of vehicle weight rests on the traction wheels to prevent wheel slip.
- **Support Points & Casters:** Two spherical casters ($r = 0.0125\text{ m}$) are located at $x = \pm 0.07\text{ m}$. The front caster rests directly on the floor (centre $0.0225\text{ m}$ below the axle), while the rear caster is offset $0.5\text{ mm}$ higher as an anti-tip guard (engaging only at $\approx 0.007\text{ rad}$ pitch). Caster friction is set to zero (lateral, spinning, rolling) to avoid parasitic yaw drag.
- **Reference Frames:** The URDF `base_link` frame is anchored at the midpoint of the driven wheel axle ($+x$ forward, $+y$ left, $+z$ upward). PyBullet internally reports pose and velocity relative to the body's inertial centre-of-mass frame. `AGV.pose()` and `AGV.reset()` explicitly apply coordinate transforms (`invertTransform` and `multiplyTransforms`) between the COM frame and the axle midpoint so that all tracking error and metric measurements remain referenced to the physical axle center.
- **Unicycle Kinematics & Wheel Speeds:** For forward velocity $v$ and commanded yaw rate $u$ (positive turning left):
  $$\omega_R = \frac{v + u \cdot W / 2}{r}, \quad \omega_L = \frac{v - u \cdot W / 2}{r}$$
  With track width $W = 0.16\text{ m}$ and wheel radius $r = 0.035\text{ m}$:
  - At cruise speed $v = 0.30\text{ m/s}$ and $u = 0$:
    $$\omega_R = \omega_L = \frac{0.30}{0.035} \approx 8.5714\text{ rad/s}$$
  - Commands are applied via `setJointMotorControlArray` in `VELOCITY_CONTROL` with a torque cap of $5.0\text{ N}\cdot\text{m}$.

### 1.2 Simulation Environment & Track Implementation
- **World & Physics Stepping (`simulation.world.Simulation`):** Configured with fixed time-step $\Delta t = 1/240\text{ s}$ ($240\text{ Hz}$) and standard gravity $g = -9.81\text{ m/s}^2$. Supports both headless batch execution (`DIRECT` mode) and visual demonstration (`GUI` mode with follow-camera tracking).
- **Line Collision Suppression:** The floor line is spawned as a static multibody from track polyline segments. To prevent the robot chassis and wheels from mechanically colliding with or bumping into the raised line ($1.5\text{ mm}$ height), `setCollisionFilterPair` explicitly disables collision between every robot link and every line segment. Ray casting (`rayTestBatch`) continues to intersect and detect the line geometry unobstructed.
- **Track Geometry:**
  - `StraightTrack`: Straight $4.0\text{ m}$ run along $+x$.
  - `CurveTrack`: $1.0\text{ m}$ straight lead-in followed by a $180^\circ$ ($\pi\text{ rad}$) semicircle arc of radius $R = 0.75\text{ m}$ turning left.

---

## 2. Technical Contribution

### Author Contribution Scope
- **Newly Authored Work:**
  1. `tests/test_robot_module.py`: Automated verification suite implementing:
     - `test_pure_yaw_commands`: Tests pure rotational motion ($v = 0, u = \pm 1.5\text{ rad/s}$), validating opposite wheel angular velocities ($\omega_L = -\omega_R$) and rotational displacement without linear translation.
     - `test_simultaneous_direct_worlds_isolation`: Validates that independent PyBullet DIRECT clients operate without cross-talk and that closing one client leaves the other fully operational.
     - `test_geometry_mismatch_rejected`: Validates that `AGV` initialization verifies URDF physical dimensions against `SimConfig` and raises `ValueError` on discrepancy.
  2. `docs/team/member-1-robot-simulation.md`: Comprehensive technical documentation and verification record for the robot simulation module.

---

## 3. Environment & Reproduction Details

- **Operating System:** Linux x86_64 (Kernel 7.0.0-38-generic)
- **Display Server:** X11 (`DISPLAY=:0`, AMD Radeon Graphics OpenGL 4.6 Core Profile)
- **Python Version:** 3.11.15 (CPython via `uv`)
- **Key Dependencies:**
  - `pybullet==3.2.7`
  - `numpy==2.4.6`
  - `scipy==1.17.1`
  - `pandas==3.1.1`
  - `matplotlib==3.11.2`
  - `pytest==9.2.0`

### Setup Command
```bash
uv venv .venv --python 3.11
source .venv/bin/activate
uv pip install -r requirements.txt
```

---

## 4. Test Verification & Results

### 4.1 Verification Commands and Suite Execution
```bash
python -m pytest -q -rs
```
**Result:** `107 passed in 17.69s` (0 skipped, 0 failed).

### 4.2 Focused Module Tests
```bash
python -m pytest -q -rs tests/test_sim.py tests/test_track.py tests/test_robot_module.py
```
**Result:** `24 passed in 0.47s` (0 skipped, 0 failed).

### 4.3 Full Test Suite (Including New Tests)
```bash
python -m pytest -q -rs
```
**Result:** `112 passed in 10.97s` (0 skipped, 0 failed).

### 4.4 GUI Smoke Tests
Executed in real-time visual mode on active X11 display:
1. **Straight Track with P-Controller:**
   ```bash
   python -m experiments.demo --track straight --controller P
   ```
   **Output:**
   - `rise_time`: 0.6000 s
   - `overshoot_pct`: 24.6636 %
   - `settling_time`: 3.2833 s
   - `iae`: 0.0542 m·s
2. **Curve Track with PID-Controller:**
   ```bash
   python -m experiments.demo --track curve --controller PID
   ```
   **Output:**
   - `mean_abs`: 0.0037 m
   - `max_abs`: 0.0125 m
   - `steady_state`: 0.0020 m
   - `iae`: 0.0291 m·s

---

## 5. Scope & Project Constraints Compliance

- **No Frozen Parameter Modifications:** `SimConfig` values in `common/config.py` remained untouched ($v = 0.30\text{ m/s}, W = 0.16\text{ m}, r = 0.035\text{ m}, \Delta t = 1/240\text{ s}, d = 0.10\text{ m}, \tau_f = 0.05\text{ s}$).
- **No Controller or Gains Altered:** P $(20, 0, 0)$, PI $(20, 5, 0)$, PID $(20, 5, 2)$ unchanged.
- **Published Results Intact:** All CSV runs in `results/runs/` and figures in `results/figures/` remain byte-exact.
- **Clean Git Boundary:** Only new verification code (`tests/test_robot_module.py`) and documentation (`docs/team/member-1-robot-simulation.md`) are contributed.

---

## 6. Proposed Root README Contribution Row

```markdown
| Harsh Kumar | Robot design and simulation environment | Added tests/test_robot_module.py (pure yaw dynamics, multi-world client isolation, URDF geometry consistency checks) and technical report docs/team/member-1-robot-simulation.md | 24 focused / 112 full tests passed (0 skipped); GUI smoke tests verified on straight (P) and curve (PID) |
```