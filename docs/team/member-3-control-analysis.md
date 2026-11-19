# Member 3 — Mathematical Modelling and Stability Analysis

- **Member:** Ishita Garg
- **GitHub:** Ishitagarg1602
- **Module:** Member 3 — Mathematical modelling and stability analysis
- **Branch:** `module/control-analysis`, based on `main` at `30831b7` (`30831b780a883ef9164016aaee03250288ae014f`)
- **Pull request:** [#5](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/5), head `4844349`, merged as `a1162ec`
- **Group:** Project P03 · Group G05 · Batch B3 (CSAI4014)

## 1. Scope and authorship

The linearised lateral model (`analysis/lateral_model.py`), frequency-domain Bode/margin script (`analysis/bode_analysis.py`), baseline model tests (`tests/test_model.py`), model documentation (`docs/MODEL.md`), and published outputs (`results/model_margins.md`, `results/figures/bode.png`, `results/figures/model_vs_sim.png`) are inherited from Prachi Agarwalla's baseline snapshot (`b158af1`, matching `3320f51`). They represent assigned module ownership for review and explanation, **not** newly authored code, and none of them were modified.

This contribution adds an independent frequency-domain cross-check CLI, its test suite, and this technical record:

| File | Status | Purpose |
|---|---|---|
| `analysis/model_check.py` | New | Read-only CLI (`python -m analysis.model_check`) independently evaluating `C(jω)·G(jω)` against `loop_tf` and checking closed-loop pole stability |
| `tests/test_model_check.py` | New | 26 tests covering independent complex evaluation, mismatch detection, CLI exit status, and stability classification |
| `docs/team/member-3-control-analysis.md` | New | This contribution, derivation, reproduction, and verification record |

No existing source files, configuration (`common/config.py`), environment files (`environment.yml`, `environment.lock.yml`), baseline tests, `README.md`, or published files under `results/` were modified.

## 2. Technical overview

### 2.1 Project architecture and module interfaces

All modules share one frozen configuration (`common/config.py`, SI units: m, s, rad, rad/s) and one sign convention:

- `e > 0` ⇔ robot axle is **left** of the line (looking along travel direction);
- `u > 0` ⇔ robot turns **left** (counter-clockwise yaw rate, rad/s);
- Control law: `u = -PID(e_sensed)`, clamped to `|u| <= 3.0 rad/s`.

Frozen baseline parameters: forward speed `v = 0.30 m/s`, physics/control step `dt = 1/240 s`, sensor look-ahead `d = 0.10 m`, derivative low-pass filter `τ_f = 0.05 s`, gains `P = (20, 0, 0)`, `PI = (20, 5, 0)`, `PID = (20, 5, 2)`.

How Member 3 connects to the other three modules:

1. **Member 1 (`simulation/`):** Provides the differential-drive AGV (`AGV`), tracks (`StraightTrack`, `CurveTrack`), and 240 Hz PyBullet world (`Simulation`). The linear model abstracts the robot kinematics into `(v, d)` and predicts the behaviour before physics effects (wheel inertia, 5 N·m torque limit, tire contact) are added.
2. **Member 2 (`controller/`):** Provides the five-sensor ray bar (`LineSensor`, 10 sub-rays per 30 mm sensor, 3 mm pitch, placed `d = 0.10 m` ahead of the axle) and discrete PID (`PID`, `LineFollowController`). Member 3 models the sensor geometry via `e_s = e + d·θ` and models `controller/pid.py`'s backward-Euler derivative filter (`α = dt/(τ_f + dt) = 1/13`) as `Kd·s/(τ_f·s + 1)`.
3. **Member 4 (`experiments/`, `analysis/metrics.py`, `analysis/plot_results.py`):** Runs the six headless trials and logs `results/runs/*.csv` using `common/schema.py`. `analysis/bode_analysis.py` reads the published straight-track CSVs (`straight_{P,PI,PID}.csv`) to overlay linear-model `e(t)` against simulated ground-truth `e_true` in `results/figures/model_vs_sim.png`.

### 2.2 Plant transfer function `G(s)`

Near a straight reference line along `+x`, let `e(t)` be the axle's signed lateral error (`e > 0` left of line) and `θ(t)` be the heading error (`θ > 0` pointing left of the line).

1. **Kinematics:** Moving forward at constant speed `v` at heading angle `θ` produces lateral velocity `de/dt = v·sin(θ)`. Under the **small-angle assumption** (`sin(θ) ≈ θ`),
   ```
   de/dt = v · θ
   ```
2. **Yaw dynamics:** Under the **instantaneous yaw-rate assumption** (ideal wheel velocity tracking, no motor lag, no wheel slip),
   ```
   dθ/dt = u
   ```
3. **Look-ahead sensor output:** The sensor bar lies distance `d` ahead of the wheel axle. For small `θ`, the lateral offset at the sensor bar is `e_s = e + d·sin(θ) ≈ e + d·θ`:
   ```
   e_s = e + d · θ
   ```
4. **Laplace transform (zero initial conditions):**
   - `s Θ(s) = U(s)` ⇒ `Θ(s) = U(s) / s`
   - `s E(s) = v Θ(s)` ⇒ `E(s) = v U(s) / s²`
   - `E_s(s) = E(s) + d Θ(s) = (v / s² + d / s) U(s)`
   ```
   G(s) = E_s(s) / U(s) = (v + d·s) / s²
   ```

The plant has a **double integrator** (`1/s²`, yaw rate → heading → lateral position) and a single **left-half-plane zero** at `s = -v/d = -3.0 rad/s` produced solely by the look-ahead distance `d`.

### 2.3 Negative feedback, loop transfer function, and filtered vs ideal D

With negative feedback `U(s) = -C(s) E_s(s)`, the open-loop transfer function is `L(s) = C(s)·G(s)` and closed-loop poles are the roots of `1 + L(s) = 0`, equivalently `den(L) + num(L) = 0`.

In `controller/pid.py`, the derivative update is:
```
d_f[k] = d_f[k-1] + α · ((e[k] - e[k-1])/dt - d_f[k-1]),   α = dt / (τ_f + dt)
```
Rearranging gives `τ_f (d_f[k] - d_f[k-1])/dt + d_f[k] = (e[k] - e[k-1])/dt`, which is the exact **backward-Euler discretisation** of the continuous-time first-order filter `τ_f df/dt + f = de/dt`, whose transfer function is `s / (τ_f·s + 1)`. Therefore:

- **Implemented controller (`controller/pid.py`):**
  `C_impl(s) = Kp + Ki/s + Kd·s / (τ_f·s + 1)` with `τ_f = 0.05 s` (for P and PI, `Kd = 0`, so the filter does not apply). In the time domain, `f` is a fourth state with `f(0) = 0`, matching `PID.reset(e0)` (no derivative kick, no algebraic loop). In closed loop at `d = 0.10 m`, it introduces one fast real pole at `-23.769` and slightly increases dominant damping (`ζ = 0.443` vs `0.431`).
- **Ideal D (`Kd·s`, theory reference only):**
  `C_ideal(s) = Kp + Ki/s + Kd·s`. Properness is lost (improper transfer function), high-frequency gain grows without bound, and in time-domain ODE integration `de_s/dt = v·θ + d·u` creates an algebraic loop solved as `u = -(Kp·e_s + Ki·z + Kd·v·θ) / (1 + Kd·d)`. It is kept strictly as a separately labelled reference.

### 2.4 Margins, conditional stability, and `d = 0` behaviour

Key properties verified from `results/model_margins.md` and `analysis/model_check.py`:

- **Default `d = 0.10 m`:** All controllers are stable:
  - **P (20, 0, 0):** poles `-1.000 ± 2.236j` (`ζ = 0.408`), phase margin `+43.9°` at `2.89 rad/s`, infinite gain margin (phase `−180° + atan(dω/v)` stays strictly above `−180°`).
  - **PI (20, 5, 0):** poles `-0.876 ± 2.302j, -0.247` (`ζ = 0.356`), phase margin `+39.0°` at `2.89 rad/s` (integrator adds phase lag), gain margin `-18.8 dB` at `0.866 rad/s`.
  - **PID filtered D (`τ_f = 0.05 s`, implemented):** poles `-23.769, -0.989 ± 2.000j, -0.254` (`ζ = 0.443`), phase margin `+56.5°` at `3.01 rad/s`, gain margin `-20.9 dB` at `0.759 rad/s`.
  - **PID ideal D (theory reference):** poles `-0.957 ± 2.004j, -0.253` (`ζ = 0.431`), phase margin `+56.0°` at `2.93 rad/s`, gain margin `-20.9 dB` at `0.76 rad/s`.
- **Why negative gain margins occur on stable PI/PID loops:** The double plant integrator plus the controller integrator start the open-loop phase at `-270°` as `ω → 0+`. As frequency rises, the controller zero and plant look-ahead zero lift the phase upward through `-180°` while `|L| > 1`. Consequently, PI and PID are **conditionally stable**: a negative gain margin (`-18.8 dB` / `-20.9 dB`) denotes a **lower stability boundary** (how far loop gain can be *reduced* before destabilising), not instability. Conversely, PI at `d = 0` reports infinite gain margin only because its phase never reaches `-180°`, even though it is unstable. Closed-loop pole locations are therefore the definitive stability check for this model.
- **Zero look-ahead (`d = 0 m`) in this linear model:**
  - **P:** characteristic equation `s² + Kp·v = 0` ⇒ poles `±2.449j` (**marginal**; phase is identically `-180°`, phase margin `0°`).
  - **PI:** characteristic equation `s³ + Kp·v·s + Ki·v = 0` has zero `s²` coefficient ⇒ Routh–Hurwitz fails, poles `+0.124 ± 2.459j, -0.247` (**unstable**).
  - **Filtered PID (`τ_f = 0.05 s`):** poles `-19.391, -0.254, -0.178 ± 2.463j` (`ζ = 0.072`, PM `+8.1°`, **stable** in the ideal linear model because `Kd·v` supplies a small positive damping term). Ideal PID is similarly weakly stable (`ζ = 0.071`, PM `+8.2°`).
  - **Scope warning:** `d = 0` PID stability holds *only* for this idealised linear model; with only `~8°` phase margin, unmodelled motor lag, sampling delay, or quantisation on nonlinear hardware or in PyBullet can easily destabilise it.

### 2.5 What the Bode plot measures and why model vs simulation differ

`results/figures/bode.png` plots the magnitude `20·log10|L(jω)|` and unwrapped phase `∠L(jω)` of the **open-loop** linearised transfer function `L(s) = C(s)G(s)` at `d = 0.10 m`. It measures frequency-domain stability margins and crossover bandwidth of the linear approximation; it is not a direct measurement from PyBullet.

Where the PyBullet simulation (`results/figures/model_vs_sim.png` and `docs/RESULTS.md`) differs from the linear model:

1. **Sensor quantisation:** 10 sub-rays across a 30 mm sensor yield a 3 mm quantisation step and a `±1.5 mm` dead band, leaving a ~1.5 mm residual steady offset that the smooth linear model does not show (accounting for ~27 % of straight-track IAE).
2. **Wheel-motor and contact dynamics:** PyBullet applies `VELOCITY_CONTROL` with a `5 N·m` torque cap and wheel-floor friction/inertia rather than instantaneous yaw rate `dθ/dt = u`.
3. **Discrete-time sampling:** The simulation runs at `240 Hz` (`dt = 1/240 s`), introducing roughly half a sample of delay and a discrete backward-Euler filter rather than exact continuous time.
4. **Nonlinearities and geometry:** Saturation (`|u| <= 3 rad/s`, anti-windup, though `max |u| ≈ 1.0 rad/s` on the nominal runs), large-angle `sin(θ)` nonlinearity, and curved-track chord geometry (`+d²/(2R) = +6.7 mm` look-ahead offset on the `R = 0.75 m` arc) are outside the straight-line linearisation.

## 3. Implementation and tests

### 3.1 `analysis/model_check.py`

A read-only diagnostic CLI (`python -m analysis.model_check`) that provides an independent frequency-domain cross-check rather than repeating known-pole lookups:

- **Independent complex evaluation:** `plant_eval`, `controller_eval`, and `loop_eval_analytic` compute `G(jω) = (v + j d ω) / (-ω²)`, `C(jω) = Kp + Ki/(jω) + Kd(jω)/(1 + jω τ_f)`, and `L(jω) = C(jω)·G(jω)` directly in complex arithmetic without calling `controller_tf`, `loop_tf`, `np.polyadd`, or `np.polymul`.
- **Cross-check against `loop_tf`:** Evaluates `np.polyval(num, jω) / np.polyval(den, jω)` from `analysis.lateral_model.loop_tf` on **13 fixed positive frequencies** (`0.1, 0.25, 0.5, 0.75, 1, 2, 2.5, 3, 5, 10, 20, 50, 100 rad/s`) spanning the integrator asymptote, phase/gain crossovers, look-ahead zero (`3 rad/s`), derivative filter corner (`20 rad/s`), and high-frequency roll-off.
- **Cases covered:** All 8 default cases—`P`, `PI`, `PID (filtered D, τ_f = 0.05 s, implemented)`, and `PID (ideal D, theory reference)`—at both `d = 0.10 m` and `d = 0 m`.
- **Justified numerical tolerances:** `DEFAULT_RTOL = 1e-12`, `DEFAULT_ATOL = 1e-12` (IEEE-754 float64 evaluation of degree ≤ 4 rational functions agrees to `~1e-16`–`1e-14` relative, leaving safe rounding headroom while immediately catching any algebraic, sign, gain, or filter discrepancy), and `STABILITY_TOL = 1e-9` matching `analysis/lateral_model.py`.
- **Stability cross-check and exit status:** Reports all closed-loop pole real parts, `max Re(p)`, and `"stable" / "marginal" / "unstable"` classification verified against `lateral_model.is_stable`. Exits `0` only if all cases pass; exits `1` on any discrepancy or stability mismatch.

### 3.2 `tests/test_model_check.py` (26 tests)

Written after reading `tests/test_model.py` so it complements rather than duplicates existing pole/time-response tests:

1. **Independent complex evaluation (11 tests):** Closed-form real/imaginary checks for `G(jω)` at `d = 0.10 m` and `d = 0 m`, filtered vs ideal D at the filter corner `ω = 1/τ_f = 20 rad/s`, optional actuator lag `τ > 0`, and input validation rejecting non-positive/non-finite frequencies and negative time constants.
2. **Default suite & separation of theory reference (2 tests):** Verifies all 8 cases (`d = 0.10 m` and `d = 0 m`), separate labelling of implemented filtered D vs theory-only ideal D, and tight complex, magnitude-dB, and phase agreement against `loop_tf`.
3. **Stability classification (7 tests):** Parametrised classification of stable, marginal, and unstable pole sets plus exact classification of all 8 default cases (including `d = 0` P marginal, PI unstable, filtered & ideal PID stable).
4. **Mismatch detection & CLI exit codes (6 tests):** Injects corrupted transfer functions (perturbed gain `1 + 1e-8`, dropped derivative filter on implemented PID, sign inversion, degenerate constant TF, zero denominator) to confirm failures are caught and `main()` returns exit code `1` on discrepancy and `0` on pass.

## 4. Verification

### 4.1 Environment

- **OS:** Windows 11 (`Windows-10-10.0.26300-SP0`)
- **Environment:** conda-forge micromamba environment `agv`, created from `environment.yml` (`environment.lock.yml` left untouched):
  - Python `3.11.18`
  - PyBullet `3.25` (conda-forge, API version `202010061`)
  - NumPy `2.4.6` (with OpenBLAS `0.3.34`)
  - SciPy `1.17.1`
  - Pandas `3.1.1`
  - Matplotlib `3.11.2`
  - pytest `9.2.0`

### 4.2 Ishita's runs: commands and recorded results

On Windows PowerShell (from the repository root, using `$py = "$HOME\micromamba\envs\agv\python.exe"`):

| Check | Command | Result |
|---|---|---|
| Independent cross-check CLI | `& $py -m analysis.model_check` | **8/8 cases passed** (exit code `0`); overall `max \|dL\| = 2.27e-13`, `max rel err = 2.65e-16` |
| New tests only | `& $py -m pytest -q -rs tests/test_model_check.py` | **26 passed, 0 skipped** (1.03 s) |
| Focused model tests | `& $py -m pytest -q -rs tests/test_model.py tests/test_model_check.py` | **40 passed, 0 skipped** (4.28 s) |
| Full test suite | `& $py -m pytest -q -rs` | **180 passed, 0 skipped** (30.35 s; 107 baseline + 47 Member 2 + 26 Member 3) |
| Disposable-copy Bode & margins reproduction | `& $py -m analysis.bode_analysis` (run inside a disposable copy outside the repo) | `results/model_margins.md` text-identical after line-ending normalisation; `results/figures/bode.png` (`141904 B`) and `results/figures/model_vs_sim.png` (`116925 B`) byte-identical (0 differing pixels) |
| Published CSV row-count check | Read-only check of `results/runs/*.csv` | Exact match to reference counts (excluding headers): straight P `3211`, PI `3212`, PID `3210`; curve P `2742`, PI `2714`, PID `2710` |

### 4.3 Actual output of `python -m analysis.model_check`

```text
Linear-model independent frequency-domain cross-check
Frequencies (13 pts) [rad/s]: 0.1, 0.25, 0.5, 0.75, 1, 2, 2.5, 3, 5, 10, 20, 50, 100
Tolerances: rtol=1.0e-12, atol=1.0e-12 (stability tol=1.0e-09)

| Case | Category | d [m] | max |dL| | max rel err | Pole Re(p) | max Re(p) | Stability | Status |
|---|---|---:|---:|---:|---|---:|---|---|
| P (implemented) | implemented | 0.1 | 4.44e-16 | 1.18e-16 | [-1.0000, -1.0000] | -1.0000 | stable | PASS |
| PI (implemented) | implemented | 0.1 | 6.66e-16 | 2.35e-16 | [-0.8764, -0.8764, -0.2473] | -0.2473 | stable | PASS |
| PID, filtered D tau_f=0.05 s (implemented) | implemented | 0.1 | 1.14e-13 | 2.39e-16 | [-23.7694, -0.9885, -0.9885, -0.2535] | -0.2535 | stable | PASS |
| PID, ideal D (theory reference) | theory reference | 0.1 | 8.88e-16 | 1.23e-16 | [-0.9566, -0.9566, -0.2535] | -0.2535 | stable | PASS |
| P (implemented) | implemented | 0 | 0.00e+00 | 0.00e+00 | [+0.0000, +0.0000] | +0.0000 | marginal | PASS |
| PI (implemented) | implemented | 0 | 1.14e-13 | 1.81e-16 | [-0.2475, +0.1237, +0.1237] | +0.1237 | unstable | PASS |
| PID, filtered D tau_f=0.05 s (implemented) | implemented | 0 | 2.27e-13 | 2.65e-16 | [-19.3907, -0.2538, -0.1777, -0.1777] | -0.1777 | stable | PASS |
| PID, ideal D (theory reference) | theory reference | 0 | 1.14e-13 | 1.46e-16 | [-0.2537, -0.1731, -0.1731] | -0.1731 | stable | PASS |

Summary: 8/8 cases passed | overall max |dL| = 2.27e-13, max rel err = 2.65e-16
```

### 4.4 Reproduction

1. Ensure you are on branch `module/control-analysis` in a clean clone.
2. Create (or reuse) the `agv` environment from `environment.yml` (`micromamba create -f environment.yml`).
3. Run `& $py -m analysis.model_check`, `& $py -m pytest -q -rs tests/test_model.py tests/test_model_check.py`, and `& $py -m pytest -q -rs`.
4. To verify `analysis.bode_analysis` without touching tracked files under `results/`, copy the repository to a separate temporary folder, run `& $py -m analysis.bode_analysis` inside that copy, and compare `results/model_margins.md`, `results/figures/bode.png`, and `results/figures/model_vs_sim.png` against the committed baseline.

### 4.5 Integration verification (team lead, locked Windows environment)

After PR #5 merged, `main` at `a1162ec` was checked in the locked reference environment
(`environment.lock.yml`: pandas `3.1.0`, pytest `9.1.1`). `python -m analysis.model_check` passed
8/8 cases, and `python -m pytest -q -rs` gave 180 passed, 0 skipped. The final integrated results
(225 passed, 6 of 6 runs reproduced) are in [docs/VERIFICATION.md](../VERIFICATION.md) §6.

## 5. Limitations

- **Verifies mathematical consistency, not physical fidelity.** `analysis/model_check.py` proves that `analysis.lateral_model.loop_tf` and `closed_loop_poles` agree with the derived continuous-time complex transfer functions `C(jω)·G(jω)`. It does not validate the PyBullet physics, motor lag, or sensor ray casting.
- **Linear `d = 0` stability does not generalise to hardware.** Filtered PID is linearly stable at `d = 0` (`ζ = 0.072`, PM `8.1°`), which is far too fragile to hold under unmodelled wheel lag, 240 Hz sampling delay, or 3 mm sensor quantisation.
- **Finite frequency grid.** The default cross-check samples 13 positive frequencies (avoiding the double pole at `ω = 0`). Because `L(s)` is a rational function of degree ≤ 4, 13 distinct points strongly overdetermine its coefficients, though custom `--freq` grids must remain strictly positive.
- **Minor package drift vs `environment.lock.yml`.** Building fresh from `environment.yml` resolved `pandas 3.1.1` and `pytest 9.2.0` (vs `3.1.0` and `9.1.1` in `environment.lock.yml`), while `python 3.11.18`, `pybullet 3.25`, `numpy 2.4.6` (OpenBLAS `0.3.34`), `scipy 1.17.1`, and `matplotlib 3.11.2` matched identically. All 180 tests and all three regenerated Bode/margin outputs matched byte-for-byte.

## 6. README entry

See [Merged module contributions](../../README.md#merged-module-contributions).
