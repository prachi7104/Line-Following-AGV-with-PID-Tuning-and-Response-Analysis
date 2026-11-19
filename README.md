# Line-Following AGV with PID Tuning and Response Analysis

PyBullet simulation of a differential-drive automated guided vehicle (AGV) that follows a floor
line with five analog ray sensors. It compares P, PI and PID control on a straight and a curved
track, measures the transient response (rise time, overshoot, settling time, IAE) and checks the
results against a linearised model with a Bode analysis.

CSAI4014 Algorithms for Intelligent Systems and Robotics · B.Tech CSE (AI & ML) Sem VII ·
Project P03 · Group G05 · Batch B3.

## Baseline provenance

The baseline implementation was written by Prachi Agarwalla in a personal reference workspace
and frozen on 16 Nov 2026 at commit `3320f51`. This repository's history does not contain that
commit or the history behind it:

- `278b866` (18 Nov 2026) starts `main` as a new root commit containing only this README.
- `b158af1` (18 Nov 2026) adds the baseline as a **source snapshot** of `3320f51`. Every file
  except this README is identical to that commit (Git blob IDs compared, 54 of 54 files). The
  code, gains, sensor geometry, experiments, CSVs, figures and documents are exactly as they were
  at `3320f51`. Only this README was rewritten for the group repository.

Documents under `docs/` still describe the work as a personal implementation, because that is how
they were written. Commit IDs they cite (for example `4bce3cb` and `e6578a5` in
`docs/VERIFICATION.md` and the plan under `docs/superpowers/plans/`) refer to the reference
workspace's history and do not exist in this repository.

## Team and module allocation

Each member owned one module and contributed through one pull request against `main`. All four
pull requests have been reviewed and merged, and the repository is frozen at the final
integration.

| Member | Assigned module | Branch | Status |
|---|---|---|---|
| Harsh Kumar | Robot design and simulation environment | `module/robot-simulation` | Merged ([PR #1](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/1)) |
| Prachi Agarwalla (team lead) | Sensors and PID controller | `module/sensors-pid` | Merged ([PR #2](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/2)) |
| Ishita Garg | Mathematical modelling and stability analysis | `module/control-analysis` | Merged ([PR #5](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/5)) |
| Nitya Shukla | Experiments and performance analysis | `module/experiments-validation` | Merged ([PR #6](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/6)) |

The inherited baseline (every file listed under Architecture) was written by Prachi Agarwalla
and is not attributed to any module contribution.

### Merged module contributions

Each contribution adds new files only; no inherited code, parameter, gain or published result
was changed. The verification column gives the results recorded when each pull request was
merged; the final integrated results are under [Testing](#testing).

| Member | Pull request | Added | Verification at merge |
|---|---|---|---|
| Harsh Kumar | [#1](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/1) | `tests/test_robot_module.py` covers pure-yaw dynamics, simultaneous PyBullet world isolation and rejection of mismatched wheel geometry. [Record](docs/team/member-1-robot-simulation.md) | 5 Member 1 tests and 24 focused simulation tests passed. The integrated Windows suite passed all 185 tests with 0 skipped and 0 failed, including all 6 CSV reproduction cases. |
| Prachi Agarwalla | [#2](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/2) | `controller/diagnostics.py`: read-only sensor characteristic (3 mm step, ±1.5 mm dead band, ±88.5 mm detection limit), PID term and limit-flag tracing (candidate saturation, integral frozen, output over limit, command at limit), line-loss and reacquisition summaries. `tests/test_sensor_controller_module.py` (47 tests). [Record](docs/team/member-2-sensors-pid.md) | 47 new / 154 full tests passed, 0 skipped. All six published scenarios reproduced exactly by the instrumented loop |
| Ishita Garg | [#5](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/5) | `analysis/model_check.py`: independent `C(jω)·G(jω)` frequency-domain cross-check and closed-loop pole stability check for `d = 0.10 m` and `d = 0`, implemented filtered-D PID and ideal-D reference. `tests/test_model_check.py` (26 tests). [Record](docs/team/member-3-control-analysis.md) | `python -m analysis.model_check`: 8/8 cases passed, max rel. error 2.65e-16. 180 full tests passed, 0 skipped |
| Nitya Shukla | [#6](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/6) | `experiments/verify_reproduction.py`: strict read-only checker that a set of six run CSVs reproduces a baseline set (schema, labels, finite values, row counts, text equal after line-ending normalisation; reports the first difference). `tests/test_reproduction.py` (23 tests) and `tests/test_metrics_closed_form.py` (17 tests checking `analysis/metrics.py` against signals with closed-form answers). [Record](docs/team/member-4-experiments-validation.md) | Nitya, macOS arm64: 40 focused passed; full suite 219 passed, 6 failed (the six byte-exact CSV tests, which also fail on `main` alone there); verifier 0/6 (last-digit PyBullet build differences). Team lead, locked Windows environment: 40 focused and 225 full passed, 0 skipped, 0 failed; verifier 6/6 |

## Architecture

| Package | Files | Role |
|---|---|---|
| `common/` | `config.py`, `schema.py` | All frozen parameters (`SimConfig`) and the CSV run schema |
| `simulation/` | `track.py`, `robot.py`, `world.py`, `assets/agv.urdf` | Straight and curved tracks, line body, differential-drive AGV, PyBullet world at 240 Hz |
| `controller/` | `sensors.py`, `pid.py`, `diagnostics.py` (PR #2) | Five-sensor ray-cast bar (10 sub-rays each), P/PI/PID with filtered D, anti-windup and output limits; read-only sensor and controller diagnostics |
| `experiments/` | `run_experiments.py`, `demo.py`, `verify_reproduction.py` (PR #6) | The six headless runs (3 controllers × 2 tracks), a live GUI demo, and a strict checker that regenerated runs reproduce the published ones |
| `analysis/` | `metrics.py`, `plot_results.py`, `lateral_model.py`, `bode_analysis.py`, `dummy_data.py`, `model_check.py` (PR #5) | Response metrics, figures and summaries, linearised model, margins and Bode plot; independent frequency-domain and pole check |
| `tests/` | 13 test modules (8 baseline, 5 added by PRs #1, #2, #5 and #6) | Unit, model, module and end-to-end tests |
| `results/` | `runs/*.csv`, `figures/*.png`, `summary_*.md`, `model_margins.md` | Published simulation output |

Signal path each step: the sensor bar, `d = 0.10 m` ahead of the axle, reads five fractional
values `r_i`; their weighted centroid gives the signed lateral error `e_sensed`; the controller
returns a yaw-rate command `u = -PID(e_sensed)`, clamped to ±3 rad/s; the robot converts `(v, u)`
into left and right wheel speeds at a constant forward speed `v = 0.30 m/s`. Sign convention:
`e > 0` means the robot is left of the line, `u > 0` turns left. Details:
[docs/DESIGN.md](docs/DESIGN.md).

## Mathematical model

For a small heading error `θ` near a straight line:

```
de/dt = v·θ,    dθ/dt = ω,    e_s = e + d·θ
G(s)  = E_s(s)/Ω(s) = (v + d·s) / s²
C(s)  = Kp + Ki/s + Kd·s/(τf·s + 1),   τf = 0.05 s
```

The look-ahead `d` adds a plant zero at `−v/d = −3 rad/s`, and that zero supplies the loop's
damping. With `d = 0`, P oscillates forever and PI is unstable. With `d = 0.10 m` all three
controllers are stable:

| Controller (Kp, Ki, Kd) | Closed-loop poles | ζ | Phase margin |
|---|---|---|---|
| P (20, 0, 0) | −1.00 ± 2.24j | 0.41 | 43.9° |
| PI (20, 5, 0) | −0.88 ± 2.30j, −0.25 | 0.36 | 39.0° |
| PID (20, 5, 2), filtered D | −0.99 ± 2.00j, −0.25, −23.8 | 0.44 | 56.5° |

Derivation, assumptions, Routh checks and how to read the Bode plot:
[docs/MODEL.md](docs/MODEL.md). Generated margins: [results/model_margins.md](results/model_margins.md).

## Setup

PyPI has no Windows build of PyBullet, so on Windows use conda-forge (micromamba works without
admin rights):

```bash
micromamba create -f environment.yml
```

`environment.yml` pins only the essentials (Python 3.11, PyBullet 3.25, OpenBLAS instead of MKL,
because MKL crashed NumPy's linear algebra on Windows). `environment.lock.yml` records the exact
package set the published results were produced and tested with:

| Package | Tested version |
|---|---|
| Python | 3.11.18 |
| pybullet (conda-forge) | 3.25 (reports itself as 3.2.5) |
| numpy | 2.4.6, with OpenBLAS 0.3.34 |
| scipy | 1.17.1 |
| pandas | 3.1.0 |
| matplotlib | 3.11.2 |
| pytest | 9.1.1 |

On Linux or macOS, `pip install -r requirements.txt` also works; PyBullet builds differ, so the
results may differ in the last digits from the published CSVs.

## Run on Windows (PowerShell)

Call the environment's Python directly; this needs no `micromamba activate` or shell setup:

```powershell
$py = "$HOME\micromamba\envs\agv\python.exe"
& $py -m pytest -q                                              # all tests, none skipped
& $py -m experiments.run_experiments --experiment all           # 6 runs -> results\runs\*.csv
& $py -m analysis.plot_results                                  # figures + results\summary_*.md
& $py -m analysis.bode_analysis                                 # margins, Bode plot, model vs sim
& $py -m experiments.demo --track curve --controller PID        # live PyBullet window
& $py -m experiments.demo --track straight --controller P
& $py -m controller.diagnostics                                 # sensor characteristic + run summaries (read-only)
& $py -m analysis.model_check                                   # independent frequency-domain and pole check (read-only)
```

To check that regenerated runs reproduce the published ones, regenerate in a disposable copy of
the repository (never in your working copy), keeping a copy of the published `results\runs`:

```powershell
& $py -m experiments.verify_reproduction <published-runs-folder> results\runs   # expect 6 of 6
```

Run these from the repository root. If `micromamba activate agv` is set up in your shell, plain
`python -m ...` works the same way.

## Run on Linux / macOS

```bash
micromamba activate agv        # or the virtualenv made from requirements.txt
pytest -q
python -m experiments.run_experiments --experiment all
python -m analysis.plot_results
python -m analysis.bode_analysis
python -m experiments.demo --track curve --controller PID
python -m controller.diagnostics
python -m analysis.model_check
```

On these platforms PyBullet builds differ from the Windows conda-forge build, so the six byte-exact
CSV tests in `tests/test_sensor_controller_module.py` and `verify_reproduction` against the
published CSVs can fail in the last digits (observed on Linux and macOS; see Limitations).

`python -m analysis.plot_results --dummy` draws the same figures from synthetic data into the
git-ignored `results/dummy/` folder; those figures are stamped "DUMMY DATA" and are never results.

## Testing

```bash
python -m pytest -q -rs
```

The suite covers the sensor model, the PID controller, the metrics, the linearised model, the CSV
schema, the tracks, the simulation and all six end-to-end runs, plus the four module
contributions. PyBullet is a hard import in every simulation test, so a missing PyBullet fails the
suite instead of skipping tests.

| Stage | Tests | Result (locked Windows environment) |
|---|---|---|
| Baseline (`b158af1`) | 107 | 107 passed, 0 skipped |
| + PR #1 robot (5), #2 sensors/PID (47), #5 model check (26) | 185 | 185 passed, 0 skipped, 0 failed |
| **Final, + PR #6 experiments (40)** | **225** | **225 passed, 0 skipped, 0 failed** |

Final reproduction check: all six runs regenerated from the final code and compared with the
published CSVs by `experiments/verify_reproduction.py`: **6 of 6 reproduce**. `python -m
analysis.model_check`: 8 of 8 cases pass. Details: [docs/VERIFICATION.md](docs/VERIFICATION.md).

## Results

P, PI and PID all follow both tracks without losing the line. PID settles fastest on a 5 cm step
(2.46 s vs 3.28 s for P). On the curve, P holds a 13.8 mm offset over the final 40 % of the arc;
PI and PID reduce that window's mean to about 2 mm, while their integrators are still converging.
Full tables, figures and explanations: [docs/RESULTS.md](docs/RESULTS.md).

## Limitations

- **Sensor resolution.** Each sensor has 10 sub-rays at a 3 mm pitch, so the sensed line position
  moves in 3 mm steps and any true offset under about 1.5 mm reads as zero. The responses settle
  about 1.5 mm off the line for that reason. The settling band (±2.5 mm) is wider than this dead
  band, so the settling times are still meaningful.
- **The model is idealised.** It leaves out wheel-motor lag, the 5 N·m torque cap, the ±3 rad/s
  saturation, the 240 Hz sampling delay, sensor quantisation and large heading angles. It
  explains the trends but does not certify the simulated numbers.
- **Curve integrators have not converged.** On the 7.9 s arc, the PI and PID integrators (time
  constant about 4 s) are still converging when the arc ends, towards a true error of about
  +6.7 mm, not 0.
- **Straight-track IAE does not rank the controllers.** About 27 % of each straight-track IAE is
  the dead-band residual, larger than the differences between controllers.
- **Platform reproducibility.** The published CSVs were produced on Windows with the conda-forge
  PyBullet build; other PyBullet builds may differ in the last digits. Team members observed this
  on Linux (PyPI PyBullet 3.2.7) and macOS arm64: the six byte-exact CSV tests fail there and
  the verifier reports 0 of 6, while the metrics agree to within a few percent and the conclusions
  are unchanged ([Member 4 record](docs/team/member-4-experiments-validation.md)). The tests are
  kept strict on purpose.

## Verification

See [docs/VERIFICATION.md](docs/VERIFICATION.md): test results, exact reproduction of the
published CSVs (identical content; only Windows line endings differ), clean-environment rebuild,
the owner's GUI demo runs and the final group integration.

## Documents

- [docs/PROJECT_SPEC.md](docs/PROJECT_SPEC.md) — frozen decisions and metric definitions
- [docs/DESIGN.md](docs/DESIGN.md) — units, sign convention, sensor model, stability check
- [docs/MODEL.md](docs/MODEL.md) — linearised model derivation and margins
- [docs/RESULTS.md](docs/RESULTS.md) — measured results and interpretation
- [docs/VERIFICATION.md](docs/VERIFICATION.md) — evidence that the results are reproducible and checked
- [docs/superpowers/plans/2026-10-27-line-following-agv.md](docs/superpowers/plans/2026-10-27-line-following-agv.md) — phase-wise plan and closeout
- Member records: [1 robot and simulation](docs/team/member-1-robot-simulation.md),
  [2 sensors and PID](docs/team/member-2-sensors-pid.md),
  [3 modelling and stability](docs/team/member-3-control-analysis.md),
  [4 experiments and validation](docs/team/member-4-experiments-validation.md)
