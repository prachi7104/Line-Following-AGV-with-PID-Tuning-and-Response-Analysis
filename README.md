# Line-Following AGV with PID Tuning and Response Analysis

PyBullet simulation of a differential-drive automated guided vehicle (AGV) that follows a floor
line with five analog ray sensors. It compares P, PI and PID control on a straight and a curved
track, measures the transient response (rise time, overshoot, settling time, IAE) and checks the
results against a linearised model with a Bode analysis.

CSAI4014 Algorithms for Intelligent Systems and Robotics · B.Tech CSE (AI & ML) Sem VII ·
Project P03 · Group G05 · Batch B3.

## Baseline provenance

The baseline implementation was written by Prachi Agarwalla in her personal reference workspace
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

Each member owns one module and contributes through a pull request against `main`. The table
below lists **assigned** responsibilities. A member's completed contribution is recorded only
once their pull request has been reviewed; until then the status says so.

| Member | Assigned module | Branch | Status |
|---|---|---|---|
| Harsh Kumar | Robot design and simulation environment | `module/robot-simulation` | Pull request #1 open, under review; not merged |
| Prachi Agarwalla (team lead) | Sensors and PID controller | `module/sensors-pid` | Assigned, no contribution merged yet |
| Ishita Garg | Mathematical modelling and stability analysis | `module/control-analysis` | Assigned, no contribution merged yet |
| Nitya Shukla | Experiments and performance analysis | `module/experiments-validation` | Assigned, no contribution merged yet |

The inherited baseline (every file listed under Architecture) was written by Prachi Agarwalla
and is not attributed to any module contribution.

## Architecture

| Package | Files | Role |
|---|---|---|
| `common/` | `config.py`, `schema.py` | All frozen parameters (`SimConfig`) and the CSV run schema |
| `simulation/` | `track.py`, `robot.py`, `world.py`, `assets/agv.urdf` | Straight and curved tracks, line body, differential-drive AGV, PyBullet world at 240 Hz |
| `controller/` | `sensors.py`, `pid.py` | Five-sensor ray-cast bar (10 sub-rays each), P/PI/PID with filtered D, anti-windup and output limits |
| `experiments/` | `run_experiments.py`, `demo.py` | The six headless runs (3 controllers × 2 tracks) and a live GUI demo |
| `analysis/` | `metrics.py`, `plot_results.py`, `lateral_model.py`, `bode_analysis.py`, `dummy_data.py` | Response metrics, figures and summaries, linearised model, margins and Bode plot |
| `tests/` | 8 test modules | Unit, model and end-to-end tests |
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
```

`python -m analysis.plot_results --dummy` draws the same figures from synthetic data into the
git-ignored `results/dummy/` folder; those figures are stamped "DUMMY DATA" and are never results.

## Testing

```bash
python -m pytest -q -rs
```

The suite covers the sensor model, the PID controller, the metrics, the linearised model, the CSV
schema, the tracks, the simulation and all six end-to-end runs. PyBullet is a hard import in every
simulation test, so a missing PyBullet fails the suite instead of skipping tests. Recorded result
for the baseline: **107 passed, 0 skipped** ([docs/VERIFICATION.md](docs/VERIFICATION.md)).

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
  PyBullet build; other PyBullet builds may differ in the last digits.

## Verification

See [docs/VERIFICATION.md](docs/VERIFICATION.md): test results, exact reproduction of the
published CSVs (identical content; only Windows line endings differ), clean-environment rebuild, independent reviews and the owner's GUI demo runs.

## Documents

- [docs/PROJECT_SPEC.md](docs/PROJECT_SPEC.md) — frozen decisions and metric definitions
- [docs/DESIGN.md](docs/DESIGN.md) — units, sign convention, sensor model, stability check
- [docs/MODEL.md](docs/MODEL.md) — linearised model derivation and margins
- [docs/RESULTS.md](docs/RESULTS.md) — measured results and interpretation
- [docs/VERIFICATION.md](docs/VERIFICATION.md) — evidence that the results are reproducible and checked
- [docs/superpowers/plans/2026-10-27-line-following-agv.md](docs/superpowers/plans/2026-10-27-line-following-agv.md) — phase-wise plan and closeout
