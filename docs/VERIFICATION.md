# Verification record

Evidence that the published results are genuine simulation output, reproducible and checked. Each
item says who ran it and on which commit. Final state: project frozen on 16 Nov 2026.

Sections 1 to 5 are the historical record of the personal baseline. Their commit IDs refer to
the reference workspace's history (see the README's "Baseline provenance"). Section 6 records
the group repository's final integration of all four member contributions (18–19 Nov 2026).

**AI assistance.** Items marked *AI coding assistant* or *AI-assisted review* were done by AI
tools in the project owner's sessions, not by people; each source review ran as a separate AI
review agent. The external review was run by the owner with a second AI tool, and the owner
reported its findings. Items marked *project owner* were run by the owner on their own laptop.
The tools are named in the README's [AI-use disclosure](../README.md#ai-use-disclosure).

## 1. Automated tests

| When | Commit | Who | Command | Result |
|---|---|---|---|---|
| Final closeout | closeout commit (after `4bce3cb`) | AI coding assistant, project env | `python -m pytest -q -rs` | **107 passed, 0 skipped**, ~27 s |
| Before closeout | `4bce3cb` | Project owner, own laptop | `python -m pytest -q` | 75 passed in 16.48 s (owner-reported) |
| Clean rebuild | `e6578a5` | AI coding assistant, fresh clone + fresh env (session log; not stored in the repo) | `pytest -q` | 74 passed |

The closeout added 32 tests:

- **End-to-end, all six controller/track runs** (`tests/test_integration.py`). Each run checks:
  - schema and labels;
  - initial conditions: 5 cm or 0 offset, heading 0, first sensed error equal to the true offset;
  - track completion: ends at the track's end point, not by timeout, in the nominal path length / v time;
  - line detected on every step;
  - tracking error and steering command stay bounded.

  It also checks the steering sign for all three controllers and the P-versus-PI/PID curve offset.
- **Filtered-PID model consistency** (`tests/test_model.py`):
  - the filtered time response decays at the filtered poles' rate;
  - the response tends to the ideal-D response as τf → 0;
  - τf is ignored when there is no D term;
  - the discrete controller's derivative low-pass matches the continuous filter `Kd s/(τf s + 1)`;
  - the Bode cases label the implemented and ideal PID correctly.

PyBullet is now a hard import in every simulation test, so a missing PyBullet fails the suite and
does not skip tests silently.

## 2. Reproducibility of the published experiments

All six runs were regenerated at closeout with `python -m experiments.run_experiments --experiment
all` and compared with the published CSVs in `HEAD`:

| File | Rows | Content vs published |
|---|---|---|
| `results/runs/straight_P.csv` | 3211 | identical |
| `results/runs/straight_PI.csv` | 3212 | identical |
| `results/runs/straight_PID.csv` | 3210 | identical |
| `results/runs/curve_P.csv` | 2742 | identical |
| `results/runs/curve_PI.csv` | 2714 | identical |
| `results/runs/curve_PID.csv` | 2710 | identical |

"Identical" means byte-identical after normalising line endings: Windows writes CRLF and Git
stores LF (`core.autocrlf = true`); `git diff` reports no change. Earlier checks:

- a same-machine rerun (`e6578a5`) gave byte-identical files;
- a fresh clone with a fresh environment built from `environment.yml` reproduced every CSV,
  summary table and figure, with an empty `git status`.

The simulation is deterministic: PyBullet DIRECT mode, a fixed time step, and no random numbers
anywhere in the pipeline.

Regenerated at closeout:

- `summary_straight.md` and `summary_curve.md`: identical numbers. The curve summary gained a
  footnote defining `steady_state` as a final-40 % window mean.
- `model_margins.md`, `bode.png` and `model_vs_sim.png`: changed on purpose. They now use the
  implemented filtered PID and label the ideal-D PID as theory only.
- `error_*.png` and `paths_*.png`: unchanged.

Tested dependency versions: `environment.lock.yml` and the table in `README.md`.

## 3. Reviews

| Review | Scope | Outcome |
|---|---|---|
| AI-assisted source review (separate AI review agent, `e6578a5`) | Sign chain, PID, metrics, model, fairness, every number in the docs | No code bugs. Fixed in `4bce3cb`: <br>• one substantive doc error, that PI/PID true error converges towards +d²/(2R), not 0; <br>• two claims narrowed: the model ranking now covers only rise time and overshoot, and a caveat says straight-track IAE does not rank the controllers; <br>• a latent curve-angle bug guarded. |
| External AI-assisted review (second AI tool, owner-run, owner-reported) | Published GitHub source; metrics recomputed from all six CSVs; closed-loop poles checked | Raised the closeout items: filtered-PID consistency in the Bode analysis, steady-state terminology, end-to-end test coverage, and the reproducibility record. |
| Final AI-assisted review (separate AI review agent, closeout) | Closeout diff | See section 5. |

## 4. Owner-executed GUI demos (owner-reported, own laptop, `4bce3cb`)

Run with the project environment's Python, called directly:

| Command | Result |
|---|---|
| `python -m experiments.demo --track curve --controller PID` | PASS; reproduced the expected metrics |
| `python -m experiments.demo --track straight --controller P` | PASS; reproduced the expected metrics |

## 5. Final review of the closeout changes

A separate AI review agent reviewed the uncommitted closeout diff on top of `4bce3cb`. It
found **no blockers**, and verified:

- the filtered model's equations and that they match `controller/pid.py`'s backward-Euler filter
  and reset;
- every changed number, recomputed: PID model 0.676 s / 22.6 % / 2.38 s; filtered and ideal poles
  and margins;
- a regenerated `model_margins.md` identical to the committed one;
- 107 passed, 0 skipped;
- `results/runs` and `summary_straight.md` untouched;
- no private paths in `environment.lock.yml`.

Its two should-fix items and seven nits were all fixed before commit:

- MODEL.md §6 now gives the filtered time-response equations, and §7 gives the implemented PID
  phase margin (56.5° at 3.01 rad/s).
- Labels for ideal and implemented D clarified in DESIGN.md and in the model-vs-sim figure title.
- README wording on reproduction aligned with this file, and the Linux activate step added.
- Two evidence claims in this file made precise.

## 6. Final group integration (18–19 Nov 2026)

The baseline was published to the group repository as `b158af1`, a source snapshot of
`3320f51`. Each member then added one module through one pull request:

| PR | Member | Merge commit | Adds |
|---|---|---|---|
| #1 | Harsh Kumar (robot and simulation) | `faf1dc4` | `tests/test_robot_module.py` (5 tests), member record |
| #2 | Prachi Agarwalla (sensors and PID) | `30831b7` | `controller/diagnostics.py`, `tests/test_sensor_controller_module.py` (47 tests), member record |
| #5 | Ishita Garg (modelling and stability) | `a1162ec` | `analysis/model_check.py`, `tests/test_model_check.py` (26 tests), member record |
| #6 | Nitya Shukla (experiments and validation) | `0352cfd` | `experiments/verify_reproduction.py`, `tests/test_reproduction.py` (23), `tests/test_metrics_closed_form.py` (17), member record |

Each member's own runs are in their record under `docs/team/`. The checks below are the team
lead's, AI-assisted (see the README's [AI-use disclosure](../README.md#ai-use-disclosure)), on
a Windows 11 laptop. They used the locked reference environment (`environment.lock.yml`): micromamba
env `agv`, Python 3.11.18, conda-forge PyBullet 3.25, numpy 2.4.6, scipy 1.17.1, pandas 3.1.0,
matplotlib 3.11.2 and pytest 9.1.1.

**Frozen implementation unchanged.** `git diff --name-status b158af1 0352cfd` lists no modified
or deleted file under `common/`, `controller/`, `simulation/`, `experiments/`, `analysis/`,
`results/`, `tests/` or the environment files. The integration only adds the files above.
The only baseline file modified is this document: AI-disclosure wording before this section,
and this section itself.

**PR #6 before merging.** PR head `59774d7`, merged without committing onto `f16f580` in a
temporary worktree: 40 focused tests passed, and the full suite gave 225 passed, 0 skipped,
0 failed. The six runs were regenerated there, and `experiments.verify_reproduction` reported
6 of 6.

**Final integrated code, fresh clone.** A fresh clone from GitHub was checked out at merge commit
`0352cfd`. Every later commit changes documentation only.

| Check | Command | Result |
|---|---|---|
| Model check | `python -m analysis.model_check` | 8/8 cases passed; max rel. error 2.65e-16 |
| Full suite | `python -m pytest -q -rs` | **225 passed, 0 skipped, 0 failed** |
| Regenerate the six runs | `python -m experiments.run_experiments --experiment all` | six files written |
| Reproduction | `python -m experiments.verify_reproduction <copy of published runs> results/runs` | **6 of 6 files reproduce** (exit 0) |
| Working tree after regeneration | `git status --short` | empty: no published file changed |

| File | Reproduced |
|---|---|
| `results/runs/straight_P.csv` | PASS |
| `results/runs/straight_PI.csv` | PASS |
| `results/runs/straight_PID.csv` | PASS |
| `results/runs/curve_P.csv` | PASS |
| `results/runs/curve_PI.csv` | PASS |
| `results/runs/curve_PID.csv` | PASS |

**Other platforms (member-reported).** Harsh (Linux, PyPI PyBullet 3.2.7) and Nitya (macOS arm64,
conda-forge PyBullet 3.25) reported that the six byte-exact CSV tests fail in the last digits.
Nitya confirmed that the same six fail on `main` without her contribution. Nitya's verifier reported 0 of 6
there, while the metrics agreed to within 3.5 % and the conclusions were unchanged. The tests were
kept strict.
