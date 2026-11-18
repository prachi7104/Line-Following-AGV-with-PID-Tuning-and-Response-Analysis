# Verification record

Evidence that the published results are genuine simulation output, reproducible and checked. Each
item says who ran it and on which commit. Final state: project frozen on 16 Nov 2026.

## 1. Automated tests

| When | Commit | Who | Command | Result |
|---|---|---|---|---|
| Final closeout | closeout commit (after `4bce3cb`) | Claude, project env | `python -m pytest -q -rs` | **107 passed, 0 skipped**, ~27 s |
| Before closeout | `4bce3cb` | Project owner, own laptop | `python -m pytest -q` | 75 passed in 16.48 s (owner-reported) |
| Clean rebuild | `e6578a5` | Claude, fresh clone + fresh env (session log; not stored in the repo) | `pytest -q` | 74 passed |

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
| Independent source review (Claude subagent, `e6578a5`) | Sign chain, PID, metrics, model, fairness, every number in the docs | No code bugs. Fixed in `4bce3cb`: <br>• one substantive doc error, that PI/PID true error converges towards +d²/(2R), not 0; <br>• two claims narrowed: the model ranking now covers only rise time and overshoot, and a caveat says straight-track IAE does not rank the controllers; <br>• a latent curve-angle bug guarded. |
| External review (ChatGPT, owner-run, owner-reported) | Published GitHub source; metrics recomputed from all six CSVs; closed-loop poles checked | Raised the closeout items: filtered-PID consistency in the Bode analysis, steady-state terminology, end-to-end test coverage, and the reproducibility record. |
| Final independent review (Claude subagent, closeout) | Closeout diff | See section 5. |

## 4. Owner-executed GUI demos (owner-reported, own laptop, `4bce3cb`)

Run with the project environment's Python, called directly:

| Command | Result |
|---|---|
| `python -m experiments.demo --track curve --controller PID` | PASS; reproduced the expected metrics |
| `python -m experiments.demo --track straight --controller P` | PASS; reproduced the expected metrics |

## 5. Final review of the closeout changes

An independent Claude subagent reviewed the uncommitted closeout diff on top of `4bce3cb`. It
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
