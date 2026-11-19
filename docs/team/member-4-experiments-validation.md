# Member 4: Experiments and Validation

- **Author:** Nitya Shukla (GitHub: NityaShukla25)
- **Branch:** `module/experiments-validation`
- **Status:** PARTIAL. My additions work and their tests pass. The published results are not reproduced bit for bit on my machine, and six baseline tests that require bit-exact reproduction fail on it (sections 5 and 6).

> **Team lead's note (added at integration, 19 Nov 2026; not part of Nitya's own record).** Merged
> in [PR #6](https://github.com/prachi7104/Line-Following-AGV-with-PID-Tuning-and-Response-Analysis/pull/6)
> (merge commit `0352cfd`). On the locked Windows reference environment, all 225 tests pass and
> the verifier reports 6 of 6 files reproduced. See section 9. Sections 1 to 8 are Nitya's
> record, unchanged, and their PARTIAL status describes her macOS machine.

## 1. What this PR contains

The group repository's `main` already holds the complete original project (published by the lead). My PR **adds three new code and test files plus this record, and changes no original file**:

| File | Purpose |
|---|---|
| `experiments/verify_reproduction.py` | Read-only CLI that compares a baseline and a candidate run directory (section 4) |
| `tests/test_reproduction.py` | 23 tests for the verifier, using small temporary fixture files |
| `tests/test_metrics_closed_form.py` | 17 tests that check `analysis/metrics.py` against signals with exactly known answers |
| `docs/team/member-4-experiments-validation.md` | this record |

I did not write any of the original modules (simulation, controller, experiments, metrics, plots, schema). Those are Prachi Agarwalla's implementation and are the baseline, not my contribution.

## 2. Authorship and sources

- I wrote these files together with Claude, an AI assistant made by Anthropic. Claude drafted them. I ran every command and test myself, reviewed the files and committed them under my own identity.
- Every commit I added, including the merge commit, has me as its author. Commits by other members reach this branch only through the merged `main`.
- An earlier version of my branch re-implemented some of the original modules. I dropped it when the lead published the full baseline to `main`, and it is not part of this PR.
- I have not yet confirmed with the faculty that this kind of AI assistance is permitted under the course rules.

## 3. Baseline and environment

- Group `main` at the time of my final checks: `f16f580`. The lead's baseline commit is `b158af1` ("publish complete verified AGV reference baseline"). `main` moved while I worked, so I merged it into my branch twice (merge commits `52aaf45` and `bdba38f`). Neither merge had conflicts. The test counts and the verifier result in section 5 were measured at `bdba38f`.
- Reference ZIP `Line-Following-AGV-with-PID-Tuning-and-Response-Analysis-main.zip`, archive revision `3320f511c2486475f53a171167c5613792db8b0b`, SHA-256 `070cc3b77558f1a8ce2415d6f9440c4ee0adefbde0cace1686245b7ac9ceb9ad`, which matches the value in the team instructions. Some of my earlier comparisons used the published results from this ZIP and the latest ones used `results/runs` on `main`. Both gave the same first differences and row counts.
- Environment: macOS 26.6.2, arm64 (Apple M4). micromamba environment from `micromamba create -f environment.yml`: Python 3.11.18, pybullet 3.25 (build time Oct 21 2025), numpy 2.4.6, pandas 3.1.1, matplotlib 3.11.2, scipy 1.17.1, pytest 9.2.0.

## 4. How the additions work

**Reproduction verifier.** `python -m experiments.verify_reproduction BASELINE_DIR CANDIDATE_DIR` checks, for each of the six standard run files (`straight_P/PI/PID.csv`, `curve_P/PI/PID.csv`):

- the file exists and is not empty;
- the header equals `common.schema.COLUMNS`, in order;
- every row has 10 values, and the numeric columns are finite numbers;
- the controller label equals the mode in the file name, `line_detected` is 0 or 1, and `segment` is `straight` or `arc` (no arc rows in a straight-track file);
- the row counts match;
- the text equals the baseline's after normalising CRLF/LF (a missing final newline is ignored).

It reports the first differing row and column. It exits 0 when everything passes, 1 on any failure and 2 when a directory is missing. It is deliberately strict: it says whether files are the same, not whether a small numerical difference is acceptable.

**Verifier tests.** The 23 tests build six valid fixture files and cover identical sets, CRLF files, a missing final newline, a missing file, a modified value (the row and column are reported), only the first difference being reported, a row-count difference, a swapped header, an empty file, a header-only file, a short row, `nan`/`inf`/`-inf`, text in a numeric column, a wrong controller label, an invalid `line_detected`, an unknown segment, an arc row in a straight file, a broken baseline, many problems in one file, and a missing directory. The fixtures are test data, not results.

**Closed-form metric tests.** Each test feeds `step_metrics`, `curve_metrics` or `summarize` a signal whose answer is known from mathematics, so the expected value does not come from the code under test. Examples: for e = E0·exp(-2t) the rise time is ln(9)/2, the settling time is ln(E0/band)/2 and the IAE is E0/2; for e = E0·exp(-t)·cos(2t) the overshoot follows from tan(2t) = -1/2. Other tests cover the sign of the initial offset, runs that never reach 10 %, a short final hold not counting as settled, and curve metrics using arc rows only. I did not compare these tests one by one with the original `tests/test_metrics.py`, so I do not claim that they are absent there.

## 5. Verification record

All commands were run on the machine in section 3.

1. **Focused tests:** `python -m pytest -q -rs tests/test_reproduction.py tests/test_metrics_closed_form.py` gave **40 passed** (23 + 17).
2. **Full suite on my branch** (`bdba38f`, merged with `main` at `f16f580`): `python -m pytest -q -rs` gave **219 passed, 6 failed, 0 skipped**.
3. **The same suite on a clean clone of `main` alone** (`f16f580`, none of my files): **179 passed, 6 failed**. The same 6 tests fail without my PR. The difference between the two runs (219 against 179) is my 40 tests, all of which pass.
4. **The 6 failures** are `test_instrumented_run_reproduces_published_csv` for the six scenarios, in `tests/test_sensor_controller_module.py` (Member 2's file, merged with `main`). The test requires a regenerated CSV to match the published CSV byte for byte, and fails at the first differing digit. The project's README (Limitations) states that the published CSVs were produced on Windows with the conda-forge PyBullet build and that other PyBullet builds may differ in the last digits. `environment.lock.yml` is marked Windows-only. I did not modify the test; whether to relax or skip it on other platforms is the lead's decision.
5. **Files changed by this PR:** before the merge, `git diff --name-status 5318422 HEAD` listed the three added test and tool files and no other file. After the latest merge, `git diff --name-status origin/main HEAD` lists four added files: those three and this record.
6. **Regenerate the six runs** with the unmodified original code, in a temporary clone of my merged branch (`bdba38f`): `python -m experiments.run_experiments --experiment all`. I did the same earlier in a disposable copy of the reference ZIP, and the metric tables below come from `python -m analysis.plot_results` there.
7. **Verifier on the real files:** `python -m experiments.verify_reproduction <published runs in my repo> results/runs` (run in the temporary clone) reported **0 of 6 files reproduce** and exited with status 1. The first difference is at row 2, column `x`, for example baseline 0.00018174108117818832 vs candidate 0.0001818174496293068 on the straight track. The three controllers on a track show the same first difference.

| Run | Rows (regenerated) | Rows (published) |
|---|---|---|
| straight P / PI / PID | 3211 / 3212 / 3210 | 3211 / 3212 / 3210 |
| curve P / PI / PID | 2741 / 2713 / 2713 | 2742 / 2714 / 2710 |

**Straight track metrics, regenerated / published**

| | P | PI | PID |
|---|---|---|---|
| Rise time [s] | 0.6007 / 0.5997 | 0.5655 / 0.5651 | 0.6663 / 0.6676 |
| Overshoot [%] | 24.52 / 24.62 | 30.57 / 30.61 | 24.48 / 24.43 |
| Settling time [s] | 3.288 / 3.279 | 3.279 / 3.279 | 2.458 / 2.458 |
| IAE | 0.05398 / 0.05402 | 0.05378 / 0.05398 | 0.05527 / 0.05522 |

**Curve track metrics (arc rows), regenerated / published**

| | P | PI | PID |
|---|---|---|---|
| Mean abs error [m] | 0.01267 / 0.01268 | 0.00371 / 0.003695 | 0.003662 / 0.003631 |
| Max abs error [m] | 0.01626 / 0.01626 | 0.01283 / 0.01282 | 0.01216 / 0.0123 |
| Tail mean [m] | -0.01384 / -0.01384 | 0.002239 / 0.002232 | 0.002024 / 0.001956 |
| IAE | 0.1022 / 0.1023 | 0.02949 / 0.02938 | 0.0291 / 0.02881 |

The largest relative difference is 3.5 % (PID tail mean, about 0.07 mm). Every other value is within about 1.1 % (the next largest is PID max error). The P settling time differs by 9 ms, about two simulation steps. The conclusions are unchanged: PID settles fastest on the straight (2.458 s against 3.288 s for P), P keeps a steady offset of about -13.8 mm on the curve, and PI and PID cut the mean curve error to about a third of P's.

8. **GUI runs** with the group's `experiments/demo.py`. Each run, made on commit `e66ce46` before the later merges, finished and printed:
   - `--track straight --controller P`: rise time 0.6007, overshoot 24.5168, settling 3.2875, IAE 0.0540.
   - `--track straight --controller PID`: rise time 0.6663, overshoot 24.4842, settling 2.4583, IAE 0.0553.
   - `--track curve --controller PID`: mean abs 0.0037, max abs 0.0122, tail mean 0.0020, IAE 0.0290.

   The curve IAE printed 0.0290 here, while an earlier run of the same command and the summary table give 0.0291. The difference is in the fourth decimal and I did not investigate its cause. I confirmed that the PyBullet window opened in an earlier run of these commands, but I did not record separate observations of the robot's motion or the camera.

## 6. Limitations and open issues

- **Not bit-identical on macOS.** The regenerated results differ from the published ones from row 2 of every file, and the curve runs end 1 to 3 rows apart. The same first difference for all three controllers on a track is consistent with platform-dependent floating-point behaviour in the physics step. The project's README documents that the published files were produced on Windows with the conda-forge PyBullet build. I could not test on Windows, so I did not confirm this cause independently. The behaviour and the conclusions are reproduced, so I report PARTIAL, not a full reproduction.
- **Six baseline tests fail on this machine** for the same reason (section 5, items 3 and 4). They fail on `main` without my PR, and I did not change them.
- **Strict verifier.** It has no tolerance mode, so on this machine it reports failure for all six regenerated files. Judging whether the numerical drift is acceptable is done by comparing the metric tables above.
- **Scope of the verifier.** It checks the six run CSVs only. It does not compare figures, summary tables or the Bode analysis.
- Everything was run on one machine (macOS, Apple Silicon). I did not run it on Windows or Linux.

## 7. Steps to reproduce

1. Create the environment: `micromamba create -f environment.yml`, then `micromamba activate agv`.
2. Run the tests: `python -m pytest -q -rs` (on a non-Windows build, expect the six byte-exact reproduction tests in `tests/test_sensor_controller_module.py` to fail).
3. Regenerate in a disposable copy of the full project (not in the working branch): `python -m experiments.run_experiments --experiment all`, then `python -m analysis.plot_results`.
4. Compare: `python -m experiments.verify_reproduction <baseline runs dir> <regenerated runs dir>`.

## 8. Proposed README row

| Member | Module | Newly authored files | Verification | Record |
|---|---|---|---|---|
| Nitya Shukla | Experiments and Validation | `experiments/verify_reproduction.py`, `tests/test_reproduction.py`, `tests/test_metrics_closed_form.py` | Focused tests 40 passed. Full suite 219 passed, 6 failed, 0 skipped; the same 6 baseline tests also fail on `main` alone on macOS. Regenerated results differ from the published ones in the last digits (PARTIAL) | `docs/team/member-4-experiments-validation.md`; PR link to be added once opened |

## 9. Team lead's Windows verification at integration

*Added by Prachi Agarwalla (team lead) when merging PR #6. Nitya did not run anything in this
section, and nothing in sections 1 to 8 was changed. The team lead's checks ran in Claude Code,
an AI coding assistant, in the team lead's session, on the team lead's Windows laptop.*

- **What was tested:** PR head `59774d7` merged without committing onto `main` at `f16f580`,
  in a temporary Git worktree. The merge had no conflicts and added exactly the four files in
  section 1. No file under `common/`, `controller/`, `simulation/`, `analysis/`, `results/`,
  `experiments/run_experiments.py` or the environment files changed.
- **Environment:** the locked reference environment (`environment.lock.yml`) on Windows 11:
  micromamba env `agv` with Python 3.11.18, conda-forge PyBullet 3.25 (build time Oct 21 2025),
  numpy 2.4.6, scipy 1.17.1, pandas 3.1.0, matplotlib 3.11.2 and pytest 9.1.1.

| Check | Command | Result |
|---|---|---|
| Member 4 focused tests | `python -m pytest -q -rs tests/test_metrics_closed_form.py tests/test_reproduction.py` | **40 passed**, 0 skipped |
| Full suite | `python -m pytest -q -rs` | **225 passed, 0 skipped, 0 failed** (185 on `main` + 40 new) |
| Regenerate the six runs | `python -m experiments.run_experiments --experiment all`, in the temporary worktree | six files written; `git diff` reports no content change in `results/` |
| Verifier on the real files | `python -m experiments.verify_reproduction <copy of published results/runs> results/runs` | **6 of 6 files reproduce the baseline**, exit status 0 |

The same verifier that reports 0 of 6 on macOS (section 5, item 7) reports 6 of 6 on the
Windows conda-forge build that produced the published CSVs. This agrees with Nitya's explanation
that the macOS differences come from the platform's PyBullet build, not from the code. The six
byte-exact tests in `tests/test_sensor_controller_module.py` were kept strict: they pass on the
reference environment, and the README documents that other PyBullet builds can differ in the
last digits.
