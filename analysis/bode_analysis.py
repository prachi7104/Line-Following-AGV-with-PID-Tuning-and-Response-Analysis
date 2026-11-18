"""Frequency-domain analysis of the linearised loop: margins, Bode plot, model-vs-simulation plot.

CLI:  python -m analysis.bode_analysis
    writes results/figures/bode.png and results/model_margins.md, and
    results/figures/model_vs_sim.png when results/runs/straight_{P,PI,PID}.csv exist.
"""
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import brentq

from analysis.lateral_model import closed_loop_poles, loop_tf, simulate_offset_response
from common.config import DEFAULT, GAINS

ROOT = Path(__file__).resolve().parents[1]
_W_GRID = np.logspace(-4, 4, 8001)          # rad/s, search grid for crossings
_W_PLOT = np.logspace(-2, 3, 2000)          # rad/s, plotted range
_MARGINAL_TOL = 1e-9


# ---------------------------------------------------------------- frequency response

def _freq_response(num, den, w):
    """|L(jw)| and the continuous (true) phase of L(jw) in degrees.

    Phase is built as a sum of factor angles, angle(jw - zero) - angle(jw - pole), so it is
    continuous by construction (a pole at the origin contributes exactly -90 deg at every w);
    np.unwrap is applied on top as a guard.
    """
    num = np.atleast_1d(np.asarray(num, dtype=float))
    den = np.atleast_1d(np.asarray(den, dtype=float))
    w = np.atleast_1d(np.asarray(w, dtype=float))
    s = 1j * w
    mag = np.abs(np.polyval(num, s) / np.polyval(den, s))
    k = num[0] / den[0]
    phase = np.full(w.shape, 0.0 if k > 0 else -np.pi)
    for z in np.roots(num):
        phase += np.angle(s - z)
    for p in np.roots(den):
        phase -= np.angle(s - p)
    if phase.size > 1:
        phase = np.unwrap(phase)
    return mag, np.degrees(phase)


def _mag_db(num, den, w):
    return 20.0 * np.log10(_freq_response(num, den, w)[0])


def margins(num, den) -> dict:
    """Gain and phase margins of the loop L = num/den.

    Returns gain_margin_db, phase_margin_deg, w_gc (|L| = 1), w_pc (phase = -180 deg).
    math.inf where there is no crossing. If the phase sits exactly on -180 deg at every
    frequency (e.g. Kp/s^2) the phase crossover is undefined and gain_margin_db, w_pc are nan.
    Where several crossings exist the most critical one is reported (smallest |GM| in dB,
    smallest PM). Crossings are found on a log grid and refined with brentq.
    """
    w = _W_GRID
    mag, phase = _freq_response(num, den, w)
    logmag = np.log10(mag)

    def f_mag(x):
        return float(np.log10(_freq_response(num, den, x)[0][0]))

    # The grid phase is continuous; evaluate a single frequency on the same branch by
    # anchoring to the nearest grid point.
    def f_phase(x):
        ph = _freq_response(num, den, np.array([w[np.searchsorted(w, x).clip(0, w.size - 1)], x]))[1]
        return float(ph[1])

    # --- gain crossovers -> phase margin
    pm, w_gc = math.inf, math.inf
    for i in np.flatnonzero(np.sign(logmag[:-1]) * np.sign(logmag[1:]) <= 0):
        if logmag[i] == 0 and i > 0 and logmag[i - 1] == 0:
            continue
        wc = w[i] if logmag[i] == 0 else brentq(f_mag, w[i], w[i + 1], xtol=1e-14, rtol=1e-12)
        ph = f_phase(wc)
        cand = (ph + 180.0 + 180.0) % 360.0 - 180.0      # PM in (-180, 180]
        if cand < pm:
            pm, w_gc = cand, wc

    # --- phase crossovers -> gain margin
    gm, w_pc = math.inf, math.inf
    if np.ptp(phase) < 1e-9 and abs((phase[0] + 180.0) % 360.0) < 1e-9:
        gm, w_pc = math.nan, math.nan                     # phase identically -180 deg
    else:
        for i in range(w.size - 1):
            lo, hi = sorted((phase[i], phase[i + 1]))
            k_lo = math.floor((lo + 180.0) / 360.0) + 1   # smallest k with -180+360k > lo
            k_hi = math.floor((hi + 180.0) / 360.0)       # largest  k with -180+360k <= hi
            for k in range(k_lo, k_hi + 1):
                target = -180.0 + 360.0 * k
                if phase[i + 1] == target:
                    wp = w[i + 1]
                else:
                    wp = brentq(lambda x: f_phase(x) - target, w[i], w[i + 1], xtol=1e-14, rtol=1e-12)
                cand = -20.0 * f_mag(wp)
                if abs(cand) < abs(gm):
                    gm, w_pc = cand, wp

    return {"gain_margin_db": gm, "phase_margin_deg": pm, "w_gc": w_gc, "w_pc": w_pc}


# ---------------------------------------------------------------- formatting helpers

def _fmt_margin_db(m):
    gm = m["gain_margin_db"]
    if math.isnan(gm):
        return "undefined (phase = -180 deg at all w)"
    if math.isinf(gm):
        return "inf (phase never crosses -180 deg)"
    return f"{gm:+.1f} dB at {m['w_pc']:.3g} rad/s"


def _fmt_margin_deg(m):
    pm = m["phase_margin_deg"]
    if math.isinf(pm):
        return "inf (|L| never crosses 1)"
    return f"{pm:+.1f} deg at {m['w_gc']:.3g} rad/s"


def _fmt_poles(poles):
    out = []
    for p in sorted(poles, key=lambda z: (z.real, -abs(z.imag))):
        re = 0.0 if abs(p.real) < 5e-4 else p.real          # no "-0.000"
        if abs(p.imag) < 1e-9:
            out.append(f"{re:.3f}")
        elif p.imag > 0:
            out.append(f"{re:.3f} ± {p.imag:.3f}j")
    return ", ".join(out)


def _damping(poles):
    pair = [p for p in poles if p.imag > 1e-9]
    if not pair:
        return None
    return min(-p.real / abs(p) for p in pair)


def _stable_text(poles):
    re = np.max(poles.real)
    if re < -_MARGINAL_TOL:
        return "yes"
    if abs(re) <= _MARGINAL_TOL:
        return "no (marginal)"
    return "no (unstable)"


# ---------------------------------------------------------------- plots

def implemented_tau_f(gains_tuple, cfg=DEFAULT) -> float:
    """Derivative-filter time constant the implemented controller uses (0 when there is no D term)."""
    return cfg.d_filter_tau if gains_tuple[2] != 0 else 0.0


def _bode_cases(gains: dict, cfg=DEFAULT):
    """(label, gains, tau_f, line style): each controller as implemented (filtered D where Kd != 0),
    then any D controller again with an ideal, unfiltered D, dashed and labelled as theory."""
    cases = []
    for name, g in gains.items():
        tau_f = implemented_tau_f(g, cfg)
        label = f"{name}, filtered D τf = {tau_f:g} s (implemented)" if tau_f > 0 else name
        cases.append((label, g, tau_f, "-"))
    for name, g in gains.items():
        if g[2] != 0:
            cases.append((f"{name}, ideal D (theory only)", g, 0.0, "--"))
    return cases


def plot_bode(gains: dict, cfg=DEFAULT, path="results/figures/bode.png") -> Path:
    """Magnitude (dB) and phase (deg) of L(jw) for each controller as implemented (solid) and the
    ideal-D PID (dashed, theory only), margins in the legend."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(9, 7.5), sharex=True)
    for name, (kp, ki, kd), tau_f, style in _bode_cases(gains, cfg):
        num, den = loop_tf(kp, ki, kd, cfg.v, cfg.d, tau_f=tau_f)
        m = margins(num, den)
        mag, phase = _freq_response(num, den, _W_PLOT)
        label = (f"{name} ({kp:g}, {ki:g}, {kd:g}): GM {_fmt_margin_db(m)}, "
                 f"PM {_fmt_margin_deg(m)}")
        (line,) = ax_m.semilogx(_W_PLOT, 20 * np.log10(mag), style, label=label)
        ax_p.semilogx(_W_PLOT, phase, style, color=line.get_color())
        if math.isfinite(m["w_gc"]):
            ph_gc = float(np.interp(np.log10(m["w_gc"]), np.log10(_W_PLOT), phase))
            ax_m.plot(m["w_gc"], 0.0, "o", color=line.get_color())
            ax_p.plot(m["w_gc"], ph_gc, "o", color=line.get_color())
    ax_m.axhline(0.0, color="0.4", lw=0.8, ls="--")
    ax_p.axhline(-180.0, color="0.4", lw=0.8, ls="--")
    ax_m.set_ylabel("|L(jω)| [dB]")
    ax_p.set_ylabel("phase L(jω) [deg]")
    ax_p.set_xlabel("ω [rad/s]")
    ax_m.set_title(f"Loop L(s) = C(s)·(v + d s)/s²,  v = {cfg.v} m/s,  d = {cfg.d} m  "
                   "(dots: gain crossover)")
    ax_m.legend(fontsize=7.5, loc="lower left")
    for ax in (ax_m, ax_p):
        ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_model_vs_sim(sim_runs: dict, gains: dict, cfg=DEFAULT,
                      path="results/figures/model_vs_sim.png") -> Path:
    """Overlay the linear-model e(t) (from the run's first e_true) on the simulated e_true, in mm."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(len(sim_runs), 1, figsize=(9, 2.8 * len(sim_runs)),
                             sharex=True, squeeze=False)
    for ax, (name, df) in zip(axes[:, 0], sim_runs.items()):
        kp, ki, kd = gains[name]
        t = df["t"].to_numpy(dtype=float)
        t = t - t[0]
        e_sim = df["e_true"].to_numpy(dtype=float)
        e_model = simulate_offset_response(kp, ki, kd, cfg.v, cfg.d, e_sim[0], t,
                                           omega_max=cfg.omega_max,
                                           tau_f=implemented_tau_f(gains[name], cfg))
        ax.plot(t, 1e3 * e_sim, label="simulation e_true")
        ax.plot(t, 1e3 * e_model, "--", label="linear model e(t), implemented C(s)")
        ax.axhspan(-1e3 * cfg.settle_band, 1e3 * cfg.settle_band, color="0.85", zorder=0)
        ax.axhline(0.0, color="0.4", lw=0.6)
        ax.set_ylabel("e [mm]")
        ax.set_title(f"{name} ({kp:g}, {ki:g}, {kd:g})", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle(f"Model vs simulation, straight track (v = {cfg.v} m/s, d = {cfg.d} m, "
                 f"model uses each controller as implemented, PID with D filter τf = {cfg.d_filter_tau} s, "
                 f"clips |ω| ≤ {cfg.omega_max} rad/s;\ngrey band ±{1e3 * cfg.settle_band:g} mm)",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


# ---------------------------------------------------------------- report

def margins_table(gains: dict, cfg=DEFAULT) -> str:
    """Markdown table for d = cfg.d and d = 0: every controller as implemented, then the ideal-D
    (unfiltered) PID as a theory-only reference row."""
    cases = [(label, g, tau_f) for label, g, tau_f, _ in _bode_cases(gains, cfg)]
    lines = [
        "| Controller | d [m] | (Kp, Ki, Kd) | Closed-loop poles | Stable | ζ (oscillatory pair) "
        "| Gain margin | Phase margin |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for d in (cfg.d, 0.0):
        for name, (kp, ki, kd), tau_f in cases:
            poles = closed_loop_poles(kp, ki, kd, cfg.v, d, tau_f=tau_f)
            m = margins(*loop_tf(kp, ki, kd, cfg.v, d, tau_f=tau_f))
            z = _damping(poles)
            z_txt = "—" if z is None else f"{z:.3f}"
            lines.append(f"| {name} | {d:g} | ({kp:g}, {ki:g}, {kd:g}) | {_fmt_poles(poles)} | "
                         f"{_stable_text(poles)} | {z_txt} | {_fmt_margin_db(m)} | {_fmt_margin_deg(m)} |")
    return "\n".join(lines)


def write_margins_md(gains: dict, cfg=DEFAULT, path="results/model_margins.md") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = f"""# Linear-model stability and margins

Generated by `python -m analysis.bode_analysis` from `common/config.py`
(v = {cfg.v} m/s, d = {cfg.d} m). Model and assumptions: `docs/MODEL.md`.

Loop: L(s) = C(s)·(v + d s)/s². Two forms of C(s) appear:

- **implemented** (what `controller/pid.py` runs): C(s) = Kp + Ki/s + Kd s/(τf s + 1) with
  τf = {cfg.d_filter_tau} s; the controller's low-pass is the backward-Euler discretisation of this
  filter. P and PI have no D term, so the filter does not apply to them.
- **ideal D, theory only**: C(s) = Kp + Ki/s + Kd s, the textbook PID, kept as a reference to show
  what the filter changes (one extra fast pole, slightly more damping).

Closed-loop poles are the roots of den(L) + num(L). "Marginal" = poles on the imaginary axis.

{margins_table(gains, cfg)}

How to read it:

- **Gain margin**: how many dB the loop gain can change before the closed loop reaches the
  stability boundary, measured where the phase crosses −180°. "inf" means the phase never
  crosses −180°. A **negative** gain margin appears for loops whose phase starts below −180° at
  low frequency (PI, PID: integrator + two plant integrators give −270°) and rises through −180°:
  such loops are *conditionally stable*, and the number says how much the gain may be
  **reduced** before instability. It is not a sign of instability on its own; the poles column is
  the definitive check. The converse also holds: PI at d = 0 shows "inf" only because its
  phase approaches −180° from below without reaching it, yet that loop is unstable.
- **ζ** is −Re/|p| of the least-damped complex pair; a negative value means a growing oscillation.
- **Phase margin**: 180° plus the phase where |L| = 1. Positive = stable for these loops,
  negative = unstable, zero = marginal.
- These are properties of the idealised linear model, not measurements of the PyBullet robot.
"""
    path.write_text(text, encoding="utf-8")
    return path


def main() -> None:
    from common.schema import read_run

    fig_dir = ROOT / "results" / "figures"
    bode = plot_bode(GAINS, DEFAULT, fig_dir / "bode.png")
    print(f"wrote {bode}")
    md = write_margins_md(GAINS, DEFAULT, ROOT / "results" / "model_margins.md")
    print(f"wrote {md}")

    run_paths = {m: ROOT / "results" / "runs" / f"straight_{m}.csv" for m in GAINS}
    missing = [str(p) for p in run_paths.values() if not p.exists()]
    if missing:
        print("skipped model_vs_sim.png: missing " + ", ".join(missing))
        return
    runs = {m: read_run(p) for m, p in run_paths.items()}
    out = plot_model_vs_sim(runs, GAINS, DEFAULT, fig_dir / "model_vs_sim.png")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
