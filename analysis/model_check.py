"""Read-only independent frequency-domain cross-check for the linearised lateral model.

CLI:
    python -m analysis.model_check

Independently evaluates C(j*w) * G(j*w) in complex arithmetic at fixed positive frequencies
without calling ``controller_tf`` or ``loop_tf``, then compares against polynomial evaluation
of ``analysis.lateral_model.loop_tf`` and reports maximum discrepancies together with
closed-loop pole real parts and stability classifications.

Covers both default look-ahead (d = 0.10 m) and zero look-ahead (d = 0 m), keeping the
implemented filtered-D PID separate from the ideal-D theory reference. Exits with a nonzero
status if any discrepancy exceeds the numerical tolerance or if stability flags disagree.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
from typing import Callable, Iterable, Sequence

import numpy as np

from analysis.lateral_model import closed_loop_poles, is_stable, loop_tf
from common.config import DEFAULT, GAINS, SimConfig

# Fixed positive test frequencies [rad/s] spanning low-frequency integrator asymptote,
# phase crossover (~0.76-0.87), natural frequency (~2.45), gain crossover (~2.9-3.0),
# look-ahead zero (v/d = 3.0), derivative filter corner (1/tau_f = 20.0), and high-frequency roll-off.
DEFAULT_FREQUENCIES: tuple[float, ...] = (
    0.1,
    0.25,
    0.5,
    0.75,
    1.0,
    2.0,
    2.5,
    3.0,
    5.0,
    10.0,
    20.0,
    50.0,
    100.0,
)

# Tight IEEE-754 float64 tolerances: direct complex evaluation vs Horner polynomial
# evaluation of degree <= 4 rational functions agrees to ~1e-14 relative error.
# 1e-12 leaves ~2 orders of magnitude headroom for rounding while catching any algebraic,
# sign, filter, or gain discrepancy.
DEFAULT_RTOL: float = 1e-12
DEFAULT_ATOL: float = 1e-12
STABILITY_TOL: float = 1e-9

TFProvider = Callable[..., tuple[np.ndarray, np.ndarray]]


@dataclass(frozen=True)
class ModelCase:
    """Single linear-model configuration to cross-check."""

    label: str
    controller: str
    gains: tuple[float, float, float]
    v: float
    d: float
    tau_f: float = 0.0
    tau: float = 0.0
    is_theory_reference: bool = False


@dataclass(frozen=True)
class CheckResult:
    """Result of cross-checking one linear-model case."""

    case: ModelCase
    frequencies: tuple[float, ...]
    max_abs_err: float
    max_rel_err: float
    max_mag_db_err: float
    max_phase_err_deg: float
    poles: tuple[complex, ...]
    pole_real_parts: tuple[float, ...]
    max_pole_real: float
    stability: str
    is_stable_flag: bool
    passed: bool
    failure_reason: str | None = None


def _validate_frequencies(w: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(w), dtype=float)
    if arr.ndim != 1 or arr.size == 0:
        raise ValueError("frequencies must be a non-empty 1-D sequence of positive numbers")
    if not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
        raise ValueError("all frequencies must be finite and strictly positive (w > 0)")
    return arr


def plant_eval(w: Iterable[float], v: float, d: float) -> np.ndarray:
    """Independently evaluate G(j*w) = (v + d*(j*w)) / (j*w)^2 at w > 0."""
    omega = _validate_frequencies(w)
    jw = 1j * omega
    return (v + d * jw) / (jw * jw)


def controller_eval(
    w: Iterable[float],
    kp: float,
    ki: float,
    kd: float,
    tau_f: float = 0.0,
) -> np.ndarray:
    """Independently evaluate C(j*w) = Kp + Ki/(j*w) + Kd*(j*w)/(tau_f*(j*w) + 1)."""
    if tau_f < 0.0:
        raise ValueError(f"tau_f must be non-negative, got {tau_f}")
    omega = _validate_frequencies(w)
    jw = 1j * omega
    c_val = np.full(omega.shape, complex(kp), dtype=complex)
    if ki != 0.0:
        c_val = c_val + ki / jw
    if kd != 0.0:
        d_term = kd * jw
        if tau_f > 0.0:
            d_term = d_term / (tau_f * jw + 1.0)
        c_val = c_val + d_term
    return c_val


def loop_eval_analytic(
    w: Iterable[float],
    kp: float,
    ki: float,
    kd: float,
    v: float,
    d: float,
    tau: float = 0.0,
    tau_f: float = 0.0,
) -> np.ndarray:
    """Independently evaluate L(j*w) = C(j*w) * G(j*w) [/ (tau*(j*w) + 1)]."""
    if tau < 0.0:
        raise ValueError(f"tau must be non-negative, got {tau}")
    omega = _validate_frequencies(w)
    l_val = controller_eval(omega, kp, ki, kd, tau_f=tau_f) * plant_eval(omega, v, d)
    if tau > 0.0:
        l_val = l_val / (tau * (1j * omega) + 1.0)
    return l_val


def loop_eval_poly(w: Iterable[float], num: np.ndarray, den: np.ndarray) -> np.ndarray:
    """Evaluate a polynomial transfer function num(j*w) / den(j*w)."""
    omega = _validate_frequencies(w)
    jw = 1j * omega
    den_val = np.polyval(den, jw)
    if np.any(den_val == 0.0):
        raise ZeroDivisionError("denominator polynomial evaluated to zero on frequency grid")
    return np.polyval(num, jw) / den_val


def classify_stability(poles: Sequence[complex], tol: float = STABILITY_TOL) -> str:
    """Classify closed-loop stability from pole real parts.

    Returns:
        "stable"   when max Re(p) < -tol
        "marginal" when |max Re(p)| <= tol
        "unstable" when max Re(p) > tol
    """
    arr = np.asarray(poles, dtype=complex)
    if arr.size == 0:
        raise ValueError("poles must be non-empty")
    max_re = float(np.max(arr.real))
    if max_re < -tol:
        return "stable"
    if abs(max_re) <= tol:
        return "marginal"
    return "unstable"


def default_cases(
    cfg: SimConfig = DEFAULT,
    gains: dict[str, tuple[float, float, float]] = GAINS,
) -> list[ModelCase]:
    """Build the default d and d=0 cases, separating implemented filtered D from ideal D."""
    cases: list[ModelCase] = []
    for d in (cfg.d, 0.0):
        for name, (kp, ki, kd) in gains.items():
            tau_f = cfg.d_filter_tau if kd != 0.0 else 0.0
            label = (
                f"{name}, filtered D tau_f={tau_f:g} s (implemented)"
                if tau_f > 0.0
                else f"{name} (implemented)"
            )
            cases.append(
                ModelCase(
                    label=label,
                    controller=name,
                    gains=(kp, ki, kd),
                    v=cfg.v,
                    d=d,
                    tau_f=tau_f,
                    tau=0.0,
                    is_theory_reference=False,
                )
            )
        for name, (kp, ki, kd) in gains.items():
            if kd != 0.0:
                cases.append(
                    ModelCase(
                        label=f"{name}, ideal D (theory reference)",
                        controller=name,
                        gains=(kp, ki, kd),
                        v=cfg.v,
                        d=d,
                        tau_f=0.0,
                        tau=0.0,
                        is_theory_reference=True,
                    )
                )
    return cases


def _wrapped_phase_diff_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest angular difference between complex arrays a and b in degrees."""
    diff = np.angle(a) - np.angle(b)
    return np.abs(np.degrees((diff + np.pi) % (2.0 * np.pi) - np.pi))


def check_case(
    case: ModelCase,
    frequencies: Iterable[float] = DEFAULT_FREQUENCIES,
    rtol: float = DEFAULT_RTOL,
    atol: float = DEFAULT_ATOL,
    tf_provider: TFProvider | None = None,
) -> CheckResult:
    """Cross-check one model case against ``loop_tf`` (or a custom ``tf_provider``)."""
    omega = _validate_frequencies(frequencies)
    kp, ki, kd = case.gains
    provider = loop_tf if tf_provider is None else tf_provider

    analytic = loop_eval_analytic(
        omega, kp, ki, kd, case.v, case.d, tau=case.tau, tau_f=case.tau_f
    )
    num, den = provider(kp, ki, kd, case.v, case.d, tau=case.tau, tau_f=case.tau_f)
    poly = loop_eval_poly(omega, num, den)

    abs_err = np.abs(poly - analytic)
    rel_err = abs_err / np.maximum(np.abs(analytic), 1e-15)
    max_abs_err = float(np.max(abs_err))
    max_rel_err = float(np.max(rel_err))

    mag_analytic = np.abs(analytic)
    mag_poly = np.abs(poly)
    if np.all(mag_analytic > 0.0) and np.all(mag_poly > 0.0):
        max_mag_db_err = float(np.max(np.abs(20.0 * np.log10(mag_poly / mag_analytic))))
    else:
        max_mag_db_err = math.inf
    max_phase_err_deg = float(np.max(_wrapped_phase_diff_deg(poly, analytic)))

    if tf_provider is None:
        poles_arr = closed_loop_poles(kp, ki, kd, case.v, case.d, tau=case.tau, tau_f=case.tau_f)
        stable_flag = is_stable(kp, ki, kd, case.v, case.d, tau=case.tau, tau_f=case.tau_f)
    else:
        poles_arr = np.roots(np.polyadd(den, num))
        stable_flag = bool(np.all(poles_arr.real < -STABILITY_TOL))

    sorted_poles = tuple(sorted((complex(p) for p in poles_arr), key=lambda z: (z.real, z.imag)))
    real_parts = tuple(float(p.real) for p in sorted_poles)
    if real_parts:
        max_pole_real = max(real_parts)
        stability = classify_stability(sorted_poles)
    else:
        max_pole_real = math.nan
        stability = "undefined"
        stable_flag = False

    freq_ok = bool(np.all(abs_err <= atol + rtol * np.abs(analytic)))
    stability_consistent = bool(real_parts) and (stable_flag == (stability == "stable"))
    passed = freq_ok and stability_consistent

    failure_reason: str | None = None
    if not freq_ok:
        failure_reason = (
            f"frequency-response discrepancy exceeds tolerance "
            f"(max_rel_err={max_rel_err:.3e}, max_abs_err={max_abs_err:.3e}, "
            f"rtol={rtol:.1e}, atol={atol:.1e})"
        )
    elif not real_parts:
        failure_reason = "degenerate characteristic polynomial (no finite closed-loop poles)"
    elif not stability_consistent:
        failure_reason = (
            f"stability classification mismatch: classify_stability={stability!r}, "
            f"is_stable={stable_flag!r}"
        )

    return CheckResult(
        case=case,
        frequencies=tuple(float(x) for x in omega),
        max_abs_err=max_abs_err,
        max_rel_err=max_rel_err,
        max_mag_db_err=max_mag_db_err,
        max_phase_err_deg=max_phase_err_deg,
        poles=sorted_poles,
        pole_real_parts=real_parts,
        max_pole_real=max_pole_real,
        stability=stability,
        is_stable_flag=stable_flag,
        passed=passed,
        failure_reason=failure_reason,
    )


def run_checks(
    cases: Sequence[ModelCase] | None = None,
    frequencies: Iterable[float] = DEFAULT_FREQUENCIES,
    rtol: float = DEFAULT_RTOL,
    atol: float = DEFAULT_ATOL,
    tf_provider: TFProvider | None = None,
) -> list[CheckResult]:
    """Run frequency-domain and pole-stability cross-checks for all specified cases."""
    selected = default_cases() if cases is None else list(cases)
    return [
        check_case(c, frequencies=frequencies, rtol=rtol, atol=atol, tf_provider=tf_provider)
        for c in selected
    ]


def _fmt_real_parts(real_parts: Sequence[float]) -> str:
    formatted = []
    for r in real_parts:
        val = 0.0 if abs(r) < 5e-7 else r
        formatted.append(f"{val:+.4f}")
    return "[" + ", ".join(formatted) + "]"


def format_report(
    results: Sequence[CheckResult],
    rtol: float = DEFAULT_RTOL,
    atol: float = DEFAULT_ATOL,
) -> str:
    """Format a human-readable summary table of all cross-check results."""
    freqs = results[0].frequencies if results else DEFAULT_FREQUENCIES
    lines = [
        "Linear-model independent frequency-domain cross-check",
        f"Frequencies ({len(freqs)} pts) [rad/s]: {', '.join(f'{w:g}' for w in freqs)}",
        f"Tolerances: rtol={rtol:.1e}, atol={atol:.1e} (stability tol={STABILITY_TOL:.1e})",
        "",
        "| Case | Category | d [m] | max |dL| | max rel err | Pole Re(p) | max Re(p) | Stability | Status |",
        "|---|---|---:|---:|---:|---|---:|---|---|",
    ]
    for res in results:
        category = "theory reference" if res.case.is_theory_reference else "implemented"
        status = "PASS" if res.passed else "FAIL"
        max_re_str = (
            "n/a"
            if math.isnan(res.max_pole_real)
            else f"{(0.0 if abs(res.max_pole_real) < 5e-7 else res.max_pole_real):+.4f}"
        )
        lines.append(
            f"| {res.case.label} | {category} | {res.case.d:g} | "
            f"{res.max_abs_err:.2e} | {res.max_rel_err:.2e} | "
            f"{_fmt_real_parts(res.pole_real_parts)} | {max_re_str} | "
            f"{res.stability} | {status} |"
        )
    overall_max_abs = max((r.max_abs_err for r in results), default=0.0)
    overall_max_rel = max((r.max_rel_err for r in results), default=0.0)
    passed_count = sum(1 for r in results if r.passed)
    lines.append("")
    lines.append(
        f"Summary: {passed_count}/{len(results)} cases passed | "
        f"overall max |dL| = {overall_max_abs:.2e}, max rel err = {overall_max_rel:.2e}"
    )
    for res in results:
        if not res.passed and res.failure_reason:
            lines.append(f"FAIL [{res.case.label}, d={res.case.d:g}]: {res.failure_reason}")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read-only frequency-domain cross-check of analysis.lateral_model.loop_tf."
    )
    parser.add_argument(
        "--rtol",
        type=float,
        default=DEFAULT_RTOL,
        help=f"Relative complex discrepancy tolerance (default: {DEFAULT_RTOL:.1e}).",
    )
    parser.add_argument(
        "--atol",
        type=float,
        default=DEFAULT_ATOL,
        help=f"Absolute complex discrepancy tolerance (default: {DEFAULT_ATOL:.1e}).",
    )
    parser.add_argument(
        "--freq",
        type=float,
        nargs="+",
        default=None,
        help="Optional list of positive test frequencies [rad/s].",
    )
    args = parser.parse_args(argv)
    freqs = DEFAULT_FREQUENCIES if args.freq is None else tuple(args.freq)
    results = run_checks(frequencies=freqs, rtol=args.rtol, atol=args.atol)
    print(format_report(results, rtol=args.rtol, atol=args.atol))
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
