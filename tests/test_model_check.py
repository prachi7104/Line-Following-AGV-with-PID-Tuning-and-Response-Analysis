"""Tests for the independent frequency-domain cross-check (analysis.model_check).

Focuses on independent complex evaluation of C(jw)*G(jw), polynomial-vs-analytic mismatch
detection, CLI exit codes, and stability classification across default d and d=0 cases
(with implemented filtered D kept separate from the theory-only ideal D reference).
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from analysis.lateral_model import loop_tf
from analysis.model_check import (
    DEFAULT_ATOL,
    DEFAULT_FREQUENCIES,
    DEFAULT_RTOL,
    ModelCase,
    check_case,
    classify_stability,
    controller_eval,
    default_cases,
    format_report,
    loop_eval_analytic,
    loop_eval_poly,
    main,
    plant_eval,
    run_checks,
)
from common.config import DEFAULT, GAINS


# ---------------------------------------------------------------- independent complex evaluation

def test_plant_eval_matches_closed_form_real_and_imag():
    """G(jw) = (v + j*d*w) / (-w^2) = -v/w^2 - j*(d/w)."""
    w = np.array([0.5, 2.0, 3.0, 10.0])
    v, d = DEFAULT.v, DEFAULT.d
    got = plant_eval(w, v, d)
    expected = (-v / (w ** 2)) + 1j * (-d / w)
    np.testing.assert_allclose(got, expected, rtol=1e-14, atol=1e-15)


def test_plant_eval_zero_lookahead_is_purely_real_negative():
    """At d = 0, G(jw) = -v/w^2 has exact phase -180 deg and zero imaginary part."""
    w = np.array([0.2, 1.0, 5.0, 25.0])
    got = plant_eval(w, DEFAULT.v, 0.0)
    np.testing.assert_allclose(got.real, -DEFAULT.v / (w ** 2), rtol=1e-14, atol=1e-15)
    np.testing.assert_allclose(got.imag, 0.0, atol=1e-15)


def test_controller_eval_filtered_vs_ideal_derivative():
    """Verify exact real/imaginary parts of filtered D vs ideal D at the corner w = 1/tau_f."""
    kp, ki, kd = GAINS["PID"]
    tau_f = DEFAULT.d_filter_tau
    w_corner = 1.0 / tau_f  # 20 rad/s, where jw*tau_f = j so jw/(1+j) = w*(1+j)/2

    c_filt = controller_eval([w_corner], kp, ki, kd, tau_f=tau_f)[0]
    expected_filt = kp + (-1j * ki / w_corner) + kd * w_corner * (0.5 + 0.5j)
    assert c_filt == pytest.approx(expected_filt, rel=1e-14, abs=1e-14)

    c_ideal = controller_eval([w_corner], kp, ki, kd, tau_f=0.0)[0]
    expected_ideal = kp + 1j * (kd * w_corner - ki / w_corner)
    assert c_ideal == pytest.approx(expected_ideal, rel=1e-14, abs=1e-14)

    # Filtered D must differ materially from ideal D at the filter corner frequency
    assert abs(c_filt - c_ideal) > 10.0


def test_loop_eval_analytic_matches_loop_tf_with_optional_actuator_lag():
    """Cross-check also works when optional first-order actuator lag tau > 0 is enabled."""
    case = ModelCase(
        label="PID with actuator lag",
        controller="PID",
        gains=GAINS["PID"],
        v=DEFAULT.v,
        d=DEFAULT.d,
        tau_f=DEFAULT.d_filter_tau,
        tau=0.02,
    )
    res = check_case(case)
    assert res.passed is True
    assert res.max_rel_err < DEFAULT_RTOL
    assert res.max_abs_err < DEFAULT_ATOL


@pytest.mark.parametrize("bad_freqs", [[], [0.0, 1.0], [-1.0, 2.0], [1.0, math.nan], [1.0, math.inf]])
def test_invalid_frequencies_rejected(bad_freqs):
    with pytest.raises(ValueError):
        plant_eval(bad_freqs, DEFAULT.v, DEFAULT.d)


def test_negative_time_constants_rejected():
    with pytest.raises(ValueError):
        controller_eval([1.0], 20.0, 5.0, 2.0, tau_f=-0.01)
    with pytest.raises(ValueError):
        loop_eval_analytic([1.0], 20.0, 5.0, 2.0, DEFAULT.v, DEFAULT.d, tau=-0.01)


# ---------------------------------------------------------------- default cases & cross-check

def test_default_cases_cover_both_lookaheads_and_separate_ideal_d():
    cases = default_cases()
    assert len(cases) == 8  # (P, PI, PID-filtered, PID-ideal) x (d=0.10, d=0)
    ds = {c.d for c in cases}
    assert ds == {DEFAULT.d, 0.0}

    implemented = [c for c in cases if not c.is_theory_reference]
    theory = [c for c in cases if c.is_theory_reference]
    assert len(implemented) == 6
    assert len(theory) == 2
    assert all(c.controller == "PID" and c.tau_f == 0.0 and "theory" in c.label.lower() for c in theory)

    pid_impl = [c for c in implemented if c.controller == "PID"]
    assert len(pid_impl) == 2
    assert all(c.tau_f == DEFAULT.d_filter_tau and "implemented" in c.label.lower() for c in pid_impl)


def test_all_default_cases_pass_independent_frequency_crosscheck():
    results = run_checks()
    assert len(results) == 8
    for res in results:
        assert res.passed is True, res.failure_reason
        assert res.max_rel_err < DEFAULT_RTOL
        assert res.max_abs_err < DEFAULT_ATOL
        assert res.max_mag_db_err < 1e-10
        assert res.max_phase_err_deg < 1e-10
        assert res.frequencies == DEFAULT_FREQUENCIES


# ---------------------------------------------------------------- stability classification

@pytest.mark.parametrize(
    ("poles", "expected"),
    [
        ([-1.0 + 2.0j, -1.0 - 2.0j], "stable"),
        ([-0.25, -0.99 + 2.0j, -0.99 - 2.0j, -23.8], "stable"),
        ([0.0 + 2.449j, 0.0 - 2.449j], "marginal"),
        ([1e-12 + 2.449j, 1e-12 - 2.449j], "marginal"),
        ([-0.247, 0.124 + 2.459j, 0.124 - 2.459j], "unstable"),
        ([0.05], "unstable"),
    ],
)
def test_classify_stability_representative_cases(poles, expected):
    assert classify_stability(poles) == expected


def test_default_cases_stability_classification_matches_theory():
    results = run_checks()
    by_key = {(r.case.controller, r.case.d, r.case.is_theory_reference): r for r in results}

    # d = 0.10 m: all four cases stable
    for ctrl, theory in [("P", False), ("PI", False), ("PID", False), ("PID", True)]:
        res = by_key[(ctrl, DEFAULT.d, theory)]
        assert res.stability == "stable"
        assert res.is_stable_flag is True
        assert res.max_pole_real < -0.2

    # d = 0 m: P marginal, PI unstable, filtered & ideal PID stable
    p0 = by_key[("P", 0.0, False)]
    assert p0.stability == "marginal"
    assert p0.is_stable_flag is False
    assert abs(p0.max_pole_real) < 1e-9

    pi0 = by_key[("PI", 0.0, False)]
    assert pi0.stability == "unstable"
    assert pi0.is_stable_flag is False
    assert pi0.max_pole_real > 0.1

    pid0_impl = by_key[("PID", 0.0, False)]
    assert pid0_impl.stability == "stable"
    assert pid0_impl.is_stable_flag is True
    assert len(pid0_impl.pole_real_parts) == 4  # extra fast filter pole

    pid0_ideal = by_key[("PID", 0.0, True)]
    assert pid0_ideal.stability == "stable"
    assert pid0_ideal.is_stable_flag is True
    assert len(pid0_ideal.pole_real_parts) == 3


# ---------------------------------------------------------------- mismatch detection

def test_mismatch_detected_for_perturbed_gain():
    case = default_cases()[0]  # P, d=0.10

    def bad_tf(kp, ki, kd, v, d, tau=0.0, tau_f=0.0):
        num, den = loop_tf(kp * (1.0 + 1e-8), ki, kd, v, d, tau=tau, tau_f=tau_f)
        return num, den

    res = check_case(case, tf_provider=bad_tf)
    assert res.passed is False
    assert res.max_rel_err == pytest.approx(1e-8, rel=1e-2)
    assert res.failure_reason is not None and "discrepancy" in res.failure_reason


def test_mismatch_detected_when_filter_dropped_on_implemented_pid():
    pid_impl = next(
        c for c in default_cases() if c.controller == "PID" and c.d == DEFAULT.d and not c.is_theory_reference
    )

    def unfiltered_tf(kp, ki, kd, v, d, tau=0.0, tau_f=0.0):
        # Buggy implementation that ignores tau_f
        return loop_tf(kp, ki, kd, v, d, tau=tau, tau_f=0.0)

    res = check_case(pid_impl, tf_provider=unfiltered_tf)
    assert res.passed is False
    assert res.max_rel_err > 0.1


def test_mismatch_detected_for_sign_inversion():
    case = default_cases()[1]  # PI, d=0.10

    def negated_tf(kp, ki, kd, v, d, tau=0.0, tau_f=0.0):
        num, den = loop_tf(kp, ki, kd, v, d, tau=tau, tau_f=tau_f)
        return -num, den

    res = check_case(case, tf_provider=negated_tf)
    assert res.passed is False
    assert res.max_rel_err == pytest.approx(2.0, rel=1e-9)
    assert res.max_phase_err_deg == pytest.approx(180.0, abs=1e-9)


def test_loop_eval_poly_rejects_zero_denominator():
    # Denominator s^2 + 1 vanishes at w = 1
    with pytest.raises(ZeroDivisionError):
        loop_eval_poly([1.0], np.array([1.0]), np.array([1.0, 0.0, 1.0]))


# ---------------------------------------------------------------- CLI behaviour

def test_cli_returns_zero_and_prints_both_categories(capsys):
    rc = main([])
    out = capsys.readouterr().out
    assert rc == 0
    assert "8/8 cases passed" in out
    assert "theory reference" in out
    assert "implemented" in out
    assert "marginal" in out
    assert "unstable" in out


def test_cli_returns_nonzero_on_discrepancy(monkeypatch, capsys):
    def broken_loop_tf(kp, ki, kd, v, d, tau=0.0, tau_f=0.0):
        num, den = loop_tf(kp, ki, kd, v, d, tau=tau, tau_f=tau_f)
        return num * 1.01, den

    monkeypatch.setattr("analysis.model_check.loop_tf", broken_loop_tf)
    rc = main([])
    out = capsys.readouterr().out
    assert rc == 1
    assert "FAIL" in out


def test_format_report_includes_failure_details():
    case = default_cases()[0]
    res = check_case(
        case,
        tf_provider=lambda kp, ki, kd, v, d, tau=0.0, tau_f=0.0: (np.array([1.0]), np.array([1.0])),
    )
    assert res.passed is False
    assert res.stability == "undefined"
    report = format_report([res])
    assert "0/1 cases passed" in report
    assert "FAIL" in report
    assert "n/a" in report
