"""Tests for the linearised lateral model (analysis.lateral_model) and margins (analysis.bode_analysis)."""
import math

import numpy as np
import pytest
from scipy.signal import find_peaks

from analysis.bode_analysis import margins
from analysis.lateral_model import (
    closed_loop_poles,
    is_stable,
    loop_tf,
    simulate_offset_response,
)

V = 0.30
D = 0.10


def _sorted(p):
    p = np.asarray(p, dtype=complex)
    return p[np.lexsort((p.imag, p.real))]


def _damping(poles):
    """Damping ratio of the least-damped complex pair."""
    pair = [p for p in poles if p.imag > 1e-9]
    return min(-p.real / abs(p) for p in pair)


def test_p_poles_match_formula():
    kp = 20.0
    expected = np.roots([1.0, kp * D, kp * V])
    got = closed_loop_poles(kp, 0.0, 0.0, V, D)
    assert len(got) == 2  # no fake integrator pole when Ki == 0
    np.testing.assert_allclose(_sorted(got), _sorted(expected), atol=1e-9)


def test_p_loop_has_no_pole_from_missing_integrator():
    num, den = loop_tf(20.0, 0.0, 0.0, V, D)
    # L = 20 (0.1 s + 0.3) / s^2: exactly two poles, both at the origin
    assert len(den) - 1 == 2
    np.testing.assert_allclose(np.roots(den), [0.0, 0.0], atol=1e-12)


def test_p_zero_lookahead_marginal():
    poles = closed_loop_poles(20.0, 0.0, 0.0, V, 0.0)
    assert np.max(np.abs(poles.real)) < 1e-9
    np.testing.assert_allclose(np.sort(np.abs(poles.imag)), [math.sqrt(6.0)] * 2, rtol=1e-9)
    assert is_stable(20.0, 0.0, 0.0, V, 0.0) is False


def test_pi_zero_lookahead_unstable():
    assert is_stable(20.0, 5.0, 0.0, V, 0.0) is False
    assert np.max(closed_loop_poles(20.0, 5.0, 0.0, V, 0.0).real) > 0


def test_pid_default_stable():
    assert is_stable(20.0, 5.0, 2.0, V, D) is True


def test_margins_known_tf():
    m = margins(np.array([1.0]), np.array([1.0, 1.0, 0.0]))
    assert m["phase_margin_deg"] == pytest.approx(51.8, abs=0.1)
    assert m["gain_margin_db"] == math.inf
    assert m["w_pc"] == math.inf
    assert m["w_gc"] == pytest.approx(math.sqrt((math.sqrt(5) - 1) / 2), rel=1e-6)


def test_offset_response_decays():
    t = np.linspace(0.0, 10.0, 1001)
    e = simulate_offset_response(20.0, 0.0, 0.0, V, D, 0.05, t)
    assert e.shape == t.shape
    assert e[0] == 0.05
    assert abs(e[-1]) < 0.005


def test_offset_response_matches_poles():
    t = np.linspace(0.0, 10.0, 10001)
    e = simulate_offset_response(20.0, 0.0, 0.0, V, D, 0.05, t)
    peaks, _ = find_peaks(np.abs(e))
    peaks = peaks[np.abs(e[peaks]) > 1e-7]
    assert len(peaks) >= 3
    # decay rate of the peak envelope |e| ~ exp(-sigma t) must equal -Re(dominant pole)
    slope = np.polyfit(t[peaks], np.log(np.abs(e[peaks])), 1)[0]
    sigma = -np.max(closed_loop_poles(20.0, 0.0, 0.0, V, D).real)
    assert -slope == pytest.approx(sigma, rel=0.10)


def test_reproduces_design_table():
    """docs/DESIGN.md section 3, v = 0.30, starting gains."""
    P, PI, PID = (20.0, 0.0, 0.0), (20.0, 5.0, 0.0), (20.0, 5.0, 2.0)

    # P, d = 0: marginal (poles on the imaginary axis), not stable
    p = closed_loop_poles(*P, V, 0.0)
    assert np.max(p.real) == pytest.approx(0.0, abs=1e-9)
    assert not is_stable(*P, V, 0.0)

    # P, d = 0.10: -1.00 +- 2.236j, zeta = 0.408
    p = closed_loop_poles(*P, V, D)
    np.testing.assert_allclose(_sorted(p), _sorted([-1 - math.sqrt(5) * 1j, -1 + math.sqrt(5) * 1j]),
                               atol=1e-9)
    assert _damping(p) == pytest.approx(0.408, abs=0.001)
    assert is_stable(*P, V, D)

    # PI, d = 0: unstable, max Re = +0.124
    p = closed_loop_poles(*PI, V, 0.0)
    assert np.max(p.real) == pytest.approx(0.124, abs=0.001)
    assert not is_stable(*PI, V, 0.0)

    # PI, d = 0.10: stable, -0.88 +- 2.30j (zeta 0.356), -0.247
    p = closed_loop_poles(*PI, V, D)
    assert is_stable(*PI, V, D)
    assert _damping(p) == pytest.approx(0.356, abs=0.001)
    assert np.min(np.abs(p[np.abs(p.imag) < 1e-9].real + 0.247)) < 0.001

    # PID, d = 0: stable but barely, zeta 0.071
    p = closed_loop_poles(*PID, V, 0.0)
    assert is_stable(*PID, V, 0.0)
    assert _damping(p) == pytest.approx(0.071, abs=0.001)

    # PID, d = 0.10: stable, -0.96 +- 2.00j (zeta 0.431), -0.253
    p = closed_loop_poles(*PID, V, D)
    assert is_stable(*PID, V, D)
    assert _damping(p) == pytest.approx(0.431, abs=0.001)
    assert np.min(np.abs(p[np.abs(p.imag) < 1e-9].real + 0.253)) < 0.001

    # PID with derivative filter tau_f = 0.05 s: zeta 0.072 (d = 0), 0.443 and a pole at -23.8 (d = 0.10)
    assert _damping(closed_loop_poles(*PID, V, 0.0, tau_f=0.05)) == pytest.approx(0.072, abs=0.001)
    p = closed_loop_poles(*PID, V, D, tau_f=0.05)
    assert _damping(p) == pytest.approx(0.443, abs=0.001)
    assert np.min(p.real) == pytest.approx(-23.8, abs=0.05)


# ---------------------------------------------------------------- filtered (implemented) PID

def test_filtered_response_decays_at_filtered_pole_rate():
    """The tau_f > 0 time response is the filtered model: its decay matches the filtered poles."""
    t = np.linspace(0.0, 10.0, 10001)
    e = simulate_offset_response(20.0, 0.0, 2.0, V, D, 0.05, t, tau_f=0.05)
    peaks, _ = find_peaks(np.abs(e))
    peaks = peaks[np.abs(e[peaks]) > 1e-7]
    assert len(peaks) >= 3
    slope = np.polyfit(t[peaks], np.log(np.abs(e[peaks])), 1)[0]
    poles = closed_loop_poles(20.0, 0.0, 2.0, V, D, tau_f=0.05)
    sigma = -np.max(poles[np.abs(poles.imag) > 1e-9].real)
    assert -slope == pytest.approx(sigma, rel=0.10)


def test_filtered_response_tends_to_ideal_as_tau_shrinks():
    t = np.linspace(0.0, 8.0, 4001)
    ideal = simulate_offset_response(20.0, 5.0, 2.0, V, D, 0.05, t)
    nearly = simulate_offset_response(20.0, 5.0, 2.0, V, D, 0.05, t, tau_f=1e-4)
    assert np.max(np.abs(nearly - ideal)) < 1e-4


def test_tau_f_ignored_without_derivative():
    t = np.linspace(0.0, 5.0, 501)
    a = simulate_offset_response(20.0, 5.0, 0.0, V, D, 0.05, t)
    b = simulate_offset_response(20.0, 5.0, 0.0, V, D, 0.05, t, tau_f=0.05)
    assert np.array_equal(a, b)


def test_discrete_controller_filter_matches_continuous_filter():
    """controller/pid.py's derivative low-pass is the discretisation of Kd s/(tau_f s + 1):
    for a unit ramp in e, the D output must follow Kd (1 - exp(-t/tau_f))."""
    from controller.pid import PID

    dt, tau = 1.0 / 240.0, 0.05
    pid = PID(kp=0.0, ki=0.0, kd=1.0, dt=dt, limit=1e9, d_tau=tau)
    pid.reset(0.0)
    n = 120
    out = np.array([pid.update((k + 1) * dt) for k in range(n)])
    t = (np.arange(n) + 1) * dt
    assert np.max(np.abs(out - (1.0 - np.exp(-t / tau)))) < 0.05


def test_bode_uses_implemented_filter_and_labels_ideal_as_theory():
    from analysis.bode_analysis import _bode_cases
    from common.config import DEFAULT, GAINS

    cases = {label: (g, tau_f, style) for label, g, tau_f, style in _bode_cases(GAINS, DEFAULT)}
    implemented = [k for k in cases if k.startswith("PID") and "implemented" in k]
    ideal = [k for k in cases if k.startswith("PID") and "theory" in k]
    assert len(implemented) == 1 and len(ideal) == 1
    assert cases[implemented[0]][1] == DEFAULT.d_filter_tau and cases[implemented[0]][2] == "-"
    assert cases[ideal[0]][1] == 0.0 and cases[ideal[0]][2] == "--"
    assert cases["P"][1] == 0.0 and cases["PI"][1] == 0.0
