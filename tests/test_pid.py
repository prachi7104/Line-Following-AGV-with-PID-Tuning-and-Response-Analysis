"""Tests for controller.pid: PID maths, anti-windup and the line-following wrapper."""
import pytest

from common.config import DEFAULT, GAINS
from controller.pid import PID, LineFollowController, make_controller


def test_p_only():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, dt=0.01, limit=10.0, d_tau=0.05)
    pid.reset(0.0)
    assert pid.update(0.1) == pytest.approx(0.2)


def test_integral_accumulates():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, dt=0.1, limit=100.0, d_tau=0.05)
    pid.reset(1.0)
    out = None
    for _ in range(10):
        out = pid.update(1.0)
    assert out == pytest.approx(1.0)


def test_no_derivative_kick():
    pid = PID(kp=0.0, ki=0.0, kd=1.0, dt=0.01, limit=100.0, d_tau=0.05)
    pid.reset(0.05)
    assert pid.update(0.05) == pytest.approx(0.0, abs=1e-12)


def test_derivative_is_filtered():
    dt, tau = 0.01, 0.05
    pid = PID(kp=0.0, ki=0.0, kd=1.0, dt=dt, limit=100.0, d_tau=tau)
    pid.reset(0.0)
    # Unit step in e: raw derivative 1/dt, low-passed by alpha = dt/(tau+dt).
    out = pid.update(1.0)
    assert out == pytest.approx((dt / (tau + dt)) * (1.0 / dt))


def test_saturation():
    pid = PID(kp=1000.0, ki=0.0, kd=0.0, dt=0.01, limit=3.0, d_tau=0.05)
    pid.reset(0.0)
    assert pid.update(1.0) == pytest.approx(3.0)
    assert pid.update(-1.0) == pytest.approx(-3.0)
    for e in (0.5, -0.2, 0.001, 10.0):
        assert abs(pid.update(e)) <= 3.0


def test_antiwindup():
    limit = 1.0
    pid = PID(kp=1.0, ki=10.0, kd=0.0, dt=0.01, limit=limit, d_tau=0.05)
    pid.reset(0.5)
    for _ in range(100):
        out = pid.update(0.5)
    assert out == pytest.approx(limit)  # driven into saturation
    steps = None
    for k in range(1, 50):
        out = pid.update(-0.5)
        if abs(out) < limit:
            steps = k
            break
    assert steps is not None and steps <= 5


def test_controller_sign():
    ctrl = make_controller("P")
    ctrl.reset(0.05)
    out = ctrl.step(0.05)
    assert out.u < 0  # robot left of the line must turn right
    assert out.line_detected is True
    assert out.e_used == pytest.approx(0.05)


def test_controller_uses_config_limit():
    ctrl = LineFollowController("PID", DEFAULT)
    ctrl.reset(1.0)
    out = ctrl.step(1.0)
    assert out.u == pytest.approx(-DEFAULT.omega_max)


def test_no_hit_holds_last_error():
    ctrl = make_controller("PID", DEFAULT)
    ctrl.reset(0.0)
    ctrl.step(0.02)
    out = ctrl.step(None)
    assert out.e_used == 0.02
    assert out.line_detected is False


def test_no_hit_after_reset_uses_e0():
    ctrl = make_controller("PI")
    ctrl.reset(0.04)
    out = ctrl.step(None)
    assert out.e_used == 0.04
    assert out.line_detected is False
    assert out.u < 0


def test_unknown_mode():
    with pytest.raises(ValueError):
        make_controller("PD")


def test_all_modes_build():
    for mode in GAINS:
        assert isinstance(make_controller(mode), LineFollowController)
