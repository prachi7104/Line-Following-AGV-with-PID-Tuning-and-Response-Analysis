"""Discrete PID with a filtered derivative and conditional-integration anti-windup, plus the
line-following wrapper that applies the project sign convention u = -PID(e).

Sign convention (common/config.py): e > 0 <=> robot LEFT of the line; u > 0 turns LEFT.
"""
from __future__ import annotations

from dataclasses import dataclass

from common.config import DEFAULT, GAINS, SimConfig


class PID:
    """Parallel-form PID: out = kp*e + ki*I + kd*d_f, clamped to +-limit.

    * Integral: I += e*dt, skipped on a step where the output would saturate and e has the
      same sign as the saturated output (conditional integration).
    * Derivative: raw (e - e_prev)/dt, low-passed with time constant d_tau
      (d_f += alpha*(raw - d_f), alpha = dt/(d_tau + dt)).
    * reset(e0) sets e_prev = e0, so the first update has no derivative kick.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float, limit: float, d_tau: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.d_tau = d_tau
        self.alpha = dt / (d_tau + dt)
        self.reset(0.0)

    def reset(self, e0: float) -> None:
        self.integral = 0.0
        self.e_prev = float(e0)
        self.d_filtered = 0.0

    def _clamp(self, value: float) -> float:
        return max(-self.limit, min(self.limit, value))

    def update(self, e: float) -> float:
        e = float(e)
        raw_d = (e - self.e_prev) / self.dt
        self.d_filtered += self.alpha * (raw_d - self.d_filtered)
        self.e_prev = e

        pd = self.kp * e + self.kd * self.d_filtered
        candidate = self.integral + e * self.dt
        unclamped = pd + self.ki * candidate
        saturated = abs(unclamped) > self.limit
        pushing_further = e * unclamped > 0.0
        if not (saturated and pushing_further):
            self.integral = candidate
        return self._clamp(pd + self.ki * self.integral)


@dataclass
class ControlOutput:
    u: float               # commanded yaw rate [rad/s], u > 0 turns left
    e_used: float          # error fed to the PID this step [m]
    line_detected: bool    # False when the sensor saw no line and the last error was held


class LineFollowController:
    """P / PI / PID line follower with gains from ``common.config.GAINS[mode]``."""

    def __init__(self, mode: str, cfg: SimConfig = DEFAULT):
        if mode not in GAINS:
            raise ValueError(f"unknown controller mode {mode!r}; expected one of {sorted(GAINS)}")
        self.mode = mode
        self.cfg = cfg
        kp, ki, kd = GAINS[mode]
        self.pid = PID(kp, ki, kd, dt=cfg.dt, limit=cfg.omega_max, d_tau=cfg.d_filter_tau)
        self.last_error = 0.0
        self.reset(0.0)

    def reset(self, e0: float) -> None:
        self.pid.reset(e0)
        self.last_error = float(e0)

    def step(self, e_sensed: float | None) -> ControlOutput:
        if e_sensed is None:
            # Line lost: hold the last valid error. Never substitute 0.0.
            e_used = self.last_error
            detected = False
        else:
            e_used = float(e_sensed)
            self.last_error = e_used
            detected = True
        u = -self.pid.update(e_used)
        return ControlOutput(u=u, e_used=e_used, line_detected=detected)


def make_controller(mode: str, cfg: SimConfig = DEFAULT) -> LineFollowController:
    """Build a controller for ``mode`` in GAINS ("P", "PI", "PID"); ValueError otherwise."""
    return LineFollowController(mode, cfg)
