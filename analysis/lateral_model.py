"""Linearised lateral-error model of the line-following AGV (derivation in docs/MODEL.md).

States near a straight line, small heading error theta:
    de/dt     = v * theta
    dtheta/dt = omega                 (commanded yaw rate, achieved instantly)
    e_s       = e + d * theta         (what the sensor bar, d ahead of the axle, reads)

Plant  E_s(s) / Omega(s) = (v + d s) / s^2
Control omega = -C(s) e_s,  C(s) = Kp + Ki/s + Kd s / (tau_f s + 1)
Optional actuator lag 1/(tau s + 1) on omega.

All polynomials are numpy coefficient arrays in descending powers of s.
"""
import numpy as np
from scipy.integrate import solve_ivp

_STABLE_TOL = 1e-9


def _trim(p) -> np.ndarray:
    p = np.atleast_1d(np.asarray(p, dtype=float))
    nz = np.flatnonzero(p)
    return p[nz[0]:] if nz.size else np.array([0.0])


def controller_tf(kp, ki, kd, tau_f=0.0) -> tuple[np.ndarray, np.ndarray]:
    """(num, den) of C(s) = Kp + Ki/s + Kd s/(tau_f s + 1).

    The 1/s factor appears only when Ki != 0 and the filter factor only when Kd != 0 and
    tau_f > 0, so no common factor (and no fake pole) is ever introduced.
    """
    num = np.array([kd, kp]) if kd != 0 else np.array([kp], dtype=float)   # Kp + Kd s
    den = np.array([1.0])
    if kd != 0 and tau_f > 0:
        # Kp + Kd s/(tau_f s + 1) = (Kp (tau_f s + 1) + Kd s) / (tau_f s + 1)
        num = np.polyadd(kp * np.array([tau_f, 1.0]), [kd, 0.0])
        den = np.array([tau_f, 1.0])
    if ki != 0:
        # (...) + Ki/s = ((...) s + Ki den) / (s den)
        num = np.polyadd(np.polymul(num, [1.0, 0.0]), ki * den)
        den = np.polymul(den, [1.0, 0.0])
    return _trim(num), _trim(den)


def loop_tf(kp, ki, kd, v, d, tau=0.0, tau_f=0.0) -> tuple[np.ndarray, np.ndarray]:
    """(num, den) of the loop L(s) = C(s) (v + d s)/s^2 [ 1/(tau s + 1) ]."""
    c_num, c_den = controller_tf(kp, ki, kd, tau_f)
    num = np.polymul(c_num, [d, v])
    den = np.polymul(c_den, [1.0, 0.0, 0.0])
    if tau > 0:
        den = np.polymul(den, [tau, 1.0])
    return _trim(num), _trim(den)


def closed_loop_poles(kp, ki, kd, v, d, tau=0.0, tau_f=0.0) -> np.ndarray:
    """Roots of the characteristic polynomial den(L) + num(L)."""
    num, den = loop_tf(kp, ki, kd, v, d, tau, tau_f)
    return np.roots(np.polyadd(den, num))


def is_stable(kp, ki, kd, v, d, tau=0.0, tau_f=0.0) -> bool:
    """True when every closed-loop pole has real part < -1e-9 (marginal counts as not stable)."""
    return bool(np.all(closed_loop_poles(kp, ki, kd, v, d, tau, tau_f).real < -_STABLE_TOL))


def simulate_offset_response(kp, ki, kd, v, d, e0, t, omega_max=None, tau_f=0.0) -> np.ndarray:
    """Axle error e(t) of the linear model released from e(0) = e0, theta(0) = 0, z(0) = 0.

    z is the integral of e_s. Two forms of the D term:

    * tau_f = 0 (ideal derivative, theory only). States [e, theta, z]. de_s/dt = v theta + d omega
      depends on omega, so the D term is solved algebraically:
          omega = -(Kp e_s + Ki z + Kd v theta) / (1 + Kd d)
    * tau_f > 0 (the implemented controller, Kd s/(tau_f s + 1)). States [e, theta, z, f] with f the
      low-passed derivative of e_s, tau_f df/dt = de_s/dt - f, f(0) = 0 (the controller resets its
      filter to 0 and e_prev to e0, so it starts with no derivative kick):
          omega = -(Kp e_s + Ki z + Kd f)

    omega is clipped to +-omega_max when given (this is the only non-linearity). The model has no
    anti-windup; it does not need it while |omega| stays below the clip.
    """
    t = np.asarray(t, dtype=float)
    filtered = kd != 0 and tau_f > 0

    def clip(omega):
        if omega_max is None:
            return omega
        return min(max(omega, -omega_max), omega_max)

    def rhs(_, x):
        if filtered:
            e, theta, z, f = x
            e_s = e + d * theta
            omega = clip(-(kp * e_s + ki * z + kd * f))
            de_s = v * theta + d * omega
            return [v * theta, omega, e_s, (de_s - f) / tau_f]
        e, theta, z = x
        e_s = e + d * theta
        omega = clip(-(kp * e_s + ki * z + kd * v * theta) / (1.0 + kd * d))
        return [v * theta, omega, e_s]

    x0 = [float(e0), 0.0, 0.0] + ([0.0] if filtered else [])
    sol = solve_ivp(rhs, (t[0], t[-1]), x0, t_eval=t, rtol=1e-8, atol=1e-11)
    if not sol.success:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")
    return sol.y[0]
