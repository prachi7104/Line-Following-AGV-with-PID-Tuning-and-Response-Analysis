"""Single source of truth for every tunable number in the project.

Units are SI throughout: metres, seconds, radians, rad/s.

Sign convention (used by every module, the plots and the model):
    e > 0  <=>  robot is LEFT of the line, looking along the direction of travel
    u > 0  <=>  robot turns LEFT (counter-clockwise yaw rate)
    control law: u = -PID(e)
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SimConfig:
    # --- simulation ---
    dt: float = 1.0 / 240.0          # physics and control step [s]

    # --- robot geometry and actuation ---
    v: float = 0.30                  # constant forward speed [m/s]
    wheel_radius: float = 0.035      # r [m]
    track_width: float = 0.16        # W, distance between wheel contact points [m]
    omega_max: float = 3.0           # |u| limit on commanded yaw rate [rad/s]
    wheel_force: float = 5.0         # max motor torque in VELOCITY_CONTROL [N m]

    # --- sensor bar (5 analog reflectance sensors) ---
    d: float = 0.10                  # look-ahead: sensor bar ahead of wheel axle [m]
    n_sensors: int = 5
    sensor_spacing: float = 0.03     # centre-to-centre [m]; footprint width = spacing
    sub_rays_per_sensor: int = 10    # rays per sensor -> reading = hit fraction in [0, 1]
    ray_z_top: float = 0.012         # ray start height, below chassis bottom [m]
    ray_z_bottom: float = -0.005     # ray end height, below the floor [m]

    # --- line ---
    line_width: float = 0.03         # >= sensor_spacing so the line never hides between sensors
    line_height: float = 0.0015

    # --- tracks ---
    straight_length: float = 4.0
    curve_lead: float = 1.0
    curve_radius: float = 0.75
    curve_angle: float = float(np.pi)  # left turn

    # --- experiments and metrics ---
    initial_offset: float = 0.05     # 5 cm lateral-reference step [m]
    settle_band: float = 0.0025      # +-2.5 mm
    max_time: float = 14.0           # run timeout [s]
    d_filter_tau: float = 0.05       # derivative low-pass time constant [s]

    def sensor_centres(self) -> np.ndarray:
        """Lateral position of each sensor centre in the robot frame, left positive [m]."""
        half = (self.n_sensors - 1) / 2
        return (np.arange(self.n_sensors) - half) * self.sensor_spacing


DEFAULT = SimConfig()

# (Kp, Ki, Kd) acting on e in metres, producing u in rad/s.
# FROZEN (Phase 2 check, 16 Nov 2026). The plan allowed one retune, only if a controller was
# unstable, lost the line, or made the comparison degenerate. None happened with these values:
#   - all six runs (3 controllers x 2 tracks) stable, line detected 100 % of the time;
#   - comparison not degenerate: on the arc P holds ~ -14 mm, PI/PID ~ +2 mm;
#   - linear model (docs/MODEL.md): all three stable at d = 0.10, zeta 0.36-0.43.
# Fairness: the same Kp in all three, the same Ki in PI and PID, so each controller differs
# from the previous one by exactly one term.
GAINS: dict[str, tuple[float, float, float]] = {
    "P": (20.0, 0.0, 0.0),
    "PI": (20.0, 5.0, 0.0),
    "PID": (20.0, 5.0, 2.0),
}
