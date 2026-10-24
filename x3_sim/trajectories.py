"""Reference trajectories shared by the surrogate experiments and the ROS 2 node."""
import numpy as np


# ---------------------------------------------------------------------------
# Square trajectory (min-jerk segments), used in E4
# ---------------------------------------------------------------------------
CORNERS = np.array([[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]], float)
T_TAKEOFF, T_SEG, T_HOLD, Z_D = 12.0, 5.0, 2.0, 2.0


def _minjerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4), (60 * s - 180 * s ** 2 + 120 * s ** 3)


def square_traj(t):
    if t < 8.0:
        return np.array([0, 0, 0.2 + 1.8 * t / 8.0]), np.array([0, 0, 1.8 / 8.0]), np.zeros(3), 0.0
    if t < T_TAKEOFF:
        return np.array([0, 0, Z_D]), np.zeros(3), np.zeros(3), 0.0
    tt = t - T_TAKEOFF
    per = T_SEG + T_HOLD
    k = int(tt // per)
    if k >= 4:
        return np.array([0, 0, Z_D]), np.zeros(3), np.zeros(3), 0.0
    r = tt - k * per
    a, b = CORNERS[k], CORNERS[k + 1]
    if r >= T_SEG:
        return np.array([b[0], b[1], Z_D]), np.zeros(3), np.zeros(3), 0.0
    s, ds, dds = _minjerk(r / T_SEG)
    p = a + (b - a) * s
    v = (b - a) * ds / T_SEG
    acc = (b - a) * dds / T_SEG ** 2
    return np.array([p[0], p[1], Z_D]), np.array([v[0], v[1], 0.0]), np.array([acc[0], acc[1], 0.0]), 0.0


