"""
controllers.py - controllers for the X3 surrogate.

OriginalGazeboController : line-by-line port of the control logic in the original
                           gazebo.py (v1, the version that crashed in Gazebo). Only the
                           ROS plumbing is removed; every number and formula is kept.
FixedController          : the corrected controller shipped as gazebo.py v2.
                           Same paper law (Eq. 8, 11-14, 22); fixes F9-F12 (see report).
"""
import numpy as np
from x3_model import (KT, CQ, MASS, G, W_MAX, J_BODY, allocation_matrix, quat_mul, quat_conj,
                      quat_to_R)


def ftppf(t, rho0, rhoT, kappa, Ts):
    """Paper Eq. (8)."""
    if t >= Ts:
        return rhoT
    t = max(t, 0.0)
    e = kappa * t / (t - Ts)
    if e < -50.0:
        return rhoT
    return max((rho0 - rhoT * (1.0 + t / Ts)) * np.exp(e) + rhoT, rhoT)


# ============================================================================
# ORIGINAL (v1) - faithful port of gazebo.py
# ============================================================================
class OriginalGazeboController:
    def __init__(self, controller_mode="constrained", fixes=()):
        """fixes: subset of {"mixer", "omega", "zdot"} - used only for the
        fault-isolation (ablation) study; fixes=() is the exact v1 behaviour."""
        self.controller_mode = controller_mode
        self.fixes = set(fixes)
        self.Ainv = np.linalg.inv(allocation_matrix())
        self.mass = 1.5; self.gravity = 9.81; self.arm_length = 0.225
        self.c_T = 8.54858e-06; self.c_Q = 0.016
        self.target_altitude = 2.0; self.Kp_z = 5.0; self.Kd_z = 3.5
        self.Kp = 1.5; self.Kd = 0.6
        self.alpha = 0.01; self.beta = 0.01; self.c = 0.5
        self.eta1 = 0.1; self.varsigma = 0.5; self.uMax = 0.20
        self.rho_q0 = 2.0; self.rho_qT = 0.30; self.kappa_q = 2.0
        self.rho_w0 = 10.0; self.rho_wT = 3.0; self.kappa_w = 2.0; self.Ts = 10.0
        self.thetaHat = 0.0
        self.start_time = None; self.last_time = None
        self.initialized = False
        self.last_z = 0.0; self.last_q = np.array([0.0, 0.0, 0.0, 1.0]); self.last_omega = np.zeros(3)
        self.z_dot = 0.0; self.z_filter = 0.10; self.omega_filter = 0.10
        self.takeoff_duration = 8.0
        self.last_cmd = np.zeros(4)
        self.log = {k: [] for k in ["t", "qv", "omega_est", "rho_q", "rho_w", "tau", "theta", "z", "T"]}

    def update(self, now, pos, q):
        if self.start_time is None:
            self.start_time = now; self.last_time = now
        dt = now - self.last_time
        if dt <= 0.0 and self.initialized:
            return self.last_cmd
        if dt > 0.1:
            dt = 0.02
        elapsed = now - self.start_time
        z = pos[2]
        q = q / max(np.linalg.norm(q), 1e-8)
        if np.dot(q, self.last_q) < 0:
            q = -q
        qv = q[:3]
        if not self.initialized:
            self.last_z = z; self.last_q = q.copy(); self.last_time = now; self.initialized = True
            return self.last_cmd
        raw_z_dot = (z - self.last_z) / dt
        self.z_dot = (1.0 - self.z_filter) * self.z_dot + self.z_filter * raw_z_dot
        if "zdot" in self.fixes:          # v1 never updates last_z (bug F14)
            self.last_z = z
        self.z_dot = np.clip(self.z_dot, -3.0, 3.0)
        if "omega" in self.fixes:
            d = quat_mul(quat_conj(self.last_q), q)
            raw_omega = 2.0 * d[:3] / dt
            omega = 0.4 * self.last_omega + 0.6 * raw_omega
        else:
            dq = (q - self.last_q) / dt
            raw_omega = 2.0 * dq[:3]
            omega = (1.0 - self.omega_filter) * self.last_omega + self.omega_filter * raw_omega
        omega = np.clip(omega, -3.0, 3.0)
        target_z = 0.2 + 1.8 * elapsed / self.takeoff_duration if elapsed < self.takeoff_duration else 2.0
        vertical_acceleration = np.clip(self.Kp_z * (target_z - z) - self.Kd_z * self.z_dot, -3.0, 3.0)
        F_vertical = self.mass * (self.gravity + vertical_acceleration)
        roll = 2.0 * q[0]; pitch = 2.0 * q[1]
        tilt_factor = np.clip(np.cos(roll) * np.cos(pitch), 0.85, 1.0)
        T_total = np.clip(F_vertical / tilt_factor, 0.0, 22.0)
        t_env = min(elapsed, self.Ts)
        rho_q = ftppf(t_env, self.rho_q0, self.rho_qT, self.kappa_q, self.Ts)
        rho_w = ftppf(t_env, self.rho_w0, self.rho_wT, self.kappa_w, self.Ts)
        s = omega + self.c * qv
        eps_q = qv / max(rho_q, 1e-6); eps_w = omega / max(rho_w, 1e-6)
        u_cons = np.zeros(3)
        if self.controller_mode == "constrained":
            for i in range(3):
                e1 = np.clip(eps_q[i], -0.90, 0.90); e2 = np.clip(eps_w[i], -0.90, 0.90)
                u_cons[i] = (self.alpha / (1 - e1 ** 2) ** 2 + self.beta / (1 - e2 ** 2) ** 2) * s[i] \
                    + self.thetaHat * np.clip(s[i] / self.varsigma, -1.0, 1.0)
            u_cons = np.clip(u_cons, -0.05, 0.05)
            eta2 = max(2.0 * np.exp(-0.5 * t_env), 0.1)
            d_theta = (1.0 / self.eta1) * (np.linalg.norm(s) - eta2 * self.thetaHat)
            if self.thetaHat <= 0.0 and d_theta < 0.0:
                d_theta = 0.0
            self.thetaHat = np.clip(self.thetaHat + d_theta * dt, 0.0, 1.0)
        else:
            self.thetaHat = 0.0
        u = -self.Kp * qv - self.Kd * omega - u_cons
        u = np.clip(u, -self.uMax, self.uMax)
        available_tau = max(0.08, min(0.15 * T_total * self.arm_length, self.uMax))
        tau = np.clip(u, -available_tau, available_tau)
        base = T_total / (4.0 * self.c_T)
        roll_term = tau[0] / (4.0 * self.c_T * self.arm_length)
        pitch_term = tau[1] / (4.0 * self.c_T * self.arm_length)
        yaw_term = tau[2] / (4.0 * self.c_Q)
        m1 = base + pitch_term - roll_term - yaw_term
        m2 = base - pitch_term - roll_term + yaw_term
        m3 = base - pitch_term + roll_term - yaw_term
        m4 = base + pitch_term + roll_term + yaw_term
        w_cmd = np.sqrt(np.maximum(0.0, [m1, m2, m3, m4]))
        if "mixer" in self.fixes:
            F = np.clip(self.Ainv @ np.array([T_total, *tau]), 0.0, KT * W_MAX ** 2)
            w_cmd = np.sqrt(F / KT)
        self.last_q = q.copy(); self.last_omega = omega.copy(); self.last_time = now
        self.last_cmd = w_cmd
        L = self.log
        L["t"].append(elapsed); L["qv"].append(qv.copy()); L["omega_est"].append(omega.copy())
        L["rho_q"].append(rho_q); L["rho_w"].append(rho_w); L["tau"].append(tau.copy())
        L["theta"].append(self.thetaHat); L["z"].append(z); L["T"].append(T_total)
        return w_cmd


def original_mixer_effective_matrix():
    """Torque actually produced per unit commanded torque by the v1 mixer on the real X3
    geometry (small-signal: F_i = kT (w0^2 + m_i) ~ linear in m_i)."""
    l = 0.225; cT = 8.54858e-06; cQ = 0.016
    # m_i coefficient rows for [tau_x, tau_y, tau_z] commands (m1..m4)
    Mcmd = np.array([[-1 / (4 * cT * l), +1 / (4 * cT * l), -1 / (4 * cQ)],
                     [-1 / (4 * cT * l), -1 / (4 * cT * l), +1 / (4 * cQ)],
                     [+1 / (4 * cT * l), -1 / (4 * cT * l), -1 / (4 * cQ)],
                     [+1 / (4 * cT * l), +1 / (4 * cT * l), +1 / (4 * cQ)]])
    dF = KT * Mcmd                       # thrust change per unit torque command (4x3)
    A = allocation_matrix()
    return A[1:, :] @ dF                 # 3x3: produced torque / commanded torque


# ============================================================================
# FIXED (v2)
# ============================================================================
class FixedController:
    """Constrained PD-type attitude controller of the paper on the X3, with the
    implementation faults of v1 removed.

    F9  mixer: rotor thrusts from the exact inverse of the X3 allocation matrix built
        from the SDF rotor positions and spin directions (v1 had wrong signs on 5 of 8
        roll/pitch entries -> zero pitch authority and a roll torque driven by the
        pitch command).
    F10 yaw: yaw torque -> thrust uses kT*cQ (v1 divided by cQ only, 5 orders of
        magnitude too small -> yaw uncontrolled).
    F11 body rate: omega = 2 Im(q_conj * dq/dt) (exact body-frame kinematics) with a
        light filter (v1: 2*dq_v/dt with a 0.10 filter = ~150 ms lag at 60 Hz).
    F12 time: dt from the message time stamp (v1 used wall-clock time.time()).
    F13 attitude error w.r.t. a reference q_d (initial yaw, or the outer position loop
        for trajectory tracking) instead of the raw world quaternion.
    F3  envelope and gains sized for the X3 (inertia 0.035-0.098 kg m^2) so that the
        FTPPF is actually active and feasible.
    """

    def __init__(self, mode="constrained", gains=None, env=None, position_hold=True,
                 trajectory=None, omega_filter=0.6, uMax=0.35):
        self.mode = mode
        g = dict(kp=4.0, kd=0.4, alpha=0.15, beta=0.12, c=1.0, eta1=5.0, varsigma=0.5,
                 theta_max=0.1, eta2_floor=0.2, gain_max=1.0,
                 Kp_z=5.0, Kd_z=3.5, Kp_xy=1.2, Kd_xy=1.6, tilt_max=0.35)
        if gains: g.update(gains)
        self.g = g
        e = dict(rho_q0=0.30, rho_qT=0.02, kappa_q=2.0, rho_w0=2.5, rho_wT=0.05, kappa_w=2.0, Ts=10.0)
        if env: e.update(env)
        self.e = e
        self.uMax = uMax
        self.Ainv = np.linalg.inv(allocation_matrix())
        self.J_scale = np.diag(J_BODY) / J_BODY[2, 2]   # per-axis cap proportional to inertia
        self.position_hold = position_hold
        self.trajectory = trajectory          # callable t -> (p_d, v_d, a_d, yaw_d) or None
        self.lam = omega_filter
        self.extra_tau = np.zeros(3)
        self.theta = 0.0
        self.t0 = None; self.last_t = None; self.last_q = None; self.last_p = None
        self.omega = np.zeros(3); self.vel = np.zeros(3)
        self.yaw_ref = None; self.p_ref = None
        self.last_qd = None; self.omega_d = np.zeros(3)
        self.cmd = np.zeros(4)
        self.log = {k: [] for k in ["t", "qe", "omega", "rho_q", "rho_w", "eps1", "eps2", "u", "tau",
                                    "theta", "p", "p_d", "T", "F", "sat_frac"]}

    @staticmethod
    def _yaw_quat(yaw):
        return np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])

    def update(self, stamp, p, q):
        q = q / np.linalg.norm(q)
        if self.t0 is None:
            self.t0 = stamp; self.last_t = stamp; self.last_q = q.copy(); self.last_p = p.copy()
            x, y, z, w = q
            self.yaw_ref = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            self.p_ref = np.array([p[0], p[1], 2.0])
            return self.cmd
        dt = stamp - self.last_t
        if dt <= 1e-6:
            return self.cmd
        t = stamp - self.t0
        if np.dot(q, self.last_q) < 0:
            q = -q
        # ---- F11: exact body rate from consecutive quaternions -------------
        dq = quat_mul(quat_conj(self.last_q), q)
        if dq[3] < 0: dq = -dq
        ang = 2.0 * np.arctan2(np.linalg.norm(dq[:3]), dq[3])
        axis = dq[:3] / (np.linalg.norm(dq[:3]) + 1e-12)
        w_raw = axis * ang / dt
        self.omega = (1 - self.lam) * self.omega + self.lam * w_raw
        v_raw = (p - self.last_p) / dt
        self.vel = 0.7 * self.vel + 0.3 * v_raw
        g = self.g; e = self.e
        # ---- reference: hover point or trajectory --------------------------
        if self.trajectory is not None:
            p_d, v_d, a_d, yaw_d = self.trajectory(t)
        else:
            z_d = 0.2 + 1.8 * t / 8.0 if t < 8.0 else 2.0
            p_d = np.array([self.p_ref[0], self.p_ref[1], z_d]); v_d = np.zeros(3); a_d = np.zeros(3)
            yaw_d = self.yaw_ref
        # ---- outer loop: desired acceleration -> thrust & desired attitude ---
        a_cmd = a_d.copy()
        a_cmd[2] += np.clip(g["Kp_z"] * (p_d[2] - p[2]) + g["Kd_z"] * (v_d[2] - self.vel[2]), -3.0, 3.0)
        if self.position_hold:
            a_xy = g["Kp_xy"] * (p_d[:2] - p[:2]) + g["Kd_xy"] * (v_d[:2] - self.vel[:2])
            n = np.linalg.norm(a_xy); lim = G * np.tan(g["tilt_max"])
            if n > lim: a_xy *= lim / n
            a_cmd[:2] += a_xy
        f_des = MASS * (a_cmd + np.array([0, 0, G]))
        zb_d = f_des / np.linalg.norm(f_des)
        xc = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
        yb_d = np.cross(zb_d, xc); yb_d /= np.linalg.norm(yb_d)
        xb_d = np.cross(yb_d, zb_d)
        q_d = rot_to_quat(np.column_stack([xb_d, yb_d, zb_d]))
        R = quat_to_R(q)
        T = float(np.clip(np.dot(f_des, R[:, 2]), 0.0, 4 * KT * W_MAX ** 2 * 0.9))
        # ---- attitude error (F13) --------------------------------------------
        if self.last_qd is not None and np.dot(q_d, self.last_qd) < 0:
            q_d = -q_d
        qe = quat_mul(quat_conj(q_d), q)
        if qe[3] < 0: qe = -qe
        qv = qe[:3]
        # desired body rate from consecutive references (feed-forward), expressed in the
        # body frame: omega_e = omega - R_e^T omega_d  (standard tracking extension, F13)
        if self.last_qd is not None:
            dqd = quat_mul(quat_conj(self.last_qd), q_d)
            if dqd[3] < 0: dqd = -dqd
            wd_raw = 2.0 * dqd[:3] / dt
            self.omega_d = (1 - self.lam) * self.omega_d + self.lam * wd_raw
        self.last_qd = q_d.copy()
        Re = quat_to_R(qe)
        w = self.omega - Re.T @ self.omega_d
        # ---- paper law, Eq. (8), (11)-(14), (22) ----------------------------
        rq = ftppf(t, e["rho_q0"], e["rho_qT"], e["kappa_q"], e["Ts"])
        rw = ftppf(t, e["rho_w0"], e["rho_wT"], e["kappa_w"], e["Ts"])
        eps1 = qv / rq; eps2 = w / rw
        s = w + g["c"] * qv
        u = -g["kp"] * qv - g["kd"] * w
        if self.mode in ("constrained", "symmetric"):
            if self.mode == "constrained":
                gain = g["alpha"] / (1 - np.minimum(eps1, 0.99)) ** 2 + g["beta"] / (1 - np.minimum(eps2, 0.99)) ** 2
            else:
                gain = g["alpha"] / (1 - np.clip(eps1, -0.99, 0.99) ** 2) ** 2 + \
                    g["beta"] / (1 - np.clip(eps2, -0.99, 0.99) ** 2) ** 2
            # sampled-data guard: the continuous-time barrier gain is unbounded as eps -> 1;
            # at 60 Hz with one sample of delay an unbounded gain destabilises the loop,
            # so the total barrier gain is capped at gain_max (see report, F15).
            gain = np.minimum(gain, g["gain_max"] * self.J_scale)
            u = u - (gain * s + self.theta * np.clip(s / g["varsigma"], -1, 1))
            eta2 = max(2.0 * np.exp(-0.5 * t), g["eta2_floor"])
            self.theta = float(np.clip(self.theta + dt * (np.linalg.norm(s) - eta2 * self.theta) / g["eta1"],
                                       0.0, g["theta_max"]))
        tau = np.clip(u, -self.uMax, self.uMax)
        # optional test torque injected through the rotors (used by the Gazebo node's
        # disturbance test; zero in the surrogate, where disturbances act physically)
        tau = tau + self.extra_tau
        # ---- F9/F10: exact allocation with thrust-priority desaturation ----------
        F = self.Ainv @ np.array([T, *tau])
        Fmax = KT * W_MAX ** 2
        sat_frac = 0.0
        if F.min() < 0 or F.max() > Fmax:
            # drop yaw first, then scale roll/pitch until feasible
            F0 = self.Ainv @ np.array([T, tau[0], tau[1], 0.0])
            k = 1.0
            if F0.min() >= 0 and F0.max() <= Fmax:
                Fy = self.Ainv @ np.array([0, 0, 0, tau[2]])
                for kk in np.linspace(1, 0, 21):
                    if (F0 + kk * Fy).min() >= 0 and (F0 + kk * Fy).max() <= Fmax:
                        k = kk; break
                F = F0 + k * Fy
            else:
                Fh = self.Ainv @ np.array([T, 0, 0, 0]); Ft = F0 - Fh
                for kk in np.linspace(1, 0, 41):
                    if (Fh + kk * Ft).min() >= 0 and (Fh + kk * Ft).max() <= Fmax:
                        k = kk; break
                F = Fh + k * Ft
            sat_frac = 1.0
        F = np.clip(F, 0, Fmax)
        self.cmd = np.sqrt(F / KT)
        self.last_t = stamp; self.last_q = q.copy(); self.last_p = p.copy()
        L = self.log
        for key, val in (("t", t), ("qe", qv.copy()), ("omega", w.copy()), ("rho_q", rq), ("rho_w", rw),
                         ("eps1", eps1.copy()), ("eps2", eps2.copy()), ("u", u.copy()), ("tau", tau.copy()),
                         ("theta", self.theta), ("p", p.copy()), ("p_d", np.array(p_d, float)), ("T", T),
                         ("F", F.copy()), ("sat_frac", sat_frac)):
            L[key].append(val)
        return self.cmd


def rot_to_quat(R):
    """Rotation matrix -> [x, y, z, w]."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S; x = (R[2, 1] - R[1, 2]) / S; y = (R[0, 2] - R[2, 0]) / S; z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S; x = 0.25 * S; y = (R[0, 1] + R[1, 0]) / S; z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S; x = (R[0, 1] + R[1, 0]) / S; y = 0.25 * S; z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S; x = (R[0, 2] + R[2, 0]) / S; y = (R[1, 2] + R[2, 1]) / S; z = 0.25 * S
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)
