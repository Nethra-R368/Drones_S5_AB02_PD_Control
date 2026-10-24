#!/usr/bin/env python3
# LEGACY - early stand-alone draft of the paper's Part 1 (kept for the record, not used as evidence).
# Differences from the paper and from paper_reproduction.py: u_max = 2.0 N m instead of 2.5, |eps| is used
# inside the barrier, varsigma = 0.05, scipy's adaptive solver with the saturation inside the ODE.
# The reproduction of record is ../paper_reproduction.py.
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# 1. Spacecraft Inertia Matrix (Section IV)
J = np.array([
    [20.0, 1.2,  0.9],
    [1.2,  17.0, 1.4],
    [0.9,  1.4,  15.0]
])
J_inv = np.linalg.inv(J)

# 2. Controller & FTPPF Parameters
kp, kd = 40.0, 20.0
alpha, beta = 1.5, 1.2
c = 0.5
zeta = 0.05
eta1 = 0.1
u_max = 2.0  # 2.0 Nm matches Fig. 6 & Fig. 9 saturation limits

rho_q0, rho_qT, kappa_q = 0.3, 0.008, 2.5
rho_w0, rho_wT, kappa_w = 0.4, 0.005, 2.5


def skew(v):
    """Cross-product matrix z^x (Eq. 2)."""
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0]
    ])


def ftppf(t, rho_0, rho_T, kappa, Ts):
    """Finite-Time Prescribed Performance Function (Eq. 8)."""
    if t < Ts:
        arg = np.clip((kappa * t) / (t - Ts), -500.0, 0.0)
        return (rho_0 - rho_T * (1.0 + t / Ts)) * np.exp(arg) + rho_T
    return rho_T


def compute_control(t, q_v, w, theta_hat, Ts):
    """Compute saturated control input u (Eq. 12, 13, 22)."""
    rho_q = ftppf(t, rho_q0, rho_qT, kappa_q, Ts)
    rho_w = ftppf(t, rho_w0, rho_wT, kappa_w, Ts)

    eps_1 = np.clip(np.abs(q_v) / rho_q, 0.0, 0.999)
    eps_2 = np.clip(np.abs(w) / rho_w, 0.0, 0.999)

    s = w + c * q_v
    barrier = alpha * (1.0 - eps_1)**(-2) + beta * (1.0 - eps_2)**(-2)
    u_cons = barrier * s + theta_hat * np.clip(s / zeta, -1.0, 1.0)

    u = -kp * q_v - kd * w - u_cons
    return np.clip(u, -u_max, u_max), s, rho_q, rho_w


def dynamics(t, state, Ts):
    """Full closed-loop ODE system: state = [q1, q2, q3, q4, w1, w2, w3, theta_hat]."""
    q_v = state[0:3]
    q_4 = state[3]
    w = state[4:7]
    theta_hat = state[7]

    u_sat, s, _, _ = compute_control(t, q_v, w, theta_hat, Ts)

    # External disturbance d(t) (Section IV)
    d = np.array([
        0.05 * np.sin(1.0 * t),
        0.10 * np.sin(1.2 * t),
        0.15 * np.sin(1.5 * t)
    ])

    # Quaternion kinematics (Eq. 1)
    dq_v = 0.5 * (q_4 * np.eye(3) + skew(q_v)) @ w
    dq_4 = -0.5 * np.dot(q_v, w)

    # Attitude dynamics (Eq. 1 & 5)
    dw = J_inv @ (-skew(w) @ (J @ w) + u_sat + d)

    # Adaptive law (Eq. 14)
    eta2 = 2.0 * np.exp(-0.5 * t)
    dtheta_hat = (1.0 / eta1) * (np.linalg.norm(s) - eta2 * theta_hat)

    return np.concatenate([dq_v, [dq_4], dw, [dtheta_hat]])


def run_simulation(Ts=15.0):
    # Initial states from Section IV-A
    q0 = np.array([0.2, -0.15, -0.25, 0.9354])
    q0 = q0 / np.linalg.norm(q0)
    w0 = np.array([0.03, 0.02, -0.01])
    x0 = np.concatenate([q0, w0, [0.0]])

    t_eval = np.linspace(0.0, 20.0, 2000)
    sol = solve_ivp(dynamics, (0.0, 20.0), x0, args=(Ts,), t_eval=t_eval, rtol=1e-6, atol=1e-8)

    u_hist, rho_q_hist, rho_w_hist = [], [], []
    for i, t in enumerate(sol.t):
        u_sat, _, rq, rw = compute_control(t, sol.y[0:3, i], sol.y[4:7, i], sol.y[7, i], Ts)
        u_hist.append(u_sat)
        rho_q_hist.append(rq)
        rho_w_hist.append(rw)

    return sol.t, sol.y, np.array(u_hist), np.array(rho_q_hist), np.array(rho_w_hist)


if __name__ == '__main__':
    t, y, u, rho_q, rho_w = run_simulation(Ts=15.0)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    # Plot Quaternion (Fig. 4)
    axes[0].fill_between(t, -rho_q, rho_q, color='lightgray', alpha=0.6)
    axes[0].plot(t, rho_q, 'k-', lw=1)
    axes[0].plot(t, -rho_q, 'k-', lw=1)
    axes[0].plot(t, y[0], 'b-', label='$q_1$')
    axes[0].plot(t, y[1], 'k--', label='$q_2$')
    axes[0].plot(t, y[2], 'r-.', label='$q_3$')
    axes[0].plot(t, y[3], 'c-', label='$q_4$')
    axes[0].set_title('Quaternion ($T_f = 15$ sec)')
    axes[0].set_xlabel('Time(s)')
    axes[0].set_ylabel('Quaternion')
    axes[0].legend()
    axes[0].grid(True)

    # Plot Angular Velocity (Fig. 5)
    axes[1].fill_between(t, -rho_w, rho_w, color='lightgray', alpha=0.6)
    axes[1].plot(t, rho_w, 'k-', lw=1)
    axes[1].plot(t, -rho_w, 'k-', lw=1)
    axes[1].plot(t, y[4], 'b-', label=r'$\omega_1$')
    axes[1].plot(t, y[5], 'k--', label=r'$\omega_2$')
    axes[1].plot(t, y[6], 'r-.', label=r'$\omega_3$')
    axes[1].set_title('Angular velocity ($T_f = 15$ sec)')
    axes[1].set_xlabel('Time(s)')
    axes[1].set_ylabel('Angular velocity (rad/s)')
    axes[1].legend()
    axes[1].grid(True)

    # Plot Control Torque (Fig. 6)
    axes[2].plot(t, u[:, 0], 'b-', label='$\\tau_1$')
    axes[2].plot(t, u[:, 1], 'k--', label='$\\tau_2$')
    axes[2].plot(t, u[:, 2], 'r-.', label='$\\tau_3$')
    axes[2].set_title('Control torque ($T_f = 15$ sec)')
    axes[2].set_xlabel('Time(s)')
    axes[2].set_ylabel('Control torque (N m)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.show()
