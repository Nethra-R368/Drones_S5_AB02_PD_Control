#!/usr/bin/env python3
"""
paper_reproduction.py
=====================
Faithful numerical reproduction of

    M. Golestani, K. A. Alattas, S. Ud Din, S. Mobayen, A. K. Alanazi, A. Fekih,
    "A Simple Structure Constrained Attitude Control for Rigid Bodies:
     A PD-Type Control", IEEE Access, vol. 10, pp. 10202-10209, 2022.

Every equation number below refers to the paper.

    Plant ............ Eq. (1)  quaternion kinematics + Euler rigid-body dynamics
    Saturation ....... Eq. (3)-(4)  sat(u_i) = sgn(u_i) min(|u_i|, u_max)
    FTPPF ............ Eq. (8)
    Normalised error . Eq. (11)  eps1 = q_v / rho_q,  eps2 = omega / rho_w
    Control law ...... Eq. (12)  u = -kp q_v - kd omega - u_cons
    Barrier term ..... Eq. (13)/(22)
                       u_cons = [alpha (1-eps1)^-2 + beta (1-eps2)^-2] s + theta_hat sat(s/varsigma)
                       s = omega + c q_v
    Adaptive law ..... Eq. (14)  theta_hat_dot = (||s|| - eta2 theta_hat) / eta1

Integration: fixed-step RK4, dt = 1 ms, control held constant over a step
(zero-order hold, as in a digital implementation).  The simulator is vectorised
over a batch dimension so that the Monte-Carlo study runs in seconds.

Parameters the paper does NOT state (documented in the report):
    c = 0.5, varsigma = 0.02, Part-2 settling time Ts = 15 s,
    Part-2 rho_q0 = 0.36 (the paper's q(0) lies exactly on rho_q0 = 0.3).

Run:   python paper_reproduction.py
Needs: numpy, matplotlib
Writes docs/figures/*.png and docs/results/metrics.{json,md}
"""
import json
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "docs", "figures")
RES = os.path.join(HERE, "docs", "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 160, "font.size": 10,
    "axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False,
    "axes.spines.right": False, "legend.fontsize": 8.5,
})
C3 = ["#1f77b4", "#d62728", "#2ca02c"]          # q1/w1, q2/w2, q3/w3
ENV = "#cfcfcf"

# ---------------------------------------------------------------------------
# Paper Section IV parameters
# ---------------------------------------------------------------------------
J = np.array([[20.0, 1.2, 0.9],
              [1.2, 17.0, 1.4],
              [0.9, 1.4, 15.0]])
J_INV = np.linalg.inv(J)
P = dict(kp=40.0, kd=20.0, alpha=1.5, beta=1.2, eta1=0.1,
         c=0.5, varsigma=0.02, umax=2.5,
         rho_q0=0.3, rho_w0=0.4, rho_qT=0.008, rho_wT=0.005,
         kappa_q=2.5, kappa_w=2.5, Ts=15.0)
PART1 = dict(q0=[0.2, -0.15, -0.25, 0.9354], w0=[0.03, 0.02, -0.01])
PART2 = dict(q0=[0.3, -0.2, -0.3, 0.8832], w0=[-0.02, -0.02, 0.025])
DT = 1e-3
T_END = 20.0


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------
def ftppf(t, rho0, rhoT, kappa, Ts):
    """Finite-time prescribed performance function, Eq. (8). Works on arrays."""
    t = np.asarray(t, dtype=float)
    out = np.full_like(t, rhoT)
    m = t < Ts
    tm = t[m]
    out[m] = (rho0 - rhoT * (1.0 + tm / Ts)) * np.exp(kappa * tm / (tm - Ts)) + rhoT
    return out


def exp_ppf(t, rho0, rhoInf, kappa):
    """Classical exponential PPF, Eq. (7)."""
    return (rho0 - rhoInf) * np.exp(-kappa * np.asarray(t)) + rhoInf


def sat_vec(u, umax):
    """Actuator saturation, Eq. (3)-(4)."""
    return np.clip(u, -umax, umax)


def eta2_fun(t):
    """Leakage gain used in the paper: eta2 = 2 exp(-0.5 t), integral = 4 < inf."""
    return 2.0 * np.exp(-0.5 * t)


def disturbance(t, scale=1.0):
    return scale * np.array([0.05 * np.sin(t), 0.10 * np.sin(1.2 * t), 0.15 * np.sin(1.5 * t)])


def rhs(q, w, u, d):
    """Eq. (1): returns q_dot (N,4) and w_dot (N,3) for a batch."""
    qv, q4 = q[:, :3], q[:, 3:4]
    qv_dot = 0.5 * (q4 * w + np.cross(qv, w))
    q4_dot = -0.5 * np.sum(qv * w, axis=1, keepdims=True)
    Jw = w @ J.T
    w_dot = (-np.cross(w, Jw) + u + d) @ J_INV.T
    return np.hstack([qv_dot, q4_dot]), w_dot


def control(t, q, w, th, p, mode):
    """Eq. (12)-(13) with Eq. (22) boundary layer. Returns u (unsaturated), s, eps1, eps2."""
    qv = q[:, :3]
    rq = ftppf(np.array([t]), p["rho_q0"], p["rho_qT"], p["kappa_q"], p["Ts"])[0]
    rw = ftppf(np.array([t]), p["rho_w0"], p["rho_wT"], p["kappa_w"], p["Ts"])[0]
    e1 = qv / rq
    e2 = w / rw
    s = w + p["c"] * qv
    u = -p["kp"] * qv - p["kd"] * w
    if mode == "constrained":            # paper form, Eq. (13)
        e1c = np.minimum(e1, 0.999)       # numerical guard only (never active when |eps|<1)
        e2c = np.minimum(e2, 0.999)
        gain = p["alpha"] / (1 - e1c) ** 2 + p["beta"] / (1 - e2c) ** 2
        u = u - (gain * s + th[:, None] * np.clip(s / p["varsigma"], -1, 1))
    elif mode == "symmetric":            # variant used in the original browser/Gazebo code
        e1c = np.clip(e1, -0.999, 0.999)
        e2c = np.clip(e2, -0.999, 0.999)
        gain = p["alpha"] / (1 - e1c ** 2) ** 2 + p["beta"] / (1 - e2c ** 2) ** 2
        u = u - (gain * s + th[:, None] * np.clip(s / p["varsigma"], -1, 1))
    elif mode == "pd":
        pass
    else:
        raise ValueError(mode)
    return u, s, e1, e2, rq, rw


def simulate(q0, w0, p=P, mode="constrained", t_end=T_END, dt=DT, dist_scale=1.0,
             record=True):
    """Batch RK4 simulation. q0: (N,4) or (4,), w0: (N,3) or (3,).
    dist_scale: scalar or (N,) array multiplying the paper's disturbance."""
    q = np.atleast_2d(np.array(q0, dtype=float))
    w = np.atleast_2d(np.array(w0, dtype=float))
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    N = q.shape[0]
    th = np.zeros(N)
    ds = np.broadcast_to(np.asarray(dist_scale, dtype=float), (N,))[:, None]
    n = int(round(t_end / dt))
    rec_every = 10 if record else n + 1
    L = {k: [] for k in ["t", "q", "w", "u", "usat", "th", "rq", "rw", "e1", "e2", "s"]}
    # running metrics for every run in the batch
    viol_q = np.zeros(N); viol_w = np.zeros(N); at_lim = np.zeros(N)
    emax1 = np.zeros(N); emax2 = np.zeros(N); energy = np.zeros(N); upeak = np.zeros(N)
    last_out = np.zeros(N)
    for k in range(n):
        t = k * dt
        u, s, e1, e2, rq, rw = control(t, q, w, th, p, mode)
        us = sat_vec(u, p["umax"])
        # metrics
        vq = np.any(np.abs(q[:, :3]) >= rq, axis=1)
        vw = np.any(np.abs(w) >= rw, axis=1)
        viol_q += vq; viol_w += vw
        at_lim += np.any(np.abs(u) >= p["umax"], axis=1)
        emax1 = np.maximum(emax1, np.max(np.abs(e1), axis=1))
        emax2 = np.maximum(emax2, np.max(np.abs(e2), axis=1))
        energy += np.sum(us ** 2, axis=1) * dt
        upeak = np.maximum(upeak, np.max(np.abs(u), axis=1))
        last_out = np.where(np.linalg.norm(q[:, :3], axis=1) > 0.01, t, last_out)
        if record and k % rec_every == 0:
            L["t"].append(t); L["q"].append(q.copy()); L["w"].append(w.copy())
            L["u"].append(u.copy()); L["usat"].append(us.copy()); L["th"].append(th.copy())
            L["rq"].append(rq); L["rw"].append(rw); L["e1"].append(e1.copy())
            L["e2"].append(e2.copy()); L["s"].append(s.copy())
        # RK4 with ZOH torque; disturbance evaluated at stage times
        def f(tt, qq, ww):
            d = disturbance(tt)[None, :] * ds
            return rhs(qq, ww, us, d)
        k1q, k1w = f(t, q, w)
        k2q, k2w = f(t + dt / 2, q + dt / 2 * k1q, w + dt / 2 * k1w)
        k3q, k3w = f(t + dt / 2, q + dt / 2 * k2q, w + dt / 2 * k2w)
        k4q, k4w = f(t + dt, q + dt * k3q, w + dt * k3w)
        q = q + dt / 6 * (k1q + 2 * k2q + 2 * k3q + k4q)
        w = w + dt / 6 * (k1w + 2 * k2w + 2 * k3w + k4w)
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        if mode != "pd":
            # adaptive law, Eq. (14) (explicit Euler; theta_hat >= 0 projection)
            th = np.maximum(th + dt * (np.linalg.norm(s, axis=1) - eta2_fun(t) * th) / p["eta1"], 0.0)
    out = dict(
        viol_q=100 * viol_q / n, viol_w=100 * viol_w / n, at_limit=100 * at_lim / n,
        eps1_max=emax1, eps2_max=emax2, energy=energy, upeak=upeak,
        settle=last_out, qv_final=np.linalg.norm(q[:, :3], axis=1),
        w_final=np.linalg.norm(w, axis=1), theta_final=th,
    )
    if record:
        for k in L:
            L[k] = np.array(L[k])
        out["log"] = L
    return out


def first(m):
    """Collapse batch-of-1 metrics to plain floats for JSON."""
    return {k: float(v[0]) for k, v in m.items() if k != "log"}


# ---------------------------------------------------------------------------
# Lyapunov function of Theorem 1 (Eq. 15) evaluated along a trajectory
# ---------------------------------------------------------------------------
def lyapunov(log, p):
    """State part of the Lyapunov function of Theorem 1, Eq. (15):
        V_x = (kp + c kd)(q_v'q_v + (1 - q4)^2) + 1/2 w'Jw + c q_v'Jw.
    The remaining term of Eq. (15) depends on theta_tilde = theta - theta_hat, and the true bound theta
    of the lumped torque T_d = d + Pi is unknown (Pi depends on how deep the actuators saturate), so it
    cannot be evaluated along a trajectory. (The paper prints (1 - q0^2) where the cited reference
    [42] has (1 - q0)^2; we use the latter, which vanishes only at q = [0,0,0,1].)"""
    q = log["q"][:, 0]; w = log["w"][:, 0]
    qv, q4 = q[:, :3], q[:, 3]
    return ((p["kp"] + p["c"] * p["kd"]) * (np.sum(qv ** 2, 1) + (1 - q4) ** 2)
            + 0.5 * np.einsum("ni,ij,nj->n", w, J, w)
            + p["c"] * np.einsum("ni,ij,nj->n", qv, J, w))


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------
def env_plot(ax, t, rho, Y, labels, ylab, title):
    ax.fill_between(t, -rho, rho, color=ENV, alpha=0.55, lw=0, label=r"$\pm\rho(t)$")
    ax.plot(t, rho, "k-", lw=0.8); ax.plot(t, -rho, "k-", lw=0.8)
    for i in range(Y.shape[1]):
        ax.plot(t, Y[:, i], color=C3[i], lw=1.4, label=labels[i])
    ax.set_xlabel("time (s)"); ax.set_ylabel(ylab); ax.set_title(title)
    ax.legend(loc="upper right", ncol=4)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, name))
    plt.close(fig)
    print("  wrote docs/figures/" + name)


# ---------------------------------------------------------------------------
# Experiments
# ---------------------------------------------------------------------------
def fig_ppf():
    t = np.linspace(0, 12, 1200)
    Ts = 8.0
    r_ft = ftppf(t, 0.3, 0.008, 2.5, Ts)
    r_ex = exp_ppf(t, 0.3, 0.008, 0.5)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(t, r_ex, "--", color="#555", label="exponential PPF, Eq. (7), $\\kappa$=0.5")
    ax.plot(t, r_ft, color="#1f77b4", lw=2, label="finite-time PPF, Eq. (8), $T_s$=8 s")
    ax.axvline(Ts, color="#d62728", ls=":", lw=1); ax.text(Ts + 0.1, 0.2, "$T_s$", color="#d62728")
    ax.axhline(0.008, color="k", lw=0.6)
    ax.set_xlabel("time (s)"); ax.set_ylabel(r"$\rho(t)$")
    ax.set_title(r"FTPPF reaches $\rho_T$ exactly at $T_s$; the exponential PPF only approaches it")
    ax.legend(); save(fig, "fig01_ppf_comparison.png")
    return dict(exp_at_Ts=float(exp_ppf(Ts, 0.3, 0.008, 0.5)), ftppf_at_Ts=0.008)


def part1():
    res = {}
    logs = {}
    for Ts in (15.0, 10.0):
        p = dict(P, Ts=Ts)
        m = simulate(PART1["q0"], PART1["w0"], p, "constrained")
        logs[Ts] = m["log"]; res[f"Ts={Ts:g}"] = first(m)
    for Ts in (15.0, 10.0):
        L = logs[Ts]; t = L["t"]
        fig, ax = plt.subplots(3, 1, figsize=(7.5, 8.5), sharex=True)
        env_plot(ax[0], t, L["rq"], L["q"][:, 0, :3], ["$q_1$", "$q_2$", "$q_3$"],
                 "quaternion $q_v$", f"Part 1, $T_s$ = {Ts:g} s  (paper Fig. 4)")
        ax[0].plot(t, L["q"][:, 0, 3] * 0 + np.nan)
        env_plot(ax[1], t, L["rw"], L["w"][:, 0], [r"$\omega_1$", r"$\omega_2$", r"$\omega_3$"],
                 r"$\omega$ (rad/s)", "angular velocity (paper Fig. 5)")
        for i in range(3):
            ax[2].plot(t, L["usat"][:, 0, i], color=C3[i], lw=1.3, label=f"$u_{i+1}$")
        ax[2].axhline(2.5, color="k", ls="--", lw=0.8); ax[2].axhline(-2.5, color="k", ls="--", lw=0.8)
        ax[2].set_ylabel("applied torque (N·m)"); ax[2].set_xlabel("time (s)")
        ax[2].set_title("control torque, saturated at ±2.5 N·m (paper Fig. 6)"); ax[2].legend(ncol=3)
        save(fig, f"fig02_part1_Ts{int(Ts)}.png")
    # zoomed comparison of convergence for both Ts
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    for Ts, ls in ((15.0, "-"), (10.0, "--")):
        L = logs[Ts]
        ax[0].semilogy(L["t"], np.linalg.norm(L["q"][:, 0, :3], axis=1), ls, label=f"$T_s$={Ts:g} s")
        ax[1].semilogy(L["t"], np.linalg.norm(L["w"][:, 0], axis=1), ls, label=f"$T_s$={Ts:g} s")
    ax[0].set_title(r"$\|q_v\|$ (log scale)"); ax[1].set_title(r"$\|\omega\|$ (log scale)")
    for a in ax: a.set_xlabel("time (s)"); a.legend()
    save(fig, "fig03_part1_convergence.png")
    return res, logs


def part2():
    p = dict(P, rho_q0=0.36, Ts=15.0)
    res, logs = {}, {}
    for mode in ("constrained", "symmetric", "pd"):
        m = simulate(PART2["q0"], PART2["w0"], p, mode)
        res[mode] = first(m); logs[mode] = m["log"]
    Lc, Lp = logs["constrained"], logs["pd"]
    t = Lc["t"]
    fig, ax = plt.subplots(3, 2, figsize=(11, 8.5), sharex=True)
    for i in range(3):
        for j, (L, name) in enumerate(((Lc, "constrained PD-type"), (Lp, "conventional PD"))):
            a = ax[i, j]
            a.fill_between(t, -L["rq"], L["rq"], color=ENV, alpha=0.55, lw=0)
            a.plot(t, L["q"][:, 0, i], color=C3[i], lw=1.4)
            a.set_ylabel(f"$q_{i+1}$")
            if i == 0: a.set_title(f"{name}: quaternion inside $\\pm\\rho_q$ (paper Fig. 7)")
    for a in ax[-1]: a.set_xlabel("time (s)")
    save(fig, "fig04_part2_quaternion.png")
    fig, ax = plt.subplots(3, 2, figsize=(11, 8.5), sharex=True)
    for i in range(3):
        for j, (L, name) in enumerate(((Lc, "constrained PD-type"), (Lp, "conventional PD"))):
            a = ax[i, j]
            a.fill_between(t, -L["rw"], L["rw"], color=ENV, alpha=0.55, lw=0)
            a.plot(t, L["w"][:, 0, i], color=C3[i], lw=1.4)
            out = np.abs(L["w"][:, 0, i]) >= L["rw"]
            if out.any():
                a.plot(t[out], L["w"][out, 0, i], ".", ms=2, color="k", label="outside envelope")
                a.legend(loc="upper right")
            a.set_ylabel(f"$\\omega_{i+1}$ (rad/s)")
            if i == 0: a.set_title(f"{name}: angular velocity (paper Fig. 8)")
    for a in ax[-1]: a.set_xlabel("time (s)")
    save(fig, "fig05_part2_angular_velocity.png")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4), sharey=True)
    for j, (L, name) in enumerate(((Lc, "constrained PD-type"), (Lp, "conventional PD"))):
        for i in range(3):
            ax[j].plot(t, L["usat"][:, 0, i], color=C3[i], lw=1.2, label=f"$u_{i+1}$")
        ax[j].set_title(f"{name}: applied torque (paper Fig. 9)"); ax[j].set_xlabel("time (s)")
        ax[j].legend(ncol=3)
    ax[0].set_ylabel("N·m")
    save(fig, "fig06_part2_torque.png")
    # normalised errors -> the single most informative plot for the constraint
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4), sharey=True)
    for j, key in enumerate(("e1", "e2")):
        for L, name, ls in ((Lc, "constrained", "-"), (Lp, "PD", "--")):
            ax[j].plot(t, np.max(np.abs(L[key][:, 0]), axis=1), ls, lw=1.4, label=name)
        ax[j].axhline(1.0, color="#d62728", lw=1); ax[j].text(0.3, 1.03, "constraint boundary |ε|=1", color="#d62728")
        ax[j].set_title(r"$\max_i|\varepsilon_{%di}(t)|$" % (j + 1)); ax[j].set_xlabel("time (s)"); ax[j].legend()
    save(fig, "fig07_part2_normalised_error.png")
    # adaptive estimate & Lyapunov function
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    ax[0].plot(t, Lc["th"][:, 0], lw=1.5); ax[0].set_title(r"adaptive estimate $\hat\theta(t)$, Eq. (14)")
    ax[0].set_xlabel("time (s)")
    V = lyapunov(Lc, p)
    ax[1].semilogy(t, V, lw=1.5); ax[1].set_title(r"state part $V_x(t)$ of the Lyapunov function, Eq. (15)")
    ax[1].set_xlabel("time (s)")
    save(fig, "fig08_theta_hat_and_lyapunov.png")
    return res, logs


def barrier_forms():
    e = np.linspace(-0.95, 0.95, 400)
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    ax.semilogy(e, 1.5 / (1 - e) ** 2, lw=2, label=r"paper Eq. (13): $\alpha(1-\varepsilon)^{-2}$")
    ax.semilogy(e, 1.5 / (1 - e ** 2) ** 2, "--", lw=2, label=r"symmetric variant: $\alpha(1-\varepsilon^2)^{-2}$")
    ax.set_xlabel(r"normalised error $\varepsilon$"); ax.set_ylabel("barrier gain"); ax.legend()
    ax.set_title(r"barrier gain vs $\varepsilon$ ($\alpha$=1.5)")
    save(fig, "fig09_barrier_forms.png")


def sensitivity():
    cases = [(15.0, 0.36), (15.0, 0.50), (10.0, 0.50), (10.0, 0.36)]
    out = []
    for Ts, rq0 in cases:
        p = dict(P, rho_q0=rq0, Ts=Ts)
        m = first(simulate(PART2["q0"], PART2["w0"], p, "constrained", record=False))
        out.append(dict(Ts=Ts, rho_q0=rq0, **m))
    p = dict(P, rho_q0=0.36, Ts=10.0)
    L = simulate(PART2["q0"], PART2["w0"], p, "constrained")["log"]
    t = L["t"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    env_plot(ax[0], t, L["rq"], L["q"][:, 0, :3], ["$q_1$", "$q_2$", "$q_3$"], "$q_v$",
             r"infeasible envelope: $T_s$=10 s, $\rho_{q0}$=0.36")
    for i in range(3):
        ax[1].plot(t, L["usat"][:, 0, i], color=C3[i], lw=1.2, label=f"$u_{i+1}$")
    ax[1].set_title("torque pinned at the 2.5 N·m limit"); ax[1].legend(ncol=3); ax[1].set_xlabel("time (s)")
    save(fig, "fig10_infeasible_envelope.png")
    return out


def wilson(k, n, z=1.96):
    ph = k / n
    den = 1 + z * z / n
    c = (ph + z * z / (2 * n)) / den
    h = z * np.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / den
    return float(c - h), float(c + h)


MODES = ("constrained", "symmetric", "pd")
MODE_LABEL = {"constrained": "constrained (paper Eq. 13)", "symmetric": "constrained (symmetric barrier)",
              "pd": "conventional PD"}
MODE_COL = {"constrained": "#1f77b4", "symmetric": "#2ca02c", "pd": "#d62728"}


def _mc_batch(q0, w0, scale, p):
    out = {}
    for mode in MODES:
        m = simulate(q0, w0, p, mode, dist_scale=scale, record=False)
        ok = (m["viol_q"] == 0) & (m["viol_w"] == 0)
        k = int(ok.sum()); N = len(ok)
        out[mode] = dict(
            N=N, violation_free=k, fraction=k / N, ci95=wilson(k, N),
            runs_with_q_violation=int((m["viol_q"] > 0).sum()),
            runs_with_w_violation=int((m["viol_w"] > 0).sum()),
            mean_viol_w=float(np.mean(m["viol_w"])), mean_viol_q=float(np.mean(m["viol_q"])),
            median_settle=float(np.median(m["settle"])), mean_energy=float(np.mean(m["energy"])),
            mean_w_final=float(np.mean(m["w_final"])),
            _ok=ok, _m=m)
    return out


def _binned(x, ok, bins):
    r = []
    for i in range(len(bins) - 1):
        sel = (x >= bins[i]) & (x < bins[i + 1])
        r.append(float(100 * ok[sel].mean()) if sel.any() else float("nan"))
    return r


def monte_carlo(N=300, seed=7):
    """WP2b robustness study (same random draws for every controller).
    Study A - the paper's operating regime: attitude anywhere inside 90 % of the
      envelope, |w_i(0)| <= 0.05 rad/s, the paper's disturbance d(t).
    Study B - disturbance amplitude: as A but d(t) scaled by U[0.5, 3].
    Study C - initial-rate stress: as A but |w_i(0)| up to 0.36 rad/s (90 % of rho_w0).
      With J ~ 17 kg m^2 and u_max = 2.5 N m the body decelerates at only
      ~0.15 rad/s^2, so large initial rates are physically unrecoverable."""
    rng = np.random.default_rng(seed)
    p = dict(P, rho_q0=0.36, Ts=15.0)

    def draw(wmax):
        qv = rng.uniform(-0.9, 0.9, (N, 3)) * p["rho_q0"]
        q4 = np.sqrt(1 - np.sum(qv ** 2, 1))
        return np.hstack([qv, q4[:, None]]), rng.uniform(-wmax, wmax, (N, 3))
    qA, wA = draw(0.05)
    A = _mc_batch(qA, wA, 1.0, p)
    sB = rng.uniform(0.5, 3.0, N)
    B = _mc_batch(qA, wA, sB, p)
    qC, wC = draw(0.9 * p["rho_w0"])
    C = _mc_batch(qC, wC, 1.0, p)

    sbins = np.array([0.5, 1.0, 1.5, 2.0, 2.5, 3.0001])
    wbins = np.linspace(0, 0.36, 7)
    wmag = np.max(np.abs(wC), axis=1)
    B_curve = {m: _binned(sB, B[m]["_ok"], sbins) for m in MODES}
    C_curve = {m: _binned(wmag, C[m]["_ok"], wbins) for m in MODES}

    fig, ax = plt.subplots(1, 3, figsize=(15.5, 3.9))
    xs = np.arange(3)
    fr = [100 * A[m]["fraction"] for m in MODES]
    lo = [fr[i] - 100 * A[m]["ci95"][0] for i, m in enumerate(MODES)]
    hi = [100 * A[m]["ci95"][1] - fr[i] for i, m in enumerate(MODES)]
    ax[0].bar(xs, fr, 0.55, yerr=[lo, hi], capsize=5, color=[MODE_COL[m] for m in MODES], edgecolor="k")
    for i, v in enumerate(fr): ax[0].text(i, v + 4, f"{v:.1f}%", ha="center")
    ax[0].set_xticks(xs); ax[0].set_xticklabels(["paper Eq.13", "symmetric", "conv. PD"]); ax[0].set_ylim(0, 115)
    ax[0].set_ylabel("violation-free runs (%)"); ax[0].set_title(f"A: paper regime, N={N}, 95% CI")
    cB = 0.5 * (sbins[1:] + sbins[:-1]); cB[-1] = 2.75
    for m in MODES:
        ax[1].plot(cB, B_curve[m], "o-", color=MODE_COL[m], label=MODE_LABEL[m])
    ax[1].set_xlabel("disturbance scale (× paper d(t))"); ax[1].set_ylabel("violation-free (%)")
    ax[1].set_title("B: disturbance amplitude sweep"); ax[1].set_ylim(-5, 105); ax[1].legend(fontsize=7.5)
    for m in MODES:
        ax[2].plot(0.5 * (wbins[1:] + wbins[:-1]), C_curve[m], "o-", color=MODE_COL[m], label=MODE_LABEL[m])
    ax[2].set_xlabel(r"initial $\max_i|\omega_i(0)|$ (rad/s)"); ax[2].set_ylabel("violation-free (%)")
    ax[2].set_title("C: initial-rate stress (feasibility limit)"); ax[2].set_ylim(-5, 105)
    save(fig, "fig11_monte_carlo.png")
    for S in (A, B, C):
        for m in S:
            S[m].pop("_ok"); S[m].pop("_m")
    return dict(study_A_paper_regime=A, study_B_disturbance_sweep=dict(per_mode=B, bins=sbins.tolist(), curve=B_curve),
                study_C_rate_stress=dict(per_mode=C, bins=wbins.tolist(), curve=C_curve))


def md_table(rows, cols, header):
    s = "| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n"
    for r in rows:
        s += "| " + " | ".join(r) + " |\n"
    return s


def main():
    t0 = time.time()
    print("FTPPF figure"); ppf = fig_ppf()
    print("Part 1"); p1, _ = part1()
    print("Part 2"); p2, _ = part2()
    print("barrier forms"); barrier_forms()
    print("sensitivity"); sens = sensitivity()
    print("Monte-Carlo"); mc = monte_carlo()
    metrics = dict(parameters=P, unstated_choices=dict(c=0.5, varsigma=0.02, part2_Ts=15.0, part2_rho_q0=0.36),
                   ppf=ppf, part1=p1, part2=p2, sensitivity=sens, monte_carlo=mc,
                   solver=dict(method="RK4, zero-order-hold torque", dt=DT, t_end=T_END))
    with open(os.path.join(RES, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    f3 = lambda x: f"{x:.3f}"
    md = "# Paper reproduction - metrics (generated by paper_reproduction.py)\n\n"
    md += "## Part 1 (constrained law)\n\n"
    rows = []
    for k, lab in [("eps1_max", "max abs eps1"), ("eps2_max", "max abs eps2"), ("viol_q", "time outside q envelope (%)"),
                   ("viol_w", "time outside w envelope (%)"), ("settle", "settling time ||qv||<0.01 (s)"),
                   ("qv_final", "final ||qv||"), ("w_final", "final ||w|| (rad/s)"), ("at_limit", "time at torque limit (%)"),
                   ("upeak", "peak commanded torque (N m)"), ("theta_final", "theta_hat at 20 s")]:
        rows.append([lab, f"{p1['Ts=15'][k]:.4g}", f"{p1['Ts=10'][k]:.4g}"])
    md += md_table(rows, None, ["metric", "Ts = 15 s", "Ts = 10 s"])
    md += "\n## Part 2 (Ts = 15 s, rho_q0 = 0.36)\n\n"
    rows = []
    for k, lab in [("eps1_max", "max abs eps1"), ("eps2_max", "max abs eps2"), ("viol_q", "time outside q envelope (%)"),
                   ("viol_w", "time outside w envelope (%)"), ("at_limit", "time at torque limit (%)"),
                   ("upeak", "peak commanded torque (N m)"), ("settle", "settling time (s)"),
                   ("w_final", "final ||w|| (rad/s)"), ("qv_final", "final ||qv||"), ("energy", "torque energy int ||sat(u)||^2 dt"),
                   ("theta_final", "final theta_hat")]:
        rows.append([lab] + [f"{p2[m][k]:.4g}" for m in ("constrained", "symmetric", "pd")])
    md += md_table(rows, None, ["metric", "constrained (paper Eq. 13)", "constrained (symmetric barrier)", "conventional PD"])
    md += "\n## Sensitivity (constrained, Part 2 initial state)\n\n"
    rows = [[f"{s['Ts']:g}", f"{s['rho_q0']:.2f}", f"{s['viol_q']:.1f}", f"{s['viol_w']:.1f}",
             f"{s['at_limit']:.1f}", f"{s['eps1_max']:.3f}"] for s in sens]
    md += md_table(rows, None, ["Ts (s)", "rho_q0", "q outside (%)", "w outside (%)", "torque at limit (%)", "max abs eps1"])
    def mc_rows(S):
        return [[MODE_LABEL[k], str(v["N"]), f"{100*v['fraction']:.1f}", f"[{100*v['ci95'][0]:.1f}, {100*v['ci95'][1]:.1f}]",
                 str(v["runs_with_q_violation"]), str(v["runs_with_w_violation"]),
                 f"{v['median_settle']:.2f}", f"{v['mean_energy']:.1f}"] for k, v in S.items()]
    hdr = ["controller", "N", "violation-free (%)", "95% CI", "runs with q violation", "runs with w violation",
           "median settling (s)", "mean torque energy"]
    md += "\n## Monte-Carlo A - paper regime (|w0|<=0.05 rad/s, paper disturbance)\n\n"
    md += md_table(mc_rows(mc["study_A_paper_regime"]), None, hdr)
    md += "\n## Monte-Carlo B - disturbance scale U[0.5, 3]\n\n"
    md += md_table(mc_rows(mc["study_B_disturbance_sweep"]["per_mode"]), None, hdr)
    bins = mc["study_B_disturbance_sweep"]["bins"]
    md += "\nViolation-free (%) by disturbance scale bin:\n\n"
    md += md_table([[MODE_LABEL[m]] + [f"{v:.0f}" for v in c] for m, c in mc["study_B_disturbance_sweep"]["curve"].items()],
                   None, ["controller"] + [f"{bins[i]:.1f}-{min(bins[i+1],3):.1f}x" for i in range(len(bins) - 1)])
    md += "\n## Monte-Carlo C - initial-rate stress (|w0| up to 0.36 rad/s)\n\n"
    md += md_table(mc_rows(mc["study_C_rate_stress"]["per_mode"]), None, hdr)
    bins = mc["study_C_rate_stress"]["bins"]
    md += "\nViolation-free (%) by initial max|w_i(0)| bin (rad/s):\n\n"
    md += md_table([[MODE_LABEL[m]] + [f"{v:.0f}" for v in c] for m, c in mc["study_C_rate_stress"]["curve"].items()],
                   None, ["controller"] + [f"{bins[i]:.2f}-{bins[i+1]:.2f}" for i in range(len(bins) - 1)])
    with open(os.path.join(RES, "metrics.md"), "w") as f:
        f.write(md)
    print(md)
    print(f"done in {time.time()-t0:.1f} s")


if __name__ == "__main__":
    main()
