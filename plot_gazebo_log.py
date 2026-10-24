#!/usr/bin/env python3
"""
plot_gazebo_log.py - turn a log written by gazebo.py (v2) into report figures + metrics.

    python3 plot_gazebo_log.py /tmp/drone_constrained_hover.npz [more.npz ...]

For every log it writes  docs/figures/gazebo_<mode>_<experiment>.png  and
docs/results/gazebo_<mode>_<experiment>.json  (same metric definitions as the
surrogate experiments, so the two can be compared line by line).
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(ROOT, "docs", "figures"); RES = os.path.join(ROOT, "docs", "results")
os.makedirs(FIG, exist_ok=True); os.makedirs(RES, exist_ok=True)
C3 = ["#1f77b4", "#d62728", "#2ca02c"]


def euler(q):
    x, y, z, w = q.T
    return np.degrees(np.array([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                                np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                                np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))]).T)


def process(path):
    d = np.load(path, allow_pickle=True)
    mode, exp = str(d["mode"]), str(d["experiment"])
    t = d["ctrl_t"]; qe = d["ctrl_qe"]; w = d["ctrl_omega"]; rq = d["ctrl_rho_q"]; rw = d["ctrl_rho_w"]
    p = d["ctrl_p"]; pd = d["ctrl_p_d"]
    st = d["raw_stamp"] - d["raw_stamp"][0]; q = d["raw_q"]; cmd = d["raw_cmd"]
    i10 = t >= 10
    vq = (np.abs(qe) >= rq[:, None]).any(1); vw = (np.abs(w) >= rw[:, None]).any(1)
    err = np.linalg.norm(p - pd, axis=1)
    m = dict(log=os.path.basename(path), mode=mode, experiment=exp, duration_s=float(t[-1]),
             samples=int(len(t)), mean_rate_hz=float(len(t) / max(t[-1], 1e-9)),
             viol_q_pct=float(100 * vq.mean()), viol_w_pct=float(100 * vw.mean()),
             eps1_max=float(np.abs(d["ctrl_eps1"]).max()), eps2_max=float(np.abs(d["ctrl_eps2"]).max()),
             alt_rms_err_after_10s=float(np.sqrt(np.mean((p[i10, 2] - pd[i10, 2]) ** 2))),
             pos_rms_err_after_10s=float(np.sqrt(np.mean(err[i10] ** 2))),
             tilt_rms_deg_after_10s=float(np.sqrt(np.mean(euler(q)[st >= 10][:, :2] ** 2))),
             theta_final=float(d["ctrl_theta"][-1]))
    fig, ax = plt.subplots(3, 2, figsize=(12, 9))
    ax[0, 0].plot(t, p[:, 2], label="z"); ax[0, 0].plot(t, pd[:, 2], "k--", lw=0.8, label="reference")
    ax[0, 0].set_title("altitude (m)"); ax[0, 0].legend()
    ax[0, 1].plot(p[:, 0], p[:, 1]); ax[0, 1].plot(pd[:, 0], pd[:, 1], "k--", lw=0.8)
    ax[0, 1].set_title("horizontal path (m)"); ax[0, 1].set_aspect("equal", adjustable="datalim")
    for k in range(3):
        ax[1, 0].plot(t, qe[:, k], color=C3[k], lw=1); ax[1, 1].plot(t, w[:, k], color=C3[k], lw=1)
    ax[1, 0].fill_between(t, -rq, rq, color="#cfcfcf", alpha=0.6, lw=0); ax[1, 0].set_title("attitude error vs envelope")
    ax[1, 1].fill_between(t, -rw, rw, color="#cfcfcf", alpha=0.6, lw=0); ax[1, 1].set_title("body rate vs envelope (rad/s)")
    for k in range(4):
        ax[2, 0].plot(st, cmd[:, k], lw=0.8, label=f"rotor {k}")
    ax[2, 0].set_title("rotor commands (rad/s)"); ax[2, 0].legend(ncol=4, fontsize=7)
    ax[2, 1].plot(t, d["ctrl_theta"]); ax[2, 1].set_title(r"$\hat\theta$")
    for a in ax.ravel()[[0, 2, 3, 4, 5]]: a.set_xlabel("time (s)")
    fig.suptitle(f"Gazebo X3 - {mode} - {exp}")
    fig.tight_layout()
    name = f"gazebo_{mode}_{exp}"
    fig.savefig(os.path.join(FIG, name + ".png"), dpi=150); plt.close(fig)
    with open(os.path.join(RES, name + ".json"), "w") as f:
        json.dump(m, f, indent=2)
    print(json.dumps(m, indent=2))


if __name__ == "__main__":
    for path in sys.argv[1:]:
        process(path)
