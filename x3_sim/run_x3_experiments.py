#!/usr/bin/env python3
"""
run_x3_experiments.py - all X3 software-in-the-loop experiments of the final report.

    E0  original gazebo.py (v1) on the X3 surrogate                -> reproduces the crash
    E1  fault isolation: v1 with each fix switched on separately    -> which fault causes what
    E2  corrected controller (v2), 180 s hover, no disturbance
    E3  disturbance injection (paper d(t) x 0.5) - paper law vs symmetric barrier vs conventional PD
    E3b disturbance at full paper amplitude - X3 yaw-authority feasibility limit
    E4  waypoint (square) trajectory tracking - constrained vs conventional PD
    E5  robustness to control rate, transport delay and time-stamp jitter

Writes docs/figures/x3_*.png and docs/results/x3_metrics.{json,md}.
Run:  python x3_sim/run_x3_experiments.py        (numpy, matplotlib; ~10 min on 2 cores)
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from x3_model import run, allocation_matrix, J_BODY, MASS, KT, CQ, W_MAX  # noqa: E402
from controllers import OriginalGazeboController, FixedController, original_mixer_effective_matrix  # noqa: E402

ROOT = os.path.dirname(HERE)
FIG = os.path.join(ROOT, "docs", "figures")
RES = os.path.join(ROOT, "docs", "results")
os.makedirs(FIG, exist_ok=True); os.makedirs(RES, exist_ok=True)
plt.rcParams.update({"figure.dpi": 110, "savefig.dpi": 160, "font.size": 10, "axes.grid": True,
                     "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False,
                     "legend.fontsize": 8.5})
C3 = ["#1f77b4", "#d62728", "#2ca02c"]
ENV = "#cfcfcf"
MODE_LABEL = {"constrained": "constrained (paper Eq. 13)", "symmetric": "constrained (symmetric barrier)",
              "pd": "conventional PD"}
MODE_COL = {"constrained": "#1f77b4", "symmetric": "#2ca02c", "pd": "#d62728"}
# tiny asymmetric torque (2e-4 N m) so that a perfectly symmetric simulation cannot hide an instability
NOISE = lambda t: 2e-4 * np.array([np.sin(3.1 * t), np.cos(2.3 * t), np.sin(1.7 * t)])


def paper_disturbance(scale, start=15.0, stop=None):
    def d(t):
        if t < start or (stop is not None and t > stop):
            return NOISE(t)
        tt = t - start
        return NOISE(t) + scale * np.array([0.05 * np.sin(tt), 0.10 * np.sin(1.2 * tt), 0.15 * np.sin(1.5 * tt)])
    return d


def save(fig, name):
    fig.tight_layout(); fig.savefig(os.path.join(FIG, name)); plt.close(fig)
    print("  wrote docs/figures/" + name, flush=True)


def clog(c):
    return {k: np.array(v) for k, v in c.log.items()}


def env_metrics(l, t_from=0.0):
    i = l["t"] >= t_from
    vq = (np.abs(l["qe"]) >= l["rho_q"][:, None]).any(1)[i]
    vw = (np.abs(l["omega"]) >= l["rho_w"][:, None]).any(1)[i]
    return dict(viol_q_pct=float(100 * vq.mean()), viol_w_pct=float(100 * vw.mean()),
                eps1_max=float(np.abs(l["eps1"][i]).max()), eps2_max=float(np.abs(l["eps2"][i]).max()),
                eps1_max_axis=np.abs(l["eps1"][i]).max(0).tolist(), eps2_max_axis=np.abs(l["eps2"][i]).max(0).tolist(),
                alloc_saturated_pct=float(100 * l["sat_frac"][i].mean()),
                theta_final=float(l["theta"][-1]))


# ---------------------------------------------------------------------------
# Jobs (run in a process pool)
# ---------------------------------------------------------------------------
def job(spec):
    kind = spec["kind"]
    t0 = time.time()
    if kind == "v1":
        c = OriginalGazeboController(fixes=spec.get("fixes", ()))
        L = run(c, t_end=spec.get("t_end", 60), ctrl_hz=spec.get("hz", 60), wind_torque=NOISE)
        out = dict(spec=spec, L=L, log=clog(c))
    else:
        c = FixedController(mode=spec.get("mode", "constrained"), trajectory=spec.get("traj") and square_traj)
        wt = paper_disturbance(spec["dist"], spec.get("d_start", 15.0), spec.get("d_stop")) if spec.get("dist") else NOISE
        L = run(c, t_end=spec.get("t_end", 180), ctrl_hz=spec.get("hz", 60), delay_samples=spec.get("delay", 1),
                wind_torque=wt, jitter=spec.get("jitter", 0.0), seed=spec.get("seed", 0))
        out = dict(spec=spec, L=L, log=clog(c))
    out["wall"] = time.time() - t0
    return out


from trajectories import square_traj, T_TAKEOFF  # noqa: E402


# ---------------------------------------------------------------------------
def main():
    t_start = time.time()
    specs = []
    # E0/E1
    ablation = [(), ("zdot",), ("mixer",), ("mixer", "zdot"), ("mixer", "zdot", "omega")]
    for fx in ablation:
        specs.append(dict(name="E1:" + "+".join(fx or ("none",)), kind="v1", fixes=fx, t_end=60))
    specs.append(dict(name="E0:v1@250Hz", kind="v1", fixes=(), t_end=60, hz=250))
    # E2
    specs.append(dict(name="E2:hover", kind="v2", mode="constrained", t_end=180))
    # E3
    for m in ("constrained", "symmetric", "pd"):
        specs.append(dict(name=f"E3:{m}", kind="v2", mode=m, dist=0.5, t_end=180))
    for m in ("constrained", "symmetric", "pd"):
        specs.append(dict(name=f"E3b:{m}", kind="v2", mode=m, dist=1.0, t_end=60))
    # E4
    for m in ("constrained", "pd"):
        specs.append(dict(name=f"E4:{m}", kind="v2", mode=m, traj=True, t_end=45))
    # E5
    for hz, dl, jit in ((60, 1, 0.0), (60, 2, 0.0), (60, 1, 0.002), (100, 1, 0.0), (250, 1, 0.0), (30, 1, 0.0)):
        for m in ("constrained", "symmetric", "pd"):
            specs.append(dict(name=f"E5:{m}:{hz}Hz-d{dl}-j{jit}", kind="v2", mode=m, dist=0.5,
                              t_end=90, hz=hz, delay=dl, jitter=jit, seed=3))
    with Pool(2) as pool:
        R = {}
        for r in pool.imap_unordered(job, specs):
            R[r["spec"]["name"]] = r
            print(f"  done {r['spec']['name']:<28s} crashed={r['L']['crashed']} ({r['wall']:.0f}s)", flush=True)
    M = {}

    # ------------------ mixer analysis -------------------------------------
    Meff = original_mixer_effective_matrix()
    M["v1_mixer_effective_torque_matrix"] = Meff.round(4).tolist()
    M["x3_constants"] = dict(mass=MASS, J_diag=np.diag(J_BODY).round(5).tolist(), kT=KT, cQ=CQ, w_max=W_MAX,
                             hover_rotor_speed=float(np.sqrt(MASS * 9.81 / 4 / KT)),
                             allocation_matrix=allocation_matrix().round(5).tolist())
    A = allocation_matrix(); Fh = MASS * 9.81 / 4; Fmax = KT * W_MAX ** 2
    auth = []
    for ax in range(3):
        e = np.zeros(4); e[ax + 1] = 1; d = np.linalg.inv(A) @ e
        auth.append(float(min((Fmax - Fh) / np.max(d), Fh / np.max(-d))))
    M["x3_max_torque_at_hover_Nm"] = auth

    # ------------------ E0 / E1 --------------------------------------------
    E1 = {}
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8))
    for fx, col in zip(ablation, ["k", "#ff7f0e", "#9467bd", "#8c564b", "#2ca02c"]):
        name = "E1:" + "+".join(fx or ("none",))
        r = R[name]; L = r["L"]
        lab = "v1 as is" if not fx else "v1 + fix " + " + ".join(fx)
        ax[0].plot(L["t"], L["p"][:, 2], color=col, lw=1.4, label=lab)
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (L["q"][:, 0] ** 2 + L["q"][:, 1] ** 2), -1, 1)))
        ax[1].semilogy(L["t"], np.maximum(tilt, 1e-3), color=col, lw=1.2, label=lab)
        i = L["t"] > 20
        E1[lab] = dict(crashed=bool(L["crashed"]), crash_time=L["crash_time"], max_altitude=float(L["p"][:, 2].max()),
                       final_altitude=float(L["p"][-1, 2]),
                       rms_tilt_deg_after_20s=(float(np.sqrt(np.mean(tilt[i] ** 2))) if i.any() and not L["crashed"] else None))
        if L["crashed"]:
            ax[0].plot(L["crash_time"], 0.05, "x", color=col, ms=10, mew=2)
    ax[0].axhline(2.0, color="grey", ls=":"); ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("altitude z (m)")
    ax[0].set_title("fault isolation on the X3 surrogate (x = crash)"); ax[0].legend(fontsize=7.5)
    ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("tilt angle (deg, log)"); ax[1].set_title("tilt from vertical")
    ax[1].set_ylim(1e-3, 200)
    save(fig, "x3_E1_fault_isolation.png")
    r = R["E0:v1@250Hz"]["L"]
    E1["v1 as is @250 Hz"] = dict(crashed=bool(r["crashed"]), crash_time=r["crash_time"], max_altitude=float(r["p"][:, 2].max()))
    M["E1_fault_isolation"] = E1
    # v1 original detailed timeline (zdot-only variant = the behaviour seen in Gazebo: climbs, then flips)
    L = R["E1:zdot"]["L"]; l = R["E1:zdot"]["log"]
    fig, ax = plt.subplots(1, 3, figsize=(14, 3.6))
    ax[0].plot(L["t"], L["p"][:, 2]); ax[0].set_title("altitude (v1 + z-velocity fix only)"); ax[0].set_ylabel("m")
    for k in range(3):
        ax[1].plot(L["t"], np.degrees(L["euler"][:, k]), color=C3[k], label=["roll", "pitch", "yaw"][k])
    ax[1].set_title("attitude diverges: pitch has no control authority"); ax[1].legend(); ax[1].set_ylabel("deg")
    ax[1].set_ylim(-190, 190)
    ax[2].plot(l["t"], l["tau"][:, 0], label=r"commanded $\tau_x$"); ax[2].plot(l["t"], l["tau"][:, 1], label=r"commanded $\tau_y$")
    ax[2].set_title("controller saturates while the vehicle tips"); ax[2].legend(); ax[2].set_ylabel("N·m")
    for a in ax: a.set_xlabel("time (s)")
    save(fig, "x3_E0_v1_failure.png")

    # ------------------ E2 hover --------------------------------------------
    r = R["E2:hover"]; L, l = r["L"], r["log"]
    i10 = L["t"] >= 10
    e2 = dict(crashed=bool(L["crashed"]),
              alt_rms_err_after_10s_m=float(np.sqrt(np.mean((L["p"][i10, 2] - 2.0) ** 2))),
              alt_max_abs_err_after_10s_m=float(np.max(np.abs(L["p"][i10, 2] - 2.0))),
              xy_max_drift_m=float(np.max(np.linalg.norm(L["p"][:, :2], axis=1))),
              tilt_rms_deg_after_10s=float(np.degrees(np.sqrt(np.mean(L["euler"][i10, :2] ** 2)))),
              yaw_max_abs_deg=float(np.degrees(np.max(np.abs(L["euler"][:, 2])))),
              rotor_speed_min=float(L["cmd"][i10].min()), rotor_speed_max=float(L["cmd"][i10].max()))
    e2.update(env_metrics(l))
    M["E2_hover_180s"] = e2
    fig, ax = plt.subplots(3, 2, figsize=(12, 9))
    ax[0, 0].plot(L["t"], L["p"][:, 2], label="z"); ax[0, 0].axhline(2, color="grey", ls=":")
    ax[0, 0].set_title("altitude (target 2.0 m after 8 s ramp)"); ax[0, 0].set_ylabel("m")
    ax[0, 1].plot(L["t"], L["p"][:, 0] * 100, label="x"); ax[0, 1].plot(L["t"], L["p"][:, 1] * 100, label="y")
    ax[0, 1].set_title("horizontal position (position hold)"); ax[0, 1].set_ylabel("cm"); ax[0, 1].legend()
    for k in range(3):
        ax[1, 0].plot(l["t"], l["qe"][:, k], color=C3[k], lw=1.1, label=f"$q_{{e,{k+1}}}$")
        ax[1, 1].plot(l["t"], l["omega"][:, k], color=C3[k], lw=1.1, label=fr"$\omega_{k+1}$")
    ax[1, 0].fill_between(l["t"], -l["rho_q"], l["rho_q"], color=ENV, alpha=0.6, lw=0)
    ax[1, 1].fill_between(l["t"], -l["rho_w"], l["rho_w"], color=ENV, alpha=0.6, lw=0)
    ax[1, 0].set_title(r"attitude error inside $\pm\rho_q(t)$"); ax[1, 0].legend(ncol=3)
    ax[1, 1].set_title(r"body rate inside $\pm\rho_\omega(t)$"); ax[1, 1].legend(ncol=3); ax[1, 1].set_ylabel("rad/s")
    ax[1, 0].set_ylim(-0.32, 0.32); ax[1, 1].set_ylim(-2.6, 2.6)
    for k in range(4):
        ax[2, 0].plot(L["t"], L["cmd"][:, k], lw=0.9, label=f"rotor {k}")
    ax[2, 0].set_title("rotor speed commands"); ax[2, 0].set_ylabel("rad/s"); ax[2, 0].legend(ncol=4)
    ax[2, 1].plot(l["t"], l["theta"]); ax[2, 1].set_title(r"adaptive estimate $\hat\theta$")
    for a in ax.ravel(): a.set_xlabel("time (s)")
    save(fig, "x3_E2_hover_180s.png")

    # ------------------ E3 disturbance --------------------------------------
    E3 = {}
    fig, ax = plt.subplots(3, 3, figsize=(14, 8.5), sharex=True)
    for j, m in enumerate(("constrained", "symmetric", "pd")):
        r = R[f"E3:{m}"]; L, l = r["L"], r["log"]
        met = env_metrics(l, t_from=15.0); met["crashed"] = bool(L["crashed"])
        i = L["t"] >= 15
        met["tilt_rms_deg"] = float(np.degrees(np.sqrt(np.mean(L["euler"][i, :2] ** 2))))
        met["yaw_rms_deg"] = float(np.degrees(np.sqrt(np.mean(L["euler"][i, 2] ** 2))))
        met["xy_max_err_m"] = float(np.max(np.linalg.norm(L["p"][i, :2], axis=1)))
        E3[m] = met
        sel = l["t"] < 60
        for k in range(3):
            a = ax[k, j]
            a.fill_between(l["t"][sel], -l["rho_w"][sel], l["rho_w"][sel], color=ENV, alpha=0.6, lw=0)
            a.plot(l["t"][sel], l["omega"][sel, k], color=MODE_COL[m], lw=1.0)
            out = np.abs(l["omega"][sel, k]) >= l["rho_w"][sel]
            if out.any(): a.plot(l["t"][sel][out], l["omega"][sel, k][out], ".", color="k", ms=2)
            a.set_ylim(-0.15, 0.15)
            a.axvline(15, color="grey", ls=":")
            if k == 0: a.set_title(MODE_LABEL[m])
            if j == 0: a.set_ylabel([r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"][k] + " (rad/s)")
    for a in ax[-1]: a.set_xlabel("time (s)  (disturbance on from 15 s)")
    save(fig, "x3_E3_disturbance_rates.png")
    M["E3_disturbance_x0.5"] = E3
    E3b = {}
    for m in ("constrained", "symmetric", "pd"):
        r = R[f"E3b:{m}"]; l = r["log"]; L = r["L"]
        met = env_metrics(l, t_from=15.0); met["crashed"] = bool(L["crashed"])
        met["yaw_torque_cmd_peak"] = float(np.max(np.abs(l["tau"][l["t"] >= 15, 2])))
        E3b[m] = met
    M["E3b_disturbance_x1.0"] = E3b
    M["E3b_note"] = ("paper yaw disturbance 0.15 N m exceeds the X3's maximum yaw torque at hover "
                     f"({auth[2]:.3f} N m): no controller can reject it")
    # bar chart for E3
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    ms = ("constrained", "symmetric", "pd")
    ax[0].bar(range(3), [E3[m]["viol_q_pct"] for m in ms], color=[MODE_COL[m] for m in ms])
    ax[1].bar(range(3), [E3[m]["viol_w_pct"] for m in ms], color=[MODE_COL[m] for m in ms])
    for a, key, tt in ((ax[0], "viol_q_pct", "time outside attitude envelope (%)"),
                       (ax[1], "viol_w_pct", "time outside rate envelope (%)")):
        a.set_xticks(range(3)); a.set_xticklabels(["paper Eq.13", "symmetric", "conv. PD"]); a.set_title(tt)
        for k, m in enumerate(ms): a.text(k, E3[m][key] + 0.5, f"{E3[m][key]:.1f}", ha="center")
    save(fig, "x3_E3_violation_bars.png")

    # ------------------ E4 trajectory ---------------------------------------
    E4 = {}
    fig = plt.figure(figsize=(13, 4.2))
    a3 = fig.add_subplot(1, 3, 1, projection="3d")
    a2 = fig.add_subplot(1, 3, 2); a1 = fig.add_subplot(1, 3, 3)
    tt = np.linspace(0, 45, 2000); ref = np.array([square_traj(x)[0] for x in tt])
    a3.plot(ref[:, 0], ref[:, 1], ref[:, 2], "k--", lw=1, label="reference")
    a2.plot(ref[:, 0], ref[:, 1], "k--", lw=1, label="reference")
    for m in ("constrained", "pd"):
        r = R[f"E4:{m}"]; L, l = r["L"], r["log"]
        err = np.linalg.norm(l["p"] - l["p_d"], axis=1)
        i = l["t"] >= T_TAKEOFF
        met = env_metrics(l, t_from=T_TAKEOFF)
        met.update(crashed=bool(L["crashed"]), rms_pos_err_m=float(np.sqrt(np.mean(err[i] ** 2))),
                   max_pos_err_m=float(err[i].max()))
        E4[m] = met
        a3.plot(L["p"][:, 0], L["p"][:, 1], L["p"][:, 2], color=MODE_COL[m], lw=1.3, label=MODE_LABEL[m])
        a2.plot(L["p"][:, 0], L["p"][:, 1], color=MODE_COL[m], lw=1.3, label=MODE_LABEL[m])
        a1.plot(l["t"], err * 100, color=MODE_COL[m], lw=1.2, label=MODE_LABEL[m])
    a3.set_title("3-D path"); a3.set_xlabel("x"); a3.set_ylabel("y"); a3.set_zlabel("z")
    a2.set_title("top view (m)"); a2.set_aspect("equal"); a2.legend(fontsize=7)
    a1.set_title("position error (cm)"); a1.set_xlabel("time (s)"); a1.legend(fontsize=7)
    save(fig, "x3_E4_square_trajectory.png")
    M["E4_square_trajectory"] = E4

    # ------------------ E5 robustness ---------------------------------------
    E5 = {}
    for k, r in R.items():
        if not k.startswith("E5:"):
            continue
        met = env_metrics(r["log"], t_from=15.0); met["crashed"] = bool(r["L"]["crashed"])
        E5[k[3:]] = met
    M["E5_robustness"] = E5

    with open(os.path.join(RES, "x3_metrics.json"), "w") as f:
        json.dump(M, f, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    # markdown
    md = "# X3 software-in-the-loop results (generated by x3_sim/run_x3_experiments.py)\n\n"
    md += "## v1 mixer: torque produced per unit torque commanded (rows x,y,z produced; cols x,y,z commanded)\n\n"
    md += "| | cmd tau_x | cmd tau_y | cmd tau_z |\n|---|---|---|---|\n"
    for k, row in zip("xyz", Meff):
        md += f"| produced tau_{k} | " + " | ".join(f"{v:+.3f}" for v in row) + " |\n"
    md += f"\nX3 maximum torque at hover (N m): roll {auth[0]:.3f}, pitch {auth[1]:.3f}, yaw {auth[2]:.3f}\n"
    md += "\n## E1 fault isolation (60 Hz)\n\n| variant | crashed | crash time (s) | max altitude (m) | final altitude (m) | RMS tilt after 20 s (deg) |\n|---|---|---|---|---|---|\n"
    for k, v in E1.items():
        md += f"| {k} | {v['crashed']} | {v['crash_time'] if v['crash_time'] is None else round(v['crash_time'],2)} | {v['max_altitude']:.2f} | {v.get('final_altitude', float('nan')):.2f} | {v.get('rms_tilt_deg_after_20s')} |\n"
    md += "\n## E2 hover 180 s (v2, paper law, no disturbance)\n\n| metric | value |\n|---|---|\n"
    for k, v in e2.items():
        md += f"| {k} | {v if not isinstance(v, float) else round(v, 5)} |\n"
    def tbl(D, keys):
        s = "| metric | " + " | ".join(MODE_LABEL[m] for m in D) + " |\n|---|" + "---|" * len(D) + "\n"
        for k in keys:
            s += f"| {k} | " + " | ".join(str(round(D[m][k], 4)) if isinstance(D[m][k], float) else str(D[m][k]) for m in D) + " |\n"
        return s
    keys = ["crashed", "viol_q_pct", "viol_w_pct", "eps1_max", "eps2_max", "alloc_saturated_pct", "theta_final"]
    md += "\n## E3 disturbance = 0.5 x paper d(t), 15-180 s\n\n" + tbl(E3, keys + ["tilt_rms_deg", "yaw_rms_deg", "xy_max_err_m"])
    md += "\n## E3b disturbance = 1.0 x paper d(t) (yaw infeasible)\n\n" + tbl(E3b, keys + ["yaw_torque_cmd_peak"])
    md += "\n## E4 square trajectory 2 m x 2 m at 2 m\n\n" + tbl(E4, ["crashed", "rms_pos_err_m", "max_pos_err_m", "viol_q_pct", "viol_w_pct"])
    md += "\n## E5 robustness (disturbance 0.5x; controller:rate-delay-jitter)\n\n| case | crashed | q outside (%) | w outside (%) | max eps1 | max eps2 |\n|---|---|---|---|---|---|\n"
    for k, v in E5.items():
        md += f"| {k} | {v['crashed']} | {v['viol_q_pct']:.2f} | {v['viol_w_pct']:.2f} | {v['eps1_max']:.3f} | {v['eps2_max']:.3f} |\n"
    with open(os.path.join(RES, "x3_metrics.md"), "w") as f:
        f.write(md)
    print(md)
    print(f"total {time.time()-t_start:.0f} s")


if __name__ == "__main__":
    main()
