# MATLAB toy model (AB02)

Small MATLAB scripts that make every block of the constrained PD-type controller tangible. Each step prints its inputs
and outputs (`[IN ]`, `[OUT]`, `[CHK]` with size, unit and meaning), reproduces the hand-calculated toy examples of
`docs/mathematical_models/Mathematical_Models.pdf`, and saves one figure. No toolbox is needed (MATLAB R2016b or newer).

| Step | Block(s) | Shows |
|---|---|---|
| `step1_ftppf.m` | 3 | finite-time envelope, rho_q(2) = 0.206042 by hand, effect of T_s and kappa |
| `step2_normalised_error.m` | 4, 5 | eps1, eps2 of the paper's Part-1 state; barrier gains, one-sided vs symmetric |
| `step3_control_law.m` | 5, 2 | s, u_PD, u_cons, u = [-10.537 5.710 10.414], sat(u) at t = 0 |
| `step4_adaptive_law.m` | 6 | theta_hat: first Euler step, drift, leakage floor |
| `step5_single_axis.m` | 1–6 | one axis: constrained vs PD with the same gains and limit |
| `step6_spacecraft_closed_loop.m` | 1–6 | independent re-implementation of the paper's Part 1 and Part 2 |
| `step7_x3_mixer.m` | 10 | X3 allocation matrix, hover 660.4 rad/s, torque authority, the v1 mixer error |
| `step8_sampled_data_limit.m` | 9 | largest stable gain on a 60 Hz loop with one sample of delay |

Run everything: `run('<repo>/toy_model/run_all.m')`. It writes `figures/*.png` and the complete printed output to
`results_log.txt`. Then `python3 docs/make_crosscheck.py` fills the Python-vs-MATLAB table of the report, and recompiling
the two PDFs includes the MATLAB figures and log automatically.
