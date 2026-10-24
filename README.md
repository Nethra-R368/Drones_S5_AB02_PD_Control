<p align="center">
  <img src="amrita.png" alt="Logo" width="400"/>
</p>

# A Simple Structure Constrained Attitude Control for Rigid Bodies: A PD-Type Control

<div align="center">

**Group AB02 | Introduction to Drones (22AIE448) | Amrita Vishwa Vidyapeetham**

</div>

---

## Team Members

| Name | Roll Number |
| :--- | :--- |
| Nethra R | CB.SC.U4AIE24147 |
| Dheeraj S | CB.SC.U4AIE24050 |
| Raaman Namputhiri | CB.SC.U4AIE24148 |
| Jyothsna | CB.SC.U4AIE24117 |

**Base paper:** M. Golestani, K. A. Alattas, S. Ud Din, S. Mobayen, A. K. Alanazi, A. Fekih, "A Simple Structure Constrained Attitude Control for Rigid Bodies: A PD-Type Control," *IEEE Access*, vol. 10, pp. 10202–10209, 2022. DOI [10.1109/ACCESS.2022.3144627](https://doi.org/10.1109/ACCESS.2022.3144627) (`base paper.pdf`).

---

## Documentation

| Document | Content |
| :--- | :--- |
| [`docs/complete_documentation/AB02_Drone_Documentation.pdf`](docs/complete_documentation/AB02_Drone_Documentation.pdf) | The full report: controller in code, reproduction of the paper, Monte-Carlo and sensitivity studies, findings, Gazebo X3 integration (why v1 crashed, how it was fixed), X3 results, demos, MATLAB toy model, claims |
| [`docs/mathematical_models/Mathematical_Models.pdf`](docs/mathematical_models/Mathematical_Models.pdf) | Learning guide: every equation boxed, with a symbol table, derivation, hand-checkable toy example and its place in our code; two equation-flow diagrams |
| [`toy_model/`](toy_model/) | MATLAB toy model (steps 1–8) that prints every toy example and plots each idea |
| [`docs/results/`](docs/results/) | Every number of the report (`metrics.md/json`, `x3_metrics.md/json`) |
| [`docs/figures/`](docs/figures/) | Every figure of the report |

---

## 1. Abstract

The base paper proposes a PD-type attitude controller that keeps **both** the attitude quaternion and the angular velocity of a rigid body inside finite-time prescribed-performance envelopes, under actuator saturation and an unknown bounded disturbance. Instead of the transformed-error machinery of classical prescribed performance control, it adds to an ordinary PD law one barrier-weighted damping term and one scalar adaptive estimate.

We implemented every equation of the paper, reproduced both of its simulation studies with quantitative metrics, tested the controller beyond the paper (three 300-run Monte-Carlo studies and a sensitivity study), and integrated it on the Gazebo **X3** quadcopter through ROS 2, where we identified and fixed the faults that made our first node crash.

## 2. Key results

| Result | Value |
| :--- | :--- |
| Part 1 (T_s = 15 s and 10 s): time outside the envelopes | **0 %** for q and ω; peak \|ε\| 0.912 / 0.942; settling 5.25 s / 4.53 s |
| Part 2, same 2.5 N·m limit: time outside the rate envelope | constrained **0 %** vs conventional PD **27.5 %** |
| Part 2: torque energy, final rate error | **−36 %** energy, final ‖ω‖ 0.0040 vs 0.0083 rad/s |
| Monte-Carlo, paper regime (N = 300) | **97.0 %** violation-free (95 % CI 94.4–98.4) vs PD 0.0 % |
| Disturbance up to 3× the paper's | symmetric barrier 97.3 % vs paper's one-sided form 21.3 % |
| X3, cause of the v1 crash | mixer signs not matching the X3 (no pitch authority) + altitude-rate bug |
| X3, corrected controller (software-in-the-loop, Gazebo constants, 60 Hz) | 180 s hover, 0 % envelope violations; 2 m square at 4.0 cm RMS |
| X3 with 0.5× paper disturbance: time outside rate envelope | PD 36.4 % → paper form 6.0 % → symmetric form 0 % |

Findings the paper does not state (details in the report): four unstated parameters (c, ς, Part-2 T_s and ρ_q0); the paper's Part-2 initial state lies on its own boundary; the barrier (1−ε)⁻² is one-sided; the envelope must be feasible for the actuator; Theorem 1 proves convergence, not envelope invariance.

## 3. Repository

| Path | What it is |
| :--- | :--- |
| `paper_reproduction.py` | Reproduction of the paper's Section IV, sensitivity and Monte-Carlo studies |
| `x3_sim/` | Software-in-the-loop model of the Gazebo X3 (`x3_model.py`), controllers v1/v2 (`controllers.py`), experiments E0–E5 |
| `gazebo.py` | ROS 2 node (v2) that flies the Gazebo X3 with the paper's controller |
| `plot_gazebo_log.py` | Figures and metrics from a `gazebo.py` log |
| `tests/test_gazebo_node_offline.py` | Runs `gazebo.py` without ROS against the X3 model |
| `toy_model/` | MATLAB toy model (`run_all.m`) |
| `Drones_Mujoco_*.ipynb`, `One_drone_alligning.ipynb` | MuJoCo 3-D demos |
| `index.html`, `app.js`, `simulation.js`, `style.css` | Interactive browser simulator (illustration, symmetric barrier) |
| `legacy/` | First versions kept for the record (`gazebo_v1.py` etc.), not used as evidence |
| `docs/` | Reports, figures, results |

## 4. Installation

What you need depends on which part you want to run. The core results (paper reproduction, X3 experiments, offline test) only need Python with NumPy and Matplotlib.

| Part | Requirements |
| :--- | :--- |
| Paper reproduction, X3 experiments, offline test of `gazebo.py` | Python 3, `numpy`, `matplotlib` |
| MuJoCo notebooks (`*.ipynb`) | Jupyter, `mujoco`, `mediapy`, `scipy`, `opencv-python`, `glfw` (plus `numpy`, `matplotlib`) |
| MATLAB toy model (`toy_model/`) | MATLAB R2016b or newer, no toolbox |
| Rebuilding the PDF reports | A LaTeX distribution with `pdflatex` (TeX Live or MiKTeX) |
| Flying the X3 in Gazebo (`gazebo.py`) | ROS 2 with `rclpy`, `geometry_msgs`, `actuator_msgs`, Gazebo Sim and `ros_gz_bridge` |
| Browser simulator (`index.html`) | Any modern web browser, nothing to install |

**1. Get the code**

```bash
git clone https://github.com/Nethra-R368/Drones_S5_AB02_PD_Control.git
cd Drones_S5_AB02_PD_Control
```

**2. Python packages** (a virtual environment is recommended)

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt    # core + MuJoCo notebook packages
```

Only the core scripts? `pip install numpy matplotlib` is enough. For the notebooks also install Jupyter (`pip install notebook`) and open them with `jupyter notebook`; they also run on Google Colab, where the first cell installs `mujoco`.

**3. MATLAB.** Nothing to install beyond MATLAB itself. Open MATLAB and run `run('toy_model/run_all.m')`.

**4. LaTeX** (only to rebuild the PDFs; the compiled PDFs are already in `docs/`)

```bash
sudo apt install texlive-latex-extra texlive-science texlive-pictures   # Ubuntu
# Windows: install MiKTeX (https://miktex.org) and let it install missing packages on first compile
```

**5. ROS 2 and Gazebo** (only to fly the X3 in Gazebo; Ubuntu). Install ROS 2 and the matching Gazebo Sim release following the official guides ([ROS 2](https://docs.ros.org/en/rolling/Installation.html), [Gazebo and ROS pairings](https://gazebosim.org/docs/latest/ros_installation/)), then:

```bash
sudo apt install ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-actuator-msgs
source /opt/ros/$ROS_DISTRO/setup.bash

# terminal 1: the X3 quadcopter world that ships with Gazebo Sim
gz sim quadcopter.sdf

# terminal 2: bridge the two topics used by gazebo.py
ros2 run ros_gz_bridge parameter_bridge \
  /world/quadcopter/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V \
  /X3/gazebo/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators

# terminal 3: start the simulation (press play in Gazebo) and run the node, see section 5
```

## 5. How to reproduce

```bash
pip install -r requirements.txt
python3 paper_reproduction.py              # ~6 min  -> docs/figures/fig*.png, docs/results/metrics.*
python3 x3_sim/run_x3_experiments.py       # ~16 min -> docs/figures/x3_*.png, docs/results/x3_metrics.*
python3 tests/test_gazebo_node_offline.py  # ~2 min
```

MATLAB (no toolbox): `run('<repo>/toy_model/run_all.m')`, then `python3 docs/make_crosscheck.py` to fill the Python-vs-MATLAB table, and rebuild the PDFs (`pdflatex`, twice, in `docs/complete_documentation` and `docs/mathematical_models`).

Gazebo (quadcopter world running, topics bridged with `ros_gz_bridge`):

```bash
python3 gazebo.py --ros-args -p controller_mode:=constrained -p experiment:=hover -p duration:=180.0
python3 gazebo.py --ros-args -p experiment:=square -p duration:=45.0
python3 gazebo.py --ros-args -p controller_mode:=pd -p disturbance_scale:=0.5
python3 plot_gazebo_log.py /tmp/drone_constrained_hover.npz
```

`controller_mode`: `constrained` (paper Eq. 13), `symmetric`, `pd`. Topics: input `/world/quadcopter/dynamic_pose/info` (`geometry_msgs/PoseArray`), output `/X3/gazebo/command/motor_speed` (`actuator_msgs/Actuators`).

## 6. Controller in one box

$$
\rho(t)=\Big(\rho_0-\rho_T\big(1+\tfrac{t}{T_s}\big)\Big)e^{\kappa t/(t-T_s)}+\rho_T\ (t<T_s),\quad \rho(t)=\rho_T\ (t\ge T_s)
$$

$$
\varepsilon_1=\frac{q_v}{\rho_q},\quad \varepsilon_2=\frac{\omega}{\rho_\omega},\quad s=\omega+c\,q_v
$$

$$
u=-k_pq_v-k_d\omega-\Big[\alpha(1-\varepsilon_1)^{-2}+\beta(1-\varepsilon_2)^{-2}\Big]s-\hat\theta\,\mathrm{sat}(s/\varsigma),\qquad
\dot{\hat\theta}=\frac{1}{\eta_1}\big(\|s\|-\eta_2(t)\hat\theta\big)
$$

On the X3 the same law runs inside an outer position loop, with the body rate computed from consecutive poses, a cap on the barrier gain for the 60 Hz sampled loop, and an exact motor mixer built from the X3 geometry. See the Mathematical Models document for every derivation.

## 7. Scope

The X3 numbers above come from the software-in-the-loop model built from Gazebo's X3 files (60 Hz, one sample of delay, no sensor noise); `gazebo.py` runs the identical controller in Gazebo, and `plot_gazebo_log.py` turns its log into the same figures and metrics. Constraint satisfaction is measured, not proven (the paper's Theorem 1 proves convergence). Section 10 of the report lists exactly what is and is not claimed.

## 8. References

1. M. Golestani, K. A. Alattas, S. Ud Din, S. Mobayen, A. K. Alanazi, A. Fekih, "A Simple Structure Constrained Attitude Control for Rigid Bodies: A PD-Type Control," *IEEE Access*, vol. 10, pp. 10202–10209, 2022. DOI [10.1109/ACCESS.2022.3144627](https://doi.org/10.1109/ACCESS.2022.3144627). **(Base paper)**
2. C. P. Bechlioulis, G. A. Rovithakis, "Robust adaptive control of feedback linearizable MIMO nonlinear systems with prescribed performance," *IEEE Transactions on Automatic Control*, vol. 53, no. 9, pp. 2090–2099, 2008. DOI [10.1109/TAC.2008.929402](https://doi.org/10.1109/TAC.2008.929402).
3. J. T.-Y. Wen, K. Kreutz-Delgado, "The attitude control problem," *IEEE Transactions on Automatic Control*, vol. 36, no. 10, pp. 1148–1162, 1991. DOI [10.1109/9.90228](https://doi.org/10.1109/9.90228).
4. H. K. Khalil, *Nonlinear Systems*, 3rd ed. Upper Saddle River, NJ: Prentice Hall, 2002.
5. Z. Chen, J. Huang, "Attitude tracking and disturbance rejection of rigid spacecraft by adaptive control," *IEEE Transactions on Automatic Control*, vol. 54, no. 3, pp. 600–605, 2009. DOI [10.1109/TAC.2008.2008350](https://doi.org/10.1109/TAC.2008.2008350). (Source of the paper's inertia matrix.)
6. E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A physics engine for model-based control," in *Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2012, pp. 5026–5033. DOI [10.1109/IROS.2012.6386109](https://doi.org/10.1109/IROS.2012.6386109).
7. Gazebo Sim: `quadcopter.sdf` example world (X3 model) and the `MulticopterMotorModel` system. [gazebosim.org](https://gazebosim.org).
8. ROS 2 and `ros_gz_bridge`. [docs.ros.org](https://docs.ros.org) · [github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz).
