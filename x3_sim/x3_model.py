"""
x3_model.py - software-in-the-loop surrogate of the Gazebo "X3 UAV" quadcopter.

Every physical constant is taken from the Gazebo model files, not tuned:
  * gz-sim test/worlds/quadcopter.sdf  (X3 base_link + rotor links)
      base_link mass 1.5 kg, ixx 0.0347563, iyy 0.07, izz 0.0977 kg m^2
      rotor_i mass 0.005 kg at
         rotor_0 ( 0.13, -0.22, 0.023)  ccw
         rotor_1 (-0.13,  0.20, 0.023)  ccw
         rotor_2 ( 0.13,  0.22, 0.023)  cw
         rotor_3 (-0.13, -0.20, 0.023)  cw
  * MulticopterMotorModel plugin parameters (examples/worlds/quadcopter.sdf)
      motorConstant 8.54858e-06, momentConstant 0.016, maxRotVelocity 800 rad/s,
      timeConstantUp 0.0125 s, timeConstantDown 0.025 s,
      rotorDragCoefficient 8.06428e-05, rollingMomentCoefficient 1e-06
  * gz-sim src/systems/multicopter_motor_model/MulticopterMotorModel.cc
      thrust_i  = motorConstant * w_i^2                 along body +z
      yaw_i     = -turningDirection_i * momentConstant * thrust_i   (ccw = +1, cw = -1)
      drag_i    = -|w_i| * rotorDragCoefficient * v_perp
      rotor speed follows the command through a first-order filter (up/down time constants)

What the surrogate does NOT model (and why that is acceptable here): rotor joint
inertia/gyroscopic effects (rotor inertia 4e-5 kg m^2 is ~0.1 % of the body's),
blade flapping, contact dynamics beyond a flat ground plane, and the ROS<->Gazebo
transport. Transport is represented by sampling the pose at a fixed rate
(SceneBroadcaster default 60 Hz) and applying the command one sample late.

Quaternion convention: q = [x, y, z, w] (ROS / Gazebo order), body -> world.
"""
import numpy as np

G = 9.81
M_BASE = 1.5
M_ROTOR = 0.005
R_POS = np.array([[0.13, -0.22, 0.023],
                  [-0.13, 0.20, 0.023],
                  [0.13, 0.22, 0.023],
                  [-0.13, -0.20, 0.023]])
TURN_DIR = np.array([+1, +1, -1, -1])          # ccw, ccw, cw, cw
KT = 8.54858e-06
CQ = 0.016
W_MAX = 800.0
TAU_UP, TAU_DOWN = 0.0125, 0.025
ROTOR_DRAG = 8.06428e-05
ROLLING = 1e-06
MASS = M_BASE + 4 * M_ROTOR
# body inertia about the CoM (base link + point-mass rotors, parallel-axis)
J_BODY = np.diag([0.0347563, 0.07, 0.0977]).astype(float)
for r in R_POS:
    J_BODY += M_ROTOR * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
J_INV = np.linalg.inv(J_BODY)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    """Hamilton product, [x,y,z,w] order."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz])


def quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def euler_from_quat(q):
    x, y, z, w = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def allocation_matrix():
    """Maps rotor thrusts F (4,) to [T, tau_x, tau_y, tau_z] about the CoM (geometric centre)."""
    A = np.zeros((4, 4))
    for i, (r, d) in enumerate(zip(R_POS, TURN_DIR)):
        A[0, i] = 1.0
        A[1, i] = r[1]            # tau_x =  y F
        A[2, i] = -r[0]           # tau_y = -x F
        A[3, i] = -d * CQ         # yaw reaction torque (MulticopterMotorModel.cc)
    return A


_A_TAU = allocation_matrix()[1:, :]


class X3:
    """6-DOF rigid body + 4 first-order rotors, integrated with RK4 at dt_phys."""

    def __init__(self, dt_phys=1e-3, pos0=(0, 0, 0.053302), yaw0=0.0, wind_torque=None):
        self.dt = dt_phys
        self.p = np.array(pos0, float)
        self.v = np.zeros(3)
        self.q = np.array([0, 0, np.sin(yaw0 / 2), np.cos(yaw0 / 2)])
        self.w = np.zeros(3)                 # body rates
        self.rotor = np.zeros(4)            # rotor speeds (rad/s)
        self.cmd = np.zeros(4)
        self.t = 0.0
        self.wind_torque = wind_torque       # callable t -> body torque (N m), for disturbance tests
        self.ground_z = 0.053302
        self.crashed = False
        self.crash_time = None
        self.airborne = False

    # -- forces --------------------------------------------------------------
    def wrench(self, p, v, q, w, rotor, t):
        R = quat_to_R(q)
        F = KT * rotor ** 2
        f_body = np.array([0.0, 0.0, F.sum()])
        # r_i x (0,0,F_i) + yaw reaction = allocation matrix rows 1..3 (identical, vectorised)
        tau = _A_TAU @ F
        # rotor drag (world) & rolling moment
        zb = R[:, 2]
        v_perp = v - np.dot(v, zb) * zb
        f_world = R @ f_body - np.abs(rotor).sum() * ROTOR_DRAG * v_perp
        tau += R.T @ (-np.abs(rotor).sum() * ROLLING * v_perp)
        if self.wind_torque is not None:
            tau = tau + self.wind_torque(t)
        return f_world, tau

    def deriv(self, s, t):
        p, v, q, w, rotor = s
        f_world, tau = self.wrench(p, v, q, w, rotor, t)
        a = f_world / MASS - np.array([0, 0, G])
        q_dot = 0.5 * quat_mul(q, np.array([w[0], w[1], w[2], 0.0]))
        w_dot = J_INV @ (tau - np.cross(w, J_BODY @ w))
        tc = np.where(self.cmd > rotor, TAU_UP, TAU_DOWN)
        r_dot = (self.cmd - rotor) / tc
        return [v, a, q_dot, w_dot, r_dot]

    def step(self):
        s = [self.p, self.v, self.q, self.w, self.rotor]
        h = self.dt
        k1 = self.deriv(s, self.t)
        k2 = self.deriv([x + h / 2 * k for x, k in zip(s, k1)], self.t + h / 2)
        k3 = self.deriv([x + h / 2 * k for x, k in zip(s, k2)], self.t + h / 2)
        k4 = self.deriv([x + h * k for x, k in zip(s, k3)], self.t + h)
        new = [x + h / 6 * (a + 2 * b + 2 * c + d) for x, a, b, c, d in zip(s, k1, k2, k3, k4)]
        self.p, self.v, self.q, self.w, self.rotor = new
        self.q /= np.linalg.norm(self.q)
        self.t += h
        # ground plane
        if self.p[2] > self.ground_z + 0.05:
            self.airborne = True
        if self.p[2] <= self.ground_z:
            impact = self.v[2]
            tilt = np.degrees(np.arccos(np.clip(quat_to_R(self.q)[2, 2], -1, 1)))
            if self.airborne and (impact < -1.0 or tilt > 30) and not self.crashed:
                self.crashed = True
                self.crash_time = self.t
            self.p[2] = self.ground_z
            if self.v[2] < 0:
                self.v[:] = 0.0
                self.w *= 0.5
        if not self.crashed:
            tilt = np.degrees(np.arccos(np.clip(quat_to_R(self.q)[2, 2], -1, 1)))
            if self.airborne and tilt > 90:          # flipped over in the air
                self.crashed = True
                self.crash_time = self.t

    def set_motor_speed(self, w_cmd):
        self.cmd = np.clip(np.asarray(w_cmd, float), 0.0, W_MAX)

    def pose(self):
        return self.p.copy(), self.q.copy()


def run(controller, t_end=180.0, ctrl_hz=60.0, dt_phys=1e-3, delay_samples=1, wind_torque=None,
        yaw0=0.0, stop_on_crash=True, jitter=0.0, seed=0):
    """Closed-loop run. The controller sees only (stamp, position, quaternion) like
    the ROS 2 node does from /world/quadcopter/dynamic_pose/info, and returns 4 rotor
    speed commands like /X3/gazebo/command/motor_speed."""
    rng = np.random.default_rng(seed)
    x3 = X3(dt_phys=dt_phys, wind_torque=wind_torque, yaw0=yaw0)
    n_phys = int(round(1.0 / (ctrl_hz * dt_phys)))
    queue = [np.zeros(4)] * delay_samples
    log = {k: [] for k in ["t", "p", "q", "w", "rotor", "cmd", "euler"]}
    k = 0
    while x3.t < t_end:
        if k % n_phys == 0:
            p, q = x3.pose()
            stamp = x3.t + (rng.normal(0, jitter) if jitter > 0 else 0.0)
            cmd = controller.update(stamp, p, q)
            queue.append(np.asarray(cmd, float))
            x3.set_motor_speed(queue.pop(0))
            log["t"].append(x3.t); log["p"].append(p); log["q"].append(q)
            log["w"].append(x3.w.copy()); log["rotor"].append(x3.rotor.copy())
            log["cmd"].append(x3.cmd.copy()); log["euler"].append(euler_from_quat(q))
        x3.step()
        k += 1
        if stop_on_crash and x3.crashed and x3.t > x3.crash_time + 1.0:
            break
    for key in log:
        log[key] = np.array(log[key])
    log["crashed"] = x3.crashed
    log["crash_time"] = x3.crash_time
    return log
