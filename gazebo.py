import rclpy
from rclpy.node import Node

import numpy as np
import time

from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators


class ConstrainedPaperController(Node):

    def __init__(self):
        super().__init__('constrained_paper_controller')

        # ============================================================
        # ROS
        # ============================================================

        self.pose_sub = self.create_subscription(
            PoseArray,
            '/world/quadcopter/dynamic_pose/info',
            self.pose_callback,
            10
        )

        self.motor_pub = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10
        )

        # ============================================================
        # EXPERIMENT SETTINGS
        # ============================================================

        self.controller_mode = 'constrained'

        # KEEP FALSE FOR THE FIRST STABILITY TEST
        self.inject_wind_disturbance = False

        # Disturbance-test configuration
        # Supported modes: 'sinusoidal' and 'pulse'
        self.disturbance_mode = 'sinusoidal'
        self.disturbance_start_time = 10.0
        self.disturbance_duration = 20.0

        # Disturbance amplitudes for roll, pitch and yaw.
        self.disturbance_amplitude = np.array(
            [0.05, 0.10, 0.15],
            dtype=float
        )

        # Run for at least 180 seconds
        self.experiment_duration = 180.0

        # ============================================================
        # X3 PARAMETERS
        # ============================================================

        self.mass = 1.5
        self.gravity = 9.81

        self.arm_length = 0.225

        self.c_T = 8.54858e-06
        self.c_Q = 0.016

        # ============================================================
        # ALTITUDE CONTROL
        # ============================================================

        self.target_altitude = 2.0

        # Conservative altitude gains
        self.Kp_z = 5.0
        self.Kd_z = 3.5

        # Hover thrust
        self.hover_thrust = (
            self.mass * self.gravity
        )

        # ============================================================
        # ATTITUDE CONTROL
        # ============================================================

        # Conservative values for X3 stabilization
        self.Kp = 1.5
        self.Kd = 0.6

        # Paper constrained terms
        self.alpha = 0.01
        self.beta = 0.01
        self.c = 0.5

        # Adaptive estimator
        self.eta1 = 0.1
        self.varsigma = 0.5

        # X3 safety torque
        self.uMax = 0.20

        # ============================================================
        # FTPPF PARAMETERS
        # ============================================================

        self.rho_q0 = 2.0
        self.rho_qT = 0.30
        self.kappa_q = 2.0

        self.rho_w0 = 10.0
        self.rho_wT = 3.0
        self.kappa_w = 2.0

        self.Ts = 10.0

        # ============================================================
        # ADAPTIVE ESTIMATOR
        # ============================================================

        self.thetaHat = 0.0

        # ============================================================
        # STATE
        # ============================================================

        self.start_time = time.time()
        self.last_time = time.time()

        self.initialized = False

        self.last_z = 0.0

        self.last_q = np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=float
        )

        self.last_omega = np.zeros(3)

        self.z_dot = 0.0

        # ============================================================
        # FILTERING
        # ============================================================

        self.z_filter = 0.10
        self.omega_filter = 0.10

        # ============================================================
        # DATA LOGGING
        # ============================================================

        self.log_time = []

        self.log_q = []
        self.log_omega = []

        self.log_rho_q = []
        self.log_rho_w = []

        self.log_tau = []
        self.log_theta = []

        self.log_altitude = []
        self.log_thrust = []

        self.log_disturbance = []
        self.log_disturbance_norm = []

        # ============================================================
        # LOG FILE
        # ============================================================

        self.log_file = (
            '/tmp/drone_'
            + self.controller_mode
            + '.npz'
        )

        # ============================================================
        # TAKEOFF
        # ============================================================

        self.takeoff_duration = 8.0

        # ============================================================
        # PRINT TIMER
        # ============================================================

        self.last_print = 0.0

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            'X3 CONSTRAINED PAPER CONTROLLER'
        )

        self.get_logger().info(
            'TARGET ALTITUDE: 2.0 m'
        )

        self.get_logger().info(
            '3-MINUTE STABILITY TEST'
        )

        self.get_logger().info(
            f'WIND: {"ON" if self.inject_wind_disturbance else "OFF"}'
        )

        self.get_logger().info(
            f'DISTURBANCE MODE: {self.disturbance_mode}'
        )

        self.get_logger().info(
            '============================================'
        )

    # ================================================================
    # FTPPF
    # ================================================================

    def ftppf(
        self,
        t,
        rho0,
        rhoT,
        kappa,
        Ts
    ):

        if t >= Ts:
            return rhoT

        t = max(t, 0.0)

        exponent = (
            kappa * t
            /
            (t - Ts)
        )

        if exponent < -50.0:
            return rhoT

        rho = (
            (
                rho0
                -
                rhoT * (
                    1.0 + t / Ts
                )
            )
            *
            np.exp(exponent)
            +
            rhoT
        )

        return max(
            rho,
            rhoT
        )

    # ================================================================
    # QUATERNION NORMALIZATION
    # ================================================================

    def normalize_q(self, q):

        n = np.linalg.norm(q)

        if n < 1e-8:
            return np.array(
                [0.0, 0.0, 0.0, 1.0]
            )

        return q / n

    # ================================================================
    # DISTURBANCE GENERATOR
    # ================================================================

    def generate_disturbance(self, elapsed):
        """Generate the configured test disturbance."""
        disturbance = np.zeros(3)

        if not self.inject_wind_disturbance:
            return disturbance

        start = self.disturbance_start_time
        end = start + self.disturbance_duration

        if elapsed < start or elapsed > end:
            return disturbance

        tau = elapsed - start
        amp = self.disturbance_amplitude

        if self.disturbance_mode == 'pulse':
            # Constant disturbance during the test window.
            disturbance = amp.copy()

        else:
            # Smooth sinusoidal disturbance.
            disturbance = np.array([
                amp[0] * np.sin(tau),
                amp[1] * np.sin(1.2 * tau),
                amp[2] * np.sin(1.5 * tau)
            ])

        return disturbance

    # ================================================================
    # MAIN CALLBACK
    # ================================================================

    def pose_callback(self, msg):

        if len(msg.poses) == 0:
            return

        now = time.time()

        dt = now - self.last_time

        if dt <= 0.0:
            return

        # Reject huge timing gaps
        if dt > 0.1:
            dt = 0.02

        elapsed = (
            now - self.start_time
        )

        # ============================================================
        # STOP AFTER EXPERIMENT
        # ============================================================

        if elapsed > self.experiment_duration:

            self.stop_motors()

            self.save_data()

            rclpy.shutdown()

            return

        pose = msg.poses[0]

        # ============================================================
        # POSITION
        # ============================================================

        z = pose.position.z

        # ============================================================
        # QUATERNION
        # ============================================================

        q = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        q = self.normalize_q(q)

        # Quaternion sign correction
        if np.dot(
            q,
            self.last_q
        ) < 0:

            q = -q

        qv = q[:3]

        # ============================================================
        # FIRST SENSOR SAMPLE
        # ============================================================

        if not self.initialized:

            self.last_z = z
            self.last_q = q.copy()
            self.last_time = now
            self.initialized = True

            return

        # ============================================================
        # VERTICAL VELOCITY
        # ============================================================

        raw_z_dot = (
            z - self.last_z
        ) / dt

        self.z_dot = (
            (1.0 - self.z_filter)
            * self.z_dot
            +
            self.z_filter
            * raw_z_dot
        )

        self.z_dot = np.clip(
            self.z_dot,
            -3.0,
            3.0
        )

        # ============================================================
        # ANGULAR VELOCITY
        # ============================================================

        dq = (
            q - self.last_q
        ) / dt

        raw_omega = (
            2.0 * dq[:3]
        )

        omega = (
            (1.0 - self.omega_filter)
            * self.last_omega
            +
            self.omega_filter
            * raw_omega
        )

        omega = np.clip(
            omega,
            -3.0,
            3.0
        )

        # ============================================================
        # ALTITUDE TARGET
        # ============================================================

        if elapsed < self.takeoff_duration:

            target_z = (
                0.2
                +
                1.8
                *
                elapsed
                /
                self.takeoff_duration
            )

        else:

            target_z = 2.0

        # ============================================================
        # ALTITUDE CONTROLLER
        # ============================================================

        altitude_error = (
            target_z - z
        )

        vertical_acceleration = (
            self.Kp_z
            *
            altitude_error
            -
            self.Kd_z
            *
            self.z_dot
        )

        # Limit acceleration request
        vertical_acceleration = np.clip(
            vertical_acceleration,
            -3.0,
            3.0
        )

        F_vertical = (
            self.mass
            *
            (
                self.gravity
                +
                vertical_acceleration
            )
        )

        # ============================================================
        # TILT COMPENSATION
        # ============================================================

        # Approximate roll/pitch from quaternion.
        #
        # For the small angles expected during hover this is
        # sufficient to prevent loss of vertical thrust.

        roll = 2.0 * q[0]
        pitch = 2.0 * q[1]

        tilt_factor = (
            np.cos(roll)
            *
            np.cos(pitch)
        )

        tilt_factor = np.clip(
            tilt_factor,
            0.85,
            1.0
        )

        T_total = (
            F_vertical
            /
            tilt_factor
        )

        # ============================================================
        # THRUST SAFETY
        # ============================================================

        T_total = np.clip(
            T_total,
            0.0,
            22.0
        )

        # ============================================================
        # FTPPF
        # ============================================================

        t_env = min(
            elapsed,
            self.Ts
        )

        rho_q = self.ftppf(
            t_env,
            self.rho_q0,
            self.rho_qT,
            self.kappa_q,
            self.Ts
        )

        rho_w = self.ftppf(
            t_env,
            self.rho_w0,
            self.rho_wT,
            self.kappa_w,
            self.Ts
        )

        # ============================================================
        # SLIDING VARIABLE
        # ============================================================

        s = (
            omega
            +
            self.c * qv
        )

        eps_q = (
            qv
            /
            max(rho_q, 1e-6)
        )

        eps_w = (
            omega
            /
            max(rho_w, 1e-6)
        )

        # ============================================================
        # CONSTRAINED CONTROLLER
        # ============================================================

        u_cons = np.zeros(3)

        if self.controller_mode == 'constrained':

            for i in range(3):

                e1 = np.clip(
                    eps_q[i],
                    -0.90,
                    0.90
                )

                e2 = np.clip(
                    eps_w[i],
                    -0.90,
                    0.90
                )

                barrier_q = (
                    self.alpha
                    /
                    (
                        1.0 - e1**2
                    )**2
                )

                barrier_w = (
                    self.beta
                    /
                    (
                        1.0 - e2**2
                    )**2
                )

                u_cons[i] = (
                    (
                        barrier_q
                        +
                        barrier_w
                    )
                    *
                    s[i]
                    +
                    self.thetaHat
                    *
                    np.clip(
                        s[i]
                        /
                        self.varsigma,
                        -1.0,
                        1.0
                    )
                )

            # Keep this contribution small
            # during X3 stabilization.

            u_cons = np.clip(
                u_cons,
                -0.05,
                0.05
            )

            # ========================================================
            # ADAPTIVE ESTIMATOR
            # ========================================================

            eta2 = max(
                2.0
                *
                np.exp(
                    -0.5 * t_env
                ),
                0.1
            )

            s_norm = np.linalg.norm(s)

            d_theta = (
                1.0
                /
                self.eta1
            ) * (
                s_norm
                -
                eta2
                *
                self.thetaHat
            )

            if (
                self.thetaHat <= 0.0
                and
                d_theta < 0.0
            ):

                d_theta = 0.0

            self.thetaHat += (
                d_theta * dt
            )

            self.thetaHat = np.clip(
                self.thetaHat,
                0.0,
                1.0
            )

        else:

            self.thetaHat = 0.0

        # ============================================================
        # ATTITUDE TORQUE
        # ============================================================

        u = (
            -self.Kp * qv
            -
            self.Kd * omega
            -
            u_cons
        )

        # ============================================================
        # OPTIONAL PAPER DISTURBANCE
        #
        # OFF for the first 3-minute hover test.
        # ============================================================

        disturbance = self.generate_disturbance(elapsed)

        if np.any(disturbance):
            u += disturbance

        # ============================================================
        # TORQUE LIMIT
        # ============================================================

        u = np.clip(
            u,
            -self.uMax,
            self.uMax
        )

        # ============================================================
        # MIXER TORQUE LIMIT
        #
        # Prevent an attitude command from stealing too much
        # thrust from one motor.
        # ============================================================

        available_tau = (
            0.15
            *
            T_total
            *
            self.arm_length
        )

        available_tau = max(
            0.08,
            min(
                available_tau,
                self.uMax
            )
        )

        tau = np.clip(
            u,
            -available_tau,
            available_tau
        )

        # ============================================================
        # MOTOR MIXER
        # ============================================================

        base = (
            T_total
            /
            (
                4.0
                *
                self.c_T
            )
        )

        roll_term = (
            tau[0]
            /
            (
                4.0
                *
                self.c_T
                *
                self.arm_length
            )
        )

        pitch_term = (
            tau[1]
            /
            (
                4.0
                *
                self.c_T
                *
                self.arm_length
            )
        )

        yaw_term = (
            tau[2]
            /
            (
                4.0
                *
                self.c_Q
            )
        )

        m1 = (
            base
            +
            pitch_term
            -
            roll_term
            -
            yaw_term
        )

        m2 = (
            base
            -
            pitch_term
            -
            roll_term
            +
            yaw_term
        )

        m3 = (
            base
            -
            pitch_term
            +
            roll_term
            -
            yaw_term
        )

        m4 = (
            base
            +
            pitch_term
            +
            roll_term
            +
            yaw_term
        )

        # ============================================================
        # MOTOR NON-NEGATIVE
        # ============================================================

        m1 = max(0.0, m1)
        m2 = max(0.0, m2)
        m3 = max(0.0, m3)
        m4 = max(0.0, m4)

        # ============================================================
        # MOTOR SPEED
        # ============================================================

        w1 = np.sqrt(m1)
        w2 = np.sqrt(m2)
        w3 = np.sqrt(m3)
        w4 = np.sqrt(m4)

        # ============================================================
        # PUBLISH
        # ============================================================

        motor_msg = Actuators()

        motor_msg.velocity = [
            float(w1),
            float(w2),
            float(w3),
            float(w4)
        ]

        self.motor_pub.publish(
            motor_msg
        )

        # ============================================================
        # LOGGING
        # ============================================================

        self.log_time.append(
            elapsed
        )

        self.log_q.append(
            qv.copy()
        )

        self.log_omega.append(
            omega.copy()
        )

        self.log_rho_q.append(
            rho_q
        )

        self.log_rho_w.append(
            rho_w
        )

        self.log_tau.append(
            tau.copy()
        )

        self.log_theta.append(
            self.thetaHat
        )

        self.log_altitude.append(
            z
        )

        self.log_thrust.append(
            T_total
        )

        self.log_disturbance.append(
            disturbance.copy()
        )

        self.log_disturbance_norm.append(
            float(np.linalg.norm(disturbance))
        )

        # ============================================================
        # TERMINAL STATUS
        # ============================================================

        if (
            elapsed
            -
            self.last_print
            >
            1.0
        ):

            self.last_print = elapsed

            print(
                f"\n"
                f"TIME      : {elapsed:7.2f} s\n"
                f"ALTITUDE  : {z:7.3f} m\n"
                f"TARGET    : {target_z:7.3f} m\n"
                f"Z DOT     : {self.z_dot:7.3f} m/s\n"
                f"THRUST    : {T_total:7.3f} N\n"
                f"MOTORS    : "
                f"{w1:7.1f} "
                f"{w2:7.1f} "
                f"{w3:7.1f} "
                f"{w4:7.1f}\n"
                f"THETA HAT : {self.thetaHat:7.3f}\n"
            )

        # ============================================================
        # UPDATE
        # ============================================================

        self.last_q = q.copy()
        self.last_omega = omega.copy()
        self.last_time = now

    # ================================================================
    # STOP MOTORS
    # ================================================================

    def stop_motors(self):

        msg = Actuators()

        msg.velocity = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.motor_pub.publish(
            msg
        )

    # ================================================================
    # SAVE DATA
    # ================================================================

    def save_data(self):

        try:

            np.savez(
                self.log_file,

                time=np.asarray(
                    self.log_time
                ),

                q=np.asarray(
                    self.log_q
                ),

                omega=np.asarray(
                    self.log_omega
                ),

                rho_q=np.asarray(
                    self.log_rho_q
                ),

                rho_w=np.asarray(
                    self.log_rho_w
                ),

                tau=np.asarray(
                    self.log_tau
                ),

                theta=np.asarray(
                    self.log_theta
                ),

                altitude=np.asarray(
                    self.log_altitude
                ),

                thrust=np.asarray(
                    self.log_thrust
                ),

                disturbance=np.asarray(
                    self.log_disturbance
                ),

                disturbance_norm=np.asarray(
                    self.log_disturbance_norm
                )
            )

            self.get_logger().info(
                f'Data saved to {self.log_file}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Could not save data: {e}'
            )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = ConstrainedPaperController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print(
            '\nStopping controller...'
        )

    finally:

        node.stop_motors()

        node.save_data()

        time.sleep(0.2)

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
import numpy as np
import matplotlib.pyplot as plt

FILE = "/tmp/drone_constrained.npz"

data = np.load(FILE)

t = data["time"]

q = data["q"]
omega = data["omega"]

rho_q = data["rho_q"]
rho_w = data["rho_w"]

tau = data["tau"]
theta = data["theta"]

altitude = data["altitude"]
thrust = data["thrust"]


# ============================================================
# FIGURE 1 — QUATERNION + FTPPF
# ============================================================

fig, ax = plt.subplots(
    3,
    1,
    figsize=(11, 8),
    sharex=True
)

names = [
    r"$q_1$",
    r"$q_2$",
    r"$q_3$"
]

for i in range(3):

    ax[i].plot(
        t,
        q[:, i],
        label="Proposed Constrained Controller"
    )

    ax[i].plot(
        t,
        rho_q,
        "--",
        label=r"$+\rho_q$"
    )

    ax[i].plot(
        t,
        -rho_q,
        "--",
        label=r"$-\rho_q$"
    )

    ax[i].axhline(
        0,
        linestyle=":"
    )

    ax[i].set_ylabel(
        names[i]
    )

    ax[i].grid(
        True,
        alpha=0.3
    )

    ax[i].legend()

ax[-1].set_xlabel(
    "Time (s)"
)

fig.suptitle(
    "Quaternion Response with Prescribed Performance Bounds"
)

plt.tight_layout()

plt.savefig(
    "Figure_7_Quaternion.png",
    dpi=300
)

plt.show()


# ============================================================
# FIGURE 2 — ANGULAR VELOCITY
# ============================================================

fig, ax = plt.subplots(
    3,
    1,
    figsize=(11, 8),
    sharex=True
)

names = [
    r"$\omega_1$",
    r"$\omega_2$",
    r"$\omega_3$"
]

for i in range(3):

    ax[i].plot(
        t,
        omega[:, i],
        label="Proposed Constrained Controller"
    )

    ax[i].plot(
        t,
        rho_w,
        "--",
        label=r"$+\rho_\omega$"
    )

    ax[i].plot(
        t,
        -rho_w,
        "--",
        label=r"$-\rho_\omega$"
    )

    ax[i].axhline(
        0,
        linestyle=":"
    )

    ax[i].set_ylabel(
        names[i]
    )

    ax[i].grid(
        True,
        alpha=0.3
    )

    ax[i].legend()

ax[-1].set_xlabel(
    "Time (s)"
)

fig.suptitle(
    "Angular Velocity Response with Prescribed Performance Bounds"
)

plt.tight_layout()

plt.savefig(
    "Figure_8_AngularVelocity.png",
    dpi=300
)

plt.show()


# ============================================================
# FIGURE 3 — CONTROL TORQUE
# ============================================================

fig, ax = plt.subplots(
    3,
    1,
    figsize=(11, 8),
    sharex=True
)

names = [
    r"$\tau_1$",
    r"$\tau_2$",
    r"$\tau_3$"
]

TORQUE_LIMIT = 0.20

for i in range(3):

    ax[i].plot(
        t,
        tau[:, i],
        label="Proposed Constrained Controller"
    )

    ax[i].axhline(
        TORQUE_LIMIT,
        linestyle="--",
        label=r"$+\tau_{max}$"
    )

    ax[i].axhline(
        -TORQUE_LIMIT,
        linestyle="--",
        label=r"$-\tau_{max}$"
    )

    ax[i].axhline(
        0,
        linestyle=":"
    )

    ax[i].set_ylabel(
        names[i]
    )

    ax[i].grid(
        True,
        alpha=0.3
    )

    ax[i].legend()

ax[-1].set_xlabel(
    "Time (s)"
)

fig.suptitle(
    "Control Torque and Actuator Constraint"
)

plt.tight_layout()

plt.savefig(
    "Figure_9_ControlTorque.png",
    dpi=300
)

plt.show()


# ============================================================
# FIGURE 4 — ADAPTIVE PARAMETER
# ============================================================

plt.figure(
    figsize=(11, 5)
)

plt.plot(
    t,
    theta,
    label=r"$\hat{\theta}$"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    r"$\hat{\theta}$"
)

plt.title(
    "Adaptive Disturbance Estimator"
)

plt.grid(
    True,
    alpha=0.3
)

plt.legend()

plt.tight_layout()

plt.savefig(
    "Figure_10_ThetaHat.png",
    dpi=300
)

plt.show()


# ============================================================
# FIGURE 5 — ALTITUDE
# ============================================================

plt.figure(
    figsize=(11, 5)
)

plt.plot(
    t,
    altitude,
    label="Drone altitude"
)

plt.axhline(
    2.0,
    linestyle="--",
    label="Target altitude = 2 m"
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Altitude (m)"
)

plt.title(
    "X3 Altitude Hold"
)

plt.grid(
    True,
    alpha=0.3
)

plt.legend()

plt.tight_layout()

plt.savefig(
    "Figure_11_Altitude.png",
    dpi=300
)

plt.show()


# ============================================================
# PERFORMANCE NUMBERS
# ============================================================

print("\n==========================================")
print("             RESULTS")
print("==========================================")

print(
    f"Simulation time : {t[-1]:.2f} s"
)

print(
    f"Minimum altitude: "
    f"{np.min(altitude):.3f} m"
)

print(
    f"Maximum altitude: "
    f"{np.max(altitude):.3f} m"
)

print(
    f"Final altitude  : "
    f"{altitude[-1]:.3f} m"
)

print(
    f"Mean altitude   : "
    f"{np.mean(altitude):.3f} m"
)

alt_error = (
    altitude - 2.0
)

print(
    f"Maximum altitude error: "
    f"{np.max(np.abs(alt_error)):.3f} m"
)

print(
    f"Maximum thrust: "
    f"{np.max(thrust):.3f} N"
)

print(
    f"Minimum thrust: "
    f"{np.min(thrust):.3f} N"
)

print(
    f"Maximum thetaHat: "
    f"{np.max(theta):.3f}"
)

print(
    f"Maximum |tau|: "
    f"{np.max(np.abs(tau)):.3f}"
)

print("\nPrescribed-performance violations:")

for i in range(3):

    violations = (
        np.abs(q[:, i])
        >
        rho_q
    )

    percentage = (
        100.0
        *
        np.mean(violations)
    )

    print(
        f"q{i+1}: "
        f"{percentage:.2f}%"
    )

for i in range(3):

    violations = (
        np.abs(omega[:, i])
        >
        rho_w
    )

    percentage = (
        100.0
        *
        np.mean(violations)
    )

    print(
        f"omega{i+1}: "
        f"{percentage:.2f}%"
    )

print("=========================================")
