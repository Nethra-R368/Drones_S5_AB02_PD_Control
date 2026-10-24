cat << 'EOF' > drone_brain.py
import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators

class ResearchPaperDroneController(Node):
    def __init__(self):
        super().__init__('research_paper_drone_controller')
        
        # Subscribe to Gazebo's ground truth poses
        self.pose_sub = self.create_subscription(
            PoseArray, '/world/quadcopter/dynamic_pose/info', self.pose_callback, 10)
        
        # Publish to Gazebo's motor speeds
        self.motor_pub = self.create_publisher(
            Actuators, '/X3/gazebo/command/motor_speed', 10)

        # X3 Physical Drone Parameters & Inertia Matrix J approximation
        # (Mapped from your spacecraft J tensor to quadrotor equivalent)
        self.J = np.array([
            [0.0232, 0.0, 0.0],
            [0.0, 0.0232, 0.0],
            [0.0, 0.0, 0.0468]
        ], dtype=float)
        self.invJ = np.linalg.inv(self.J)

        self.mass = 1.5
        self.gravity = 9.81
        self.l = 0.225
        self.c_T = 8.54858e-06
        self.c_Q = 0.016
        self.hover_thrust = self.mass * self.gravity

        # --- YOUR RESEARCH PAPER TUNING PARAMETERS ---
        self.Kp = 40.0
        self.Kd = 20.0
        self.alpha = 1.5
        self.beta = 1.2
        self.c = 1.0          # Sliding surface coefficient: s = omega + c * qv
        self.eta1 = 0.1
        self.varsigma = 0.01  # Boundary layer for chattering reduction
        self.uMax = 15.0      # Torque saturation limit (Nm)

        # FTPPF Envelope Parameters
        self.rho_q0 = 0.3
        self.rho_qT = 0.008
        self.kappa_q = 2.5
        self.rho_w0 = 0.4
        self.rho_wT = 0.005
        self.kappa_w = 2.5
        self.Ts = 15.0        # Settling time (seconds)

        # State initialization
        self.thetaHat = 0.0
        self.last_time = time.time()
        self.last_q = np.array([0.0, 0.0, 0.0, 1.0])
        
        self.get_logger().info("Research Paper Constrained Controller Online!")

    def ftppf(self, t, rho0, rhoT, kappa, Ts):
        """Finite-Time Prescribed Performance Function."""
        if t >= Ts - 1e-9:
            return rhoT
        exponent = (kappa * t) / (t - Ts)
        if exponent < -50:
            return rhoT
        return (rho0 - rhoT * (1.0 + t / Ts)) * np.exp(exponent) + rhoT

    def sat_func(self, x, delta):
        """Saturation function for boundary layers: sat(x/delta)."""
        return np.clip(x / delta, -1.0, 1.0)

    def pose_callback(self, msg):
        if len(msg.poses) < 1:
            return
            
        current_time = time.time()
        dt = current_time - self.last_time
        if dt < 0.005: # Run at ~200Hz
            return
            
        # Get current orientation quaternion [x, y, z, w] from Gazebo ground truth
        pose = msg.poses[0]
        q = np.array([pose.orientation.x, pose.orientation.y, 
                      pose.orientation.z, pose.orientation.w])
        qv = q[0:3]
        q4 = q[3]
        
        # Numerically derive angular velocity (omega)
        dq = (q - self.last_q) / dt
        omega = 2.0 * dq[:3]

        # Calculate dynamic envelope boundaries at current time t
        t = current_time % self.Ts # Keep within envelope bounds safely
        rho_q = self.ftppf(t, self.rho_q0, self.rho_qT, self.kappa_q, self.Ts)
        rho_w = self.ftppf(t, self.rho_w0, self.rho_wT, self.kappa_w, self.Ts)

        # Sliding surface error vector: s = omega + c * qv
        s = omega + self.c * qv

        # --- YOUR PAPER'S CONSTRAINED PD CONTROLLER WITH BARRIER FUNCTIONS ---
        eps1 = qv / rho_q
        eps2 = omega / rho_w

        uCons = np.zeros(3)
        for i in range(3):
            e1_clamped = np.clip(eps1[i], -0.9999, 0.9999)
            e2_clamped = np.clip(eps2[i], -0.9999, 0.9999)

            term1 = self.alpha / ((1.0 - e1_clamped**2)**2)
            term2 = self.beta / ((1.0 - e2_clamped**2)**2)

            uCons[i] = (term1 + term2) * s[i] + self.thetaHat * self.sat_func(s[i], self.varsigma)

        # Proposed Controller Torques [tau_x, tau_y, tau_z]
        u = -self.Kp * qv - self.Kd * omega - uCons
        uSat = np.clip(u, -self.uMax, self.uMax)

        # --- ADAPTIVE DISTURBANCE ESTIMATOR UPDATE LAW ---
        eta2 = 2.0 * np.exp(-0.5 * t)
        s_norm = np.linalg.norm(s)
        dThetaHat = (1.0 / self.eta1) * (s_norm - eta2 * self.thetaHat)
        if self.thetaHat <= 0.0 and dThetaHat < 0.0:
            dThetaHat = 0.0
        
        self.thetaHat += dThetaHat * dt

        # --- CONTROL ALLOCATION MIXER (Torques & Thrust -> 4 Motor Speeds) ---
        T_total = self.hover_thrust * 1.15 # 15% lift boost for takeoff
        tau = uSat

        m1 = (T_total / (4 * self.c_T)) + (tau[1] / (4 * self.c_T * self.l)) - (tau[0] / (4 * self.c_T * self.l)) - (tau[2] / (4 * self.c_Q))
        m2 = (T_total / (4 * self.c_T)) - (tau[1] / (4 * self.c_T * self.l)) - (tau[0] / (4 * self.c_T * self.l)) + (tau[2] / (4 * self.c_Q))
        m3 = (T_total / (4 * self.c_T)) - (tau[1] / (4 * self.c_T * self.l)) + (tau[0] / (4 * self.c_T * self.l)) - (tau[2] / (4 * self.c_Q))
        m4 = (T_total / (4 * self.c_T)) + (tau[1] / (4 * self.c_T * self.l)) + (tau[0] / (4 * self.c_T * self.l)) + (tau[2] / (4 * self.c_Q))
        
        # Prevent negative square roots
        m1, m2, m3, m4 = max(0, m1), max(0, m2), max(0, m3), max(0, m4)
        w1, w2, w3, w4 = np.sqrt(m1), np.sqrt(m2), np.sqrt(m3), np.sqrt(m4)

        # --- SEND TO SIMULATOR ---
        motor_msg = Actuators()
        motor_msg.velocity = [float(w1), float(w2), float(w3), float(w4)]
        self.motor_pub.publish(motor_msg)

        self.last_time = current_time
        self.last_q = q

def main(args=None):
    rclpy.init(args=args)
    node = ResearchPaperDroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF