#!/usr/bin/env python3
"""
gazebo.py (v2) - ROS 2 node that flies the Gazebo "X3" quadcopter with the
constrained PD-type attitude controller of

    M. Golestani et al., "A Simple Structure Constrained Attitude Control for
    Rigid Bodies: A PD-Type Control", IEEE Access 10 (2022) 10202-10209.

Topics (gz-sim quadcopter world, bridged with ros_gz_bridge):
    in   /world/quadcopter/dynamic_pose/info   geometry_msgs/PoseArray  (pose of the X3, sim-time stamp)
    out  /X3/gazebo/command/motor_speed         actuator_msgs/Actuators  (4 rotor speeds, rad/s)

The control law itself (paper Eq. 8, 11-14, 22, plus the X3 mixer and outer
position loop) lives in x3_sim/controllers.py -> FixedController, so that the
node, the software-in-the-loop surrogate (x3_sim/) and the offline test
(tests/test_gazebo_node_offline.py) all run exactly the same code.
The first version of this node, which crashed in Gazebo, is kept unchanged in
legacy/gazebo_v1.py; the faults found in it (F9-F14) are listed in the docs.

ROS parameters (ros2 run ... --ros-args -p name:=value):
    controller_mode    'constrained' (paper Eq. 13) | 'symmetric' | 'pd'      default 'constrained'
    experiment         'hover' (2 m hover) | 'square' (2 m x 2 m square)      default 'hover'
    duration           experiment length in seconds                            default 180.0
    disturbance_scale  multiplies the paper's d(t) = [0.05 sin t, 0.1 sin 1.2t, 0.15 sin 1.5t] N m,
                       injected as an extra body torque through the rotors     default 0.0
    disturbance_start  time (s) at which the disturbance is switched on        default 15.0
    log_file           .npz log, processed by plot_gazebo_log.py              default /tmp/drone_<mode>_<experiment>.npz
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "x3_sim"))
from controllers import FixedController          # noqa: E402  (same law as the surrogate)
from trajectories import square_traj             # noqa: E402


def paper_disturbance(t):
    """Paper Section IV disturbance d(t) in N m."""
    return np.array([0.05 * np.sin(t), 0.10 * np.sin(1.2 * t), 0.15 * np.sin(1.5 * t)])


class ConstrainedPaperController(Node):

    def __init__(self):
        super().__init__("constrained_paper_controller")
        for name, default in (("controller_mode", "constrained"), ("experiment", "hover"),
                              ("duration", 180.0), ("disturbance_scale", 0.0),
                              ("disturbance_start", 15.0), ("log_file", "")):
            self.declare_parameter(name, default)
        self.mode = str(self.get_parameter("controller_mode").value)
        self.experiment = str(self.get_parameter("experiment").value)
        self.duration = float(self.get_parameter("duration").value)
        self.dist_scale = float(self.get_parameter("disturbance_scale").value)
        self.dist_start = float(self.get_parameter("disturbance_start").value)
        self.log_file = str(self.get_parameter("log_file").value) or \
            f"/tmp/drone_{self.mode}_{self.experiment}.npz"
        if self.mode not in ("constrained", "symmetric", "pd"):
            raise ValueError(f"unknown controller_mode {self.mode}")
        if self.experiment not in ("hover", "square"):
            raise ValueError(f"unknown experiment {self.experiment}")

        self.ctrl = FixedController(mode=self.mode,
                                    trajectory=square_traj if self.experiment == "square" else None)
        self.pose_sub = self.create_subscription(PoseArray, "/world/quadcopter/dynamic_pose/info",
                                                 self.pose_callback, 10)
        self.motor_pub = self.create_publisher(Actuators, "/X3/gazebo/command/motor_speed", 10)
        self.t0 = None
        self.finished = False
        self.raw = {"stamp": [], "p": [], "q": [], "cmd": []}
        self.get_logger().info(f"mode={self.mode} experiment={self.experiment} duration={self.duration}s "
                               f"disturbance={self.dist_scale}x from {self.dist_start}s -> {self.log_file}")

    # ------------------------------------------------------------------
    def pose_callback(self, msg):
        if self.finished or len(msg.poses) == 0:
            return
        # F12: time from the simulator stamp, never from the wall clock
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.t0 is None:
            self.t0 = stamp
        t = stamp - self.t0
        if t > self.duration:
            self.finish()
            return
        pose = msg.poses[0]
        p = np.array([pose.position.x, pose.position.y, pose.position.z], float)
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], float)

        # optional disturbance test: extra body torque realised by the rotors
        if self.dist_scale > 0.0 and t >= self.dist_start:
            self.ctrl.extra_tau = self.dist_scale * paper_disturbance(t - self.dist_start)
        else:
            self.ctrl.extra_tau = np.zeros(3)

        cmd = self.ctrl.update(stamp, p, q)
        out = Actuators()
        out.velocity = [float(c) for c in cmd]
        self.motor_pub.publish(out)

        self.raw["stamp"].append(stamp); self.raw["p"].append(p)
        self.raw["q"].append(q); self.raw["cmd"].append(np.array(cmd, float))

    # ------------------------------------------------------------------
    def stop_motors(self):
        out = Actuators()
        out.velocity = [0.0, 0.0, 0.0, 0.0]
        self.motor_pub.publish(out)

    def save_data(self):
        L = {k: np.asarray(v) for k, v in self.ctrl.log.items()}
        try:
            np.savez(self.log_file, mode=self.mode, experiment=self.experiment,
                     disturbance_scale=self.dist_scale,
                     raw_stamp=np.asarray(self.raw["stamp"]), raw_p=np.asarray(self.raw["p"]),
                     raw_q=np.asarray(self.raw["q"]), raw_cmd=np.asarray(self.raw["cmd"]),
                     **{"ctrl_" + k: v for k, v in L.items()})
            self.get_logger().info(f"log written to {self.log_file}")
        except Exception as exc:                       # never lose the flight because of the log
            self.get_logger().error(f"could not save log: {exc}")

    def finish(self):
        self.finished = True
        self.stop_motors()
        self.save_data()
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ConstrainedPaperController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.finished:
            node.stop_motors()
            node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
