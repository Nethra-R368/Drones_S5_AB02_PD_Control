#!/usr/bin/env python3
"""
Offline test of gazebo.py (v2) without ROS: rclpy and the two message types are
replaced by minimal stand-ins, and the node is driven by the X3 surrogate exactly
as Gazebo would drive it (PoseArray with a sim-time stamp at 60 Hz -> Actuators).
Checks: the node flies the surrogate for the full duration, writes its log, and
plot_gazebo_log.py can process that log.

    python3 tests/test_gazebo_node_offline.py
"""
import os, sys, types
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT); sys.path.insert(0, os.path.join(ROOT, "x3_sim"))

# ---- stand-ins for rclpy / geometry_msgs / actuator_msgs --------------------
class _Logger:
    def info(self, s): pass
    def error(self, s): print(s)
class _Param:
    def __init__(self, v): self.value = v
class Node:
    _overrides = {}
    def __init__(self, name): self._p = {}; self._pub = None
    def declare_parameter(self, n, v): self._p[n] = Node._overrides.get(n, v)
    def get_parameter(self, n): return _Param(self._p[n])
    def create_subscription(self, *a): return None
    def create_publisher(self, typ, topic, qos):
        self._pub = types.SimpleNamespace(publish=lambda m: setattr(self, "_last", m)); return self._pub
    def get_logger(self): return _Logger()
    def get_clock(self): raise RuntimeError("stamp must come from the message")
    def destroy_node(self): pass
rclpy = types.ModuleType("rclpy"); rclpy_node = types.ModuleType("rclpy.node"); rclpy_node.Node = Node
rclpy.node = rclpy_node; rclpy.init = lambda args=None: None; rclpy.ok = lambda: False
gm = types.ModuleType("geometry_msgs"); gmm = types.ModuleType("geometry_msgs.msg"); gmm.PoseArray = object; gm.msg = gmm
am = types.ModuleType("actuator_msgs"); amm = types.ModuleType("actuator_msgs.msg")
class Actuators:
    def __init__(self): self.velocity = []
amm.Actuators = Actuators; am.msg = amm
sys.modules.update({"rclpy": rclpy, "rclpy.node": rclpy_node, "geometry_msgs": gm, "geometry_msgs.msg": gmm,
                    "actuator_msgs": am, "actuator_msgs.msg": amm})

import gazebo                       # the real node file
from x3_model import X3


def pose_msg(t, p, q):
    ns = types.SimpleNamespace
    sec = int(t); nsec = int(round((t - sec) * 1e9))
    pose = ns(position=ns(x=p[0], y=p[1], z=p[2]), orientation=ns(x=q[0], y=q[1], z=q[2], w=q[3]))
    return ns(header=ns(stamp=ns(sec=sec, nanosec=nsec)), poses=[pose])


def fly(mode, experiment, duration, dist):
    Node._overrides = dict(controller_mode=mode, experiment=experiment, duration=duration, disturbance_scale=dist,
                           log_file=f"/tmp/test_{mode}_{experiment}.npz")
    node = gazebo.ConstrainedPaperController()
    x3 = X3()
    pending = np.zeros(4)
    k = 0
    try:
        while True:
            if k % 17 == 0:                       # ~60 Hz pose messages (1 ms physics)
                p, q = x3.pose()
                x3.set_motor_speed(pending)       # one-sample transport delay
                node.pose_callback(pose_msg(x3.t, p, q))
                if hasattr(node, "_last"): pending = np.array(node._last.velocity)
            x3.step(); k += 1
            assert not x3.crashed, f"crashed at {x3.t:.1f}s"
    except SystemExit:
        pass
    assert os.path.exists(node.log_file)
    return x3, node


if __name__ == "__main__":
    x3, node = fly("constrained", "hover", 40.0, 0.0)
    print(f"hover: final position {x3.p.round(3)}  (target z = 2.0)")
    assert abs(x3.p[2] - 2.0) < 0.05
    x3, node = fly("constrained", "square", 45.0, 0.0)
    print(f"square: final position {x3.p.round(3)}")
    assert np.linalg.norm(x3.p - [0, 0, 2]) < 0.1
    x3, node = fly("symmetric", "hover", 40.0, 0.5)
    print(f"hover + 0.5x disturbance (injected through rotors): final position {x3.p.round(3)}")
    import plot_gazebo_log
    # write test plots to /tmp so surrogate output can never be mistaken for a Gazebo result
    plot_gazebo_log.FIG = plot_gazebo_log.RES = "/tmp"
    plot_gazebo_log.process("/tmp/test_constrained_hover.npz")
    print("ALL OFFLINE NODE TESTS PASSED")
