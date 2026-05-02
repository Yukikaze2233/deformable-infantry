#!/usr/bin/env python3

import argparse
import json
import math
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

try:
    import zenoh
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "zenoh Python package is required. Install with: python3 -m pip install eclipse-zenoh"
    ) from exc

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


JOINT_EFFORT_TOPICS = [
    "/sim/chassis/left_front_joint/effort_cmd",
    "/sim/chassis/left_back_joint/effort_cmd",
    "/sim/chassis/right_back_joint/effort_cmd",
    "/sim/chassis/right_front_joint/effort_cmd",
]

WHEEL_EFFORT_TOPICS = [
    "/sim/chassis/left_front_wheel/effort_cmd",
    "/sim/chassis/left_back_wheel/effort_cmd",
    "/sim/chassis/right_back_wheel/effort_cmd",
    "/sim/chassis/right_front_wheel/effort_cmd",
]


def build_zenoh_config(mode: str, endpoint: str):
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(mode))
    if mode == "client":
        config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    else:
        config.insert_json5("listen/endpoints", json.dumps([endpoint]))
    return config


@dataclass
class CommandState:
    cmd_vel: Optional[Dict] = None
    base_target: Optional[List[float]] = None
    joint_efforts: Dict[str, float] = field(default_factory=dict)
    wheel_efforts: Dict[str, float] = field(default_factory=dict)


class ZenohRos2Bridge(Node):
    def __init__(self, args):
        super().__init__("rmcs_zenoh_ros2_bridge")
        self._args = args
        self._command_state = CommandState()
        self._command_lock = threading.Lock()

        self._session = zenoh.open(build_zenoh_config(args.mode, args.endpoint))
        self._joint_state_pub = self.create_publisher(JointState, args.joint_state_topic, 10)
        self._imu_pub = self.create_publisher(Imu, args.imu_topic, 10)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd_vel, reliable_qos)
        self.create_subscription(
            Float64MultiArray, args.base_target_topic, self._on_base_target, reliable_qos
        )

        for topic in JOINT_EFFORT_TOPICS:
            self.create_subscription(Float64, topic, self._make_effort_cb(topic, True), reliable_qos)
        for topic in WHEEL_EFFORT_TOPICS:
            self.create_subscription(Float64, topic, self._make_effort_cb(topic, False), reliable_qos)

        self._joint_state_pub_zenoh = self._session.declare_publisher(args.zenoh_joint_state_key)
        self._imu_pub_zenoh = self._session.declare_publisher(args.zenoh_imu_key)
        self._command_pub_zenoh = self._session.declare_publisher(args.zenoh_command_key)

        self._joint_state_sub = self._session.declare_subscriber(
            args.zenoh_joint_state_key, self._on_zenoh_joint_state
        )
        self._imu_sub = self._session.declare_subscriber(args.zenoh_imu_key, self._on_zenoh_imu)

        self._command_timer = self.create_timer(args.command_rate, self._publish_commands)
        self.get_logger().info(
            f"zenoh bridge ready: mode={args.mode} endpoint={args.endpoint} "
            f"ros_topics=({args.joint_state_topic}, {args.imu_topic}, {args.cmd_vel_topic})"
        )

    def _on_cmd_vel(self, msg: Twist):
        with self._command_lock:
            self._command_state.cmd_vel = {
                "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
                "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
                "stamp": time.time(),
            }

    def _on_base_target(self, msg: Float64MultiArray):
        with self._command_lock:
            self._command_state.base_target = list(msg.data)

    def _make_effort_cb(self, topic: str, is_joint: bool):
        def _cb(msg: Float64):
            with self._command_lock:
                if is_joint:
                    self._command_state.joint_efforts[topic] = msg.data
                else:
                    self._command_state.wheel_efforts[topic] = msg.data

        return _cb

    def _publish_commands(self):
        with self._command_lock:
            payload = {
                "cmd_vel": self._command_state.cmd_vel,
                "base_target_physical_angle": self._command_state.base_target,
                "joint_efforts": self._command_state.joint_efforts,
                "wheel_efforts": self._command_state.wheel_efforts,
            }
        self._command_pub_zenoh.put(json.dumps(payload))

    def _on_zenoh_joint_state(self, sample):
        data = json.loads(sample.payload.to_bytes().decode("utf-8"))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = data.get("name", [])
        msg.position = data.get("position", [])
        msg.velocity = data.get("velocity", [])
        msg.effort = data.get("effort", [])
        self._joint_state_pub.publish(msg)

    def _on_zenoh_imu(self, sample):
        data = json.loads(sample.payload.to_bytes().decode("utf-8"))
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = data.get("frame_id", "")
        ori = data.get("orientation", {})
        msg.orientation.x = ori.get("x", 0.0)
        msg.orientation.y = ori.get("y", 0.0)
        msg.orientation.z = ori.get("z", 0.0)
        msg.orientation.w = ori.get("w", 1.0)
        ang = data.get("angular_velocity", {})
        msg.angular_velocity.x = ang.get("x", 0.0)
        msg.angular_velocity.y = ang.get("y", 0.0)
        msg.angular_velocity.z = ang.get("z", 0.0)
        lin = data.get("linear_acceleration", {})
        msg.linear_acceleration.x = lin.get("x", 0.0)
        msg.linear_acceleration.y = lin.get("y", 0.0)
        msg.linear_acceleration.z = lin.get("z", 0.0)
        self._imu_pub.publish(msg)

    def close(self):
        self._joint_state_sub.undeclare()
        self._imu_sub.undeclare()
        self._joint_state_pub_zenoh.undeclare()
        self._imu_pub_zenoh.undeclare()
        self._command_pub_zenoh.undeclare()
        self._session.close()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["client", "peer"], default="client")
    parser.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    parser.add_argument("--command-rate", type=float, default=0.02)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--base-target-topic", default="/sim/chassis/base_target_physical_angle")
    parser.add_argument("--zenoh-command-key", default="rmcs/cmd")
    parser.add_argument("--zenoh-joint-state-key", default="rmcs/state/joint_states")
    parser.add_argument("--zenoh-imu-key", default="rmcs/state/imu")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    bridge = ZenohRos2Bridge(args)

    shutdown = threading.Event()

    def _stop(*_args):
        shutdown.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(bridge, timeout_sec=0.1)
    finally:
        bridge.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
