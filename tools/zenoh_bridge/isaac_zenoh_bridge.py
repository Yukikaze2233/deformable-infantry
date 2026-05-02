"""
Run this inside Isaac Sim's Script Editor or Execute Script.

It avoids Windows-local DDS interop by talking zenoh directly and touching
Isaac Sim Python APIs for articulation state / command IO.
"""

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

try:
    import zenoh
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise RuntimeError(
        "zenoh Python package is required inside Isaac Sim. "
        "Install with: C:\\isaac-sim\\python.bat -m pip install eclipse-zenoh"
    ) from exc

import omni.timeline
from omni.physx import get_physx_interface
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.sensors.physics import _sensor


def build_zenoh_config(mode: str, endpoint: str):
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(mode))
    if mode == "client":
        config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    else:
        config.insert_json5("listen/endpoints", json.dumps([endpoint]))
    return config


@dataclass
class IsaacZenohBridgeConfig:
    mode: str = "peer"
    endpoint: str = "tcp/0.0.0.0:7447"
    articulation_prim: str = "/World/deformable_infantry_omni/base_link"
    imu_prim: str = "/World/deformable_infantry_omni/base_link/sensor"
    imu_fallback_rigid_prim: str = "/World/deformable_infantry_omni/base_link"
    actuated_dof_names: List[str] = field(
        default_factory=lambda: [
            "left_back_joint",
            "left_front_joint",
            "right_back_joint",
            "right_front_joint",
            "left_back_wheel",
            "left_front_wheel",
            "right_back_wheel",
            "right_front_wheel",
        ]
    )
    zenoh_command_key: str = "rmcs/cmd"
    zenoh_joint_state_key: str = "rmcs/state/joint_states"
    zenoh_imu_key: str = "rmcs/state/imu"


class IsaacZenohBridge:
    def __init__(self, cfg: IsaacZenohBridgeConfig):
        self.cfg = cfg
        self.session = zenoh.open(build_zenoh_config(cfg.mode, cfg.endpoint))
        self.timeline = omni.timeline.get_timeline_interface()
        self.imu = _sensor.acquire_imu_sensor_interface()
        self.articulation = SingleArticulation(prim_path=cfg.articulation_prim, name="rmcs_bridge_articulation")
        self.base_rigid = SingleRigidPrim(cfg.imu_fallback_rigid_prim, "rmcs_bridge_base")
        self._command_lock = threading.Lock()
        self._latest_command = {}
        self._pub_joint = self.session.declare_publisher(cfg.zenoh_joint_state_key)
        self._pub_imu = self.session.declare_publisher(cfg.zenoh_imu_key)
        self._sub_cmd = self.session.declare_subscriber(cfg.zenoh_command_key, self._on_command)
        self._initialized = False
        self._last_init_error = 0.0
        self._last_runtime_error = 0.0
        self._actuated_indices: List[int] = []
        self._actuated_names: List[str] = []
        self._sub = get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        print(f"IsaacZenohBridge ready mode={cfg.mode} endpoint={cfg.endpoint}")

    def _ensure_initialized(self):
        if self._initialized:
            return True
        try:
            self.articulation.initialize()
            self.base_rigid.initialize()
        except Exception as exc:
            now = time.time()
            if now - self._last_init_error > 1.0:
                print(f"IsaacZenohBridge waiting for physics scene: {exc}")
                self._last_init_error = now
            return False
        dof_names = list(self.articulation.dof_names)
        self._actuated_indices = []
        self._actuated_names = []
        missing_names = []
        for name in self.cfg.actuated_dof_names:
            if name in dof_names:
                self._actuated_indices.append(dof_names.index(name))
                self._actuated_names.append(name)
            else:
                missing_names.append(name)
        self._initialized = True
        print("IsaacZenohBridge initialized")
        print("DOF names:", dof_names)
        print("Actuated DOFs:", self._actuated_names)
        if missing_names:
            print("Missing actuated DOFs:", missing_names)
        return True

    def _on_command(self, sample):
        with self._command_lock:
            self._latest_command = json.loads(sample.payload.to_bytes().decode("utf-8"))

    def _publish_joint_state(self):
        positions_all = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
        velocities_all = np.asarray(self.articulation.get_joint_velocities()).reshape(-1)
        try:
            efforts_all = np.asarray(self.articulation.get_applied_joint_efforts()).reshape(-1)
        except Exception:
            efforts_all = np.zeros_like(positions_all)
        positions = positions_all[self._actuated_indices].tolist()
        velocities = velocities_all[self._actuated_indices].tolist()
        efforts = efforts_all[self._actuated_indices].tolist()
        payload = {
            "name": self._actuated_names,
            "position": positions,
            "velocity": velocities,
            "effort": efforts,
            "stamp": time.time(),
        }
        self._pub_joint.put(json.dumps(payload))

    def _publish_imu(self):
        try:
            reading = self.imu.get_sensor_reading(self.cfg.imu_prim)
        except Exception:
            reading = None
        if reading is not None and getattr(reading, "is_valid", False):
            payload = {
                "frame_id": self.cfg.imu_prim,
                "orientation": {
                    "x": float(reading.orientation[0]),
                    "y": float(reading.orientation[1]),
                    "z": float(reading.orientation[2]),
                    "w": float(reading.orientation[3]),
                },
                "angular_velocity": {
                    "x": float(reading.ang_vel_x),
                    "y": float(reading.ang_vel_y),
                    "z": float(reading.ang_vel_z),
                },
                "linear_acceleration": {
                    "x": float(reading.lin_acc_x),
                    "y": float(reading.lin_acc_y),
                    "z": float(reading.lin_acc_z),
                },
                "stamp": time.time(),
            }
        else:
            _, orientation = self.base_rigid.get_world_pose()
            payload = {
                "frame_id": self.cfg.imu_fallback_rigid_prim,
                "orientation": {
                    "x": float(orientation[1]),
                    "y": float(orientation[2]),
                    "z": float(orientation[3]),
                    "w": float(orientation[0]),
                },
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                "stamp": time.time(),
            }
        self._pub_imu.put(json.dumps(payload))

    def _apply_efforts(self):
        with self._command_lock:
            cmd = dict(self._latest_command)
        dof_names = list(self.articulation.dof_names)
        efforts = np.zeros((len(dof_names),), dtype=np.float64)
        for topic, value in cmd.get("joint_efforts", {}).items():
            name = topic.split("/")[-2]
            if name in self._actuated_names and name in dof_names:
                efforts[dof_names.index(name)] = float(value)
        for topic, value in cmd.get("wheel_efforts", {}).items():
            name = topic.split("/")[-2]
            if name in self._actuated_names and name in dof_names:
                efforts[dof_names.index(name)] = float(value)
        if efforts.any():
            self.articulation.set_joint_efforts(efforts)

    def _on_physics_step(self, _dt):
        if not self.timeline.is_playing():
            return
        if not self._ensure_initialized():
            return
        try:
            self._apply_efforts()
            self._publish_joint_state()
            self._publish_imu()
        except Exception as exc:
            self._initialized = False
            now = time.time()
            if now - self._last_runtime_error > 1.0:
                print(f"IsaacZenohBridge lost physics view, reinitializing: {exc}")
                self._last_runtime_error = now

    def close(self):
        self._sub = None
        self._sub_cmd.undeclare()
        self._pub_joint.undeclare()
        self._pub_imu.undeclare()
        self.session.close()


# Keep a single global instance alive in Script Editor state.
global RMCS_ZENOH_BRIDGE
try:
    RMCS_ZENOH_BRIDGE.close()
except Exception:
    pass

RMCS_ZENOH_BRIDGE = IsaacZenohBridge(IsaacZenohBridgeConfig())
