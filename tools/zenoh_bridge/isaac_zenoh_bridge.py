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
from isaacsim.core.prims import SingleArticulation
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
    imu_fallback_frame_prim: str = "/World/deformable_infantry_omni/base_link"
    logical_actuated_dof_names: List[str] = field(
        default_factory=lambda: [
            "left_front_joint",
            "left_back_joint",
            "right_back_joint",
            "right_front_joint",
            "left_front_wheel",
            "left_back_wheel",
            "right_back_wheel",
            "right_front_wheel",
        ]
    )
    # Map RMCS logical motor names to the actual Isaac DOF names.
    # The current imported asset appears mirrored left/right, so the defaults
    # below route logical left commands to the actual right-side DOFs and
    # logical right commands to the actual left-side DOFs.
    actual_actuated_dof_names: List[str] = field(
        default_factory=lambda: [
            "left_front_joint",
            "left_back_joint",
            "right_back_joint",
            "right_front_joint",
            "left_front_wheel",
            "left_back_wheel",
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
        self._command_lock = threading.Lock()
        self._latest_command = {}
        self._pub_joint = self.session.declare_publisher(cfg.zenoh_joint_state_key)
        self._pub_imu = self.session.declare_publisher(cfg.zenoh_imu_key)
        self._sub_cmd = self.session.declare_subscriber(cfg.zenoh_command_key, self._on_command)
        self._initialized = False
        self._last_init_error = 0.0
        self._last_runtime_error = 0.0
        self._actuated_indices: List[int] = []
        self._logical_names: List[str] = []
        self._actual_names: List[str] = []
        self._actual_index_by_logical_name: Dict[str, int] = {}
        self._effort_mode_configured = False
        self._effort_mode_indices: List[int] = []
        self._sub = get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        print(f"IsaacZenohBridge ready mode={cfg.mode} endpoint={cfg.endpoint}")

    def _ensure_initialized(self):
        if self._initialized:
            return True
        try:
            self.articulation.initialize()
        except Exception as exc:
            now = time.time()
            if now - self._last_init_error > 1.0:
                print(f"IsaacZenohBridge waiting for physics scene: {exc}")
                self._last_init_error = now
            return False
        if len(self.cfg.logical_actuated_dof_names) != len(self.cfg.actual_actuated_dof_names):
            print(
                "IsaacZenohBridge configuration error: logical_actuated_dof_names and "
                "actual_actuated_dof_names must have the same length."
            )
            return False

        dof_names = list(self.articulation.dof_names)
        self._actuated_indices = []
        self._logical_names = []
        self._actual_names = []
        self._actual_index_by_logical_name = {}
        self._effort_mode_indices = []
        missing_names = []
        for logical_name, actual_name in zip(
            self.cfg.logical_actuated_dof_names, self.cfg.actual_actuated_dof_names
        ):
            if actual_name in dof_names:
                index = dof_names.index(actual_name)
                self._actuated_indices.append(index)
                self._logical_names.append(logical_name)
                self._actual_names.append(actual_name)
                self._actual_index_by_logical_name[logical_name] = index
            else:
                missing_names.append((logical_name, actual_name))
        self._initialized = True
        print("IsaacZenohBridge initialized")
        print("DOF names:", dof_names)
        print("Logical actuated DOFs:", self._logical_names)
        print("Actual actuated DOFs:", self._actual_names)
        print("Actuated DOF mapping:")
        for logical_name, actual_name, index in zip(
            self._logical_names, self._actual_names, self._actuated_indices
        ):
            print(f"  {logical_name} -> {actual_name} (dof_index={index})")

        # For effort control, disable USD joint drives not only on the mapped
        # actuation DOFs, but also on coupled carrier joints imported from URDF
        # mimic chains. Otherwise those hidden drives can fight external
        # set_joint_efforts() commands.
        candidate_names = set(self._actual_names)
        for name in dof_names:
            if name.endswith("_carrier_joint"):
                candidate_names.add(name)
        self._effort_mode_indices = [dof_names.index(name) for name in candidate_names if name in dof_names]
        print("Effort mode DOFs:")
        for index in sorted(self._effort_mode_indices):
            print(f"  {dof_names[index]} (dof_index={index})")

        try:
            dof_props = self.articulation.dof_properties
            joint_positions = np.asarray(self.articulation.get_joint_positions()).reshape(-1)
            print("DOF properties:")
            for i, name in enumerate(dof_names):
                prop = dof_props[i]
                pos = float(joint_positions[i]) if i < len(joint_positions) else float("nan")
                print(
                    "  "
                    f"{name}: index={i}, pos={pos:.6f}, "
                    f"type={prop['type']}, driveMode={prop['driveMode']}, "
                    f"maxEffort={prop['maxEffort']:.6f}, maxVelocity={prop['maxVelocity']:.6f}, "
                    f"stiffness={prop['stiffness']:.6f}, damping={prop['damping']:.6f}, "
                    f"lower={prop['lower']:.6f}, upper={prop['upper']:.6f}"
                )
        except Exception as exc:
            print(f"IsaacZenohBridge failed to dump DOF properties: {exc}")
        if missing_names:
            print("Missing actuated DOFs:", missing_names)
        return True

    def _configure_effort_mode(self):
        if self._effort_mode_configured or not self._effort_mode_indices:
            return
        try:
            zero_gains = np.zeros((len(self._effort_mode_indices),), dtype=np.float32)
            joint_indices = np.asarray(self._effort_mode_indices, dtype=np.int64)
            self.articulation._articulation_view.set_gains(
                kps=zero_gains.reshape(1, -1),
                kds=zero_gains.reshape(1, -1),
                joint_indices=joint_indices,
            )
            self._effort_mode_configured = True
            print("IsaacZenohBridge configured effort mode DOFs (stiffness=0, damping=0).")
        except Exception as exc:
            now = time.time()
            if now - self._last_runtime_error > 1.0:
                print(f"IsaacZenohBridge failed to configure effort mode: {exc}")
                self._last_runtime_error = now

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
            "name": self._logical_names,
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
            try:
                _, orientation = self.articulation.get_world_pose()
                frame_id = self.cfg.imu_fallback_frame_prim or self.cfg.articulation_prim
            except Exception as exc:
                now = time.time()
                if now - self._last_runtime_error > 1.0:
                    print(f"IsaacZenohBridge IMU fallback pose unavailable, using identity: {exc}")
                    self._last_runtime_error = now
                orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                frame_id = self.cfg.articulation_prim
            payload = {
                "frame_id": frame_id,
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
        efforts = np.zeros((len(self.articulation.dof_names),), dtype=np.float64)
        applied_debug = []
        for topic, value in cmd.get("joint_efforts", {}).items():
            logical_name = topic.split("/")[-2]
            if logical_name in self._actual_index_by_logical_name:
                index = self._actual_index_by_logical_name[logical_name]
                efforts[index] = float(value)
                if abs(float(value)) > 1e-9:
                    applied_debug.append((topic, logical_name, index, float(value)))
        for topic, value in cmd.get("wheel_efforts", {}).items():
            logical_name = topic.split("/")[-2]
            if logical_name in self._actual_index_by_logical_name:
                index = self._actual_index_by_logical_name[logical_name]
                efforts[index] = float(value)
                if abs(float(value)) > 1e-9:
                    applied_debug.append((topic, logical_name, index, float(value)))
        if applied_debug:
            now = time.time()
            if not hasattr(self, "_last_effort_debug_time"):
                self._last_effort_debug_time = 0.0
            if now - self._last_effort_debug_time > 0.5:
                print("IsaacZenohBridge non-zero effort commands:")
                for topic, logical_name, index, value in applied_debug:
                    actual_name = self.articulation.dof_names[index]
                    print(
                        f"  {topic} -> {logical_name} -> {actual_name} "
                        f"(dof_index={index}) = {value}"
                    )
                self._last_effort_debug_time = now
        if efforts.any():
            self.articulation.set_joint_efforts(efforts)

    def _on_physics_step(self, _dt):
        if not self.timeline.is_playing():
            return
        if not self._ensure_initialized():
            return
        try:
            self._configure_effort_mode()
            self._apply_efforts()
            self._publish_joint_state()
            self._publish_imu()
        except Exception as exc:
            self._initialized = False
            self._effort_mode_configured = False
            now = time.time()
            if now - self._last_runtime_error > 1.0:
                print(f"IsaacZenohBridge lost physics view, reinitializing: {exc}")
                self._last_runtime_error = now

    def close(self):
        self._sub = None
        for attr_name in ("_sub_cmd", "_pub_joint", "_pub_imu"):
            handle = getattr(self, attr_name, None)
            if handle is None:
                continue
            try:
                handle.undeclare()
            except Exception:
                pass
            setattr(self, attr_name, None)
        session = getattr(self, "session", None)
        if session is not None:
            try:
                session.close()
            except Exception:
                pass
            self.session = None


# Keep a single global instance alive in Script Editor state.
global RMCS_ZENOH_BRIDGE
try:
    RMCS_ZENOH_BRIDGE.close()
except Exception:
    pass

RMCS_ZENOH_BRIDGE = IsaacZenohBridge(IsaacZenohBridgeConfig())
