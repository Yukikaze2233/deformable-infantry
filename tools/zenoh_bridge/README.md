# Zenoh Python Bridge

This directory contains a custom bridge that avoids DDS communication across
the Windows/WSL boundary:

- WSL/container side: ROS 2 <-> zenoh
- Windows/Isaac side: zenoh <-> Isaac Sim Python APIs

## Topics

Isaac -> ROS 2:

- `/joint_states`
- `/imu`

ROS 2 -> Isaac:

- `/cmd_vel`
- `/sim/chassis/base_target_physical_angle`
- `/sim/chassis/*/effort_cmd`

The Isaac-side bridge only maps the 8 actuated DOFs:

- `left_front_joint`
- `left_back_joint`
- `right_back_joint`
- `right_front_joint`
- `left_front_wheel`
- `left_back_wheel`
- `right_back_wheel`
- `right_front_wheel`

Passive helper joints such as `*_carrier_joint` and `*_lower_joint` are ignored.

## Python dependency

Install the official zenoh Python package in both environments:

```bash
python3 -m pip install eclipse-zenoh
```

For Isaac Sim, run with its bundled Python:

```powershell
C:\isaac-sim\python.bat -m pip install eclipse-zenoh
```

## Run order

1. Start the Isaac script in Script Editor or with `Execute Script`.
2. Start the WSL ROS 2 bridge.
3. Launch RMCS with the sim config.
