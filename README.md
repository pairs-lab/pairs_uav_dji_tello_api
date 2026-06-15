# pairs_uav_dji_tello_api

Hardware API plugin that lets the PAIRS UAV stack fly a DJI Tello drone. It
implements the `pairs_uav_hw_api::PairsUavHwApi` plugin interface for the Tello,
and ships a Python wrapper (built on the `tellopy` protocol library) that talks
to the drone over Wi-Fi and bridges its telemetry, camera, and command channels
into ROS. With this package the rest of the PAIRS control and estimation stack
can command a Tello the same way it commands any other supported platform.

## Contents

- `pairs_uav_dji_tello_api::Api` — pluginlib plugin loaded by the HW API
  manager, exporting to `pairs_uav_hw_api`.
- `tellopy_wrapper.py` — Python node that drives the Tello over its native
  protocol and publishes pose, twist, IMU, height, battery, and camera streams
  while accepting arm/throw/command inputs (installed as the `api` entry point).
- Vendored `tellopy` Python package (protocol, state, video stream).
- Launch files: `hw_api.launch.py` (composable HW API container),
  `tellopy_wrapper.launch.py`, `flame_ros.launch.py`.
- Configs: `config/tello_api.yaml`, plus a `platform_config` UAV-system profile.
- A bundled tmux session under `tmux/`.

## Branches

- `ros1` — ROS 1 Noetic (catkin).
- `ros2` — ROS 2 Jazzy (ament_cmake).

## Install (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-pairs-uav-dji-tello-api
```

## Usage

```bash
ros2 launch pairs_uav_dji_tello_api hw_api.launch.py
```
