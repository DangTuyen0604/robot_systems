# Reproducible environment

## Supported baseline

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 (Jammy), 64-bit |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| GCC | 11.x |
| CMake | 3.22.x |
| Gazebo | Gazebo Classic 11 through `ros-humble-gazebo-ros-pkgs` |
| OpenCV (ROS/system) | Ubuntu package 4.5.4 |
| NumPy (AI venv) | 1.26.4; do not upgrade to 2.x on Humble |

The inspected development machine is x86_64, Ubuntu 22.04.5, Python 3.10.12,
GCC 11.4.0, and CMake 3.22.1. CUDA is optional; no working NVIDIA driver was
detected during the environment audit.

## Laptop installation

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-image-transport-plugins \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-venv

sudo rosdep init  # Skip if already initialized.
rosdep update

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-laptop.txt

source /opt/ros/humble/setup.bash
cd laptop_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The pinned Torch package is the CPU-compatible PyPI build. For NVIDIA CUDA,
install the matching Torch wheel from the official PyTorch index first, then
install the remaining requirements without replacing Torch. Keep the exact
CUDA, driver, Torch, and GPU versions in deployment records.

## Raspberry Pi installation

Use 64-bit Ubuntu 22.04 with ROS 2 Humble matching the laptop's DDS domain.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-image-transport \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-serial \
  python3-rosdep

python3 -m pip install --user -r requirements-raspberry.txt
source /opt/ros/humble/setup.bash
cd raspberry_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Configure a udev rule that exposes the Arduino as `/dev/robot_arduino`. Add the
ROS user to `dialout`, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

## Network

Set the same domain on both computers and ensure multicast/UDP is permitted:

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
```

For repeatability, store these values in a dedicated robot environment script,
not in source files. Synchronize time on both devices with systemd-timesyncd or
chrony.

## Production models

Only these files are loaded by `traffic_sign_node.py`:

| Purpose | File | SHA-256 |
|---|---|---|
| Traffic signs | `viet.pt` | `eeab84b4ef1c5818417bb34abaac237c95857449b57de66cbf432267caaef8f7` |
| Traffic lights | `traffic_light.pt` | `360fbc9e8f87c222a5b52b39eead63e04ddcbc2984f2d795087b4a57904a3d94` |

`viet.pt` is the source YOLOv8 traffic-sign checkpoint. The current Gazebo
world contains no traffic lights, so `simulation.yaml` disables the
separate YOLO11 `traffic_light.pt` model. Enable it only with an Ultralytics
release that provides the `C3k2` layer.

For stable CPU inference, simulation executes the exported
`yolov8n.onnx` model through ONNX Runtime. Its SHA-256 is
`6bd6b0deb7671c10754cd7df8b85fb43689f9830435a2c7d8cc9353980b2b026`.
The `.pt` file is retained as the source checkpoint and is not used in the
simulation runtime path.
The ONNX graph has a fixed `320x320` input; keep the simulation `imgsz`
parameter at 320 unless the model is exported again with dynamic shapes.

The other weight files are experiments and should be moved to an external model
registry after their provenance has been checked. Do not delete them solely by
filename. Store production weights with Git LFS or an artifact registry and
verify their hashes before deployment.

## Clean rebuild

Build outputs are intentionally ignored and may be regenerated:

```bash
cd laptop_ws
rm -rf build install log
colcon build --symlink-install
```

Repeat in `raspberry_ws`. Never run the cleanup command from an unresolved path.
