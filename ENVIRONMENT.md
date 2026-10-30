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
# Run colcon with the venv interpreter so ROS entry points use it and can
# import the venv packages; plain `colcon` binds them to /usr/bin/python3.
python -m colcon build --symlink-install
source install/setup.bash
```

If `python3 -m venv` fails because `ensurepip` is missing and `python3-venv`
cannot be installed, create the venv with `--without-pip` and install pip into
it from the official pip wheel.

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
not in source files.

Synchronize time on both devices with systemd-timesyncd or chrony. This is
required: the lane and sign nodes drop camera images whose capture stamp is
more than `max_image_age` (0.5 s) away from the laptop clock, so a clock offset
larger than that stops the robot and logs `Dropping camera image ... synchronise
clocks`. Check the offset with `chronyc tracking` or `timedatectl
timesync-status` on both machines. Set `max_image_age: 0.0` only to rule the
check out while debugging.

## Production models

Fetch and verify the weights from the GitHub release `models-v2` with
`python3 scripts/fetch_models.py`, then rebuild `laptop_ws`. Weights are never
committed; publish a new release (for example `models-v3`) and update
`MODELS` and `RELEASE_URL` in the script when they change.

`traffic_sign_node.py` loads a single traffic-sign detector; the system has no
traffic-light model.

| Purpose | File | SHA-256 |
|---|---|---|
| Traffic signs (robot) | `viet.pt` | `eeab84b4ef1c5818417bb34abaac237c95857449b57de66cbf432267caaef8f7` |
| Traffic signs (simulation) | `viet_320.onnx` | `28b9f86995812745035d578bba3ffac8028bdd05de7e1f3d0f9f73e2f0f9db83` |

`viet.pt` is the source YOLOv8 traffic-sign checkpoint. Its class names are
Vietnamese (`cam_re_phai`, `di_cham`, `dung_lai`, `re_trai`,
`toc_do_toi_da_20`); `perception_pkg/sign_labels.py` maps them to the labels
the decision node uses.

Simulation runs `viet_320.onnx`, `viet.pt` exported to ONNX at 320 px, through
ONNX Runtime. Its SHA-256 is
`28b9f86995812745035d578bba3ffac8028bdd05de7e1f3d0f9f73e2f0f9db83`.
Its raw outputs match the PyTorch checkpoint to within 0.001 px (boxes) and
1e-5 (class scores). On the 321 test images of the training dataset (Roboflow
`datn-stoh6` version 9), mAP@50 is 0.993 for `viet_320.onnx` and 0.994 for
`viet.pt` at 640 px; mAP@50-95 drops from 0.981 to 0.965.
The ONNX graph has a fixed `320x320` input; keep the simulation `imgsz`
parameter at 320 unless the model is exported again with dynamic shapes.
Re-export with:

```bash
yolo export model=viet.pt format=onnx imgsz=320 simplify=False
```

Experimental weights from earlier development (`yolov8n_2.*`, `yolov11n.onnx`,
`lane_best.pt`, `traffic_light*`, ...) were removed from the repository history;
no node uses them.

## Clean rebuild

Build outputs are intentionally ignored and may be regenerated:

```bash
cd laptop_ws
rm -rf build install log
python -m colcon build --symlink-install  # with .venv activated
```

Repeat in `raspberry_ws`. Never run the cleanup command from an unresolved path.
