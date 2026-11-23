# RWRUMI

**Team 4 · [Real World Robotics (RWR)](https://rwr.ethz.ch/) · Task X: UMI for the ORCA hand v2**
Project page: **https://erhell.github.io/RWRUMI/**

This README is the working guide for Team 4: how the repository is structured, how to set
up the environment, and how we add code and files.

> **Status:** OAK-D Pro driver and AprilTag localization working ([section 7](#7-camera-and-localization)). Next: test with a printed tag, then `rwrumi_description`.

---

## 1. Setup (once per machine)

We use **ROS 2 Humble from [RoboStack](https://robostack.github.io)** inside a `mamba`
environment. Don't install ROS from apt, and don't use Docker.

```bash
# 1. Install mamba: Miniforge (https://github.com/conda-forge/miniforge) or micromamba
#    (micromamba users: replace `mamba` with `micromamba` everywhere)

# 2. Clone
git clone https://github.com/erhell/RWRUMI.git
cd RWRUMI

# 3. Create the environment from environment.yml
mamba env create -f environment.yml

# 4. Build
mamba activate rwrumi
colcon build --symlink-install
```

Our machines: Ubuntu 26.04 and macOS (Apple silicon). The environment uses Python 3.12.

## 2. Daily workflow

```bash
cd RWRUMI
mamba activate rwrumi
export ROS_DOMAIN_ID=4                                # our team's ID on the lab network
git pull
mamba env update -f environment.yml --prune           # only if environment.yml changed
colcon build --symlink-install                        # always from the repo root
source install/setup.zsh                              # or install/setup.bash
```

## 3. Repository structure

```text
RWRUMI/
├── README.md                this guide
├── docs/index.html          project page (GitHub Pages), same content as this README
├── environment.yml          the shared environment (ROS 2 + all dependencies)
├── .gitignore
└── src/                     all ROS 2 packages
    ├── rwrumi_description/  robot model: URDF, meshes (incl. files copied from ORCA)
    │   ├── urdf/
    │   ├── meshes/
    │   ├── LICENSE.orca     ORCA's license for the copied files
    │   └── SOURCE.md        where copied files come from + what we changed
    ├── rwrumi_interfaces/   custom messages (e.g. TactileArray.msg)
    ├── rwrumi_camera/       OAK-D Pro driver
    ├── rwrumi_joint_sensors/
    ├── rwrumi_tactile/
    ├── rwrumi_pose/         global + relative pose estimation
    ├── rwrumi_recorder/     recording + bag → HDF5 export
    └── rwrumi_bringup/      launch files, configs, RViz setup
```

| Package | Content | Status |
|---|---|---|
| `rwrumi_description` | URDF + meshes | planned |
| `rwrumi_interfaces` | custom `.msg` / `.srv` | planned |
| `rwrumi_camera` | OAK-D Pro: color, aligned depth, calibration, IMU | working |
| `rwrumi_joint_sensors` | joint sensor driver | planned |
| `rwrumi_tactile` | tactile sensor driver | planned |
| `rwrumi_pose` | AprilTag global + relative pose | working, not yet tested with a printed tag |
| `rwrumi_recorder` | recording + export | planned |
| `rwrumi_bringup` | launch files, configs | camera + localization |

## 4. How we add things

**A new ROS package**
```bash
cd src
ros2 pkg create --build-type ament_python rwrumi_<name>   # Python nodes
ros2 pkg create --build-type ament_cmake  rwrumi_<name>   # messages, URDF/mesh-only packages
```
- Every package lives directly in `src/` and starts with `rwrumi_`.
- List everything it uses in its `package.xml` (`<depend>`, `<exec_depend>`).
- Add the package to the structure above in the same pull request.

**A new dependency (library, ROS package, tool)**
- Add it to `environment.yml` (RoboStack / conda-forge name, e.g. `ros-humble-<name>`;
  pip-only packages under a `pip:` section), then run
  `mamba env update -f environment.yml --prune` and commit the file.
- Never `mamba install` or `pip install` by hand: your teammates wouldn't get it.

**Files from ORCA**
- Copy only the files you need from
  [`orcahand/orcahand_description`](https://github.com/orcahand/orcahand_description)
  into `src/rwrumi_description/` (`urdf/` or `meshes/`). No submodule.
- Note the source (repo, commit, path) and every change you make in `SOURCE.md`.
- Keep `LICENSE.orca` next to the copied files (ORCA is MIT-licensed).
- Current source version: `orcahand_description` commit `b9b349a2`, folder `v2/`, right hand.

**Referencing files in code**
- Always by package name, never by path:
  `package://rwrumi_description/meshes/…` in URDF, `FindPackageShare('rwrumi_description')`
  in launch files, `get_package_share_directory('rwrumi_description')` in Python.

## 5. Conventions

| What | Rule |
|---|---|
| Packages | `rwrumi_<name>`, all in `src/` |
| Topics | under `/umi`, e.g. `/umi/joint_states`, `/umi/tactile` |
| Frames | [REP 105](https://www.ros.org/reps/rep-0105.html): `world` → `umi_base` → hand links |
| Joint / link names | readable names (`index_mcp`, …), not ORCA's generated ones |
| ROS network | `ROS_DOMAIN_ID=4` |
| Recordings | `rosbag2` with MCAP (`ros2 bag record -s mcap`) |

## 6. Git workflow

- `main` only changes through pull requests. Work on a branch:
  `feature/<topic>` or `setup/<topic>`.
- Never commit `build/`, `install/` or `log/` (already in `.gitignore`).
- Commit `environment.yml` whenever you change it.
- If your PR adds, moves or removes a package, or settles an open decision, update this
  README and `docs/index.html` in the same PR.

## 7. Camera and localization

The OAK-D Pro gives color, depth aligned to color, and IMU. AprilTags (36h11) define a metric
world frame, so the pose recovers immediately whenever a known tag is back in view.

**Before the first run:** print a 36h11 tag, measure the side of its **black square**, and enter
the ID and size in `src/rwrumi_bringup/config/tag_map.yaml`. Every position scales with this size.

```bash
ros2 launch rwrumi_bringup camera_localization.launch.py rviz:=true
ros2 service call /umi/pose/set_anchor std_srvs/srv/Trigger   # zero the relative pose, e.g. at episode start
```

Launch arguments: `fps` (default 15, use 30 on USB 3), `ir_dot_intensity` (0.0 to 1.0, default off,
helps depth on plain surfaces), `mxid`, `tag_map`, `rviz`.

| Topic | Type | Content |
|---|---|---|
| `/umi/camera/color/image_raw` | `sensor_msgs/Image` | 960 × 540 BGR |
| `/umi/camera/depth/image_raw` | `sensor_msgs/Image` | depth in mm, aligned to color, same stamp |
| `/umi/camera/{color,depth}/camera_info` | `sensor_msgs/CameraInfo` | factory calibration |
| `/umi/camera/imu` | `sensor_msgs/Imu` | raw accelerometer + gyroscope, 200 Hz |
| `/umi/pose/camera` | `geometry_msgs/PoseStamped` | camera in `world`; also on TF |
| `/umi/pose/relative` | `geometry_msgs/PoseStamped` | camera relative to the anchor |
| `/umi/pose/status` | `diagnostic_msgs/DiagnosticStatus` | `ok`, `no_tags`, `unknown_tags`, `ambiguous`, `high_error` |
| `/umi/pose/debug_image` | `sensor_msgs/Image` | detections drawn on the image |

- No pose is published while no known tag is visible, so check `/umi/pose/status` when recording.
- The first valid pose becomes the anchor until `set_anchor` is called.
- Run the pose tests with `cd src/rwrumi_pose && python -m pytest test`.

## Open decisions

- [x] Camera: OAK-D Pro
- [ ] Tracking without visible tags (visual-inertial odometry / SLAM)
- [ ] Object detection model and output format
- [ ] Joint sensor hardware and interface
- [ ] Tactile sensor hardware and message format
- [ ] Dataset format (episode structure, sampling rate)

## Links

- Course: [Real World Robotics](https://rwr.ethz.ch/)
- ORCA hand: [orcahand.com](https://orcahand.com) ·
  [`orcahand_description`](https://github.com/orcahand/orcahand_description) (MIT)
- [RoboStack](https://robostack.github.io) · [ROS 2 Humble docs](https://docs.ros.org/en/humble/)
