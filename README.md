# RWRUMI — a Universal Manipulation Interface for the ORCA hand v2

**Team 4 · Real World Robotics (RWR)**
Project page: **https://erhell.github.io/RWRUMI/**

We are a team in the course **Real World Robotics (RWR)**, which works with the
[ORCA hand](https://orcahand.com). Our course task, **Task X**, is to build a
**Universal Manipulation Interface (UMI) for the ORCA hand v2**: a hand-held device that
records joint angles, tactile data and the global and relative pose while a person does a
manipulation task, so the ORCA hand can learn it.

We develop **our own ROS 2 packages for the UMI system**. The ORCA hand v2 model (URDF and
meshes, MIT license) is our starting point: we copy the files we need into our own
package `rwrumi_description`, and build everything else ourselves.

The instructions in this README are for the collaborators of Team 4.

> **Status:** development environment ready (ROS 2 Humble via RoboStack). Next: our own
> description package `rwrumi_description` (based on ORCA v2) and the first ROS packages.
> The repository layout below is the *planned* structure; each entry shows whether it
> already exists.

---

## System overview

```mermaid
flowchart LR
    subgraph UMI["RWRUMI hand-held device"]
        J[Joint sensors]
        T[Tactile sensors]
    end
    C["Camera / tracker<br/>(TBD)"]

    J -->|/umi/joint_states| R
    T -->|/umi/tactile| R
    C --> P[Pose estimation<br/>global + relative]
    P -->|/tf, /umi/pose| R

    D["rwrumi_description<br/>our URDF + meshes<br/>(based on ORCA v2)"] --> S[robot_state_publisher]
    J --> S
    S -->|/tf| R

    R[(Recorder<br/>rosbag2 / MCAP)] --> E[Export<br/>HDF5 dataset]
    E --> L[Policy learning<br/>transfer to ORCA hand]
```

| Subsystem | What it does | Owner | Status |
|---|---|---|---|
| Robot description | Our own URDF + meshes of the UMI, derived from ORCA v2, used by every other part | Team 4 | planned |
| Joint sensing | Reads our joint sensors and publishes `sensor_msgs/JointState` | Team 4 | planned |
| Tactile sensing | Reads our tactile sensors and publishes a custom tactile message | Team 4 | planned |
| Pose estimation | Global pose of the UMI and relative UMI ↔ object pose | Team 4 | planned (camera/tracker TBD) |
| Recording | Synced recording of all streams to rosbag2 (MCAP) | Team 4 | planned |
| Dataset export | Converts bags into per-episode HDF5 for training | Team 4 | planned |

---

## Repository layout

```text
RWRUMI/
├── README.md                     project overview (this file)
├── docs/                         GitHub Pages project site
│   └── index.html
├── environment.yml               environment: ROS 2 Humble + all dependencies (RoboStack)
├── .gitignore
└── src/                          ROS 2 workspace sources                         [planned]
    ├── rwrumi_description/       our robot model, based on ORCA v2
    │   ├── urdf/                 UMI URDF/xacro (ORCA hand + our parts, clean names)
    │   ├── meshes/               STL meshes copied from ORCA (+ our own)
    │   ├── LICENSE.orca          ORCA's MIT license for the copied files
    │   └── SOURCE.md             origin of the copied files + all our changes
    ├── rwrumi_interfaces/        custom messages (e.g. TactileArray.msg)
    ├── rwrumi_joint_sensors/     joint sensor driver node
    ├── rwrumi_tactile/           tactile sensor driver node
    ├── rwrumi_pose/              global + relative pose estimation
    ├── rwrumi_recorder/          recording + bag → HDF5 export
    └── rwrumi_bringup/           launch files, configs, RViz setup
```

**Naming conventions (proposed):** packages are prefixed `rwrumi_`; all UMI topics live
under the `/umi` namespace; frames follow [REP 105](https://www.ros.org/reps/rep-0105.html)
(`world` → `umi_base` → hand links from `rwrumi_description`). Files from other packages
are always referenced by package name (`package://…`, `FindPackageShare`), never by path.

---

## Tech stack

| | |
|---|---|
| OS | Ubuntu 26.04 LTS |
| Middleware | ROS 2 **Humble**, installed through [RoboStack](https://robostack.github.io) (conda packages), not apt |
| Environment | `mamba` (or `micromamba`) with one shared `environment.yml` (ROS, Python and system libraries) |
| Language | Python 3.12 (`rclpy`) |
| Robot model | own package `rwrumi_description`, based on ORCA v2 right hand (MIT) |
| Recording | `rosbag2` with the MCAP storage plugin |
| Dataset | HDF5 (`h5py`) |
| Pose estimation | TBD (depends on the camera/tracker choice) |

---

## Getting started

No system ROS install is needed. ROS 2 comes inside a conda environment from
[RoboStack](https://robostack.github.io).

```bash
# 1. Install mamba (once per machine): Miniforge or micromamba
#    https://github.com/conda-forge/miniforge  (micromamba users: replace `mamba` with `micromamba`)

# 2. Clone
git clone https://github.com/erhell/RWRUMI.git
cd RWRUMI

# 3. Create the environment (ROS 2 Humble + all dependencies)
mamba env create -f environment.yml
mamba activate rwrumi
export ROS_DOMAIN_ID=4             # only talk to our team's ROS nodes on the lab network

# 4. Build and use (from the repo root)
colcon build --symlink-install
source install/setup.zsh           # or install/setup.bash, depending on your shell
```

### Keeping everyone's environment identical (no Docker)

| Layer | How it is pinned | How to update after `git pull` |
|---|---|---|
| ROS 2, Python and system libraries | `environment.yml` (versions pinned) | `mamba env update -f environment.yml --prune` |
| Robot model (incl. ORCA-based files) | committed in `src/rwrumi_description` | nothing extra, it comes with `git pull` |

To add a dependency, add it to `environment.yml` (conda-forge / RoboStack name, or under a
`pip:` section if it only exists on PyPI), run the update command and commit the file.
Don't `mamba install` or `pip install` packages by hand, and don't install ROS from
apt as well, so the two installs never get mixed up.

---

## Open decisions

- [ ] Camera / tracker for global pose estimation
- [ ] Method for relative UMI ↔ object pose
- [ ] Joint sensor hardware and interface (serial, I²C, …)
- [ ] Tactile sensor hardware and message format
- [ ] Dataset format details (episode structure, sampling rate)
- [ ] License for our own code

---

## Contributing

- Changes to `main` go through pull requests.
- Only invited collaborators can push; anyone can clone or fork.
- Update this README (and `docs/index.html`) in the same PR whenever you add,
  move or remove a package, or settle one of the open decisions.
- When you copy or change anything that came from ORCA, update
  `src/rwrumi_description/SOURCE.md` in the same PR.

## Acknowledgements

- [ORCA hand](https://orcahand.com): RWRUMI's robot model is based on ORCA's open-source
  [`orcahand_description`](https://github.com/orcahand/orcahand_description) (MIT, © 2025 ORCA).
- Chi et al., *Universal Manipulation Interface*, RSS 2024 — the original UMI concept.
