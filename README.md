# RWRUMI

**Team 4 · [Real World Robotics (RWR)](https://rwr.ethz.ch/) · Task X: UMI for the ORCA hand v2**
Project page: **https://erhell.github.io/RWRUMI/**

This README is the working guide for Team 4: how the repository is structured, how to set
up the environment, and how we add code and files.

> **Status:** environment ready (ROS 2 Humble). Next: `rwrumi_description` and the first packages.

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

Our machines: Ubuntu 26.04. The environment uses Python 3.12.

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
| `rwrumi_joint_sensors` | joint sensor driver | planned |
| `rwrumi_tactile` | tactile sensor driver | planned |
| `rwrumi_pose` | pose estimation | planned (camera/tracker TBD) |
| `rwrumi_recorder` | recording + export | planned |
| `rwrumi_bringup` | launch files, configs | planned |

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

## Open decisions

- [ ] Camera / tracker for pose estimation
- [ ] Joint sensor hardware and interface
- [ ] Tactile sensor hardware and message format
- [ ] Dataset format (episode structure, sampling rate)

## Links

- Course: [Real World Robotics](https://rwr.ethz.ch/)
- ORCA hand: [orcahand.com](https://orcahand.com) ·
  [`orcahand_description`](https://github.com/orcahand/orcahand_description) (MIT)
- [RoboStack](https://robostack.github.io) · [ROS 2 Humble docs](https://docs.ros.org/en/humble/)
