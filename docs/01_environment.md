# 01. Environment

### 1. Objective
Establish a read-only, reproducible baseline of the WSL2 host, ROS 2, Gazebo, Clearpath packages, development tools, and graphics environment before inspecting simulation interfaces.

### 2. System Configuration
- **Inspection date:** 2026-11-12.
- **Operating system:** Ubuntu 24.04.5 LTS (Noble), x86_64.
- **WSL:** Linux kernel `6.18.58.1-microsoft-standard-WSL2`. `wsl.exe --status` reported default WSL version 2; `wsl.exe --list --verbose` showed `Ubuntu-24.04` running as version 2.
- **CPU and resources:** AMD Ryzen 7 7800X3D; 16 logical CPUs available to WSL. `free -h` reported 7.3 GiB RAM and 2.0 GiB swap. Root filesystem: 1007 GiB total, 948 GiB available.
- **GPU:** NVIDIA GeForce RTX 4060, driver `610.88`, 8188 MiB reported memory.
- **ROS 2:** Jazzy is installed under `/opt/ros/jazzy`; `ros2` is `/opt/ros/jazzy/bin/ros2`. The workspace overlay is `/home/feras/clearpath_ws/install`. ROS setup files were not referenced by the checked shell startup files. In the inspection shell, `ROS_DISTRO`, `ROS_DOMAIN_ID`, and `RMW_IMPLEMENTATION` were unset. Sourcing the Jazzy setup and workspace overlay in a temporary subshell set `ROS_DISTRO=jazzy`; the other two variables remained unset.
- **Gazebo:** After sourcing ROS, `gz` resolves to `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`; `gz sim --version` reports Gazebo Sim `8.15.0` (Harmonic series). The standalone `gz-harmonic` Debian metapackage was not found, but the ROS Gazebo tool and simulation vendor packages are installed. No OSRF Gazebo APT source was present in the inspected source files.
- **ROS–Gazebo packages:** `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-image`, `ros-jazzy-ros-gz-interfaces`, and `ros-jazzy-ros-gz-sim` are version `1.0.24`; `ros-jazzy-gz-sim-vendor` is `0.0.13`.
- **Clearpath packages:** Installed Jazzy packages and versions: `clearpath-bt-joy` `2.9.17`, `clearpath-common` `2.9.17`, `clearpath-config` `2.9.8`, `clearpath-control` `2.9.17`, `clearpath-description` `2.9.17`, `clearpath-diagnostics` `2.9.17`, `clearpath-generator-common` `2.9.17`, `clearpath-generator-gz` `2.9.4`, `clearpath-gz` `2.9.4`, `clearpath-manipulators` `2.9.17`, `clearpath-manipulators-description` `2.9.17`, `clearpath-mounts-description` `2.9.17`, `clearpath-platform-description` `2.9.17`, `clearpath-platform-msgs` `2.7.0`, `clearpath-sensors-description` `2.9.17`, `clearpath-simulator` `2.9.4`, and `clearpath-viz` `2.9.1`. The sourced workspace overlay resolves `clearpath_description` from the workspace install tree; the other checked package prefixes resolve under `/opt/ros/jazzy`.
- **RViz2 and development tools:** `ros-jazzy-rviz2` `14.1.24` is installed. `colcon-core` `0.21.3`, `rosdep` `0.27.0`, Git `2.43.0`, and Python `3.12.3` are available. GitHub CLI (`gh`) and `glxinfo` were not available on the current PATH.
- **WSLg:** `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, `XDG_RUNTIME_DIR=/run/user/1000`, and `WSL2_GUI_APPS_ENABLED=1`; `/dev/dxg` and `/dev/dri` devices exist. `nvidia-smi` sees the RTX 4060. A prior RViz log reports OpenGL 4.5 and a prior Gazebo log records loading `gz-rendering-ogre2`; these are historical log observations, not tests run in this inspection. Hardware-backed OpenGL rendering was not independently measured because `glxinfo` is unavailable.
- **APT sources:** Ubuntu Noble, Noble updates/backports/security, and `packages.ros.org/ros2/ubuntu` (Noble) were present. No APT refresh or package operation was run.
- **Git repositories:** The project root was at commit `857d10833f1e8a86b1b446d460081d7e695cc1be` on `main` tracking `origin/main` before these documentation changes. The four clean nested repositories were on branch `jazzy`:
  - `clearpath_common` — `ca416eaf752e1db86fb1c725122dc5a33c758170`, `https://github.com/clearpathrobotics/clearpath_common.git`
  - `clearpath_config` — `5b861f8e5b8d9f12ffcc795f79a7e74f348b9e97`, `https://github.com/clearpathrobotics/clearpath_config.git`
  - `clearpath_generator_tests` — `de9a45058a402b500ffba6db27674de6f3ee7ea0`, `https://github.com/clearpathrobotics/clearpath_generator_tests.git`
  - `clearpath_msgs` — `0014d0afafea2715d1336ff58f2562450b6debda`, `https://github.com/clearpathrobotics/clearpath_msgs.git`

### 3. Implementation or Inspection Procedure
Read OS and WSL information, CPU/memory/disk statistics, relevant environment variables, package-manager metadata, APT source fields, tool versions, GPU/WSLg device information, existing graphics-log excerpts, and nested-repository branch/commit/remote metadata. Sourced `/opt/ros/jazzy/setup.bash` and `/home/feras/clearpath_ws/install/setup.bash` only inside temporary subshells to inspect ROS packages and run `ros2 --help` and `gz sim --version`. Nested remote URLs were sanitized before recording.

No simulation, ROS node, RViz application, graphics benchmark, package operation, source-repository operation, or persistent environment/configuration change was performed.

### 4. Results
- **PASS:** Ubuntu 24.04.5, WSL2, architecture, CPU, RAM, and disk information were read successfully.
- **PASS:** ROS 2 Jazzy setup works when sourced temporarily; the ROS CLI responded and the sourced overlay exposed 447 ROS package names.
- **PASS:** `gz sim --version` reported Gazebo Sim 8.15.0 when the ROS setup was sourced. The command was used only to print the version; Gazebo was not launched.
- **PASS:** Clearpath and ROS–Gazebo packages, the four nested repository commits/remotes, installed tools, APT sources, NVIDIA visibility, and WSLg environment were inspected.
- **NOT VERIFIED:** Current GPU-backed OpenGL renderer selection. WSLg devices and NVIDIA visibility are present, and historical RViz/Gazebo logs show OpenGL 4.5/OGRE2, but `glxinfo` is unavailable and no graphics probe was run.
- **NOT TESTED:** Simulation launch, ROS nodes/topics, robot interfaces, RViz launch, and sensor data. The previously reported successful Husky keyboard-control session was not reproduced during this baseline inspection.

### 5. Verification
- **PASS** — `ros2 --help` and `ros2 pkg list` in a temporary Jazzy + workspace-overlay shell.
- **PASS** — `gz sim --version` returned Gazebo Sim 8.15.0 without launching a simulation.
- **PASS** — `nvidia-smi` identified the RTX 4060; WSLg environment variables and device nodes were present.
- **PASS** — Four nested source repositories were on `jazzy` with clean worktrees; their remotes contained no embedded credentials in the recorded output.
- **NOT TESTED** — ROS runtime communication, Husky simulation behavior, RViz in this task, and hardware renderer selection.

### 6. Problems and Solutions
- ROS environment variables and ROS/Gazebo commands were absent from the initial inspection shell. No ROS setup references were found in the checked shell startup files. A temporary `source /opt/ros/jazzy/setup.bash` (followed by the workspace overlay) exposed the installed commands; no persistent fix was made.
- `gz-harmonic` was not listed as an installed Debian metapackage and the initial unsourced shell could not find `gz`. After sourcing ROS, the `gz` executable supplied by `ros-jazzy-gz-tools-vendor` reported version 8.15.0. This inspection found no evidence of a conflicting Gazebo installation.
- Existing logs contain Gazebo warnings about unsupported Ogre material scripts and ignored Ogre2 visibility-mask bits. Their impact was not investigated; no related fatal rendering error appeared in the sampled excerpts.
- `glxinfo` and `gh` were unavailable. Neither was installed as part of this task.

### 7. Evidence
- [System and WSL output](../evidence/command_outputs/2026-11-12-system-wsl.txt)
- [ROS, Gazebo, and package output](../evidence/command_outputs/2026-11-12-ros-gazebo.txt)
- [Tools, GPU, WSLg, graphics logs, and APT output](../evidence/command_outputs/2026-11-12-tools-gpu-apt.txt)
- [Sanitized Clearpath repository metadata](../evidence/command_outputs/2026-11-12-clearpath-repositories.txt)

### 8. Reproducibility
From WSL, run:

```bash
cat /etc/os-release
uname -a
uname -m
nproc
free -h
df -h /
wsl.exe --status
wsl.exe --list --verbose
printenv ROS_DISTRO
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 --help
ros2 pkg list
gz sim --version
nvidia-smi
```

The two `source` commands are session-local; do not add them to shell startup files based on this inspection alone. On Windows, the WSL version can also be inspected with `wsl --status` and `wsl --list --verbose` in PowerShell. These commands do not start the Husky simulation.

### 9. Thesis Writing Notes
This baseline can support the thesis implementation/environment chapter. It establishes the WSL2, Ubuntu, ROS 2 Jazzy, Gazebo Sim 8.15.0, Clearpath, and GPU environment, while distinguishing installed packages from tests actually run. Note the constrained WSL memory allocation (7.3 GiB reported) and that GPU-backed rendering was not directly verified. The reported earlier Husky keyboard-control result remains a user-reported observation for later independent verification.

Official references:
- [ROS 2 Jazzy installation on Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Harmonic installation on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [Clearpath Husky A300 documentation](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/)
- [Microsoft: Run Linux GUI apps with WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
- [NVIDIA CUDA on WSL User Guide](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
- [Clearpath source repositories](https://github.com/clearpathrobotics)
