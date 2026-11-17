# Headless native GLIM dump reload validator

This standalone validation utility consumes installed GLIM 1.2.2 APIs. It is not a ROS package or a SLAM component and defines no serialization format or file parser. `GlobalMapping::load()` invokes GLIM's own `SubMap::load()`/GTSAM Points loading and native GTSAM graph/value deserialization.

The program registers console-only loggers, selects the installed native `ISAM2ExtDummy` backend in memory, and never calls save, configuration dump, ROS initialization, viewers or async mapping. The native loader's `optimize` log label still appears, but a callback verifies the dummy backend before its non-optimizing update. Saved configuration files are not edited. Native errors and corruption/recovery diagnostics fail validation even if `load()` returns true.

Before constructing/loading GLIM objects, Linux Landlock ABI >= 3 is required. An additional ruleset denies write-open, truncation, file creation, removal and rename/link operations. A denied creation probe verifies enforcement. Logging/results use inherited stdout/stderr only. If enforcement fails, no dump is loaded. The utility does not modify file permissions or use a writable copy of the input dump. This is a Linux-specific validation tool, not a generic untrusted-code sandbox.

Callbacks inspect public native submaps, frames, point attributes, graph factors and values. Counts must match the three explicit expected-count arguments. Frame timestamps/poses, compact arrays, factor/value key consistency and final submap poses are checked. In-memory `export_points()` must agree with the expected point count. The four separately saved trajectory text files are not input to the native map loader; their hashes and previous SLAM-003B checks are recorded separately.

## Build and run separately from the ROS workspace

Use a new private build/result directory; do not use colcon or write into the original dump. No installation or package rebuild is needed. For the installed PPA package, use the intact CMake export directory `/opt/ros/jazzy/share/glim`: the alternative config under its `cmake/` subdirectory references a missing `glim-targets.cmake` file.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S /home/feras/clearpath_ws/tools/glim_dump_reload \
  -B /home/feras/glim_ws/validation/slam003c_b_001/build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dglim_DIR=/opt/ros/jazzy/share/glim
cmake --build /home/feras/glim_ws/validation/slam003c_b_001/build --parallel 2
```

The single SLAM-003C-B execution used the following command, with its working directory set to `/home/feras/glim_ws/validation/slam003c_b_001`, `OMP_NUM_THREADS=2`, and stdout/stderr exclusively captured to new private files by the parent process:

```bash
timeout --signal=TERM --kill-after=5s 30s \
  /home/feras/glim_ws/validation/slam003c_b_001/build/glim_dump_reload \
  /home/feras/glim_ws/runs/mode_a_mapping/slam003b_001/glim_dump \
  1 300 2648
```

Invocation: `glim_dump_reload DUMP EXPECTED_SUBMAPS EXPECTED_FRAMES EXPECTED_POINTS`. Reproduction requires a fresh output directory/log filenames, before/after SHA256 checks and an execution bound; do not overwrite the recorded validation results. CMake/compiler checks are build steps, not dump-reload runs.

## Recorded result

Exactly one bounded reload completed with exit 0: 1 submap, 300 frames, 2,648 loaded/exported points, 4 graph factors and 4 values. Native error count was zero and no corruption/recovery was detected. Three optional global-mapping parameters used their native defaults (six warning lines). All 22 original artifact hashes were unchanged.

See [sanitized evidence](../../evidence/command_outputs/2026-11-17-slam-003c-b-native-reload.txt). Build outputs and full validation captures remain outside Git. No live mapping, GUI, simulation, movement, accuracy test, commit or push is part of this utility.
