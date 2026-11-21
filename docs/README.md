# Project Documentation Guide

This documentation records verifiable development and inspection of the thesis project. Keep observations separate from assumptions, and do not describe an unrun test as successful.

## Documentation files

- `01_environment.md`: operating system, hardware, software versions, and environment setup.
- `02_husky_simulation.md`: Husky A300 simulation launch and operation.
- `03_ros2_communication.md`: ROS 2 nodes, topics, message flow, and commands.
- `04_odometry_tf.md`: odometry and TF frames and their verification.
- `05_ros_gz_integration.md`: ROS 2 and Gazebo integration.
- `06_rviz_visualization.md`: RViz configuration and visualized data.
- `07_sensor_integration.md`: sensor setup, data interfaces, and checks.
- `08_slam_integration.md`: SLAM implementations and their integration.
- `09_switching_manager.md`: SLAM monitoring and switching manager design and behavior.
- `10_navigation.md`: Nav2 setup and navigation behavior.
- `11_evaluation.md`: evaluation methods, metrics, and results.
- `decisions.md`: engineering decisions and their rationale.
- `development_log.md`: chronological project progress.

## Evidence storage

- Save reusable maps, plots and diagrams by task in `../evidence/figures/<TASK-ID>/`; start with the [figure index](../evidence/figures/README.md) and [task figure template](../evidence/figures/TEMPLATE.md).
- Save screenshots in `../evidence/screenshots/`.
- Save logs in `../evidence/logs/`.
- Save concise command output captures in `../evidence/command_outputs/`.
- Put reusable configuration snapshots in `../configs/` and experiment descriptions or small result artifacts in `../experiments/`.

Use descriptive filenames that include a date or task identifier where helpful. Reference evidence from a document with a relative Markdown link, for example `[node list](../evidence/command_outputs/2026-11-12-ros-nodes.txt)`. Add evidence only when it exists; do not create placeholder links.

## Updating documentation during development

For each future development task, update the relevant topic document with the actual configuration, procedure, results, verification status, issues, evidence, and reproducibility details. Add a dated entry to `development_log.md` after the task is complete. Add to `decisions.md` only when a decision is actually made. Documentation should describe the code and configuration that exist; it should not imply that planned work has been implemented.

When a task produces or supplies diagrams, archive the actual exports in its figure folder, preserve older versions, record captions/provenance/frames/units and update the central figure index. Keep editable sources when available. Figures produced in another ChatGPT conversation must be downloaded or located before they can be archived; do not record a chat preview as a saved file. Existing legacy paths remain in place and may be indexed rather than moved. A task with no figures does not need placeholder images.

## Recording test outcomes

In each topic document's Verification section, label each check `PASS`, `FAIL`, or `NOT TESTED`. Record what was tested and the observed outcome. Include the command, relevant conditions, and evidence link when available. Record failures and their causes or unresolved status rather than omitting them; change a failure to a resolved result only after repeating the check.

## From development records to thesis writing

Use the topic documents as traceable records of system design, implementation, and validation. The thesis-writing notes should identify findings that may support a thesis chapter, but the thesis should synthesize and critically discuss verified evidence rather than copy unverified plans or claims.

## Keep generated data out of Git

Do not store large rosbag recordings, generated build/install output, or other reproducible bulk artifacts in Git. Keep them in an appropriate external or managed data location, and document their location, provenance, and reproduction procedure. Commit only small, relevant evidence or configuration snapshots when project policy permits; do not change ignore rules as part of routine documentation updates.
