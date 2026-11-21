# SLAM-003D preparation

Static preparation only. No runtime authorization is implied by these files.

- [Runtime procedure and exact proposed commands](../../../docs/experiments/slam003d_b_procedure.md).
- `plan.json`: route, limits, reservation identity and authorization gates.
- `recording_qos.yaml`: recorder subscriptions only; leaves all publishers and GLIM QoS unchanged.
- `ground_truth_bridge.yaml`: one-way existing Gazebo pose stream to a dedicated recording topic, never `/tf` or `/tf_static`.
- [Portable mapping snapshot](../../glim/mode_a_cpu_mapping_moving/): 15 JSON files, byte-identical to the SLAM-003A portable CPU mapping snapshot. Operational copies differ only in `logging.log_dir`.

Reproduce in a fresh run: copy the portable snapshot to a fresh private configuration directory; replace only `logging.log_dir` with a new absolute `<run>/logs/glim` path. Reserve `<run>/logs/ros`, use explicit absolute `config_path`/`dump_path`, and refuse existing bag/dump/log filenames. Record source and destination hashes and the semantic logging-only delta. Do not launch the portable copy directly: relative `logs/glim` resolves against process working directory.

The prepared operational copy is `mode_a_cpu_mapping_slam003d_001`, reserved for `mode_a_mapping/slam003d_001` outside Git. The private `preparation_manifest.json` records its full paths, 15 source/destination hashes, 187 protected hashes, 22 SLAM-003B artifact hashes and 41 native-reload build/result hashes. No raw logs, private networking values or runtime artifacts belong in this directory.
