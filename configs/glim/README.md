# SLAM-003A portable configuration review

The current `mode_a_cpu_mapping/` snapshot is the portable copy prepared for version control. It contains only the 15 JSON configuration files; no private filesystem paths, run manifest, logs or runtime artifacts are included. Files remain unstaged/untracked for review; no commit or push was made.

Relative to validated Mode A, exactly three values change:

| File / key | Mode A | Portable mapping snapshot |
|---|---|---|
| config_ros.json / glim_ros.enable_local_mapping | false | true |
| config_ros.json / glim_ros.enable_global_mapping | false | true |
| config_logging.json / logging.log_dir | /tmp | logs/glim |

The other 13 files are byte-identical. CPU modules, mapping defaults, sensor topics, QoS, timestamps, frames and exact extrinsic are unchanged. Extensions remain empty and publish_imu2lidar remains false. Inactive example configs are retained; active references select CPU files.

The logging directory is relative to the process working directory, not the config directory. This snapshot is a portable record, not a replacement for the separately prepared private operational configuration. For a future authorized run, use a fresh private copy with a unique absolute module logging path and explicit ROS_LOG_DIR/dump_path. Do not launch this copy directly from the thesis repository.

The private operational configuration and its run manifest remain unchanged. The previous snapshot with an absolute logging path was preserved intact outside Git. Earlier SLAM-003A documentation/evidence described byte equality before this portability correction; current repository/operational copies differ only in logging.log_dir. See the supplemental static review evidence for exact locations, preservation and differences.

No existing file contents were overwritten. No SLAM-003B action was performed; runtime remains pending user review and authorization.
