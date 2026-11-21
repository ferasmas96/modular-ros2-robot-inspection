# Bounded operator input for SLAM-004C

This is a manual experiment utility, not a motion planner or SLAM component.
It never generates a route or a nonzero command without fresh operator input.
Run it directly using the installed ROS Python dependencies; no package install
or normal workspace build is required.

`manual_guard.py` subscribes to `/a300_00000/manual/raw_cmd_vel`, clamps X/yaw to
**±0.10 m/s / ±0.10 rad/s**, zeros the other four components, stamps with the
advancing simulation clock, and publishes at 20 Hz using a steady wall timer.
`control_limits.py` contains the pure logic. `test_control_limits.py` tests caps,
nonfinite rejection, gate transitions, input freshness, clock rewind/stall and
the manual-window limit. No test publishes ROS movement commands.

Two outputs use the **same bounded tuple**:

- `/a300_00000/manual/limited_preview`: bounded outgoing messages for disconnected
  user-keyboard verification. It does not feed the mux/controller.
- `/a300_00000/cmd_vel`: exactly zero while disarmed; after authorized arming, the
  same bounded preview tuple. The existing mux/controller path is retained.

The service `/a300_00000/manual/arm` (`std_srvs/srv/SetBool`) defaults false.
Arming requires exclusive graph publishers, an advancing clock, no fault, and
released (zero) input. It discards previous input; the operator must create a
fresh event after arming. No configuration parameter raises the hard caps.
Input older than **0.35 wall seconds** becomes zero. A clock stall over **0.50 s**,
clock rewind, invalid input or expired **120 s wall** manual window latches a
disarmed fault. Do not automatically rearm or retry after a fault.

Before the bridge/guard are used, stop only the current simulation's task-owned
`cmd_vel_bridge`, `teleop_twist_joy_node` and `twist_server_node` processes. Their
launch definitions do not respawn them. This prevents bypass and higher-priority
inputs without modifying protected files, mux parameters, controller, EKF or TF.
The raw replacement bridge publishes on the isolated input topic. Arming refuses
any other publisher on the robot mux input, any unexpected controller-input/raw
publisher, or any publisher on the three other mux input topics. An independent
observer must also inspect **actual mux/controller commands** throughout driving.
Unexpected publishers/commands require disarm and abort; graph checks are not a
security boundary against a hostile ROS participant.

Use the existing Gazebo Teleop Keyboard panel, topic `/a300_00000/cmd_vel`.
Even if GUI settings remain at their historical 1.0/0.5 values, the robot path is
clamped. Keyboard auto-repeat is required for continuous input; the initial
repeat delay or losing keyboard focus can cause the freshness stop. Release keys
to stop; confirm wheel velocity before continuing. Do not use latched Buttons or
Sliders for sustained motion with this freshness rule.

Before motion approval, ask the user to exercise forward and left keys **only
while disarmed**. Measure raw input, actual preview publications, zero robot
commands and stationary wheels. Nonzero robot-path commands are checked during
the subsequently authorized manual drive, not inferred from intended GUI values.

```bash
source /opt/ros/jazzy/setup.bash
python3 -B -m unittest discover -s tools/manual_drive_guard -p 'test_*.py' -v
# Only after simulation/input approval and replacement of bypass inputs:
python3 -B tools/manual_drive_guard/manual_guard.py
```

Safety fallback: disarm first and verify zero output/wheel stop. If another
publisher wins the mux or the guard dies with motion, immediately stop the
identified task-owned simulation. Do not rely on controller timeouts with a
stalled clock. No automatic motion or recovery sequence is authorized.
