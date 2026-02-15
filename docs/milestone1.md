# Milestone 1 — Existing simulated robot, observable execution

## Objective and boundary

Run a real simulated UR5e through existing ROS control interfaces. Demonstrate measured
motion and an execution result before building an execution supervisor.

## Reused components

- `ur_gazebo` / `ur_description` 1.5.0 Noetic binary distribution.
- `joint_trajectory_controller` / `effort_controllers` 0.22.0.
- System Gazebo 11.15.1, `gazebo_ros_control`, `controller_manager`, ROS messages/actionlib.
- The upstream UR5e URDF/xacro, inertial properties, limits, and controller gains are unchanged.

Project changes are a small launch composition, startup sequencer, dependency overlay,
empty-world physics configuration, and integration smoke checker. No upstream files are patched.

## Evidence from the investigation

| Experiment | Observed result | Artifact |
| --- | --- | --- |
| Upstream concurrent startup, effort interface, default timestep | Arm left intended pose; first command aborted with wrist path error | `upstream-effort-baseline-failure.json` |
| Concurrent startup, position interface | Nearly constant joint positions with nonzero reported shoulder velocity; goal aborted | `upstream-position-baseline-failure.json` |
| Concurrent startup, effort interface, 100 us step | Smaller timestep alone did not fix startup; path error | `small-step-concurrent-start-failure.json` |
| Sequenced startup, effort interface, 100 us step | Three moves passed | `small-step-sequenced-start-pass.json` |
| Sequenced startup, effort interface, 1 ms step | Goal tolerance failure persisted | `default-step-sequenced-start-failure.json` |
| Sequenced startup, effort interface, 100 us step, repeated | Three moves passed | `sequenced-start-repeat.json` |

The final `milestone1.json` adds direct Gazebo joint queries to the successful setup.
Failed experiments retain their original verifier configuration and do not represent
the final acceptance run.

## Startup sequencing

The upstream bringup starts controller spawners concurrently with robot spawning.
The installed `spawn_model` implementation applies initial joint configuration after
spawning, including a one-second wall sleep workaround. Logs showed controllers starting
before that configuration was applied. This permits the controller's initial hold state
and the later physical pose to disagree.

We instead:

1. Start physics paused.
2. Run the existing spawn utility with initial joint positions and wait for it to exit successfully.
3. Load the existing joint-state and effort trajectory controllers.
4. Queue controller activation and unpause physics so an update cycle can complete the switch.
5. Require the expected measured initial pose before sending motion commands.

The switch service runs in a thread because it requires a physics update; blocking on it
while physics remains paused would deadlock initialization. Initialization is externally
bounded by the runner. The thread is not a general asynchronous execution manager.

## Numerical integration choice

The default 1 ms physics step still failed with sequenced startup. A 100 us step with
the same original effort gains passed repeated runs. We retain that smaller step as
an empirically verified configuration. These experiments establish a working numerical
configuration, not a complete stability analysis of the controller or physics solver.

`real_time_update_rate=10000` is a requested simulation pacing rate, not a measured
deadline guarantee. Physics time and wall time are recorded separately.

## Why the position-interface fallback was rejected

The upstream DefaultRobotHWSim position mode directly sets joint positions while reading
velocities from Gazebo. The measured trace had almost unchanged shoulder position with
reported velocity around 0.069 rad/s, above the controller's 0.05 rad/s stop threshold.
We retained the failure and did not relax that threshold to obtain a pass.

Source inspected:
https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp

## Acceptance and scope

The final checker requires action success, fresh finite measured position/velocity,
settling, actual displacement, advancing physics time, and direct Gazebo confirmation.
The Gazebo query uses a separate API but shares the same simulator; it is not an
independent physical sensor or proof of real-world correctness.

This milestone does not yet implement recovery, feedback corruption injection,
replacement races, collision avoidance, Cartesian planning, or physical safety.
Those boundaries keep the baseline inspectable and reusable.
