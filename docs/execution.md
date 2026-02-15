# Execution architecture and interview guide

## Data path

```text
Scenario / Python caller
  -> ExecutionSupervisor.submit(joint_positions, duration)
  -> actionlib FollowJointTrajectory
  -> upstream effort JointTrajectoryController
  -> gazebo_ros_control EffortJointInterface
  -> Gazebo / ODE UR5e joints
  -> joint_states + controller action feedback/result
  -> measured completion or latched fault
```

The project owns admission, liveness, command identity, measured-completion policy,
recovery policy, and evidence collection. It does not implement a servo loop, physics,
trajectory interpolation, inverse kinematics, or robot middleware.

`submit` returns a standard-library Future. Only one command executes at a time.
Positions use the fixed documented six-joint ordering in radians. Limits are loaded
from the upstream robot description. Non-finite values, wrong dimensions, stale local
issue timestamps, limit violations, and clearly infeasible durations are rejected.
The duration check is a conservative endpoint velocity-envelope check, not collision
checking or a complete acceleration/jerk feasibility proof.

Minimal use in an initialized simulation:

```python
import rospy
from robot_control.execution import ExecutionSupervisor

rospy.init_node("my_scenario")
supervisor = ExecutionSupervisor("artifacts/my-events.jsonl")
try:
    supervisor.wait_ready()
    target = supervisor.observer.snapshot()["q"]
    target[0] += 0.12
    result = supervisor.submit(target, duration=2.0).result(timeout=15)
    print(result)
finally:
    supervisor.close()
```

## State and concurrency

- `IDLE -> EXECUTING` after validated admission.
- Execution ends in `SUCCEEDED`, acknowledged `CANCELLED`, or `FAULT`.
- Success/cancellation return the supervisor to `IDLE`; faults remain latched.
- New commands during a fault are rejected until explicit `recover()` succeeds.
- Replacement requests cancellation, waits for the old worker's terminal result,
  then starts a new action client with a distinct command ID.
- Each worker's callbacks close over its own command/outcome. Old callbacks cannot
  complete a replacement Future. Action subscriptions are stopped at worker exit.
- An already-successful action racing with cancellation proceeds through measured
  verification; cancellation is not falsely claimed after completion.

No command queue, automatic replay, autonomous retry, or general-purpose transport
layer is introduced. The caller owns high-level intent.

## Timing and feedback

The upstream controller schedules trajectory points in simulation time. Completion
requires a 0.25 simulation-second interval within 0.03 rad and 0.05 rad/s.

Communication freshness (0.5 seconds), execution deadlines, connection waits, and
cancellation waits use monotonic wall time. A paused simulator therefore cannot
freeze the watchdog. Callbacks reject non-finite, malformed, out-of-order, and
duplicate-stamp joint samples. A simulator clock reset requires a fresh supervisor
session; automatic cross-epoch recovery is intentionally unsupported.

Controller feedback is logged at a bounded rate, including desired/actual/error
positions. It does not replace the separate joint-state completion check.
The baseline and normal regression additionally query Gazebo joint properties.

## Fault policy

| Condition | Response | Recovery requirement |
| --- | --- | --- |
| Stale command | Reject before dispatch | Submit fresh intent |
| Feedback delivery stops / simulation pauses | Latch `STALE_FEEDBACK`; request cancellation | Restore advancing state, running controller, and settled feedback |
| Controller is stopped or returns failure | Latch controller fault | Restore controller, then explicit recovery |
| Controller reports success but observed target is wrong | Latch `EXECUTION_MISMATCH` after catch-up allowance | Investigate observation/execution inconsistency, then explicit recovery |
| Wall deadline expires | Latch `EXECUTION_WALL_TIMEOUT`; request cancellation | Fresh settled state and explicit recovery |
| Cancellation cannot be confirmed | Latch `CANCEL_UNCONFIRMED` | Establish robot/controller state; never assume a stop |

Cancellation acknowledgement means an action-protocol acknowledgement. It is not proof
of zero velocity, certified safety, or a hardware stop. Physical deceleration belongs
to the upstream controller. Recovery additionally checks low measured velocity.

Recovery cannot prove that an arbitrary sensor corruption is repaired; the mismatch
regression explicitly removes its injection before recovering. No automatic trust
restoration or physical safety claim is made.

## Verification scope

`tests/integration.py` runs on the actual simulated robot. Pausing physics and switching
the controller use real Gazebo/controller-manager services. Feedback drop/offset are
explicit test-only seams in the supervisor's feedback consumer. They exercise loss and
inconsistency handling without claiming to emulate every network or sensor failure.

The harness tests a successful fresh command after feedback-loss recovery. Each test
restores feedback, unpauses physics, starts the controller, and checks readiness/settling
before the next scenario. It does not reset the entire world between scenarios, so the
ordering is fixed and documented; a fresh world is used on each suite invocation.

## Milestone acceptance mapping

| Milestone | Deliverable | Verification | Dependencies / complexity / AIM relevance |
| --- | --- | --- | --- |
| 1: simulation | Pinned overlay, launch, startup sequencing, smoke checker | Three observed moves with direct Gazebo queries; repeated startup | Existing Noetic/Gazebo; medium; integration diagnosis |
| 2: control integration | Reusable action client, joint/units/limit validation, result records | Normal execution and rejected malformed/stale inputs | M1; low-medium; robot API integration |
| 3: execution management | Futures, one active command, cancellation/replacement, separate clocks | Cancellation/replacement and wall deadline regressions | M2; medium; asynchronous execution and timing |
| 4: fault handling | Latched faults, bounded cancellation, explicit recovery | Feedback loss, paused physics, controller deactivation, mismatch | M3; medium; failure diagnosis and honest stop semantics |
| 5: regression verification | Real-simulator unittest suite and machine-readable results | Eight passing scenarios through the runner | M4; medium; reproducible evaluation |
| 6: demonstration/docs | Narrated demo, setup guide, architecture and decision notes | Demo move -> replace -> fault -> recover -> fresh move | M5; low-medium; explainable evidence and maintainability |

## Interview discussion

Be ready to explain why a successful API response is insufficient, how controller
startup can disagree with initial robot state, why wall-clock watchdogs matter in
paused simulation, and what cancellation acknowledgement does and does not prove.
Walk through one command UUID in the event log and connect intent, desired/actual
feedback, terminal result, and observed joints. Use the saved failed baseline traces
to explain the investigation instead of claiming the first example worked unchanged.

## Remaining limitations

- Simulation only: no hardware driver, fieldbus, electrical faults, real sensor noise,
  calibrated dynamics, certified stop, or hardware-in-the-loop validation.
- No collision-aware planner, task-space goals, perception, AI policy, or VLA model.
- No hard real-time performance guarantee, comprehensive stress testing, or coverage
  of arbitrary external clients competing for the same action server.
- ROS 1 Noetic/Gazebo Classic are legacy pinned dependencies, selected to reuse the
  available environment. Migration to ROS 2 is a separate future exercise.
- This is a small single-process integration prototype, not a production robot SDK.
