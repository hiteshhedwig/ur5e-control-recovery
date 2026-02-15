"""Bounded, observed joint execution on the existing ROS trajectory action.

One active command per supervisor. The supervisor owns admission and liveness;
the upstream controller owns interpolation, motor control, and cancellation motion.
"""
from concurrent.futures import Future
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
CONTROLLER = "eff_joint_traj_controller"

@dataclass(frozen=True)
class ExecutionResult:
    command_id: str
    status: str
    reason: str
    # This means the action server acknowledged cancellation, not physical safety.
    cancel_acknowledged: bool = False

class JointObserver:
    def __init__(self):
        self.lock = threading.Lock()
        self.sample = None
        # Explicit test seam at the feedback-consumer boundary, never a physics mock.
        self.test_drop = False
        self.test_offset = 0.0
        self.subscription = rospy.Subscriber("/joint_states", JointState, self._receive, queue_size=20)

    def _receive(self, msg):
        with self.lock:
            if self.test_drop:
                return
            indexes = {name: i for i, name in enumerate(msg.name)}
            if not all(name in indexes for name in JOINTS):
                return
            if len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
                return
            q = [msg.position[indexes[j]] + self.test_offset for j in JOINTS]
            v = [msg.velocity[indexes[j]] for j in JOINTS]
            if not all(math.isfinite(x) for x in q + v):
                return
            stamp = msg.header.stamp.to_sec()
            if self.sample is not None and stamp <= self.sample["sim"]:
                return  # Old, duplicate, or reset-clock samples require a new session.
            self.sample = {"q": q, "v": v, "sim": stamp, "wall": time.monotonic()}

    def snapshot(self):
        with self.lock:
            return None if self.sample is None else dict(self.sample)

    def close(self):
        self.subscription.unregister()

class ExecutionSupervisor:
    def __init__(self, log_path, feedback_timeout=0.5):
        self.lock = threading.RLock()
        self.log_lock = threading.Lock()
        self.observer = JointObserver()
        self.feedback_timeout = feedback_timeout
        self.state = "IDLE"
        self.active = None
        self.closed = False
        self.worker = None
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        # Keep previous sessions; command IDs distinguish runs.
        self.log = self.log_path.open("a", buffering=1)
        model = ET.fromstring(rospy.get_param("/robot_description"))
        self.limits = {}
        for joint in model.findall("joint"):
            if joint.attrib.get("name") in JOINTS:
                limit = joint.find("limit")
                self.limits[joint.attrib["name"]] = (
                    float(limit.attrib["lower"]), float(limit.attrib["upper"]),
                    float(limit.attrib["velocity"]))
        if len(self.limits) != len(JOINTS):
            raise RuntimeError("Robot description lacks expected joint limits")
        self._event("SESSION", joint_names=JOINTS, limits=self.limits)

    def _event(self, event, **fields):
        record = {"event": event, "wall_monotonic": time.monotonic(),
                  "sim_time": rospy.Time.now().to_sec(), **fields}
        with self.log_lock:
            if not self.log.closed:
                self.log.write(json.dumps(record, allow_nan=False) + "\n")

    @staticmethod
    def _bounded_call(function, timeout):
        result = {}
        ready = threading.Event()
        def call():
            try:
                result["value"] = function()
            except Exception as exc:
                result["error"] = exc
            finally:
                ready.set()
        threading.Thread(target=call, daemon=True).start()
        if not ready.wait(timeout):
            raise RuntimeError("ROS interface wall timeout")
        if "error" in result:
            raise RuntimeError(str(result["error"]))
        return result["value"]

    def _controller_running(self):
        def query():
            proxy = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
            try:
                return proxy()
            finally:
                proxy.close()
        result = self._bounded_call(query, 1.0)
        return any(c.name == CONTROLLER and c.state == "running" for c in result.controller)

    def wait_ready(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sample = self.observer.snapshot()
            if sample and time.monotonic() - sample["wall"] <= self.feedback_timeout:
                if self._controller_running():
                    return
            time.sleep(0.02)
        raise RuntimeError("Robot readiness timeout")

    def submit(self, target, duration=2.0, wall_timeout=12.0, issued_at=None, replace=False):
        now = time.monotonic()
        issued_at = now if issued_at is None else issued_at
        if not math.isfinite(issued_at) or issued_at > now or now - issued_at > 0.5:
            raise ValueError("Command is stale or has an invalid issue time")
        target = list(target)
        if len(target) != len(JOINTS) or not all(math.isfinite(x) for x in target):
            raise ValueError("Expected six finite joint positions in radians")
        if not math.isfinite(duration) or duration < 0.5 or not math.isfinite(wall_timeout) or wall_timeout <= 0:
            raise ValueError("Invalid execution duration or wall timeout")
        for joint, value in zip(JOINTS, target):
            low, high, _ = self.limits[joint]
            if not low <= value <= high:
                raise ValueError("Joint limit exceeded: " + joint)
        # No queue. Replacement waits for the old worker's terminal result before
        # allocating a new action client; late callbacks cannot finish the new goal.
        with self.lock:
            if self.closed:
                raise RuntimeError("Supervisor is closed")
            active = self.active
            if self.state == "FAULT":
                raise RuntimeError("Explicit recovery required")
            if active and not replace:
                raise RuntimeError("A command is already active")
        if active:
            self.cancel(reason="REPLACED")
            active["future"].result(timeout=2)
        with self.lock:
            if self.state == "FAULT":
                raise RuntimeError("Replacement failed; explicit recovery required")
            if self.active is not None:
                raise RuntimeError("Another caller submitted a command")
            sample = self.observer.snapshot()
            if not sample or time.monotonic() - sample["wall"] > self.feedback_timeout:
                raise RuntimeError("Fresh joint feedback required")
            # Reject clearly infeasible point-to-point timing; the controller still
            # owns the actual interpolation. A zero-endpoint cubic has peak 1.5*d/T.
            for joint, q, desired in zip(JOINTS, sample["q"], target):
                if 1.5 * abs(desired-q) / duration > 0.8 * self.limits[joint][2]:
                    raise ValueError("Duration exceeds conservative joint velocity envelope")
            future = Future()
            command = {"id": uuid.uuid4().hex, "target": target, "duration": duration,
                       "wall_timeout": wall_timeout, "future": future,
                       "cancel": threading.Event(), "cancel_reason": "CANCELLED"}
            self.active = command
            self.state = "EXECUTING"
            self._event("COMMAND", command_id=command["id"], target=target,
                        duration=duration, wall_timeout=wall_timeout)
            self.worker = threading.Thread(target=self._execute, args=(command,), daemon=True)
            self.worker.start()
            return future

    def cancel(self, reason="CANCELLED"):
        with self.lock:
            if self.active:
                self.active["cancel_reason"] = reason
                self.active["cancel"].set()

    def _execute(self, command):
        client = None
        done = threading.Event()
        outcome = {}
        sent = False
        terminal = None
        start = time.monotonic()
        settled_since = None
        def request_cancel():
            try:
                client.cancel_goal()
                return done.wait(0.75) and outcome.get("status") in (
                    GoalStatus.PREEMPTED, GoalStatus.RECALLED)
            except Exception:
                return False  # Loss of transport never proves the robot stopped.
        try:
            client = actionlib.SimpleActionClient("/" + CONTROLLER + "/follow_joint_trajectory",
                                                 FollowJointTrajectoryAction)
            if not self._controller_running():
                raise RuntimeError("CONTROLLER_NOT_RUNNING")
            connected = self._bounded_call(lambda: client.wait_for_server(), 2.0)
            if not connected:
                raise RuntimeError("ACTION_SERVER_UNAVAILABLE")
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = list(JOINTS)
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=command["target"], velocities=[0.0]*6,
                time_from_start=rospy.Duration(command["duration"]))]
            def completed(status, result):
                outcome.update(status=status, error_code=result.error_code if result else None,
                               error_string=result.error_string if result else "missing result")
                done.set()
            last_feedback_logged = [0.0]
            def feedback(message):
                now = time.monotonic()
                if now-last_feedback_logged[0] >= 0.1:
                    last_feedback_logged[0] = now
                    self._event("CONTROLLER_FEEDBACK", command_id=command["id"],
                                stamp=message.header.stamp.to_sec(),
                                desired=list(message.desired.positions),
                                actual=list(message.actual.positions),
                                error=list(message.error.positions))
            client.send_goal(goal, done_cb=completed, feedback_cb=feedback)
            sent = True
            self._event("SENT", command_id=command["id"])
            while not rospy.is_shutdown():
                now = time.monotonic()
                sample = self.observer.snapshot()
                if command["cancel"].is_set() and not done.is_set():
                    acknowledged = request_cancel()
                    if not acknowledged:
                        raise RuntimeError("CANCEL_UNCONFIRMED")
                    terminal = ExecutionResult(command["id"], "CANCELLED", command["cancel_reason"], True)
                    break
                if sample is None or now - sample["wall"] > self.feedback_timeout:
                    raise RuntimeError("STALE_FEEDBACK")
                if now - start > command["wall_timeout"]:
                    raise RuntimeError("EXECUTION_WALL_TIMEOUT")
                if done.is_set():
                    if outcome.get("status") != GoalStatus.SUCCEEDED or outcome.get("error_code") != 0:
                        raise RuntimeError("CONTROLLER_EXECUTION_ERROR: " + str(outcome))
                    error = max(abs(a-b) for a,b in zip(sample["q"], command["target"]))
                    quiet = max(abs(v) for v in sample["v"]) <= 0.05
                    if error <= 0.03 and quiet:
                        if settled_since is None:
                            settled_since = sample["sim"]
                        if sample["sim"] - settled_since >= 0.25:
                            terminal = ExecutionResult(command["id"], "SUCCEEDED", "MEASURED_TARGET_REACHED")
                            break
                    else:
                        settled_since = None
                    # The server can finish before our subscription catches up.
                    if "finished_wall" not in outcome:
                        outcome["finished_wall"] = now
                    if now - outcome["finished_wall"] > 1.0:
                        raise RuntimeError("EXECUTION_MISMATCH")
                time.sleep(0.02)
            if terminal is None:
                raise RuntimeError("ROS_SHUTDOWN")
        except Exception as exc:
            acknowledged = False
            if sent and not done.is_set():
                acknowledged = request_cancel()
            terminal = ExecutionResult(command["id"], "FAULT", str(exc), acknowledged)
        finally:
            if client is not None:
                client.stop_tracking_goal()
                client.action_client.stop()
            self._event("TERMINAL", command_id=command["id"], status=terminal.status,
                        reason=terminal.reason, cancel_acknowledged=terminal.cancel_acknowledged,
                        controller_result=outcome, observed=self.observer.snapshot())
            with self.lock:
                if self.active is command:
                    self.active = None
                    self.state = "FAULT" if terminal.status == "FAULT" else "IDLE"
                command["future"].set_result(terminal)

    def recover(self):
        with self.lock:
            if self.active:
                raise RuntimeError("Cannot recover while execution is active")
            if self.closed:
                raise RuntimeError("Supervisor is closed")
            if not self._controller_running():
                raise RuntimeError("Recovery requires a running controller")
            self.wait_ready(timeout=3)
            # Require fresh, low-velocity state; never replay a stored command.
            deadline = time.monotonic() + 3
            stable_since = None
            while time.monotonic() < deadline:
                sample = self.observer.snapshot()
                if (sample and time.monotonic()-sample["wall"] <= self.feedback_timeout
                        and max(abs(v) for v in sample["v"]) <= 0.05):
                    if stable_since is None:
                        stable_since = sample["sim"]
                    if sample["sim"] - stable_since >= 0.25:
                        self.state = "IDLE"
                        self._event("RECOVERED", observed=sample)
                        return
                else:
                    stable_since = None
                time.sleep(0.02)
            raise RuntimeError("Recovery requires settled fresh feedback")

    def close(self):
        with self.lock:
            self.closed = True
        self.cancel()
        if self.worker:
            self.worker.join(timeout=5)
            if self.worker.is_alive():
                raise RuntimeError("Execution worker did not shut down")
        self.observer.close()
        with self.log_lock:
            self.log.close()
