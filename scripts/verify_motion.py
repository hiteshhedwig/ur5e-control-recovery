#!/usr/bin/python3
"""Milestone 1 smoke check against real Gazebo joint feedback, not a mock."""
import argparse
import json
import math
import hashlib
from pathlib import Path
import threading
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetPhysicsProperties, GetJointProperties
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CONTROLLER = "eff_joint_traj_controller"

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    report = {"milestone": 1, "robot": "UR5e", "simulator": "Gazebo Classic",
              "passed": False, "moves": [], "trace": [],
              "criteria": {"position_error_rad": 0.03, "velocity_rad_s": 0.05,
                           "settle_sim_seconds": 0.25, "feedback_age_wall_seconds": 0.5}}
    lock = threading.Lock()
    latest = {}

    def observe(msg):
        indexes = {name: i for i, name in enumerate(msg.name)}
        if not all(name in indexes for name in JOINTS):
            return
        q = [msg.position[indexes[name]] for name in JOINTS]
        v = [msg.velocity[indexes[name]] for name in JOINTS]
        with lock:
            latest.update(q=q, v=v, sim=msg.header.stamp.to_sec(), wall=time.monotonic())
            if not report["trace"] or latest["sim"] - report["trace"][-1]["sim"] >= 0.02:
                report["trace"].append({"sim": latest["sim"], "q": q, "v": v})

    def snapshot():
        with lock:
            return dict(latest)

    def wait_until(predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(0.02)
        raise RuntimeError(message)

    try:
        rospy.init_node("milestone1_verify", anonymous=True)
        subscriber = rospy.Subscriber("/joint_states", JointState, observe, queue_size=100)
        wait_until(lambda: bool(snapshot()) and rospy.Time.now().to_sec() > 0, 30,
                   "No advancing simulator joint feedback")
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=10)
        controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)()
        report["controllers"] = [{"name": c.name, "state": c.state, "type": c.type}
                                 for c in controllers.controller]
        require(any(c.name == CONTROLLER and c.state == "running"
                    for c in controllers.controller), "Trajectory controller is not running")
        report["controller_config"] = rospy.get_param("/" + CONTROLLER)
        report["actuator_model"] = "Gazebo DefaultRobotHWSim effort interface"
        report["ros_distro"] = "noetic"
        report["use_sim_time"] = rospy.get_param("/use_sim_time", False)
        require(report["use_sim_time"], "Simulation clock is not enabled")
        physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)()
        require(physics.success, "Cannot read physics configuration")
        report["physics"] = {"time_step": physics.time_step,
                             "max_update_rate": physics.max_update_rate,
                             "gravity": [physics.gravity.x, physics.gravity.y, physics.gravity.z]}
        report["robot_description_sha256"] = hashlib.sha256(
            rospy.get_param("/robot_description").encode()).hexdigest()
        client = actionlib.SimpleActionClient("/" + CONTROLLER + "/follow_joint_trajectory",
                                             FollowJointTrajectoryAction)
        connected = threading.Event()
        def connect():
            if client.wait_for_server():
                connected.set()
        threading.Thread(target=connect, daemon=True).start()
        require(connected.wait(15), "Trajectory action server unavailable")
        # Let the upstream spawn_model initial-configuration workaround finish.
        initial_time = snapshot()["sim"]
        wait_until(lambda: snapshot()["sim"] - initial_time >= 2.0, 10, "Physics clock stalled")
        home = snapshot()["q"]
        require(max(abs(a-b) for a,b in zip(home, [0, -1.57, 0, -1.57, 0, 0])) <= 0.03,
                "Initial pose does not match configured upright pose")
        report["initial_q"] = home
        poses = [list(home), list(home), list(home)]
        poses[0][0] += 0.15
        poses[1][0] -= 0.10
        poses[1][4] += 0.12
        for index, target in enumerate(poses):
            before = snapshot()
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = JOINTS
            point = JointTrajectoryPoint()
            point.positions = target
            point.velocities = [0.0] * len(JOINTS)
            point.time_from_start = rospy.Duration(3.0)
            goal.trajectory.points = [point]
            done = threading.Event()
            outcome = {}
            def completed(status, result):
                outcome.update(status=status, error_code=result.error_code,
                               error_string=result.error_string)
                done.set()
            client.send_goal(goal, done_cb=completed)
            require(done.wait(15), "Action result wall timeout")
            require(outcome["status"] == GoalStatus.SUCCEEDED and outcome["error_code"] == 0,
                    "Controller did not succeed: " + str(outcome))
            settled_since = None
            def settled():
                nonlocal settled_since
                sample = snapshot()
                fresh = time.monotonic() - sample["wall"] < 0.5
                finite = all(math.isfinite(x) for x in sample["q"] + sample["v"])
                error = max(abs(a - b) for a, b in zip(sample["q"], target))
                quiet = max(abs(v) for v in sample["v"]) <= 0.05
                if fresh and finite and error <= 0.03 and quiet:
                    if settled_since is None:
                        settled_since = sample["sim"]
                    return sample["sim"] - settled_since >= 0.25
                settled_since = None
                return False
            wait_until(settled, 5, "Fresh measured joints did not settle at target")
            after = snapshot()
            gazebo_joint = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
            direct_positions = []
            for name in JOINTS:
                properties = gazebo_joint("robot::" + name)
                require(properties.success and bool(properties.position),
                        "Cannot observe simulated joint directly: " + name)
                direct_positions.append(properties.position[0])
            require(max(abs(a-b) for a,b in zip(direct_positions, target)) <= 0.03,
                    "Gazebo joint positions do not confirm target execution")
            measured_motion = max(abs(a - b) for a, b in zip(after["q"], before["q"]))
            require(measured_motion > 0.05, "Action succeeded without expected measured movement")
            require(after["sim"] > before["sim"], "Physics time did not advance")
            move = {"index": index, "target": target, "before": before, "after": after,
                    "gazebo_joint_positions": direct_positions,
                    "result": outcome, "max_error_rad": max(abs(a-b) for a,b in zip(after["q"], target)),
                    "max_measured_motion_rad": measured_motion}
            report["moves"].append(move)
            print("Move {} verified: max error {:.6f} rad".format(index + 1, move["max_error_rad"]), flush=True)
        report["passed"] = True
    except Exception as exc:
        report["error"] = str(exc)
        raise
    finally:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with lock:
            output.write_text(json.dumps(report, indent=2) + "\n")
        rospy.signal_shutdown("Milestone 1 verification finished")

if __name__ == "__main__":
    main()
