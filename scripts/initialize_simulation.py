#!/usr/bin/python3
"""Sequence upstream spawn and controller APIs without a startup pose race."""
import subprocess
import threading
import time
import rospy
from controller_manager_msgs.srv import LoadController, SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty

def main():
    subprocess.run(["rosrun", "gazebo_ros", "spawn_model", "-urdf", "-param", "robot_description",
                    "-model", "robot", "-z", "0.1", "-J", "shoulder_pan_joint", "0",
                    "-J", "shoulder_lift_joint", "-1.57", "-J", "elbow_joint", "0",
                    "-J", "wrist_1_joint", "-1.57", "-J", "wrist_2_joint", "0",
                    "-J", "wrist_3_joint", "0"], check=True, timeout=35)
    rospy.init_node("initialize_simulation", anonymous=True)
    rospy.wait_for_service("/controller_manager/load_controller", timeout=15)
    load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
    names = ["joint_state_controller", "eff_joint_traj_controller"]
    for name in names:
        if not load(name).ok:
            raise RuntimeError("Cannot load controller " + name)
    request = SwitchControllerRequest()
    request.start_controllers = names
    request.strictness = SwitchControllerRequest.STRICT
    result = {}
    def start():
        try:
            result["ok"] = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)(request).ok
        except Exception as exc:
            result["error"] = str(exc)
    thread = threading.Thread(target=start, daemon=True)
    thread.start()
    # Controller switching requires an update cycle; unpause after queuing it.
    time.sleep(0.1)
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=10)
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    thread.join(timeout=10)
    if not result.get("ok"):
        raise RuntimeError("Controller startup failed: " + str(result))
    print("Spawned paused, configured joints, loaded and started upstream effort controllers", flush=True)

if __name__ == "__main__":
    main()
