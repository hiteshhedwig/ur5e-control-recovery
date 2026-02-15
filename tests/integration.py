#!/usr/bin/python3
"""Integration regressions: every motion runs in real Gazebo through ros_control."""
from dataclasses import asdict
import json
from pathlib import Path
import time
import unittest
import sys

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from gazebo_msgs.srv import GetJointProperties
import rospy
from std_srvs.srv import Empty

from robot_control.execution import ExecutionSupervisor, JOINTS, CONTROLLER

ROOT = Path(__file__).resolve().parents[1]

class RobotIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("robot_control_regression", anonymous=True)
        cls.supervisor = ExecutionSupervisor(ROOT / "artifacts/execution-events.jsonl")
        cls.supervisor.wait_ready()
        # Baseline startup has settled in the runner's Milestone 1 check.
        cls.home = cls.supervisor.observer.snapshot()["q"]
        cls.results = []

    @classmethod
    def tearDownClass(cls):
        cls.supervisor.close()
        (ROOT / "artifacts/regression-results.json").write_text(json.dumps(cls.results, indent=2)+"\n")

    def setUp(self):
        self.s = self.supervisor

    def tearDown(self):
        with self.s.observer.lock:
            self.s.observer.test_drop = False
            self.s.observer.test_offset = 0.0
        self.service("/gazebo/unpause_physics")
        self.set_controller(True)
        if self.s.active:
            future = self.s.active["future"]
            self.s.cancel()
            future.result(timeout=4)
        self.s.recover()

    @staticmethod
    def service(name):
        return ExecutionSupervisor._bounded_call(lambda: rospy.ServiceProxy(name, Empty)(), 2)

    @staticmethod
    def set_controller(running):
        request = SwitchControllerRequest()
        request.strictness = SwitchControllerRequest.BEST_EFFORT
        request.start_controllers = [CONTROLLER] if running else []
        request.stop_controllers = [] if running else [CONTROLLER]
        response = ExecutionSupervisor._bounded_call(
            lambda: rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)(request), 2)
        if not response.ok:
            raise RuntimeError("Test could not switch controller")

    def target(self, offset=0.12):
        q = list(self.home)
        q[0] += offset
        return q

    def record(self, result):
        self.results.append({"test": self.id(), **asdict(result)})
        return result

    def wait_sent(self, timeout=3):
        # Synchronize injection with actual goal publication, not an arbitrary sleep.
        deadline = time.monotonic()+timeout
        command_id = self.s.active["id"]
        while time.monotonic() < deadline:
            events = [json.loads(line) for line in self.s.log_path.read_text().splitlines()]
            if any(e["event"] == "SENT" and e.get("command_id") == command_id for e in events):
                return
            time.sleep(0.01)
        self.fail("Command was not sent before injection")

    def test_01_normal_execution_and_gazebo_confirmation(self):
        target = self.target()
        result = self.record(self.s.submit(target).result(timeout=15))
        self.assertEqual(result.status, "SUCCEEDED")
        query = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
        for name, expected in zip(JOINTS, target):
            actual = query("robot::"+name)
            self.assertTrue(actual.success)
            self.assertLess(abs(actual.position[0]-expected), 0.03)

    def test_02_invalid_and_stale_commands_are_not_sent(self):
        before = len(self.s.log_path.read_text().splitlines())
        for target in ([0]*5, [float("nan")]*6, [100]*6):
            with self.assertRaises(ValueError):
                self.s.submit(target)
        with self.assertRaises(ValueError):
            self.s.submit(self.target(), issued_at=time.monotonic()-1)
        self.assertIsNone(self.s.active)
        self.assertEqual(len(self.s.log_path.read_text().splitlines()), before)
        self.results.append({"test": self.id(), "status": "PASSED", "rejections": 4})

    def test_03_cancel_and_replace(self):
        old = self.s.submit(self.target(0.22), duration=4)
        self.wait_sent()
        replacement = self.s.submit(self.target(-0.12), replace=True)
        replaced = self.record(old.result(timeout=3))
        self.assertEqual((replaced.status, replaced.reason), ("CANCELLED", "REPLACED"))
        self.assertTrue(replaced.cancel_acknowledged)
        completed = self.record(replacement.result(timeout=15))
        self.assertEqual(completed.status, "SUCCEEDED")
        self.assertNotEqual(replaced.command_id, completed.command_id)
        pending = self.s.submit(self.target(0.22), duration=4)
        self.wait_sent()
        self.s.cancel()
        cancelled = self.record(pending.result(timeout=3))
        self.assertEqual(cancelled.status, "CANCELLED")

    def test_04_feedback_loss_latches_until_recovery(self):
        future = self.s.submit(self.target(0.2), duration=4)
        self.wait_sent()
        injected = time.monotonic()
        with self.s.observer.lock:
            self.s.observer.test_drop = True
        result = self.record(future.result(timeout=3))
        self.assertEqual((result.status, result.reason), ("FAULT", "STALE_FEEDBACK"))
        self.assertLess(time.monotonic()-injected, 2)
        with self.assertRaises(RuntimeError):
            self.s.submit(self.target())
        with self.s.observer.lock:
            self.s.observer.test_drop = False
        self.s.recover()
        self.assertEqual(self.record(self.s.submit(self.target()).result(timeout=15)).status, "SUCCEEDED")

    def test_05_paused_physics_detected_by_wall_clock(self):
        future = self.s.submit(self.target(-0.2), duration=4)
        self.wait_sent()
        self.service("/gazebo/pause_physics")
        injected = time.monotonic()
        result = self.record(future.result(timeout=3))
        self.assertEqual((result.status, result.reason), ("FAULT", "STALE_FEEDBACK"))
        self.assertLess(time.monotonic()-injected, 2)
        self.service("/gazebo/unpause_physics")

    def test_06_controller_deactivation(self):
        future = self.s.submit(self.target(0.2), duration=4)
        self.wait_sent()
        self.set_controller(False)
        result = self.record(future.result(timeout=4))
        self.assertEqual(result.status, "FAULT")
        self.assertIn("CONTROLLER_EXECUTION_ERROR", result.reason)
        with self.assertRaises(RuntimeError):
            self.s.recover()

    def test_07_success_result_with_mismatched_observation(self):
        future = self.s.submit(self.target())
        self.wait_sent()
        with self.s.observer.lock:
            self.s.observer.test_offset = 0.2
        result = self.record(future.result(timeout=15))
        self.assertEqual((result.status, result.reason), ("FAULT", "EXECUTION_MISMATCH"))

    def test_08_wall_deadline(self):
        future = self.s.submit(self.target(-0.2), duration=4, wall_timeout=0.4)
        result = self.record(future.result(timeout=3))
        self.assertEqual((result.status, result.reason), ("FAULT", "EXECUTION_WALL_TIMEOUT"))

if __name__ == "__main__":
    started = time.monotonic()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RobotIntegration)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    summary = {"passed": result.wasSuccessful(), "tests_run": result.testsRun,
               "wall_seconds": time.monotonic()-started,
               "failures": [{"test": case.id(), "traceback": detail} for case,detail in result.failures],
               "errors": [{"test": case.id(), "traceback": detail} for case,detail in result.errors]}
    (ROOT / "artifacts/regression-summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    sys.exit(0 if result.wasSuccessful() else 1)
