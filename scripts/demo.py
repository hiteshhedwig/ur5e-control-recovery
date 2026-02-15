#!/usr/bin/python3
"""Narrated simulation-only demonstration using the reusable execution client."""
from dataclasses import asdict
import json
from pathlib import Path
import time

import rospy
from robot_control.execution import ExecutionSupervisor

ROOT = Path(__file__).resolve().parents[1]

def main():
    rospy.init_node("execution_demo", anonymous=True)
    supervisor = ExecutionSupervisor(ROOT / "artifacts/demo-events.jsonl")
    steps = []
    passed = False
    def show(label, future, expected):
        result = future.result(timeout=15)
        steps.append({"step": label, **asdict(result)})
        print("{}: {} ({})".format(label, result.status, result.reason), flush=True)
        if result.status != expected:
            raise RuntimeError("Unexpected demonstration result: " + str(result))
    try:
        supervisor.wait_ready()
        home = supervisor.observer.snapshot()["q"]
        def pose(offset):
            q = list(home)
            q[0] += offset
            return q
        show("1. Verified move", supervisor.submit(pose(0.12)), "SUCCEEDED")
        old = supervisor.submit(pose(0.25), duration=4)
        time.sleep(0.4)
        new = supervisor.submit(pose(-0.12), replace=True)
        show("2. Replaced command", old, "CANCELLED")
        show("3. Replacement reaches target", new, "SUCCEEDED")
        fault = supervisor.submit(pose(0.2), duration=4)
        time.sleep(0.4)
        with supervisor.observer.lock:
            supervisor.observer.test_drop = True
        show("4. Feedback delivery interrupted at consumer", fault, "FAULT")
        with supervisor.observer.lock:
            supervisor.observer.test_drop = False
        supervisor.recover()
        print("5. Feedback restored; explicit recovery completed", flush=True)
        show("6. Fresh command after recovery", supervisor.submit(home), "SUCCEEDED")
        passed = True
    finally:
        supervisor.close()
        (ROOT / "artifacts/demo-results.json").write_text(
            json.dumps({"passed": passed, "steps": steps}, indent=2)+"\n")

if __name__ == "__main__":
    main()
