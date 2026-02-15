#!/usr/bin/python3
"""Start upstream simulation, verify motion, and stop only our process group."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/milestone1.json")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--regression", action="store_true", help="Run execution and fault regressions after baseline")
    parser.add_argument("--demo", action="store_true", help="Run the narrated execution/recovery demonstration")
    parser.add_argument("--record-clips", action="store_true", help="Record and annotate three real Gazebo GUI demonstrations")
    parser.add_argument("--keep-running", action="store_true", help="Keep simulation open after verification; Ctrl-C stops it")
    args = parser.parse_args()
    # Refuse to reuse or terminate another simulation using these ports.
    for port in (11321, 11355):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
    log_path = ROOT / ".runtime/launch.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        simulation = subprocess.Popen(
            ["roslaunch", str(ROOT / "launch/simulation.launch"), "gui:=" + str(args.gui or args.record_clips).lower()],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            socket.setdefaulttimeout(1)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if simulation.poll() is not None:
                    raise RuntimeError("Simulation launcher exited; inspect " + str(log_path))
                try:
                    master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
                    if master.getPid("milestone1_runner")[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                time.sleep(0.1)
            else:
                raise RuntimeError("ROS master startup timeout; inspect " + str(log_path))
            subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/initialize_simulation.py")],
                           check=True, timeout=65)
            subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/verify_motion.py"),
                            "--output", str(ROOT / args.output)], check=True, timeout=100)
            if args.regression:
                subprocess.run(["/usr/bin/python3", str(ROOT / "tests/integration.py")],
                               check=True, timeout=180)
            if args.demo:
                subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/demo.py")],
                               check=True, timeout=90)
            if args.record_clips:
                subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/record_clips.py")],
                               check=True, timeout=240)
            if args.keep_running:
                print("Verification passed. Simulation remains open; Ctrl-C stops it.", flush=True)
                try:
                    simulation.wait()
                except KeyboardInterrupt:
                    pass
        finally:
            if simulation.poll() is None:
                os.killpg(simulation.pid, signal.SIGINT)
                try:
                    simulation.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(simulation.pid, signal.SIGKILL)
                    simulation.wait()

if __name__ == "__main__":
    main()
