#!/usr/bin/python3
"""Record actual Gazebo window pixels; subtitles come from live execution events.

Requires a working X11 Gazebo GUI, ffmpeg, xdotool, and xwininfo. No generated
robot animation is used. Raw window capture and timestamped evidence are retained.
"""
from dataclasses import asdict
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import rospy
from robot_control.execution import ExecutionSupervisor

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/demo-clips"
RAW = ROOT / ".runtime/recordings"

def command(*args):
    return subprocess.check_output(args, text=True).strip()

def ass_time(seconds):
    centis = max(0, round(seconds*100))
    return "{}:{:02d}:{:02d}.{:02d}".format(centis//360000, centis//6000%60, centis//100%60, centis%100)

class Clip:
    def __init__(self, supervisor, window, width, height, slug, title, subtitle):
        self.s = supervisor
        self.slug, self.title, self.subtitle = slug, title, subtitle
        self.path = RAW / (slug + ".mp4")
        self.stages, self.samples, self.results = [], [], []
        self.running = True
        self.target = None
        self.start = time.monotonic()
        # Capture only this application's window; never the desktop or other apps.
        self.capture = subprocess.Popen([
            "ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab", "-framerate", "30",
            "-draw_mouse", "0", "-window_id", str(window), "-video_size", "{}x{}".format(width,height),
            "-i", os.environ.get("DISPLAY", ":0"), "-vf",
            "crop={}:{}:270:74".format((width-292)//2*2, (height-134)//2*2),
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "18", "-pix_fmt", "yuv420p",
            str(self.path)], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=(RAW / (slug + "-capture.log")).open("w"))
        self.thread = threading.Thread(target=self.observe, daemon=True)
        self.thread.start()
        time.sleep(0.8)
        if self.capture.poll() is not None:
            raise RuntimeError("Window recording failed: " + slug)

    def observe(self):
        while self.running:
            sample = self.s.observer.snapshot()
            if sample:
                now = time.monotonic()
                self.samples.append({"t": now-self.start, "state": self.s.state,
                                     "sample": sample, "feedback_age": now-sample["wall"],
                                     "target": self.target})
            time.sleep(0.12)

    def stage(self, text):
        print(self.slug + ": " + text, flush=True)
        self.stages.append({"t": time.monotonic()-self.start, "text": text})

    def move(self, target, duration=4):
        self.target = list(target)
        return self.s.submit(target, duration=duration, wall_timeout=20)

    def expect(self, future, status):
        result = future.result(timeout=23)
        self.results.append(asdict(result))
        if result.status != status:
            raise RuntimeError("Unexpected recorded outcome: " + str(result))
        return result

    def finish(self, passed=True):
        self.running = False
        self.thread.join(timeout=2)
        if self.capture.poll() is None:
            self.capture.stdin.write(b"q\n")
            self.capture.stdin.flush()
        self.capture.wait(timeout=15)
        if self.capture.returncode:
            raise RuntimeError("Capture encoding failed")
        duration = float(command("ffprobe", "-v", "error", "-show_entries", "format=duration",
                                 "-of", "default=nw=1:nk=1", str(self.path)))
        evidence = {"passed": passed, "title": self.title, "slug": self.slug,
                    "capture": "Actual Gazebo X11 window, continuous wall-time capture at 30 fps",
                    "overlay": "Post-produced from live event timestamps; no motion retiming",
                    "duration_seconds": duration, "stages": self.stages,
                    "results": self.results, "samples": self.samples}
        (OUTPUT / (self.slug + ".json")).write_text(json.dumps(evidence,indent=2)+"\n")
        if not passed:
            return
        ass = self.subtitles(duration)
        target = OUTPUT / (self.slug + ".mp4")
        subprocess.run([
            "ffmpeg", "-y", "-loglevel", "error", "-i", str(self.path),
            "-vf", "scale=1280:696,pad=1280:900:0:80:color=0x101820,ass=" + str(ass),
            "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", "-an", str(target)], check=True, timeout=60)
        print("Saved " + str(target), flush=True)

    def subtitles(self, duration):
        header = """[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 900
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Title,DejaVu Sans,27,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,0,0,7,28,20,13,1
Style: Subtitle,DejaVu Sans,15,&H00BFD65A,&H00FFFFFF,&H00101820,&H00101820,0,0,0,0,100,100,0,0,1,0,0,7,28,20,51,1
Style: Stage,DejaVu Sans,22,&H00FFFFFF,&H00FFFFFF,&H00101820,&H00101820,-1,0,0,0,100,100,0,0,1,0,0,7,28,28,793,1
Style: Data,DejaVu Sans Mono,15,&H00CBD0D3,&H00FFFFFF,&H00101820,&H00101820,0,0,0,0,100,100,0,0,1,0,0,7,28,20,840,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
        lines = [header]
        def line(start,end,style,text):
            if end > start:
                lines.append("Dialogue: 0,{},{},{},,0,0,0,,{}\n".format(ass_time(start),ass_time(end),style,text))
        line(0,duration,"Title",self.title)
        line(0,duration,"Subtitle",self.subtitle + "  |  ACTUAL GAZEBO CAPTURE  |  SIMULATION ONLY")
        stages = [{"t": 0, "text": "Robot ready | existing ROS effort trajectory controller"}] + self.stages
        for i,stage in enumerate(stages):
            end = stages[i+1]["t"] if i+1 < len(stages) else duration
            line(stage["t"],end,"Stage",stage["text"])
        for i,sample in enumerate(self.samples):
            end = self.samples[i+1]["t"] if i+1<len(self.samples) else duration
            q = sample["sample"]["q"]
            text = "STATE {:<9} | sim {:6.2f}s | feedback age {:.2f}s".format(
                sample["state"], sample["sample"]["sim"], sample["feedback_age"])
            if sample["target"]:
                error = max(abs(a-b) for a,b in zip(q,sample["target"]))
                text += " | max joint error {:.4f} rad".format(error)
            text += "\\NJoint positions [rad]: " + "  ".join("{:+.2f}".format(x) for x in q)
            line(sample["t"],end,"Data",text)
        path = OUTPUT / (self.slug + ".ass")
        path.write_text("".join(lines))
        return path

def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    # Gazebo opens a splash first; wait for the actual visible main window.
    deadline = time.monotonic()+30
    window = None
    while time.monotonic() < deadline:
        result = subprocess.run(["xdotool", "search", "--onlyvisible", "--name", "^Gazebo$"],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            window = int(result.stdout.splitlines()[-1])
            break
        time.sleep(0.2)
    if window is None:
        raise RuntimeError("No visible Gazebo main window")
    subprocess.run(["xdotool", "windowactivate", "--sync", str(window)], check=True)
    geometry = command("xwininfo", "-id", str(window))
    width = int(re.search(r"Width: (\d+)",geometry)[1])
    height = int(re.search(r"Height: (\d+)",geometry)[1])
    if width < 1200 or height < 800:
        raise RuntimeError("Gazebo window too small for the verified recording layout")
    rospy.init_node("record_robot_clips", anonymous=True)
    supervisor = ExecutionSupervisor(OUTPUT / "execution-events.jsonl")
    supervisor.wait_ready()
    home = [0,-1.57,0,-1.57,0,0]
    pose_a = [0.35,-1.08,-0.72,-1.30,0.45,0]
    pose_b = [-0.40,-1.22,-0.35,-1.65,-0.30,0.20]
    current = None
    try:
        current = Clip(supervisor,window,width,height,"01-verified-motion",
                       "01 / Verified robot motion", "ROS Noetic + ros_control + UR5e")
        current.stage("Send joint target A through FollowJointTrajectory")
        current.expect(current.move(pose_a),"SUCCEEDED")
        current.stage("Target A verified from measured joint position and velocity")
        time.sleep(1.6)
        current.stage("Send target B | controller performs timed interpolation")
        current.expect(current.move(pose_b),"SUCCEEDED")
        current.stage("Target B verified | return to the initial pose")
        current.expect(current.move(home),"SUCCEEDED")
        current.stage("PASS | three measured motions; successful API calls alone are insufficient")
        time.sleep(2)
        current.finish()
        current = Clip(supervisor,window,width,height,"02-command-replacement",
                       "02 / Replace an executing command", "Asynchronous goals + explicit cancellation")
        current.stage("Goal A executing | six-second trajectory")
        first = current.move(pose_a,6)
        time.sleep(2.2)
        current.stage("New intent arrives | cancel A and submit a distinct goal B")
        current.target = pose_b
        second = supervisor.submit(pose_b,duration=4,wall_timeout=20,replace=True)
        current.expect(first,"CANCELLED")
        current.expect(second,"SUCCEEDED")
        current.stage("PASS | A acknowledged as replaced; B verified at its own target")
        time.sleep(2.2)
        current.finish()
        if supervisor.submit(home,duration=4,wall_timeout=20).result(timeout=23).status != "SUCCEEDED":
            raise RuntimeError("Cannot restore recording start pose")
        current = Clip(supervisor,window,width,height,"03-fault-and-recovery",
                       "03 / Detect a fault and recover", "Feedback loss injected at the supervisor consumer")
        current.stage("Robot executing | fresh joint-state feedback is arriving")
        pending = current.move(pose_a,6)
        time.sleep(1.8)
        current.stage("INJECT: stop feedback delivery to the supervisor; physics continues")
        with supervisor.observer.lock:
            supervisor.observer.test_drop = True
        fault = current.expect(pending,"FAULT")
        if fault.reason != "STALE_FEEDBACK":
            raise RuntimeError("Expected stale-feedback detection")
        current.stage("FAULT LATCHED | stale feedback detected; cancellation requested")
        time.sleep(2)
        try:
            supervisor.submit(pose_b)
        except RuntimeError:
            current.stage("New command rejected while fault is latched")
        else:
            raise RuntimeError("Fault did not block new motion")
        time.sleep(1.5)
        current.stage("Restore feedback | explicitly verify readiness and recover")
        with supervisor.observer.lock:
            supervisor.observer.test_drop = False
        supervisor.recover()
        current.stage("Recovered | accept fresh intent, without replaying the old goal")
        current.expect(current.move(pose_b,4),"SUCCEEDED")
        current.stage("PASS | fresh command reaches its measured target after recovery")
        time.sleep(2.5)
        current.finish()
        current = None
    except Exception:
        if current and current.running:
            current.finish(passed=False)
        raise
    finally:
        with supervisor.observer.lock:
            supervisor.observer.test_drop = False
        supervisor.close()
    subprocess.run(["/usr/bin/python3",str(ROOT / "scripts/finalize_clips.py")],check=True,timeout=90)

if __name__ == "__main__":
    main()
