#!/usr/bin/python3
"""Make slower instructional replays from the tracked, real-time recordings.

No new simulation results are claimed: source footage is slowed to 0.5x and
explicitly held for explanations. Source clocks and feedback age freeze too.
"""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import textwrap

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts/demo-clips"
WORK = ROOT / ".runtime/explained-clips"

def run(args):
    subprocess.run(args, check=True, stdout=subprocess.DEVNULL, timeout=180)

def ts(seconds):
    n = max(0, round(seconds*100))
    return "{}:{:02d}:{:02d}.{:02d}".format(n//360000, n//6000%60, n//100%60, n%100)

def wrap(text, width=38):
    return r"\N".join(textwrap.wrap(text, width=width))

def cards(slug, stages):
    # Each card is tied to a recorded event, not a newly invented motion.
    def event(i, hold, heading, body, evidence):
        return (stages[i]["t"]+0.16, hold, heading, body, evidence)
    if slug.startswith("01"):
        return [
            (0,6,"What are we testing?", "Can a Python command move all six simulated joints to a requested pose, and can we verify that it actually arrived?", "Watch the elbow bend, the base turn, and the wrist change orientation."),
            event(0,4,"1 / Request pose A", "Send six joint angles through the existing ROS trajectory action. The controller interpolates a four-second move in simulation time.", "A = [+0.35, -1.08, -0.72, -1.30, +0.45, 0.00] rad"),
            event(1,5,"2 / Verify arrival at A", "The action reports success. The supervisor also checks fresh measured position and velocity over a settling interval before declaring success.", "Required: error <= 0.03 rad, speed <= 0.05 rad/s, settled for 0.25 simulated seconds."),
            event(2,4,"3 / Request pose B", "Now send a different configuration: turn the base in the opposite direction, reduce the elbow bend, and change wrist orientation.", "B = [-0.40, -1.22, -0.35, -1.65, -0.30, +0.20] rad"),
            event(3,4,"4 / Return to home", "B has passed the same measured checks. Send the near-upright home pose and verify the return motion too.", "Home = [0.00, -1.57, 0.00, -1.57, 0.00, 0.00] rad"),
            event(4,6,"Result / Three verified moves", "A, B, and home each completed successfully. This is joint-space control: there is no object, grasp, or Cartesian planning task in this clip.", "Evidence: three SUCCEEDED results, each with reason MEASURED_TARGET_REACHED."),
        ]
    if slug.startswith("02"):
        return [
            (0,7,"What are we testing?", "What happens when a new command arrives before the current motion has finished? B must replace A, rather than wait in a queue.", "Watch for the change of direction before the arm reaches A."),
            event(0,4,"1 / A is still executing", "Start toward pose A with a six-second trajectory. The program waits only 2.2 wall seconds before requesting a replacement.", "A is unfinished when the replacement request arrives."),
            event(1,8,"2 / Cancel A; start B", "Request cancellation and wait for the controller's acknowledgement. Mark A as CANCELLED / REPLACED, then send B with a distinct command ID.", "Acknowledgement is a protocol event. It does not mean the joints instantly reached zero velocity."),
            event(2,7,"Result / B owns the outcome", "The arm has reached B, not A. B is declared successful only after its own measured settling checks. The old goal cannot complete the new Future.", "Recorded outcomes: A = CANCELLED (REPLACED); B = SUCCEEDED."),
        ]
    return [
        (0,7,"What are we testing?", "If joint-state feedback disappears during motion, does the supervisor detect it, block new commands, and recover without replaying old intent?", "The injected fault is feedback delivery at the consumer. The simulator remains running."),
        event(0,3,"1 / Motion and feedback", "Begin a six-second move toward A. Joint feedback is arriving and the supervisor is EXECUTING.", "The arm is moving; feedback age is normally close to zero."),
        event(1,6,"2 / Interrupt feedback", "Stop delivering joint-state samples to the supervisor. Physics and the controller continue. The displayed last-received joint values now freeze.", "Watch feedback age rise past the 0.50 wall-second watchdog limit."),
        event(2,8,"3 / Detect and latch FAULT", "About 0.52 wall seconds after injection, the supervisor reports STALE_FEEDBACK and requests cancellation. The recorded controller acknowledges cancellation.", "Cancellation permits controller deceleration. It is not proof of an instantaneous or certified physical stop."),
        event(3,6,"4 / Reject new intent", "A new move request is deliberately attempted. The supervisor rejects it because the fault is latched. This is a software decision: no new motion should appear.", "Clearing the last error message alone does not re-enable commands."),
        event(4,6,"5 / Restore; explicitly recover", "Restore sample delivery and call recover(). It requires a running controller, fresh advancing feedback, and low velocity over a settling interval.", "Recovery re-establishes readiness; it does not resume unfinished A."),
        event(5,4,"6 / Send a fresh goal B", "After recovery succeeds, submit B as new intent. This is a different target from the interrupted A. Watch the arm move again.", "B = [-0.40, -1.22, -0.35, -1.65, -0.30, +0.20] rad"),
        event(6,7,"Result / Recovery verified", "The fresh command reaches B and passes measured completion checks. The sequence is: detect, latch, reject, restore, recover, then execute new intent.", "Recorded outcomes: interrupted A = FAULT (STALE_FEEDBACK); fresh B = SUCCEEDED."),
    ]

def build(slug):
    evidence = json.loads((OUT/(slug+".json")).read_text())
    if not evidence["passed"]:
        raise RuntimeError("Cannot explain an unsuccessful recording")
    duration = evidence["duration_seconds"]
    checkpoints = cards(slug, evidence["stages"])
    target = OUT/(slug+"-explained.mp4")
    folder = WORK/slug
    folder.mkdir(parents=True, exist_ok=True)
    segments, paths = [], []
    clock = 0.0
    for index, card in enumerate(checkpoints):
        source, hold, heading, body, proof = card
        following = checkpoints[index+1][0] if index+1<len(checkpoints) else duration
        for frozen, start, length in [(True,source,hold),(False,source,2*(following-source))]:
            if length <= 0:
                continue
            part = folder/("{:02d}.mp4".format(len(paths)))
            if frozen:
                filters = "select=eq(n\\,0),setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={},trim=duration={},fps=30".format(length,length)
                input_duration = 0.08
            else:
                filters = "setpts=2*(PTS-STARTPTS),fps=30"
                input_duration = following-source
            run(["ffmpeg","-y","-v","error","-ss",str(start),"-t",str(input_duration),
                 "-i",str(OUT/(slug+".mp4")),"-vf",filters,"-an","-c:v","libx264",
                 "-preset","ultrafast","-crf","18","-threads","2",str(part)])
            # Use actual encoded duration to avoid accumulating frame-rounding drift.
            length = float(subprocess.check_output(["ffprobe","-v","error","-show_entries",
                "format=duration","-of","default=nw=1:nk=1",str(part)],text=True))
            paths.append(part)
            segments.append({"start":clock,"end":clock+length,"source_start":source,
                             "source_end":source if frozen else following,"freeze":frozen,
                             "heading":heading,"body":body,"proof":proof})
            clock += length
    playlist = folder/"concat.txt"
    playlist.write_text("".join("file '{}'\n".format(p) for p in paths))
    subtitles = OUT/(slug+"-explained.ass")
    write_subtitles(subtitles, segments, evidence, clock)
    run(["ffmpeg","-y","-v","error","-f","concat","-safe","0","-i",str(playlist),
         "-vf","crop=868:696:206:80,scale=1000:802,pad=1600:1000:600:90:color=0x101820,ass="+str(subtitles),
         "-an","-c:v","libx264","-preset","fast","-crf","21","-threads","3",
         "-pix_fmt","yuv420p","-movflags","+faststart",str(target)])
    manifest = {"source":slug+".mp4","output":target.name,"duration_seconds":clock,
                "editorial_note":"0.5x source playback with explicitly labeled explanation freezes; no new execution results",
                "segments":segments}
    (OUT/(slug+"-explained.json")).write_text(json.dumps(manifest,indent=2)+"\n")
    print("Rendered",target.name,"({:.1f} s)".format(clock),flush=True)
    return target

def write_subtitles(path, segments, evidence, total):
    lines = ["[Script Info]\nScriptType: v4.00+\nPlayResX: 1600\nPlayResY: 1000\n[V4+ Styles]\n",
        "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n"]
    for name,size,color,bold in [("Title",30,"FFFFFF",-1),("Accent",20,"BFD65A",-1),
                                  ("Heading",29,"FFFFFF",-1),("Body",25,"E7E5E2",0),
                                  ("Proof",22,"BFD65A",0),("Data",19,"CBCBCB",0)]:
        lines.append("Style: {},DejaVu Sans,{},&H00{},&H00FFFFFF,&H00101820,&H00101820,{},0,0,0,100,100,0,0,1,0,0,7,0,0,0,1\n".format(name,size,color,bold))
    lines += ["[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n"]
    def text(start,end,style,x,y,value):
        lines.append("Dialogue: 0,{},{},{},,0,0,0,,{{\\an7\\pos({},{})\\q2}}{}\n".format(ts(start),ts(end),style,x,y,value))
    text(0,total,"Title",28,20,"UR5e Control & Recovery | "+evidence["title"].split(" / ")[-1])
    text(0,total,"Accent",28,62,"EXPLANATORY REPLAY  |  Source motion at 0.5x speed + labeled reading pauses  |  SIMULATION ONLY")
    samples = evidence["samples"]
    for segment in segments:
        start,end=segment["start"],segment["end"]
        mode = "PAUSED FOR EXPLANATION" if segment["freeze"] else "SOURCE FOOTAGE PLAYING AT 0.5x"
        text(start,end,"Accent",28,132,mode)
        text(start,end,"Heading",28,183,wrap(segment["heading"],31))
        text(start,end,"Body",28,285,wrap(segment["body"],37))
        text(start,end,"Proof",28,565,wrap(segment["proof"],40))
        text(start,end,"Data",28,795,wrap("Reading pause = frozen video frame. It is not an extra robot stop or a new simulation event.",43))
        t=start
        while t<end-0.005:
            source = segment["source_start"] if segment["freeze"] else segment["source_start"]+(t-start)/2
            sample=min(samples,key=lambda s:abs(s["t"]-source))
            q=sample["sample"]["q"]
            row="Source {:05.2f}s | Recorded state: {} | Last feedback age: {:.2f}s".format(source,sample["state"],sample["feedback_age"])
            row+=r"\NLast received q [rad]: "+"  ".join("{:+.2f}".format(v) for v in q)
            text(t,min(t+0.24,end),"Data",600,918,row)
            t+=0.24
    path.write_text("".join(lines))

def main():
    WORK.mkdir(parents=True,exist_ok=True)
    slugs=["01-verified-motion","02-command-replacement","03-fault-and-recovery"]
    with ThreadPoolExecutor(max_workers=2) as pool:
        paths=list(pool.map(build,slugs))
    playlist=WORK/"all.txt"
    playlist.write_text("".join("file '{}'\n".format(p) for p in paths))
    combined=OUT/"00-explained-demo.mp4"
    run(["ffmpeg","-y","-v","error","-f","concat","-safe","0","-i",str(playlist),
         "-c","copy","-movflags","+faststart",str(combined)])
    results=[]
    for path in [combined]+paths:
        run(["ffmpeg","-v","error","-xerror","-i",str(path),"-f","null","-"])
        metadata=json.loads(subprocess.check_output(["ffprobe","-v","error","-show_entries",
            "stream=codec_name,width,height,nb_frames:format=duration,size","-of","json",str(path)],text=True))
        results.append({"file":path.name,"fully_decoded":True,**metadata})
    (OUT/"explained-media-validation.json").write_text(json.dumps({"passed":True,"files":results},indent=2)+"\n")
    print("All explained videos fully decoded without errors",flush=True)

if __name__=="__main__":
    main()
