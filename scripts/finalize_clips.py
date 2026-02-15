#!/usr/bin/python3
"""Join the recorded chapters and validate every delivered MP4 by full decoding."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/demo-clips"
SLUGS = ["01-verified-motion", "02-command-replacement", "03-fault-and-recovery"]

def main():
    playlist = OUTPUT / "playlist.txt"
    playlist.write_text("".join("file '{}.mp4'\n".format(slug) for slug in SLUGS))
    combined = OUTPUT / "00-full-demo.mp4"
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-f", "concat", "-safe", "0",
                    "-i", str(playlist), "-c", "copy", "-movflags", "+faststart", str(combined)], check=True)
    report = {"passed": False, "graphical_simulation_verified": True, "files": []}
    for slug in ["00-full-demo"]+SLUGS:
        path = OUTPUT / (slug+".mp4")
        metadata = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-show_entries",
            "stream=codec_name,width,height,r_frame_rate,nb_frames:format=duration,size", "-of", "json", str(path)], text=True))
        stream = metadata["streams"][0]
        if (stream["codec_name"],stream["width"],stream["height"]) != ("h264",1280,900):
            raise RuntimeError("Unexpected video encoding: " + slug)
        if float(metadata["format"]["duration"]) < 5:
            raise RuntimeError("Video unexpectedly short: " + slug)
        subprocess.run(["ffmpeg", "-v", "error", "-xerror", "-i", str(path),
                        "-f", "null", "-"], check=True, timeout=60)
        if slug in SLUGS:
            evidence = json.loads((OUTPUT / (slug+".json")).read_text())
            if not evidence["passed"]:
                raise RuntimeError("Execution did not pass: " + slug)
            expected = {"01-verified-motion": ["SUCCEEDED"]*3,
                        "02-command-replacement": ["CANCELLED","SUCCEEDED"],
                        "03-fault-and-recovery": ["FAULT","SUCCEEDED"]}[slug]
            if [r["status"] for r in evidence["results"]] != expected:
                raise RuntimeError("Unexpected execution sequence: " + slug)
        report["files"].append({"file":path.name,"decoded_without_errors":True,**metadata})
    report["passed"] = True
    (OUTPUT / "media-validation.json").write_text(json.dumps(report,indent=2)+"\n")
    print("Validated four H.264 videos by full decoding; all recorded execution sequences passed",flush=True)

if __name__ == "__main__":
    main()
