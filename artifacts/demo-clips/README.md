# Robot control demonstrations: what is happening?

**Start with the [explained walkthrough — 3 minutes 11 seconds](00-explained-demo.mp4).**
The earlier 44-second edit did not leave enough time to understand the software events.
These instructional versions introduce the question, slow the original footage to
0.5×, and hold a frame at each important decision so the explanation can be read.

| Scenario | Explained walkthrough | Original real-time source |
| --- | --- | --- |
| Verified robot motion | [68 seconds](01-verified-motion-explained.mp4) | [19 seconds](01-verified-motion.mp4) |
| Command replacement | [47 seconds](02-command-replacement-explained.mp4) | [11 seconds](02-command-replacement.mp4) |
| Fault detection and recovery | [76 seconds](03-fault-and-recovery-explained.mp4) | [15 seconds](03-fault-and-recovery.mp4) |

## Read before watching

A and B are named six-joint configurations. The [main README](../../README.md) gives
the actual angles, source-clock timelines, completion checks, and recorded errors.
A reading pause freezes both the image and displayed telemetry. It is an editing
pause, not an extra robot stop. The source timestamp in the footer lets you locate
the same event in the original recording.

### Verified robot motion

The program sends A, waits for measured completion, sends B, verifies it, then returns
home and verifies that too. Watch the elbow bend, the base turn in opposite directions,
and the wrist reorient. Each filmed goal uses a four-second simulated trajectory;
wall-clock completion also includes simulation pacing and settling checks.

A pass requires controller success plus fresh joint measurements within 0.03 rad,
velocity at most 0.05 rad/s, and a 0.25 simulated-second settling interval. The three
recorded outcomes are `SUCCEEDED / MEASURED_TARGET_REACHED`.

### Command replacement

A starts on a six-second simulated trajectory. About 2.2 wall seconds later, before
A finishes, B is submitted with replacement enabled. A receives a cancellation request
and an acknowledgement; its result is `CANCELLED / REPLACED`. Then B is sent with a
new command ID and passes its own measured checks. Watch the arm change toward B
before finishing A. It is not a queue of two completed motions.

### Fault detection and recovery

The robot begins moving toward A. At source time 2.6 s, the script drops joint-state
samples at the supervisor's consumer; the simulator and controller continue. Last
received joint values freeze while sample age rises. About 0.52 wall seconds after
injection, the supervisor latches `FAULT / STALE_FEEDBACK` and requests cancellation.

The script next attempts another command and confirms rejection. That rejection is a
software event with no new robot motion. It then restores sample delivery and explicitly
calls `recover()`, which checks controller readiness and fresh, settled state. Finally
it submits B as new intent. B succeeds; the interrupted A is never replayed.

Cancellation acknowledgement is not an instantaneous or certified hardware stop.
Controller deceleration and the video editor's labeled reading pauses are separate
from fault detection and recovery.

## Recording provenance and evidence

The original clips are actual, continuous Gazebo window captures at wall-time playback
speed. Their robot motion is unchanged. The explained versions crop and slow that same
footage and insert explicitly labeled holds. They do not claim a new simulation run.
All clips are silent and captioned. No physical robot or certified safety claim is involved.

Each original chapter has a JSON receipt containing stages, outcomes, and sampled state.
`execution-events.jsonl` correlates commands and results. Each explained chapter also
has a JSON source-to-playback timeline and ASS caption source.

- `media-validation.json`: successful full decoding of the four original MP4s.
- `explained-media-validation.json`: successful full decoding of the four explained MP4s.
- `../regression-summary.json`: eight passing integration tests before original recording.

Rebuild the explanatory edit with `python3 scripts/explain_clips.py` from the repository
root, using system Python and ffmpeg. No running robot simulation is needed for this edit.
