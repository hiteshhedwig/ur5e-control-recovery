# Recorded Gazebo demonstrations

Watch [the complete 44-second demo](00-full-demo.mp4), or individual chapters:

1. [Verified robot motion](01-verified-motion.mp4): three joint targets executed through
   the existing ROS trajectory controller, with measured settling checks.
2. [Command replacement](02-command-replacement.mp4): cancel an executing goal, acknowledge
   its replacement, and verify the new goal independently.
3. [Fault detection and recovery](03-fault-and-recovery.mp4): interrupt feedback delivery
   at the supervisor's consumer, latch a stale-feedback fault, reject new intent,
   restore feedback, explicitly recover, and complete a fresh command.

These are actual continuous Gazebo window captures at wall-time playback speed.
Captions and measured-state overlays were added afterward from timestamped live data;
robot motion is neither synthesized nor retimed. During feedback loss, displayed joint
values are the last received sample; the increasing feedback-age field marks them stale.
No physical robot or certified safety claim is involved. Clips are silent and captioned.

Each chapter has a matching JSON file containing stages, outcomes, and sampled state,
plus an ASS caption source. `execution-events.jsonl` correlates commands and results.
`media-validation.json` records successful full decoding and video encoding metadata.
The combined file simply joins the three chapters, without changing their timing.

The baseline and all eight regression tests passed immediately before recording,
with the graphical client running. See `../regression-summary.json` for test results.
