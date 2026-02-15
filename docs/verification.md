# Verified results — 2026-11-15

Final command, from a fresh simulation:

```bash
source scripts/env.sh
/usr/bin/python3 scripts/run_milestone1.py --regression --record-clips
```

Result: exit code 0. All spawned simulation processes were shut down afterward.

## Baseline

Three 3-second joint moves passed controller success, measured settling, actual
displacement, feedback freshness, and direct Gazebo joint queries. Maximum final
joint-position error was approximately 0.00013 rad, against a 0.03 rad threshold.
Gravity was enabled (z = -9.8 m/s²); physics used a 0.0001 s step and upstream effort gains.
See `artifacts/milestone1.json` for raw positions, velocities, configuration, and trace.

## Regression suite

Eight tests passed, with zero failures and zero errors, in approximately 17.7 wall seconds
with the Gazebo graphical client also running:

1. Normal execution with direct Gazebo confirmation.
2. Wrong dimensions, non-finite targets, joint-limit violations, and stale command rejection.
3. Active-goal replacement and explicit cancellation.
4. Feedback loss, latched fault, explicit recovery, and a successful fresh command.
5. Paused physics detected using wall time.
6. Actual controller deactivation, fault detection, and recovery rejection while stopped.
7. Controller success with inconsistent observed joint state detected as a mismatch.
8. Execution wall deadline and cancellation request.

Machine-readable records: `artifacts/regression-summary.json`,
`artifacts/regression-results.json`, and `artifacts/execution-events.jsonl`.

## Demonstration

The narrated demo passed normal motion, active-command replacement, verified replacement
motion, feedback-loss fault detection, explicit recovery, and verified fresh motion.
See `artifacts/demo-results.json` and `artifacts/demo-events.jsonl`.

The graphical demonstration additionally records three larger-motion scenarios:
verified joint motion, active-command replacement, and fault/recovery. All recorded
commands passed their expected terminal-state assertions. Exported videos are 1280x900,
30 fps, H.264/yuv420p, with fast-start MP4 metadata:

| Video | Duration |
| --- | --- |
| `00-full-demo.mp4` | 44.467 seconds |
| `01-verified-motion.mp4` | 19.467 seconds |
| `02-command-replacement.mp4` | 10.500 seconds |
| `03-fault-and-recovery.mp4` | 14.500 seconds |

All four videos were fully decoded with ffmpeg using error-fatal mode. Representative
frames from each clip and a full-resolution fault frame were visually inspected for
robot visibility, actual motion, and readable captions. See
`artifacts/demo-clips/media-validation.json` for stream metadata and decoding results.

## Checks and limitations

Python compilation, shell syntax, and launch/world XML parsing passed. Repeated baseline
startup runs passed at the final numerical configuration. Earlier failed configurations
remain in `artifacts/` and are explained in `docs/milestone1.md`.

These results validate this machine's headless and graphical configurations and the listed scenarios.
They do not establish hardware correctness, certified safety, hard real-time timing,
or unlimited reliability. No physical hardware, cloud account, or paid service
was used. System packages and other repositories were not modified.
