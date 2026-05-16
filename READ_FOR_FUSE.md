# RacingBrain Fused Localization Agent Brief

This file is for Codex-like coding agents. It is not a user-facing README, a
research proposal, or a place to write project plans. Your job is to make the
project produce a usable fused-localization result for short GNSS/INS outages,
evaluate it against GNSS/INS truth, and leave behind executable code and
commands.

## Mission

Implement and evaluate a fused localization test where GNSS/INS has the highest
priority, and the existing simple LIO pipeline is used only as a temporary
fallback when GNSS/INS is artificially unavailable.

In this project, the complete GNSS/INS pose is treated as the true vehicle pose
and the complete GNSS/INS trajectory is treated as the true trajectory. Fusion
quality is judged by how well the fused trajectory matches that complete truth.

The expected behavior is:

- When GNSS/INS is available, fused localization outputs GNSS/INS.
- When GNSS/INS is artificially masked or missing, fused localization switches
  to LIO.
- At the start of each GNSS/INS outage, initialize or align LIO from the most
  recent available GNSS/INS pose.
- During an outage, do not keep injecting hidden GNSS/INS truth into the fused
  pose. LIO should carry the pose through the gap.
- When GNSS/INS becomes available again, switch fused localization back to
  GNSS/INS.
- Always use the complete, unmasked GNSS/INS trajectory for final evaluation.

Use the current simple LIO implementation that already produced a useful result
for this dataset. Do not replace it with a large opaque localization framework
unless the existing LIO path is proven unusable for the fusion test.

Avoid creating unrelated documents, PPTs, reports, duplicate READMEs, or
planning-only artifacts. Keep work focused on executable fusion code,
evaluation scripts, result HTML, and the minimal command surface needed to
reproduce them.

## Current Dataset

Primary dataset:

```bash
/media/yupeng/Ventoy/rosbag2_2026_05_10-15_02_50
```

The GNSS/INS device documentation may be useful for field definitions, status
bits, coordinate conventions, heading/yaw interpretation, and time sync:

```bash
/media/yupeng/新加卷/0-3-Projects/AutoElec/ADAS-Project/共迹A602组合定位设备资料
```

Before tuning the fusion behavior, confirm that the dataset still supports the
test:

- LiDAR, IMU, and GNSS/INS topics exist and contain enough messages.
- Timestamps and frequencies are usable for synchronized comparison.
- Complete GNSS/INS truth is continuous enough to serve as ground truth.
- The artificial outage windows are inside time ranges where LIO can be run.
- Coordinate frame conventions match the existing LIO evaluation.

If the dataset cannot support the fusion test, such as missing LiDAR/IMU/GNSS
topics, severe timestamp disorder, invalid GNSS/INS states, or motion that does
not support LIO fallback, stop tuning and write the evidence clearly.

## Existing LIO Baseline

The LIO fallback should use the simple offline LIO direction already validated
in this repo:

```bash
scripts/offline_simple_lio_eval.py
scripts/run_dataset_simple_lio_eval.sh
```

The important behavior to preserve:

- GNSS/INS may initialize LIO pose/yaw at a bounded initialization point.
- LiDAR and IMU carry the pose after initialization.
- GNSS/INS is not continuously injected as the LIO pose source.
- Outputs must be synchronized and evaluated against complete GNSS/INS truth.

If the fusion implementation shares code with the LIO evaluator, keep the
shared behavior explicit and inspectable. Do not silently change the LIO
algorithm in a way that makes old LIO results irreproducible.

## Expected Fusion Direction

The preferred first target is a simple, controllable offline fusion evaluation:

- Read the dataset directly or replay it in a reproducible way.
- Build the complete GNSS/INS truth trajectory.
- Create configurable artificial GNSS/INS outage windows.
- Produce a masked GNSS/INS availability stream.
- During available GNSS/INS samples, fused pose equals GNSS/INS.
- During outage samples, fused pose equals LIO fallback initialized from the
  latest pre-outage GNSS/INS pose.
- Record which source produced each fused sample: `gnss_ins` or `lio_fallback`.
- Report error of the fused output against the complete GNSS/INS truth.

Good outage scenarios include several short gaps at different parts of the
track, such as straight segments and turns. The outage schedule should be
visible in outputs and easy to change from command-line parameters or a compact
configuration block.

Do not make the fused trajectory look good by using complete GNSS/INS truth
during the artificially masked intervals. The whole point is to verify whether
LIO can fill those gaps.

## Evaluation Output

The final evaluation must produce a dynamic HTML result in the same interaction
style as the LIO evaluation HTML:

- Play/pause button.
- Time slider.
- Speed selector.
- Reset-view control.
- Draggable and zoomable global maps.
- Current-time marker during playback.
- Instantaneous position error over time.
- Cumulative mean position error over time.
- Compact numeric status panel with only values the pipeline actually has.

For the fusion test, the large map area should preferably be split into two
stacked global maps:

- Top map: complete GNSS/INS truth trajectory.
- Bottom map: fused trajectory made from available GNSS/INS segments plus LIO
  fallback segments in the artificial GNSS/INS outage windows.

The viewer should make the artificial GNSS/INS outage intervals visible. During
an outage, the bottom map should show the GNSS/INS segment disappearing or
being unavailable, and the LIO fallback segment filling the missing part.

Use distinct colors for:

- Complete GNSS/INS truth.
- Available GNSS/INS portions used by fusion.
- LIO fallback portions used during outages.
- Current vehicle marker.
- Current position error, if drawn on the map.

Keep the HTML self-contained when possible. It should embed sampled
trajectory/error data as JSON and run without a server.

## Recommended HTML Shape

Minimum data per synchronized sample:

```json
{
  "t": 12.34,
  "truth": {"x": 1.0, "y": 2.0, "yaw": 0.1},
  "fused": {"x": 0.9, "y": 2.2, "yaw": 0.08, "source": "lio_fallback"},
  "gnss_available": false,
  "outage_id": "gap_01",
  "error_m": 0.22,
  "mean_error_m": 0.15
}
```

The HTML should compute or display:

- sample count
- elapsed time
- current truth x/y
- current fused x/y
- current fused source
- GNSS/INS availability at the current time
- current outage id, if any
- current position error
- cumulative mean position error
- max position error
- RMSE, if enough synchronized samples exist
- total artificial outage duration
- LIO fallback sample count

If yaw error is reliable, it may be added. If yaw conventions are uncertain, do
not pretend it is valid; explain the uncertainty in the generated result or
evaluation summary.

## Success Criteria

A useful agent run should leave the project in a state where another agent can:

1. Run the fusion evaluation on the dataset.
2. Reproduce the artificial GNSS/INS outage schedule.
3. Generate synchronized truth and fused trajectory samples.
4. Open a dynamic HTML showing the complete GNSS/INS truth and the fused
   GNSS/INS-plus-LIO fallback trajectory.
5. Read quantitative fused-vs-truth error results.
6. Know whether remaining error is caused by LIO fallback drift, fusion
   switching logic, outage schedule choices, parameter choices, integration
   bugs, or dataset problems.

Strong results are not defined by appearance. They are defined by lower fused
pose error against complete GNSS/INS truth, stable gap filling, clear outage
visualization, and clear diagnosis.

## Existing Useful Commands

Current LIO evaluation command:

```bash
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50 OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html ./scripts/run_dataset_simple_lio_eval.sh
```

Current LIO result viewer command:

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/simple_lio_2026_05_10-15_02_50.html
```

Future fusion commands should be similarly complete and should write an HTML
result under `results/`, for example:

```bash
OUTPUT_DIR=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50 OUTPUT_HTML=/home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50.html ./scripts/run_dataset_fuse_eval.sh
```

```bash
xdg-open /home/yupeng/GitHub/RacingBrain/results/fuse_eval_2026_05_10-15_02_50.html
```
