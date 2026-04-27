# RacingBrain Research Report Entry

## 1. Motivation

RacingBrain targets autonomous formula racing under strong real-time and
reliability constraints. The system does not rely on pure SLAM: localization is
GNSS-RTK/INS aided, while mapping focuses on building a stable cone map from
camera, LiDAR, fusion, and vehicle pose.

The research question is:

> How can a racing stack use fast learned perception while preventing learned
> perception failures from contaminating the map used by downstream planning?

This is a useful research angle because high-speed racing is not only about
accuracy on average. It is about detecting when the current perception result is
unsafe, limiting the damage online, and leaving replay evidence that explains
what happened.

## 2. Paper-Driven Ideas

The current mainline is shaped by four literature themes:

| Theme | What it suggests | RacingBrain implementation |
|---|---|---|
| Learned 3D detection | Replace slow handcrafted LiDAR pipelines when inference hardware is available | PointPillars path with clustering fallback |
| Open-world robustness | Learned models fail under blur, sparsity, calibration drift, latency, and domain shift | Online failure state and backend arbitration |
| Cross-modal consistency | Camera and LiDAR should agree in time, projection, and class evidence | Fusion consistency score, drift score, stamp-offset checks |
| Runtime safety monitors | A planner should not consume unqualified perception as truth | Mapping gate, map pollution benchmark, confidence map layers |

The important point is not that RacingBrain simply "uses deep learning". The
insight is that deep learning is treated as a high-performance but fallible
subsystem. The project therefore adds monitors and conservative map interfaces
around it.

## 2.5 Recent References Worth Keeping

These newer references fit the project direction naturally:

- [Robustness Evaluation of Localization Techniques for Autonomous Racing (2024)](https://arxiv.org/abs/2401.07658):
  highlights that racing localization must be judged under slip and aggressive
  dynamics, not only under nominal logs.
- [Resilient Sensor Fusion Under Adverse Sensor Failures via Multi-Modal Expert Fusion (CVPR 2025)](https://openaccess.thecvf.com/content/CVPR2025/html/Park_Resilient_Sensor_Fusion_Under_Adverse_Sensor_Failures_via_Multi-Modal_Expert_CVPR_2025_paper.html):
  suggests modality-quality-aware routing rather than one monolithic fusion path.
- [Cost-Sensitive Uncertainty-Based Failure Recognition for Object Detection (UAI 2024)](https://proceedings.mlr.press/v244/kassem-sbeyti24a.html):
  suggests that failure recognition should depend on downstream cost, not only on
  detector confidence.
- [Perceive With Confidence (CoRL 2025)](https://proceedings.mlr.press/v270/dixit25a.html):
  suggests calibrated uncertainty as a bridge from perception outputs to
  planner-level safety guarantees.
- [A re-calibration method for object detection with multi-modal alignment bias in autonomous driving (2024)](https://arxiv.org/abs/2405.16848):
  supports the current calibration-drift monitor and points toward online
  re-calibration as a future module.
- [Can't Slow Me Down: Learning Robust and Hardware-Adaptive Object Detectors against Latency Attacks for Edge Devices (CVPR 2025)](https://openaccess.thecvf.com/content/CVPR2025/html/Wang_Cant_Slow_Me_Down_Learning_Robust_and_Hardware-Adaptive_Object_Detectors_CVPR_2025_paper.html):
  is a reminder that real-time robustness is not only about perception accuracy,
  but also about compute-time collapse and cascading latency failures.
- [Curvature-Integrated MPCC Local Trajectory Planning Method (2025)](https://arxiv.org/abs/2502.03695):
  makes the next step clear once the track graph is stable: convert it into a
  racing-aware local planner with curvature-sensitive speed targets.

## 3. Current System Claim

RacingBrain now has a reliability-aware perception-to-map path:

```text
Camera + LiDAR + GNSS/INS
        |
YOLO + PointPillars / clustering fallback
        |
Camera-LiDAR fusion consistency checks
        |
Perception failure state and system health bus
        |
Risk-aware mapping gate
        |
Stable map + candidate map + rejected observation layer
        |
Sparse track graph for planning
```

This makes the project more than a replay pipeline. It gives the system a way to
answer: "Should this observation be trusted enough to change the long-lived map?"

## 4. Evidence Already in the Repo

The latest local validation runs show:

| Experiment | Evidence |
|---|---|
| Build | 11 ROS packages build successfully |
| Mapping gate A/B | With the same replay, stable cones stayed 20/20 while candidate residue dropped from 11 to 7 |
| Map stability score | Gate-on score improved from 0.545 to 0.721 in the tested replay |
| Confidence layers | Stable, candidate, and rejected-observation map layers all published during replay |
| Planning interface | Track graph and planning input state published 138 frames; 84 frames were ready |

These numbers are self-consistency evidence, not ground-truth accuracy. A later
research report should add annotated cone positions or surveyed track geometry to
measure absolute map error.

## 4.5 Current Mainline Progress Evaluation

The current mainline should be viewed as follows:

| Direction | Status | Comment |
|---|---|---|
| Real-time perception-to-map chain | Strong | Already runnable, benchmarked, and inspectable |
| Reliability-aware perception fallback | Strong | Backend arbitration and health bus are in place |
| Failure-to-map protection | Strong | Risk gate and map-pollution evaluation already form a coherent story |
| Planning-facing representation | Medium | Sparse track graph is landed, but not yet a real racing planner |
| Absolute localization/mapping accuracy benchmark | Medium-weak | Self-consistency is strong; ground-truth error is still missing |
| Online calibration correction | Medium-weak | Drift is monitored, but not yet corrected online |
| End-to-end racing performance story | Medium | The control/planning side is not the bottleneck yet, but it is still incomplete |

So the project is already attractive as an engineering research prototype. Its
weak point is no longer "the system is old". The remaining gap is that the
current story ends at a conservative planning interface instead of a full
closed-loop racing stack.

## 5. Next Research Commits

Good next experiments should stay close to the racing task:

1. Add scenario-level fault sweeps for blur, camera dropout, LiDAR timestamp
   skew, GNSS skew, and calibration perturbation.
2. Convert `map_stability_score` into a paper-quality metric with ablations:
   gate off, gate downweight only, gate freeze, backend fallback only.
3. Add a small annotated cone-map fixture to report absolute position error,
   duplicate rate, and false candidate lifetime.
4. Replace the greedy track-graph pairing with a constrained boundary matching
   method that respects cone color, ordering, and expected track width.
5. Feed the sparse track graph into a first local planner, while keeping the
   current planning input state as a safety gate.
6. Add latency-aware failure budgeting so that perception failures include
   compute collapse, not only semantic mismatch or calibration drift.

## 6. How to Present It

The project can be presented as:

> A real-time intelligent racing stack that combines learned perception with
> conservative runtime monitors, then protects the GNSS/INS-aided cone map from
> perception failures before exposing it to planning.

This is aligned with real-scene embodied autonomy because the main contribution
is not a single model. It is an inspectable system loop: perception, failure
judgement, map protection, evaluation, and planning-facing representation.
