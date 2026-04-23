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

## 6. How to Present It

The project can be presented as:

> A real-time intelligent racing stack that combines learned perception with
> conservative runtime monitors, then protects the GNSS/INS-aided cone map from
> perception failures before exposing it to planning.

This is aligned with real-scene embodied autonomy because the main contribution
is not a single model. It is an inspectable system loop: perception, failure
judgement, map protection, evaluation, and planning-facing representation.
