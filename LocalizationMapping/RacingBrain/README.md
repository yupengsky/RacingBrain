# RacingBrain Orchestration

`RacingBrain` is the ROS 2 orchestration layer for the localization and mapping stack.

```text
RacingBrain
└── LocalizationMapping
    ├── Perception  -> run_perception + cone detector + fusion
    ├── Mapping     -> GNSS/INS-aided localization and cone-map generation
    ├── Health      -> unified runtime health bus for perception and mapping
    └── Planning    -> reserved planner interface
```

The lower-level implementation lives under:

- `LocalizationMapping/perception/`
- `LocalizationMapping/PointPillars/`
- `LocalizationMapping/slam/`
- `LocalizationMapping/gnss/`

## Launch Surface

```bash
ros2 launch racingbrain localization_mapping.launch.py \
  enable_perception:=true \
  enable_mapping:=true \
  enable_health:=true \
  enable_planning:=false \
  lidar_backend:=auto \
  track:=acceleration
```

CLI form after `source install/setup.bash`:

```bash
ros2 run racingbrain racingbrain mapping
```

Planning is intentionally a placeholder so the planner can later attach to the same localization and mapping entry point.
Health is enabled by default and publishes `/racingbrain/health/system`.
Use `lidar_backend:=auto` to let the perception arbiter prefer PointPillars when
available and fall back to clustering when the learning backend is unavailable or unhealthy.
