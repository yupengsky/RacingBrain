# Perception Function

Wraps the existing perception launch:

```bash
ros2 launch run_perception system_run.launch.py
```

RacingBrain entry:

```bash
ros2 launch racingbrain localization_mapping.launch.py enable_perception:=true enable_mapping:=false
```

Arguments passed through:

- `lidar_backend`: `pointpillars` or `cluster`
- `eval_debug`: enable evaluation metrics publishers
