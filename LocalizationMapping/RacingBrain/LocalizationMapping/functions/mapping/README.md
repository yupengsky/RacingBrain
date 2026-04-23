# Mapping Function

Wraps the existing GNSS/INS-aided mapping launch:

```bash
ros2 launch slam slam.launch.py
```

RacingBrain entry:

```bash
ros2 launch racingbrain localization_mapping.launch.py enable_perception:=false enable_mapping:=true
```

Arguments passed through:

- `track`: `acceleration`, `autocross`, or `skidpad`
- `rviz`: launch RViz
- `eval_debug`: enable evaluation metrics publishers
