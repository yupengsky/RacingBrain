# Health Function

Aggregates lightweight runtime metrics from:

- YOLO
- LiDAR backend (`pointpillars` or `cluster`)
- camera-LiDAR fusion
- GNSS/INS-aided mapping

Primary topic:

```bash
ros2 topic echo /racingbrain/health/system
```

RacingBrain entry:

```bash
ros2 launch racingbrain localization_mapping.launch.py enable_health:=true
```

Useful arguments:

- `enable_health`: turn the online health monitor on or off
- `health_period`: publish period for the aggregated system-health topic
- `health_stale_timeout`: stale timeout used by the health monitor
