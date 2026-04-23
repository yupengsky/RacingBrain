# LocalizationMapping

This is the function-facing side of RacingBrain. It composes the packages needed to turn raw race-car sensor data into pose and map outputs.

Current function folders:

- `functions/perception`: camera, LiDAR, cone detection, and fusion.
- `functions/mapping`: GNSS/INS-aided localization and cone-map generation.
- `functions/health`: online health aggregation for perception and mapping.
- `functions/planning`: reserved planner interface.
- `functions/localization_mapping_stack`: composition of the full stack.

Primary entry point:

```bash
ros2 launch racingbrain localization_mapping.launch.py
```
