# Interfaces

## Input Topics

Default topic names are defined in `slam/slam/config/params.yaml`.

- `/gongji_gnss_ins_64`: GNSS/INS input, type
  `gnss_ins_msg/msg/Gnssins64`.
- `/perception/fusion/map`: perception cone map input, type
  `drd25_msgs/msg/Map`.

## Output Topics

- `/global_map`: `visualization_msgs/msg/MarkerArray`, stable global cones.
- `/vehicle_path`: `nav_msgs/msg/Path`.
- `/vehicle_odom`: `nav_msgs/msg/Odometry`.
- TF: `map` to `base_link`.

## `gnss_ins_msg` Fields Used By Code

The code includes `gnss_ins_msg/msg/gnssins64.hpp` and uses
`gnss_ins_msg::msg::Gnssins64`. The current `slam_node.cpp` reads these fields:

- `header`
- `latitude`
- `longitude`
- `roll`
- `pitch`
- `yaw`
- `vel_e`
- `vel_n`
- `imu_gyro_z`

A minimal compatibility message would need at least:

```text
std_msgs/Header header
float64 latitude
float64 longitude
float64 roll
float64 pitch
float64 yaw
float64 vel_e
float64 vel_n
float64 imu_gyro_z
```

Prefer the real project/team `gnss_ins_msg` package for real data or rosbag
compatibility.

## Perception Message Shape

`drd25_msgs/msg/Map`:

```text
std_msgs/Header header
Cone[] track
```

`drd25_msgs/msg/Cone`:

```text
float64 x
float64 y

uint8 BLUE=0
uint8 RED=1
uint8 YELLOW_BIG=2
uint8 YELLOW_SMALL=3
uint8 UNKNOWN=4
uint8 color
```
