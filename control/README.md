# RacingBrain Control Stack

`control/` is now reserved for planning and control code. The downloaded demo
workspace has been reduced to the modules that actually belong to this layer:

```text
control/src/
  path_planner/              # local path planning for autocross, acceleration, skidpad
  cpp_controller/            # primary C++ lateral/longitudinal controller
  simple_pid_controller/     # Python fallback controller
  racing_control_adapters/   # RacingBrain map/odom -> planner interface adapter
  racing_control_bringup/    # launch files and mission configs
```

Removed from the imported workspace:

- perception/LiDAR clustering code
- SLAM and ground-truth map publishers
- duplicated `drd25_msgs`
- FSDS track editor tooling
- generated `build/`, `install/`, `log/`, and zip artifacts

The active message interface remains the project-level `drd25_msgs` package
under `LocalizationMapping/slam/drd25_msgs`.

## Data Flow

With RacingBrain adapters enabled:

```text
/global_map        -> racing_control_adapters -> /drd25/map
/vehicle_odom      -> racing_control_adapters -> /testing_only/odom
/drd25/map         -> path_planner            -> /drd25/path
/testing_only/odom -> cpp_controller          -> /control_command
```

The controller output uses `fs_msgs/ControlCommand`, the command interface
expected by the Formula Student Driverless Simulator ROS bridge.

FSDS ground-truth debug flow:

```text
/testing_only/track -> fsds_track_to_map_adapter -> /drd25/map
/testing_only/odom  -> path_planner/controller   -> /control_command
```

## Build

The control stack is optional because the controller depends on FSDS `fs_msgs`.
Build the normal RacingBrain stack as before:

```bash
./scripts/build_ros_clean.sh
```

Build the planning-control stack after sourcing/building FSDS or another
workspace that provides `fs_msgs`:

```bash
source /opt/ros/humble/setup.bash
source /path/to/Formula-Student-Driverless-Simulator/ros2/install/setup.bash
INCLUDE_CONTROL_STACK=true ./scripts/build_ros_clean.sh
```

## Launch

RacingBrain map/odom to control command:

```bash
ros2 launch racing_control_bringup racing_control.launch.py \
  mode:=autocross \
  use_racingbrain_adapters:=true
```

Planner/controller only, when another stack already publishes `/drd25/map` and
`/testing_only/odom`:

```bash
ros2 launch racing_control_bringup racing_control.launch.py \
  mode:=autocross \
  use_racingbrain_adapters:=false
```

FSDS ground-truth track quick debug:

```bash
# For the local FSDS v2.2.0 ROS2 bridge, start this launch before starting the
# bridge so the volatile one-shot `/testing_only/track` message is received.
ros2 launch racing_control_bringup racing_control.launch.py \
  mode:=autocross \
  use_racingbrain_adapters:=false \
  use_fsds_track_adapter:=true
```

Use the Python fallback controller:

```bash
ros2 launch racing_control_bringup racing_control.launch.py \
  mode:=autocross \
  dummy_control:=true
```

## FSDS Notes

FSDS is an Unreal Engine 4 + AirSim based Formula Student Driverless simulator.
The local v2.2.0 ROS2 bridge publishes simulated sensors and ground-truth testing
topics, then subscribes to `/control_command` for throttle, steering, and brake.
In this tagged release the main bridge topics are rooted at `/` (for example
`/testing_only/odom` and `/testing_only/track`), while camera topics are under
`/fsds/cam*`. Newer documentation/branches may show a `/fsds/...` namespace; if
you use those builds, override `fsds_track_topic`, `planner_odom_topic`, and
`control_topic` in the launch command, and set `fsds_track_transient_local:=true`
when the bridge offers a transient-local track publisher. The command values are
dimensionless: throttle/brake in `[0, 1]`, steering in `[-1, 1]`.

Key references:

- FSDS repository: <https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator>
- ROS bridge docs: <https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/ros-bridge/>
- ROS setup docs: <https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/getting-started-with-ros/>
