#!/usr/bin/env python3
"""Bridge RacingBrain map/odometry topics into the imported control stack.

The imported planner consumes `drd25_msgs/Map` and `/testing_only/odom`, while
RacingBrain currently publishes a visualization `MarkerArray` map and
`/vehicle_odom`. This adapter keeps that boundary explicit instead of mixing
mapping code into the control packages.
"""

from __future__ import annotations

from typing import Any, Optional

import rclpy
from drd25_msgs.msg import Cone, Map
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def marker_color_id(marker: Marker) -> int:
    cone_schema = Cone()
    blue = getattr(cone_schema, "BLUE", 0)
    right = getattr(cone_schema, "YELLOW", getattr(cone_schema, "RED", 1))
    unknown = getattr(cone_schema, "UNKNOWN", 4)

    r = float(marker.color.r)
    g = float(marker.color.g)
    b = float(marker.color.b)
    if b > 0.7 and r < 0.35 and g < 0.35:
        return int(blue)
    if r > 0.7 and b < 0.45:
        return int(right)
    return int(unknown)


class RacingBrainMapOdomAdapter(Node):
    def __init__(self) -> None:
        super().__init__("racingbrain_map_odom_adapter")
        self.declare_parameter("marker_map_topic", "/global_map")
        self.declare_parameter("planner_map_topic", "/drd25/map")
        self.declare_parameter("odom_input_topic", "/vehicle_odom")
        self.declare_parameter("planner_odom_topic", "/testing_only/odom")
        self.declare_parameter("republish_odom", True)

        marker_map_topic = str(self.get_parameter("marker_map_topic").value)
        planner_map_topic = str(self.get_parameter("planner_map_topic").value)
        odom_input_topic = str(self.get_parameter("odom_input_topic").value)
        planner_odom_topic = str(self.get_parameter("planner_odom_topic").value)
        republish_odom = bool(self.get_parameter("republish_odom").value)

        latched_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(Map, planner_map_topic, latched_map_qos)
        self.create_subscription(MarkerArray, marker_map_topic, self.cb_marker_map, 10)

        self.odom_pub: Optional[Any] = None
        if republish_odom:
            self.odom_pub = self.create_publisher(Odometry, planner_odom_topic, 10)
            self.create_subscription(Odometry, odom_input_topic, self.cb_odom, 10)

        self.get_logger().info(
            f"Adapting map {marker_map_topic} -> {planner_map_topic}, "
            f"odom {odom_input_topic} -> {planner_odom_topic}"
        )

    def cb_marker_map(self, msg: MarkerArray) -> None:
        out = Map()
        header_set = False
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if not header_set:
                out.header = marker.header
                header_set = True
            cone = Cone()
            cone.x = float(marker.pose.position.x)
            cone.y = float(marker.pose.position.y)
            cone.color = marker_color_id(marker)
            out.track.append(cone)
        if not header_set:
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "map"
        self.map_pub.publish(out)

    def cb_odom(self, msg: Odometry) -> None:
        if self.odom_pub is not None:
            self.odom_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = RacingBrainMapOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
