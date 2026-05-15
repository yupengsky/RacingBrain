import numpy as np
from threading import Thread
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from drd25_msgs.msg import Map
from drd25_msgs.msg import Waypoint
from drd25_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from .mid_line_planner import MidLinePlanner
from .skidpad_planner import SkidPadPlanner
from .acceleration_planner import AccelerationPlanner
from .visualization import publish_tf, publish_ego_marker, publish_cones_markers, publish_path_markers


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


class PathPlanner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('rviz_visualization', False)
        self.declare_parameter('mode', 'autocross')
        self.declare_parameter('waypointsPublishInterval', 0.2)
        self.declare_parameter('map_topic', '/drd25/map')
        self.declare_parameter('odom_topic', '/testing_only/odom')
        self.declare_parameter('path_topic', '/drd25/path')
        self.declare_parameter('state_indicator_topic', '/drd25/state_indicator')
        self.declare_parameter('off_track_topic', '/drd25/off_track')
        self.declare_parameter('brake_topic', '/drd25/brake_command')

        self.rviz_visualization = self.get_parameter('rviz_visualization').value
        self.mode = self.get_parameter('mode').value
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.state_indicator_topic = self.get_parameter('state_indicator_topic').value
        self.off_track_topic = self.get_parameter('off_track_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value
        if self.mode == 'autocross':
            self.planner = MidLinePlanner(self)
        elif self.mode == 'skidpad':
            self.planner = SkidPadPlanner(self)
        elif self.mode == 'acceleration':
            self.planner = AccelerationPlanner(self)
        else:
            raise ValueError("Invalid mode. Choose from 'autocross', 'skidpad', 'acceleration'")
        self.handle_parameters()

        latched_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(Map, self.map_topic, self.map_callback, latched_map_qos,
                                                callback_group=MutuallyExclusiveCallbackGroup())
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1,
                                                 callback_group=MutuallyExclusiveCallbackGroup())

        self.waypointsPublishInterval = self.get_parameter('waypointsPublishInterval').value
        self.wp_pub_timer = self.create_timer(self.waypointsPublishInterval, self.timer_callback,
                                              callback_group=MutuallyExclusiveCallbackGroup())

        self.route_pub = self.create_publisher(Path, self.path_topic, 1,
                                               callback_group=MutuallyExclusiveCallbackGroup())
        self.state_indicator_pub = self.create_publisher(Bool, self.state_indicator_topic, 1,
                                                         callback_group=MutuallyExclusiveCallbackGroup())
        self.offtrack_pub = self.create_publisher(Bool, self.off_track_topic, 1)
        self.brake_pub = self.create_publisher(Bool, self.brake_topic, 1)
        
        # rviz2 visualization publishers
        if self.rviz_visualization:
            self.conesVisualPub = self.create_publisher(MarkerArray, "/visual/cones", 1,
                                                        callback_group=MutuallyExclusiveCallbackGroup())
            self.egoVisualPub = self.create_publisher(Marker, "/visual/ego_marker", 1,
                                                      callback_group=MutuallyExclusiveCallbackGroup())
            self.treeVisualPub = self.create_publisher(MarkerArray, "/visual/tree_markers", 1,
                                                       callback_group=MutuallyExclusiveCallbackGroup())
            self.bestBranchVisualPub = self.create_publisher(Marker, "/visual/best_tree_branch", 1,
                                                             callback_group=MutuallyExclusiveCallbackGroup())
            self.filteredBranchVisualPub = self.create_publisher(Marker, "/visual/filtered_tree_branch", 1,
                                                                 callback_group=MutuallyExclusiveCallbackGroup())
            self.delaunayLinesVisualPub = self.create_publisher(Marker, "/visual/delaunay_lines", 1,
                                                                callback_group=MutuallyExclusiveCallbackGroup())
            self.pathVisualPub = self.create_publisher(Marker, "/visual/path", 1,
                                                       callback_group=MutuallyExclusiveCallbackGroup())
            self.savedwaypointsVisualPub = self.create_publisher(Marker, "/visual/saved_waypoints", 1,
                                                                 callback_group=MutuallyExclusiveCallbackGroup())
            self.circleVisualPub = self.create_publisher(Marker, "/visual/circle", 1,
                                                         callback_group=MutuallyExclusiveCallbackGroup())
        # parameters
        self.add_on_set_parameters_callback(self.parameters_cb)

    def map_callback(self, map_msg):
        cones = [cone for cone in map_msg.track]
        if self.rviz_visualization:
            cones_markers_thread = Thread(target=publish_cones_markers, args=(self, cones))
            cones_markers_thread.daemon = True
            cones_markers_thread.start()
        self.planner.update_map(cones)

    def odom_callback(self, odom_msg):
        if self.rviz_visualization:
            tf_thread = Thread(target=publish_tf, args=(self, odom_msg))
            tf_thread.daemon = True
            tf_thread.start()
            ego_marker_thread = Thread(target=publish_ego_marker, args=(self,))
            ego_marker_thread.daemon = True
            ego_marker_thread.start()
        egopose = np.array([0.0, 0.0, 0.0])
        egopose[0] = odom_msg.pose.pose.position.x  # x
        egopose[1] = odom_msg.pose.pose.position.y  # y
        egopose[2] = yaw_from_quaternion(odom_msg.pose.pose.orientation)
        self.planner.update_egopose(egopose)

    def timer_callback(self):
        waypoints = self.planner.plan()
        off_track_msg = Bool()
        off_track_msg.data = self.planner.recovery_mode
        self.offtrack_pub.publish(off_track_msg)
        if waypoints:
            if self.rviz_visualization:
                path_markers_thread = Thread(target=publish_path_markers, args=(self, waypoints))
                path_markers_thread.daemon = True
                path_markers_thread.start()
            self.publish_waypoints(waypoints)

    def publish_waypoints(self, waypoints):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        if waypoints:
            for waypoint in waypoints:
                wp = Waypoint()
                wp.x = waypoint[0]
                wp.y = waypoint[1]
                path.waypoints.append(wp)
        self.route_pub.publish(path)

    def handle_parameters(self):
        member_variables = self.planner.cfg.__dict__.keys()
        for param in member_variables:
            self.declare_parameter(param, value=self.planner.cfg.__dict__[param])
            self.planner.cfg.__dict__[param] = self.get_parameter(param).value

    def parameters_cb(self, params):
        for param in params:
            if param.name in self.planner.cfg.__dict__.keys():
                old_value = self.planner.cfg.__dict__[param.name]
                self.planner.cfg.__dict__[param.name] = param.value
                print(f"{param.name} changed from {old_value} to {param.value}")
            elif param.name == 'waypointsPublishInterval':
                old_value = self.waypointsPublishInterval
                self.destroy_timer(self.wp_pub_timer)
                self.waypointsPublishInterval = param.value
                self.wp_pub_timer = self.create_timer(self.waypointsPublishInterval, self.timer_callback,
                                                      callback_group=MutuallyExclusiveCallbackGroup())
                print(f"Waypoints publish interval changed from {old_value} to {self.waypointsPublishInterval}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PathPlanner("planner_node")
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
