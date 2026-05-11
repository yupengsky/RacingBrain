from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.duration import Duration
import numpy as np


def publish_tf(node, odom_msg):
    tf_pub = TransformBroadcaster(node)
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = "map"
    t.child_frame_id = "fsds/FSCar"
    if odom_msg is not None:
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        tf_pub.sendTransform(t)


def publish_ego_marker(node):
    ego_marker = Marker()
    ego_marker.header.frame_id = "fsds/FSCar"
    ego_marker.header.stamp = node.get_clock().now().to_msg()
    ego_marker.lifetime = Duration(seconds=1.0).to_msg()
    ego_marker.ns = "ego"
    ego_marker.id = 0
    ego_marker.type = Marker.CUBE
    ego_marker.action = Marker.ADD
    ego_marker.pose.position.x = 0.0
    ego_marker.pose.position.y = 0.0
    ego_marker.pose.position.z = 0.0
    ego_marker.pose.orientation.w = 1.0
    ego_marker.pose.orientation.x = 0.0
    ego_marker.pose.orientation.y = 0.0
    ego_marker.pose.orientation.z = 0.0
    ego_marker.scale.x = 2.0
    ego_marker.scale.y = 1.2
    ego_marker.scale.z = 1.1
    ego_marker.color.a = 0.6
    ego_marker.color.r = 1.0
    ego_marker.color.g = 0.0
    ego_marker.color.b = 0.0
    node.egoVisualPub.publish(ego_marker)


def publish_cones_markers(node, cones):
    cones_markers = MarkerArray()
    for i, cone in enumerate(cones):
        cone_marker = Marker()
        cone_marker.header.frame_id = "map"
        cone_marker.header.stamp = node.get_clock().now().to_msg()
        cone_marker.lifetime = Duration(seconds=1.0).to_msg()
        cone_marker.ns = "cones"
        cone_marker.id = i
        cone_marker.type = Marker.CYLINDER
        cone_marker.action = Marker.ADD
        cone_marker.pose.position.x = cone.x
        cone_marker.pose.position.y = cone.y
        cone_marker.pose.position.z = 0.0
        cone_marker.pose.orientation.w = 1.0
        cone_marker.pose.orientation.x = 0.0
        cone_marker.pose.orientation.y = 0.0
        cone_marker.pose.orientation.z = 0.0
        cone_marker.scale.x = 0.2
        cone_marker.scale.y = 0.2
        cone_marker.scale.z = 0.3
        cone_marker.color.a = 1.0
        cone_marker.color.r = 1.0
        cone_marker.color.g = 0.5
        cone_marker.color.b = 0.0
        cones_markers.markers.append(cone_marker)
    node.conesVisualPub.publish(cones_markers)


def publish_tree_markers(ros_node, nodeList, leafNodes):
    if nodeList is None and leafNodes is None:
        return
    tree_markers = MarkerArray()
    # tree lines marker
    tree_line_marker = Marker()
    tree_line_marker.header.frame_id = "map"
    tree_line_marker.header.stamp = ros_node.get_clock().now().to_msg()
    tree_line_marker.lifetime = Duration(seconds=1.0).to_msg()
    tree_line_marker.ns = "tree"
    tree_line_marker.id = 0
    tree_line_marker.type = Marker.LINE_LIST
    tree_line_marker.action = Marker.ADD
    tree_line_marker.scale.x = 0.03
    tree_line_marker.pose.orientation.w = 1.0
    tree_line_marker.color.a = 0.7
    tree_line_marker.color.g = 0.7
    for node in nodeList:
        if node.parent is not None:
            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.0
            tree_line_marker.points.append(p1)
            p2 = Point()
            p2.x = nodeList[node.parent].x
            p2.y = nodeList[node.parent].y
            p2.z = 0.0
            tree_line_marker.points.append(p2)
    tree_markers.markers.append(tree_line_marker)
    # leaves nodes marker
    leaves_marker = Marker()
    leaves_marker.header.frame_id = "map"
    leaves_marker.header.stamp = ros_node.get_clock().now().to_msg()
    leaves_marker.lifetime = Duration(seconds=1.0).to_msg()
    leaves_marker.ns = "tree-leaves"
    leaves_marker.id = 1
    leaves_marker.type = Marker.SPHERE_LIST
    leaves_marker.action = Marker.ADD
    leaves_marker.pose.orientation.w = 1.0
    leaves_marker.scale.x = 0.15
    leaves_marker.scale.y = 0.15
    leaves_marker.scale.z = 0.15
    leaves_marker.color.a = 0.5
    leaves_marker.color.b = 0.1
    for node in leafNodes:
        p = Point()
        p.x = node.x
        p.y = node.y
        p.z = 0.0
        leaves_marker.points.append(p)
    tree_markers.markers.append(leaves_marker)

    ros_node.treeVisualPub.publish(tree_markers)


def publish_bestbranch_markers(ros_node, bestBranch, type=0):
    bestbranch_marker = Marker()
    bestbranch_marker.header.frame_id = "map"
    bestbranch_marker.header.stamp = ros_node.get_clock().now().to_msg()
    bestbranch_marker.lifetime = Duration(seconds=1.0).to_msg()
    bestbranch_marker.ns = "best-branch"
    bestbranch_marker.id = 0
    bestbranch_marker.type = Marker.LINE_STRIP
    bestbranch_marker.action = Marker.ADD
    bestbranch_marker.pose.orientation.w = 1.0
    if type == 0:
        bestbranch_marker.scale.x = 0.05
        bestbranch_marker.color.a = 0.3
        bestbranch_marker.color.r = 1.0
        bestbranch_marker.color.g = 1.0
    elif type == 1:
        bestbranch_marker.scale.x = 0.07
        bestbranch_marker.color.a = 0.7
        bestbranch_marker.color.b = 1.0
    for i in range(len(bestBranch)):
        p = Point()
        p.x = bestBranch[i].x
        p.y = bestBranch[i].y
        p.z = 0.0
        bestbranch_marker.points.append(p)
    if type == 0:
        ros_node.bestBranchVisualPub.publish(bestbranch_marker)
    elif type == 1:
        ros_node.filteredBranchVisualPub.publish(bestbranch_marker)


def publish_delaunay_markers(node, edges):
    delaunay_marker = Marker()
    delaunay_marker.header.frame_id = "map"
    delaunay_marker.header.stamp = node.get_clock().now().to_msg()
    delaunay_marker.lifetime = Duration(seconds=1.0).to_msg()
    delaunay_marker.ns = "delaunay"
    delaunay_marker.id = 0
    delaunay_marker.type = Marker.LINE_LIST
    delaunay_marker.action = Marker.ADD
    delaunay_marker.pose.orientation.w = 1.0
    delaunay_marker.scale.x = 0.05
    delaunay_marker.color.a = 0.5
    delaunay_marker.color.r = 1.0
    delaunay_marker.color.b = 1.0
    for edge in edges:
        p1 = Point()
        p1.x = edge.x1
        p1.y = edge.y1
        p1.z = 0.0
        delaunay_marker.points.append(p1)
        p2 = Point()
        p2.x = edge.x2
        p2.y = edge.y2
        p2.z = 0.0
        delaunay_marker.points.append(p2)
    node.delaunayLinesVisualPub.publish(delaunay_marker)


def publish_path_markers(node, path):
    path_marker = Marker()
    path_marker.header.frame_id = "map"
    path_marker.header.stamp = node.get_clock().now().to_msg()
    path_marker.lifetime = Duration(seconds=1.0).to_msg()
    path_marker.ns = "path"
    path_marker.id = 0
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation.w = 1.0
    path_marker.scale.x = 0.1
    path_marker.color.a = 1.0
    path_marker.color.r = 1.0
    for i in range(len(path)):
        p = Point()
        p.x = path[i][0]
        p.y = path[i][1]
        p.z = 0.0
        path_marker.points.append(p)
    node.pathVisualPub.publish(path_marker)


def publish_savedwaypoints_markers(node, savedwaypoints):
    savedwaypoints_marker = Marker()
    savedwaypoints_marker.header.frame_id = "map"
    savedwaypoints_marker.header.stamp = node.get_clock().now().to_msg()
    savedwaypoints_marker.lifetime = Duration(seconds=1.0).to_msg()
    savedwaypoints_marker.ns = "saved-waypoints"
    savedwaypoints_marker.id = 0
    savedwaypoints_marker.type = Marker.SPHERE_LIST
    savedwaypoints_marker.action = Marker.ADD
    savedwaypoints_marker.pose.orientation.w = 1.0
    savedwaypoints_marker.scale.x = 0.3
    savedwaypoints_marker.scale.y = 0.3
    savedwaypoints_marker.scale.z = 0.3
    savedwaypoints_marker.color.a = 1.0
    savedwaypoints_marker.color.b = 1.0
    for waypoint in savedwaypoints:
        p = Point()
        p.x = waypoint[0]
        p.y = waypoint[1]
        p.z = 0.0
        savedwaypoints_marker.points.append(p)
    node.savedwaypointsVisualPub.publish(savedwaypoints_marker)


def publish_circle_markers(node, circle):
    circle_marker = Marker()
    circle_marker.header.frame_id = "map"
    circle_marker.header.stamp = node.get_clock().now().to_msg()
    circle_marker.lifetime = Duration(seconds=1.0).to_msg()
    circle_marker.ns = "circle"
    circle_marker.id = 0
    circle_marker.type = Marker.LINE_STRIP
    circle_marker.action = Marker.ADD
    circle_marker.pose.orientation.w = 1.0
    circle_marker.scale.x = 0.05
    circle_marker.color.a = 1.0
    circle_marker.color.r = 1.0
    circle_marker.color.g = 0.75
    circle_marker.color.b = 0.8
    angles = np.linspace(0, 2 * np.pi, 100, endpoint=True)
    for angle in angles:
        p = Point()
        p.x = circle[0] + circle[2] * np.cos(angle)
        p.y = circle[1] + circle[2] * np.sin(angle)
        p.z = 0.0
        circle_marker.points.append(p)
    node.circleVisualPub.publish(circle_marker)
    
