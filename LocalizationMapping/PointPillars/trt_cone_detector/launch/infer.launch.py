from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    eval_debug = LaunchConfiguration('eval_debug')
    health_metrics = LaunchConfiguration('health_metrics')

    return LaunchDescription([
        DeclareLaunchArgument(
            'eval_debug',
            default_value='false',
            description='Enable JSON timing metrics for offline evaluation.'
        ),
        DeclareLaunchArgument(
            'health_metrics',
            default_value='false',
            description='Enable lightweight runtime health metrics for online monitoring.'
        ),
        Node(
            package='trt_cone_detector',
            executable='trt_infer_node',
            name='pointpillars_cone_detector',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_topic': '/lidar_points',
                'output_topic': '/cone_detection_custom',
                'marker_topic': '/detected_cones_markers',
                'metrics_topic': '/perception/lidar/evaluation/metrics',
                'score_thresh': 0.25,
                'big_cone_score_thresh': 0.25,
                'nms_thresh': 0.10,
                'max_raw_points': 300000,
                'max_pre_nms': 1024,
                'max_output_boxes': 200,
                'intensity_scale': -1.0,
                'print_latency': ParameterValue(eval_debug, value_type=bool),
                'evaluation.enable_debug_metrics': ParameterValue(eval_debug, value_type=bool),
                'runtime_health.enable_metrics': ParameterValue(health_metrics, value_type=bool),
            }]
        )
    ])
