import os
from configparser import ConfigParser
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def find_path_config():
    installed_config = Path(get_package_share_directory('run_perception')) / 'config' / 'hardcoded_paths.ini'
    if installed_config.exists():
        return installed_config

    env_config = os.environ.get('DRD26_PATH_CONFIG')
    if env_config and Path(env_config).exists():
        return Path(env_config)

    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'config' / 'hardcoded_paths.ini'
        if candidate.exists():
            return candidate
        candidate = parent / 'LocalizationMapping' / 'config' / 'hardcoded_paths.ini'
        if candidate.exists():
            return candidate
    candidate = Path.cwd() / 'config' / 'hardcoded_paths.ini'
    if candidate.exists():
        return candidate
    candidate = Path.cwd() / 'LocalizationMapping' / 'config' / 'hardcoded_paths.ini'
    if candidate.exists():
        return candidate
    raise FileNotFoundError('LocalizationMapping/config/hardcoded_paths.ini not found')


def load_hardcoded_paths():
    config_path = find_path_config()
    parser = ConfigParser()
    parser.read(config_path, encoding='utf-8')
    return parser, config_path


def configured_path(parser, section, option):
    value = parser.get(section, option)
    path = Path(value)
    if not path.exists():
        print(f"Warning: configured path does not exist: {path}")
    return str(path)


def _perform(context, value):
    if hasattr(value, "perform"):
        return value.perform(context)
    return str(value)


def build_fusion_launch(context, eval_debug, health_metrics, fusion_calibration_file):
    try:
        fusion_pkg_share = get_package_share_directory('fs_fusion_box')
        fusion_launch_path = os.path.join(fusion_pkg_share, 'launch', 'fusion_box.launch.py')
        default_fusion_calibration = os.path.join(fusion_pkg_share, 'config', 'calibration.yaml')
        calibration_file = _perform(context, fusion_calibration_file).strip() or default_fusion_calibration
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fusion_launch_path),
                launch_arguments={
                    'eval_debug': _perform(context, eval_debug),
                    'health_metrics': _perform(context, health_metrics),
                    'calibration_file': calibration_file,
                }.items()
            )
        ]
    except Exception as e:
        print(f"Error: 找不到 fs_fusion_box 功能包。请确保它已编译并 source。错误信息: {e}")
        return []

def generate_launch_description():
    eval_debug = LaunchConfiguration('eval_debug')
    health_metrics = LaunchConfiguration('health_metrics')
    camera_topic = LaunchConfiguration('camera_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    fusion_calibration_file = LaunchConfiguration('fusion_calibration_file')
    lidar_backend = LaunchConfiguration('lidar_backend')
    
    # ==========================================
    # 1. 配置 2D YOLO 节点
    # ==========================================
    paths, paths_file = load_hardcoded_paths()
    model_path = configured_path(paths, 'models', 'yolo_runtime')
    dataset_path = configured_path(paths, 'datasets', 'rosbag_2026_02_05')
    print(f"Using hardcoded paths from: {paths_file}")
    print(f"Loading model from: {model_path}")
    print(f"Default rosbag dataset: {dataset_path}")
    
    yolo_node = Node(
        package='cone_detector',       # 包名
        executable='yolo_detector',    # 可执行文件名
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'image_topic': camera_topic,
            'conf_threshold': 0.5,
            'max_fps': 10.0,
            'evaluation.enable_debug_metrics': ParameterValue(eval_debug, value_type=bool),
            'runtime_health.enable_metrics': ParameterValue(health_metrics, value_type=bool),
        }]
    )

    # ==========================================
    # 2. 配置 3D 雷达检测节点
    # ==========================================
    pointpillars_lidar_node = Node(
        package='trt_cone_detector',
        executable='trt_infer_node',
        name='pointpillars_cone_detector',
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_backend, "' == 'pointpillars'"])),
        parameters=[{
            'input_topic': lidar_topic,
            'output_topic': '/cone_detection_custom',
            'marker_topic': '/detected_cones_markers',
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

    cluster_lidar_node = Node(
        package='test_cone_segmentation',
        executable='cone_segmentation_node',
        name='cone_segmentation_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", lidar_backend, "' == 'cluster'"])),
        parameters=[{
            'input_topic': lidar_topic,
            'use_csf': False,   # 关闭 CSF
            'evaluation.enable_debug_metrics': ParameterValue(eval_debug, value_type=bool),
            'runtime_health.enable_metrics': ParameterValue(health_metrics, value_type=bool),
        }]
    )

    # ==========================================
    # 3. 配置 融合节点 (引用已安装的 fs_fusion_box)
    # ==========================================
    # 只要 fs_fusion_box 编译并 source 过，这里就能找到
    default_fusion_calibration = os.path.join(
        get_package_share_directory('fs_fusion_box'),
        'config',
        'calibration.yaml'
    )

    # ==========================================
    # 4. 组合启动
    # ==========================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'eval_debug',
            default_value='false',
            description='Enable sidecar evaluation metrics publishers in perception nodes.'
        ),
        DeclareLaunchArgument(
            'health_metrics',
            default_value='false',
            description='Enable lightweight runtime health metrics publishers in perception nodes.'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera1/image_raw',
            description='Camera topic for YOLO input.'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/lidar_points',
            description='LiDAR topic for the selected cone-detection backend.'
        ),
        DeclareLaunchArgument(
            'fusion_calibration_file',
            default_value=default_fusion_calibration,
            description='Optional override for the fusion calibration YAML file.'
        ),
        DeclareLaunchArgument(
            'lidar_backend',
            default_value='pointpillars',
            description='LiDAR cone detector backend: pointpillars or cluster.'
        ),

        # 1. 启动 YOLO
        yolo_node,
        
        # 2. 启动 LiDAR 锥桶检测
        pointpillars_lidar_node,
        cluster_lidar_node,
        
        # 3. 延迟 2 秒启动融合 (等待传感器节点就绪)
        TimerAction(
            period=2.0,
            actions=[
                OpaqueFunction(
                    function=build_fusion_launch,
                    kwargs={
                        'eval_debug': eval_debug,
                        'health_metrics': health_metrics,
                        'fusion_calibration_file': fusion_calibration_file,
                    },
                )
            ],
        )
    ])
