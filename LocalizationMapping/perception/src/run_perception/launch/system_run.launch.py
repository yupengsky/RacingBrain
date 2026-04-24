import ctypes
import os
from configparser import ConfigParser
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_directory


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


def _as_bool(context, value):
    return _perform(context, value).strip().lower() in {'1', 'true', 'yes', 'on'}


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


def has_ros_executable(package, executable):
    try:
        prefix = Path(get_package_prefix(package))
    except Exception:
        return False
    return (prefix / 'lib' / package / executable).exists()


def cuda_runtime_available():
    try:
        cudart = ctypes.CDLL('libcudart.so')
        device_count = ctypes.c_int(0)
        status = cudart.cudaGetDeviceCount(ctypes.byref(device_count))
        if status != 0:
            message = 'unknown error'
            try:
                cudart.cudaGetErrorString.restype = ctypes.c_char_p
                raw = cudart.cudaGetErrorString(status)
                if raw:
                    message = raw.decode('utf-8', errors='replace')
            except Exception:
                pass
            print(
                'Warning: PointPillars CUDA runtime unavailable: '
                f'cudaGetDeviceCount={status} ({message}).'
            )
            return False
        if device_count.value <= 0:
            print('Warning: PointPillars CUDA runtime unavailable: no CUDA devices found.')
            return False
        return True
    except Exception as exc:
        print(f'Warning: PointPillars CUDA runtime check failed: {exc}')
        return False


def pointpillars_node(lidar_topic, output_topic, metrics_topic, eval_debug, health_metrics, marker_topic='/detected_cones_markers'):
    return Node(
        package='trt_cone_detector',
        executable='trt_infer_node',
        name='pointpillars_cone_detector',
        output='screen',
        parameters=[{
            'input_topic': lidar_topic,
            'output_topic': output_topic,
            'marker_topic': marker_topic,
            'metrics_topic': metrics_topic,
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


def cluster_node(lidar_topic, output_topic, metrics_topic, eval_debug, health_metrics):
    return Node(
        package='test_cone_segmentation',
        executable='cone_segmentation_node',
        name='cone_segmentation_node',
        output='screen',
        parameters=[{
            'input_topic': lidar_topic,
            'output_topic': output_topic,
            'metrics_topic': metrics_topic,
            'use_csf': False,
            'evaluation.enable_debug_metrics': ParameterValue(eval_debug, value_type=bool),
            'runtime_health.enable_metrics': ParameterValue(health_metrics, value_type=bool),
        }]
    )


def build_lidar_launch(context, eval_debug, health_metrics, lidar_topic, lidar_backend):
    backend = _perform(context, lidar_backend).strip().lower()
    eval_debug_value = _as_bool(context, eval_debug)
    health_metrics_value = _as_bool(context, health_metrics)
    lidar_topic_value = _perform(context, lidar_topic)
    pointpillars_executable_available = False
    pointpillars_runtime_available = False
    if backend in {'pointpillars', 'auto'}:
        pointpillars_executable_available = has_ros_executable('trt_cone_detector', 'trt_infer_node')
        pointpillars_runtime_available = pointpillars_executable_available and cuda_runtime_available()
    pointpillars_available = pointpillars_executable_available and pointpillars_runtime_available

    generic_output = '/cone_detection_custom'
    generic_metrics = '/perception/lidar/evaluation/metrics'
    pointpillars_output = '/perception/lidar/pointpillars/cones'
    pointpillars_metrics = '/perception/lidar/pointpillars/evaluation/metrics'
    cluster_output = '/perception/lidar/cluster/cones'
    cluster_metrics = '/perception/lidar/cluster/evaluation/metrics'

    if backend == 'cluster':
        return [
            cluster_node(lidar_topic_value, generic_output, generic_metrics, eval_debug_value, health_metrics_value)
        ]

    if backend == 'pointpillars' and pointpillars_available:
        return [
            pointpillars_node(lidar_topic_value, generic_output, generic_metrics, eval_debug_value, health_metrics_value)
        ]

    if backend not in {'pointpillars', 'auto'}:
        print(f"Warning: unsupported lidar_backend={backend}; falling back to cluster.")

    if backend == 'pointpillars' and not pointpillars_executable_available:
        print("Warning: PointPillars executable is unavailable; launching cluster through the arbiter fallback path.")
    elif backend == 'pointpillars' and not pointpillars_runtime_available:
        print("Warning: PointPillars CUDA runtime is unavailable; launching cluster through the arbiter fallback path.")

    actions = [
        cluster_node(lidar_topic_value, cluster_output, cluster_metrics, eval_debug_value, health_metrics_value),
        Node(
            package='racingbrain',
            executable='lidar_backend_arbiter',
            name='lidar_backend_arbiter',
            output='screen',
            parameters=[{
                'mode': 'auto',
                'preferred_backend': 'pointpillars',
                'fallback_backend': 'cluster',
                'learning_backend_enabled': ParameterValue(pointpillars_available, value_type=bool),
                'primary_topic': pointpillars_output,
                'fallback_topic': cluster_output,
                'output_topic': generic_output,
                'primary_metrics_topic': pointpillars_metrics,
                'fallback_metrics_topic': cluster_metrics,
                'output_metrics_topic': generic_metrics,
                'backend_stale_timeout_sec': 3.0,
                'fusion_consistency_min': 0.45,
                'fusion_drift_warn': 0.70,
            }]
        )
    ]
    if pointpillars_available:
        actions.insert(
            0,
            pointpillars_node(
                lidar_topic_value,
                pointpillars_output,
                pointpillars_metrics,
                eval_debug_value,
                health_metrics_value,
                marker_topic='/perception/lidar/pointpillars/markers',
            )
        )
    return actions

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
            description='LiDAR cone detector backend: pointpillars, cluster, or auto.'
        ),

        # 1. 启动 YOLO
        yolo_node,
        
        # 2. 启动 LiDAR 锥桶检测或自动仲裁
        OpaqueFunction(
            function=build_lidar_launch,
            kwargs={
                'eval_debug': eval_debug,
                'health_metrics': health_metrics,
                'lidar_topic': lidar_topic,
                'lidar_backend': lidar_backend,
            },
        ),
        
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
