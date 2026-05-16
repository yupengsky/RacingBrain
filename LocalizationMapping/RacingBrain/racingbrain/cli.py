import argparse
import subprocess
import sys


def _bool_text(value):
    return "true" if value else "false"


def _launch(package, launch_file, launch_args):
    cmd = ["ros2", "launch", package, launch_file]
    cmd.extend(f"{key}:={value}" for key, value in launch_args.items() if value is not None)
    return subprocess.call(cmd)


def run_mapping(args):
    return _launch(
        "racingbrain",
        "localization_mapping.launch.py",
        {
            "track": args.track,
            "rviz": _bool_text(args.rviz),
            "eval_debug": _bool_text(args.eval_debug),
            "lidar_backend": args.lidar_backend,
            "fusion_mode": args.fusion_mode,
            "enable_perception": _bool_text(args.perception),
            "enable_mapping": _bool_text(args.mapping),
            "enable_planning": _bool_text(args.planning),
            "enable_health": _bool_text(args.health),
            "mapping_gate": _bool_text(args.mapping_gate),
            "health_period": args.health_period,
            "health_stale_timeout": args.health_stale_timeout,
            "health_expected_perception": _bool_text(args.health_expected_perception),
        },
    )


def run_lio_eval(args):
    return _launch(
        "racingbrain",
        "lio_dataset_eval.launch.py",
        {
            "run_simple_lio": _bool_text(args.run_simple_lio),
            "run_pointcloud_adapter": _bool_text(args.pointcloud_adapter),
            "run_imu_adapter": _bool_text(args.imu_adapter),
            "run_error_eval": _bool_text(args.error_eval),
            "input_cloud_topic": args.input_cloud_topic,
            "adapted_cloud_topic": args.adapted_cloud_topic,
            "gnss_topic": args.gnss_topic,
            "input_imu_topic": args.input_imu_topic,
            "imu_topic": args.imu_topic,
            "lio_odom_topic": args.lio_odom_topic,
            "output_dir": args.output_dir,
            "lidar_frame": args.lidar_frame,
            "n_scan": args.n_scan,
            "scan_period_sec": args.scan_period_sec,
            "imu_gyro_scale": args.imu_gyro_scale,
        },
    )


def add_mapping_args(parser):
    parser.add_argument("--track", default="acceleration")
    parser.add_argument("--lidar-backend", default="pointpillars", choices=("pointpillars", "cluster", "auto"))
    parser.add_argument("--fusion-mode", default="camera_lidar", choices=("camera_lidar", "lidar_only"))
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--eval-debug", action="store_true")
    parser.add_argument("--no-perception", dest="perception", action="store_false")
    parser.add_argument("--no-mapping", dest="mapping", action="store_false")
    parser.add_argument("--no-health", dest="health", action="store_false")
    parser.add_argument("--no-mapping-gate", dest="mapping_gate", action="store_false")
    parser.add_argument("--no-health-expected-perception", dest="health_expected_perception", action="store_false")
    parser.add_argument("--health-period", type=float, default=1.0)
    parser.add_argument("--health-stale-timeout", type=float, default=3.0)
    parser.add_argument("--planning", action="store_true")
    parser.set_defaults(
        perception=True,
        mapping=True,
        health=True,
        mapping_gate=True,
        health_expected_perception=True,
    )


def add_lio_eval_args(parser):
    parser.add_argument("--no-run-simple-lio", dest="run_simple_lio", action="store_false")
    parser.add_argument("--no-pointcloud-adapter", dest="pointcloud_adapter", action="store_false")
    parser.add_argument("--no-imu-adapter", dest="imu_adapter", action="store_false")
    parser.add_argument("--no-error-eval", dest="error_eval", action="store_false")
    parser.add_argument("--input-cloud-topic", default="/lidar_points")
    parser.add_argument("--adapted-cloud-topic", default="/points")
    parser.add_argument("--gnss-topic", default="/gongji_gnss_ins_64")
    parser.add_argument("--input-imu-topic", default="/imu")
    parser.add_argument("--imu-topic", default="/imu_lio")
    parser.add_argument("--lio-odom-topic", default="/racingbrain/simple_lio/odometry")
    parser.add_argument("--output-dir", default="log/benchmark/lio_gnss/latest")
    parser.add_argument("--lidar-frame", default="base_link")
    parser.add_argument("--n-scan", default="64")
    parser.add_argument("--scan-period-sec", default="0.1")
    parser.add_argument("--imu-gyro-scale", default="0.04348764102608839")
    parser.set_defaults(run_simple_lio=True, pointcloud_adapter=False, imu_adapter=False, error_eval=True)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="racingbrain")
    subparsers = parser.add_subparsers(dest="command", required=True)

    mapping = subparsers.add_parser("mapping", help="Run real-time localization and mapping.")
    add_mapping_args(mapping)
    mapping.set_defaults(func=run_mapping)

    lio_eval = subparsers.add_parser("lio-eval", help="Run Simple LIO and the GNSS/LIO evaluator.")
    add_lio_eval_args(lio_eval)
    lio_eval.set_defaults(func=run_lio_eval)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
