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
            "enable_perception": _bool_text(args.perception),
            "enable_mapping": _bool_text(args.mapping),
            "enable_planning": _bool_text(args.planning),
            "enable_health": _bool_text(args.health),
            "health_period": args.health_period,
            "health_stale_timeout": args.health_stale_timeout,
        },
    )


def add_mapping_args(parser):
    parser.add_argument("--track", default="acceleration")
    parser.add_argument("--lidar-backend", default="pointpillars", choices=("pointpillars", "cluster", "auto"))
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--eval-debug", action="store_true")
    parser.add_argument("--no-perception", dest="perception", action="store_false")
    parser.add_argument("--no-mapping", dest="mapping", action="store_false")
    parser.add_argument("--no-health", dest="health", action="store_false")
    parser.add_argument("--health-period", type=float, default=1.0)
    parser.add_argument("--health-stale-timeout", type=float, default=3.0)
    parser.add_argument("--planning", action="store_true")
    parser.set_defaults(perception=True, mapping=True, health=True)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="racingbrain")
    subparsers = parser.add_subparsers(dest="command", required=True)

    mapping = subparsers.add_parser("mapping", help="Run real-time localization and mapping.")
    add_mapping_args(mapping)
    mapping.set_defaults(func=run_mapping)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
