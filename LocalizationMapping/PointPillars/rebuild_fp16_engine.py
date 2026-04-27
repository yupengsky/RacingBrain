#!/usr/bin/env python3
import argparse
from pathlib import Path

import tensorrt as trt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build a TensorRT FP16 PointPillars engine from ONNX."
    )
    parser.add_argument("--onnx", required=True, help="Path to the ONNX model.")
    parser.add_argument("--output", required=True, help="Path to the output .engine file.")
    parser.add_argument("--min-voxels", type=int, default=512, help="Minimum voxel profile.")
    parser.add_argument("--opt-voxels", type=int, default=12000, help="Optimal voxel profile.")
    parser.add_argument("--max-voxels", type=int, default=40000, help="Maximum voxel profile.")
    parser.add_argument(
        "--workspace-gib",
        type=float,
        default=2.0,
        help="TensorRT workspace size in GiB.",
    )
    return parser.parse_args()


def build_engine(onnx_path: Path, output_path: Path, min_voxels: int, opt_voxels: int, max_voxels: int,
                 workspace_gib: float):
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, logger)
    config = builder.create_builder_config()

    model_bytes = onnx_path.read_bytes()
    if not parser.parse(model_bytes):
        raise RuntimeError(
            "Failed to parse ONNX:\n" +
            "\n".join(str(parser.get_error(i)) for i in range(parser.num_errors))
        )

    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE,
        int(workspace_gib * (1024 ** 3)),
    )
    if builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)

    profile = builder.create_optimization_profile()
    profile.set_shape("voxels", (min_voxels, 32, 4), (opt_voxels, 32, 4), (max_voxels, 32, 4))
    profile.set_shape("voxel_coords", (min_voxels, 4), (opt_voxels, 4), (max_voxels, 4))
    profile.set_shape("voxel_num_points", (min_voxels,), (opt_voxels,), (max_voxels,))
    config.add_optimization_profile(profile)

    engine_bytes = builder.build_serialized_network(network, config)
    if engine_bytes is None:
        raise RuntimeError("TensorRT failed to build the serialized engine.")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(engine_bytes)


def main():
    args = parse_args()
    onnx_path = Path(args.onnx).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()

    if not onnx_path.exists():
        raise FileNotFoundError(f"ONNX file not found: {onnx_path}")

    build_engine(
        onnx_path=onnx_path,
        output_path=output_path,
        min_voxels=args.min_voxels,
        opt_voxels=args.opt_voxels,
        max_voxels=args.max_voxels,
        workspace_gib=args.workspace_gib,
    )
    print(f"Built FP16 engine: {output_path}")


if __name__ == "__main__":
    main()
