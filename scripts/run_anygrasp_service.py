"""Run the remote AnyGrasp HTTP service used by ScaleBench."""

from __future__ import annotations

import argparse
import math
import threading
from typing import Any

import numpy as np
from flask import Flask, jsonify, request
from gsnet import create_detector

PROTOCOL_VERSION = 3


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint_path", required=True)
    parser.add_argument("--max_gripper_width", type=float, default=0.10)
    parser.add_argument("--gripper_height", type=float, default=0.03)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    return parser


def _depth_to_point_cloud(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    scale: float,
    depth_trunc_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return camera-frame points and the image mask defining their order."""

    image = np.asarray(depth, dtype=np.float32)
    if image.ndim != 2:
        raise ValueError(f"depth must have shape (H, W), got {image.shape}")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if scale <= 0.0 or depth_trunc_m <= 0.0:
        raise ValueError("scale and depth_trunc must be positive")

    height, width = image.shape
    columns, rows = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    z = image / scale
    valid = np.isfinite(z) & (z > 0.0) & (z < depth_trunc_m)
    points = np.stack(
        (
            (columns - cx) / fx * z,
            (rows - cy) / fy * z,
            z,
        ),
        axis=-1,
    )
    return points[valid].astype(np.float32), valid


def _target_region(
    valid_depth_mask: np.ndarray,
    target_pixel_indices: list[int],
) -> np.ndarray:
    """Map row-major image indices onto the flattened valid point cloud."""

    if not target_pixel_indices:
        raise ValueError("target_pixel_indices must not be empty")
    if any(type(index) is not int for index in target_pixel_indices):
        raise TypeError("target_pixel_indices must contain integers")
    pixel_count = int(valid_depth_mask.size)
    indices = np.asarray(target_pixel_indices, dtype=np.int64)
    if int(indices.min()) < 0 or int(indices.max()) >= pixel_count:
        raise ValueError("target_pixel_indices contains an out-of-range index")
    target_image_mask = np.zeros(pixel_count, dtype=np.bool_)
    target_image_mask[indices] = True
    region = target_image_mask.reshape(valid_depth_mask.shape)[valid_depth_mask]
    if not region.any():
        raise ValueError("target pixels contain no valid scene depth")
    return region


def _aligned_point_colors(
    colors: Any,
    valid_depth_mask: np.ndarray,
) -> np.ndarray:
    """Validate aligned RGB and return colors for the valid XYZ points."""

    image = np.asarray(colors)
    expected_shape = (*valid_depth_mask.shape, 3)
    if image.shape != expected_shape:
        raise ValueError(f"colors must have shape {expected_shape}, got {image.shape}")
    if not np.issubdtype(image.dtype, np.integer):
        raise TypeError("colors must contain integer RGB values")
    if np.any(image < 0) or np.any(image > 255):
        raise ValueError("colors must contain values in [0, 255]")
    return image.astype(np.float32)[valid_depth_mask] / 255.0


def _serialize_grasp(grasp: Any) -> dict[str, object]:
    """Serialize one grasp from the proprietary SDK's untyped object."""

    translation = np.asarray(grasp.translation, dtype=np.float64)
    rotation = np.asarray(grasp.rotation_matrix, dtype=np.float64)
    depth_m = float(grasp.depth)
    tip_position = translation + depth_m * rotation[:, 0]
    return {
        "score": float(grasp.score),
        "width": float(grasp.width),
        "height": float(grasp.height),
        "depth": depth_m,
        "translation": translation.tolist(),
        "rotation_matrix": rotation.tolist(),
        "tip_position": tip_position.tolist(),
        "object_id": int(grasp.object_id),
    }


def create_app(detector: Any) -> Flask:
    """Bind Flask routes to the proprietary SDK detector instance."""

    app = Flask(__name__)
    inference_lock = threading.Lock()

    @app.get("/health")
    def health():
        return jsonify(
            {
                "service": "AnyGrasp",
                "status": "ok",
                "protocol_version": PROTOCOL_VERSION,
                "capabilities": ["target_pixel_indices", "aligned_rgb"],
            }
        )

    @app.post("/process")
    def process():
        try:
            data = request.get_json(force=True)
            if not isinstance(data, dict):
                raise TypeError("request JSON must be an object")

            depth = np.asarray(data["depths"], dtype=np.float32)
            points, valid_depth_mask = _depth_to_point_cloud(
                depth,
                float(data["fx"]),
                float(data["fy"]),
                float(data["cx"]),
                float(data["cy"]),
                float(data.get("scale", 1000.0)),
                float(data.get("depth_trunc", 2.0)),
            )
            if not len(points):
                raise ValueError("depth image contains no valid points")
            point_colors = _aligned_point_colors(
                data["colors"],
                valid_depth_mask,
            )

            # New ScaleBench clients identify target pixels while keeping the
            # full scene for collision detection. Legacy clients omit the
            # field and intentionally treat every supplied point as the region.
            if "target_pixel_indices" in data:
                target_pixel_indices = data["target_pixel_indices"]
                if not isinstance(target_pixel_indices, list):
                    raise TypeError("target_pixel_indices must be a list")
                region_steering = _target_region(
                    valid_depth_mask,
                    target_pixel_indices,
                )
                input_mode = "scene_with_target_region"
            else:
                region_steering = np.ones(len(points), dtype=np.bool_)
                input_mode = "legacy_all_points_region"

            dense_grasp = bool(data.get("dense_grasp", False))
            optional_params = {
                "dense_grasp": dense_grasp,
                "collision_detection": bool(data.get("collision_detection", True)),
                "region_steering": region_steering,
                # The two steered pools provide a vector; the broad pool omits
                # it to request the SDK's documented unrestricted mode.
                "approach_steering": data.get("approach_steering"),
                "approach_thresh": float(data.get("approach_thresh", math.pi)),
            }
            with inference_lock:
                grasps = detector.get_grasp(points, optional_params)

            if grasps is None:
                grasp_list = []
            else:
                if not dense_grasp:
                    grasps = grasps.nms()
                grasps = grasps.sort_by_score()
                top_k = int(data.get("top_k", 20))
                if top_k <= 0:
                    raise ValueError("top_k must be positive")
                grasps = grasps[: min(top_k, len(grasps))]
                grasp_list = [_serialize_grasp(grasp) for grasp in grasps]

            return jsonify(
                {
                    "protocol_version": PROTOCOL_VERSION,
                    "input_mode": input_mode,
                    "num_points": len(points),
                    "num_color_points": len(point_colors),
                    "num_target_points": int(region_steering.sum()),
                    "num_grasps": len(grasp_list),
                    "grasp_groups": grasp_list,
                }
            )
        except (KeyError, TypeError, ValueError) as error:
            return jsonify({"error": str(error), "grasp_groups": []}), 400
        except Exception as error:
            app.logger.exception("AnyGrasp inference failed")
            return jsonify({"error": str(error)}), 500

    return app


def main() -> None:
    args = _build_parser().parse_args()
    detector = create_detector(args)
    if detector is None:
        raise RuntimeError("failed to create AnyGrasp detector")
    create_app(detector).run(
        host=args.host,
        port=args.port,
        threaded=True,
    )


if __name__ == "__main__":
    main()
