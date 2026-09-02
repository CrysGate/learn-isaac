"""Small validated HTTP client for the external AnyGrasp inference service."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np

from scale_bench.config.models.grasp import AnyGraspConfig


class AnyGraspServiceError(RuntimeError):
    """An unavailable service or invalid inference response."""


@dataclass(frozen=True, slots=True)
class AnyGraspDetection:
    """One validated GraspNet-frame detection returned by the service."""

    score: float
    width_m: float
    height_m: float
    depth_m: float
    translation_camera_m: np.ndarray
    rotation_camera: np.ndarray
    tip_position_camera_m: np.ndarray
    object_id: int


class AnyGraspClient:
    """Send metric depth images to a model that stays resident on the server."""

    def __init__(self, config: AnyGraspConfig) -> None:
        self._config = config

    def detect(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        intrinsic_matrix_px: np.ndarray,
        target_mask: np.ndarray,
    ) -> tuple[AnyGraspDetection, ...]:
        depth = _validated_depth(depth_m, self._config.depth_trunc_m)
        colors = _validated_rgb(rgb, depth.shape)
        target = np.asarray(target_mask, dtype=np.bool_)
        if target.shape != depth.shape:
            raise AnyGraspServiceError(
                "target mask must have the same shape as the depth image"
            )
        target_pixel_indices = np.flatnonzero(target & (depth > 0.0))
        if not target_pixel_indices.size:
            raise AnyGraspServiceError("target mask contains no valid depth pixels")
        intrinsics = np.asarray(intrinsic_matrix_px, dtype=np.float64)
        if intrinsics.shape != (3, 3) or not np.isfinite(intrinsics).all():
            raise AnyGraspServiceError(
                "camera intrinsic matrix must be finite with shape (3, 3)"
            )
        fx = float(intrinsics[0, 0])
        fy = float(intrinsics[1, 1])
        cx = float(intrinsics[0, 2])
        cy = float(intrinsics[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            raise AnyGraspServiceError("camera focal lengths must be positive")

        payload = {
            # The SDK consumes XYZ only. Protocol v3 also retains aligned RGB
            # so the exact inference input can be inspected as a color cloud.
            "colors": colors.tolist(),
            # Isaac Lab already exposes metric depth. A scale of one avoids a
            # lossy metres -> millimetres -> metres round trip on the server.
            "depths": depth.tolist(),
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "scale": 1.0,
            "depth_trunc": self._config.depth_trunc_m,
            "dense_grasp": self._config.dense_grasp,
            "collision_detection": self._config.collision_detection,
            "top_k": self._config.top_k,
            # Existing servers ignore this extra field and run on the full
            # scene. Updated servers use it as region_steering while retaining
            # all scene points for collision detection.
            "target_pixel_indices": target_pixel_indices.tolist(),
        }
        document = self._post_json("/process", payload)
        groups = document.get("grasp_groups")
        if not isinstance(groups, list):
            raise AnyGraspServiceError(
                "AnyGrasp response field 'grasp_groups' must be a list"
            )
        detections = []
        detection_keys = set()
        for index, group in enumerate(groups):
            try:
                detection = _parse_detection(group)
            except (KeyError, TypeError, ValueError) as error:
                raise AnyGraspServiceError(
                    f"invalid AnyGrasp detection at index {index}: {error}"
                ) from error
            key = (
                round(detection.score, 6),
                round(detection.width_m, 6),
                round(detection.height_m, 6),
                round(detection.depth_m, 6),
                detection.object_id,
                *np.round(detection.translation_camera_m, 6).tolist(),
                *np.round(detection.rotation_camera, 6).reshape(-1).tolist(),
            )
            if key in detection_keys:
                continue
            detection_keys.add(key)
            detections.append(detection)
        return tuple(detections)

    def _post_json(self, route: str, payload: dict[str, Any]) -> dict[str, Any]:
        body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode(
            "utf-8"
        )
        request = Request(
            f"{self._config.service_url}{route}",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=self._config.request_timeout_s) as response:
                response_body = response.read()
        except HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            raise AnyGraspServiceError(
                f"AnyGrasp returned HTTP {error.code}: {detail}"
            ) from error
        except (TimeoutError, URLError) as error:
            raise AnyGraspServiceError(
                f"could not reach AnyGrasp at {self._config.service_url}: {error}"
            ) from error
        try:
            document = json.loads(response_body)
        except json.JSONDecodeError as error:
            raise AnyGraspServiceError("AnyGrasp returned invalid JSON") from error
        if not isinstance(document, dict):
            raise AnyGraspServiceError("AnyGrasp response must be a JSON object")
        return document


def _validated_depth(depth_m: np.ndarray, depth_trunc_m: float) -> np.ndarray:
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise AnyGraspServiceError(
            f"depth image must have shape (H, W) or (H, W, 1), got {depth.shape}"
        )
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < depth_trunc_m)
    return np.where(valid, depth, 0.0).astype(np.float32, copy=False)


def _validated_rgb(rgb: np.ndarray, depth_shape: tuple[int, int]) -> np.ndarray:
    colors = np.asarray(rgb)
    expected_shape = (*depth_shape, 3)
    if colors.shape != expected_shape:
        raise AnyGraspServiceError(
            f"RGB image must have shape {expected_shape}, got {colors.shape}"
        )
    if colors.dtype != np.uint8:
        raise AnyGraspServiceError(
            f"RGB image must use uint8 values, got {colors.dtype}"
        )
    return colors


def _parse_detection(group: Any) -> AnyGraspDetection:
    if not isinstance(group, dict):
        raise TypeError("detection must be an object")
    score = float(group["score"])
    width = float(group["width"])
    height = float(group["height"])
    depth = float(group["depth"])
    object_id = int(group["object_id"])
    translation = _finite_array(group["translation"], (3,), "translation")
    rotation = _finite_array(group["rotation_matrix"], (3, 3), "rotation_matrix")
    if not math.isfinite(score) or not 0.0 <= score <= 1.0:
        raise ValueError("score must be finite and in [0, 1]")
    if not math.isfinite(width) or width < 0.0:
        raise ValueError("width must be finite and non-negative")
    if not math.isfinite(height) or height < 0.0:
        raise ValueError("height must be finite and non-negative")
    if not math.isfinite(depth) or depth < 0.0:
        raise ValueError("depth must be finite and non-negative")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-4):
        raise ValueError("rotation_matrix must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-4):
        raise ValueError("rotation_matrix must have determinant +1")

    computed_tip = translation + depth * rotation[:, 0]
    if "tip_position" in group:
        returned_tip = _finite_array(group["tip_position"], (3,), "tip_position")
        if not np.allclose(returned_tip, computed_tip, atol=1.0e-5):
            raise ValueError("tip_position is inconsistent with translation/depth")
    return AnyGraspDetection(
        score=score,
        width_m=width,
        height_m=height,
        depth_m=depth,
        translation_camera_m=translation,
        rotation_camera=rotation,
        tip_position_camera_m=computed_tip,
        object_id=object_id,
    )


def _finite_array(value: Any, shape: tuple[int, ...], field_name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{field_name} must be finite with shape {shape}")
    return array


__all__ = [
    "AnyGraspClient",
    "AnyGraspDetection",
    "AnyGraspServiceError",
]
