"""AprilTag localization without ROS, so it can be tested on images alone.

Transforms are 4x4 matrices named T_a_b: they map points from frame b into frame a.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml


def make_transform(rotation: np.ndarray, translation) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(translation, dtype=float).ravel()
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    rotation = T[:3, :3].T
    return make_transform(rotation, -rotation @ T[:3, 3])


def rpy_deg_to_rotation(rpy_deg) -> np.ndarray:
    """Rotation from roll, pitch, yaw in degrees (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    roll, pitch, yaw = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] with w >= 0."""
    trace = np.trace(R)
    largest = int(np.argmax([R[0, 0], R[1, 1], R[2, 2], trace]))
    q = np.zeros(4)
    if largest == 3:
        q[:] = R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1], 1 + trace
    else:
        i, j, k = largest, (largest + 1) % 3, (largest + 2) % 3
        q[i] = 1 + 2 * R[i, i] - trace
        q[j] = R[j, i] + R[i, j]
        q[k] = R[k, i] + R[i, k]
        q[3] = R[k, j] - R[j, k]
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    cosine = (np.trace(R_a.T @ R_b) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def tag_corners(size_m: float) -> np.ndarray:
    """Corners in the tag frame (x right, y up, z out), in OpenCV's detection order."""
    half = size_m / 2
    return np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


@dataclass(frozen=True)
class TagDefinition:
    tag_id: int
    size_m: float
    T_world_tag: np.ndarray


@dataclass
class LocalizationResult:
    T_world_camera: np.ndarray | None
    status: str  # ok | no_tags | unknown_tags | ambiguous | high_error
    visible_ids: list[int] = field(default_factory=list)
    used_ids: list[int] = field(default_factory=list)
    reprojection_px: float = float("nan")
    ambiguity: float = float("nan")
    corners: list[np.ndarray] = field(default_factory=list)
    corner_ids: np.ndarray | None = None  # IDs in the same order as corners


def load_tag_map(path: str | Path) -> dict[int, TagDefinition]:
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("family") != "tag36h11":
        raise ValueError("Only the tag36h11 family is supported")

    tags: dict[int, TagDefinition] = {}
    for entry in config["tags"]:
        tag_id = int(entry["id"])
        size_m = float(entry["size_m"])
        if tag_id in tags:
            raise ValueError(f"Tag {tag_id} is defined twice")
        if size_m <= 0:
            raise ValueError(f"Tag {tag_id} needs a positive size_m")
        rotation = rpy_deg_to_rotation(entry.get("rpy_deg", [0, 0, 0]))
        tags[tag_id] = TagDefinition(tag_id, size_m, make_transform(rotation, entry.get("xyz", [0, 0, 0])))
    if not tags:
        raise ValueError("The tag map contains no tags")
    return tags


class TagLocalizer:
    def __init__(
        self,
        tags: dict[int, TagDefinition],
        camera_matrix: np.ndarray,
        distortion: np.ndarray,
        max_reprojection_px: float = 3.0,
        max_ambiguity: float = 0.6,
    ):
        self.tags = tags
        self.K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(distortion, dtype=np.float64).ravel()
        self.max_reprojection_px = max_reprojection_px
        self.max_ambiguity = max_ambiguity

        parameters = cv2.aruco.DetectorParameters()
        # Gave about 4x lower median position error than SUBPIX on noisy renders.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def _reprojection_error(self, object_points, image_points, rvec, tvec) -> float:
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.K, self.D)
        return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))

    def _solve_single(self, tag: TagDefinition, image_points, prior):
        object_points = tag_corners(tag.size_m)
        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points, image_points, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        candidates = []
        for rvec, tvec in zip(rvecs, tvecs):
            error = self._reprojection_error(object_points, image_points, rvec, tvec)
            T_camera_tag = make_transform(cv2.Rodrigues(rvec)[0], tvec)
            candidates.append((error, tag.T_world_tag @ invert_transform(T_camera_tag)))
        candidates.sort(key=lambda candidate: candidate[0])

        best_error, best_pose = candidates[0]
        if len(candidates) < 2:
            return best_pose, best_error, 0.0, "ok"

        second_error, second_pose = candidates[1]
        ambiguity = best_error / second_error if second_error > 1e-9 else 1.0
        if ambiguity <= self.max_ambiguity:
            return best_pose, best_error, ambiguity, "ok"
        if prior is None:
            return None, best_error, ambiguity, "ambiguous"

        # A planar tag can flip when seen head-on: keep the solution closest to the last pose.
        best_angle = rotation_angle_deg(best_pose[:3, :3], prior[:3, :3])
        second_angle = rotation_angle_deg(second_pose[:3, :3], prior[:3, :3])
        if second_angle < best_angle:
            return second_pose, second_error, ambiguity, "ok"
        return best_pose, best_error, ambiguity, "ok"

    def _solve_multi(self, observations):
        object_points = np.vstack(
            [
                (self.tags[tag_id].T_world_tag[:3, :3] @ tag_corners(self.tags[tag_id].size_m).T).T
                + self.tags[tag_id].T_world_tag[:3, 3]
                for tag_id in observations
            ]
        )
        image_points = np.vstack(list(observations.values()))
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, self.K, self.D, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None, float("inf")
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, self.K, self.D, rvec, tvec)
        error = self._reprojection_error(object_points, image_points, rvec, tvec)
        T_camera_world = make_transform(cv2.Rodrigues(rvec)[0], tvec)
        return invert_transform(T_camera_world), error

    def localize(self, gray: np.ndarray, prior: np.ndarray | None = None) -> LocalizationResult:
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return LocalizationResult(None, "no_tags")

        visible_ids = [int(tag_id) for tag_id in ids.ravel()]
        observations = {
            tag_id: np.asarray(corner, dtype=np.float64).reshape(4, 2)
            for tag_id, corner in zip(visible_ids, corners)
            if tag_id in self.tags and visible_ids.count(tag_id) == 1  # a repeated ID cannot be trusted
        }
        result = LocalizationResult(None, "unknown_tags", sorted(visible_ids), corners=list(corners), corner_ids=ids)
        if not observations:
            return result

        result.used_ids = sorted(observations)
        if len(observations) == 1:
            tag_id, image_points = next(iter(observations.items()))
            pose, error, ambiguity, status = self._solve_single(self.tags[tag_id], image_points, prior)
            result.ambiguity = ambiguity
            result.reprojection_px = error
            if status != "ok":
                result.status = status
                return result
        else:
            pose, error = self._solve_multi(observations)
            result.ambiguity = 0.0
            result.reprojection_px = error

        if pose is None or error > self.max_reprojection_px:
            result.status = "high_error"
            return result
        result.T_world_camera = pose
        result.status = "ok"
        return result
