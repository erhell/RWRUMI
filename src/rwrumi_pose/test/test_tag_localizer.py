"""Renders tags at known poses and checks that the localizer recovers them."""

import cv2
import numpy as np
import pytest

from rwrumi_pose.tag_localizer import (
    TagDefinition,
    TagLocalizer,
    invert_transform,
    load_tag_map,
    make_transform,
    rotation_angle_deg,
    rotation_to_quaternion,
    rpy_deg_to_rotation,
    tag_corners,
)

K = np.array([[772.7, 0.0, 480.0], [0.0, 772.7, 270.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)
SIZE = (540, 960)
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def render(tags, T_world_camera, supersample=4):
    """Renders at higher resolution and averages down, which models pixel integration."""
    height, width = SIZE
    K_high = K.copy()
    K_high[:2, :2] *= supersample
    K_high[:2, 2] = (K[:2, 2] + 0.5) * supersample - 0.5
    size_high = (width * supersample, height * supersample)
    image = np.full(size_high[::-1], 235, dtype=np.uint8)
    T_camera_world = invert_transform(T_world_camera)
    rvec = cv2.Rodrigues(T_camera_world[:3, :3])[0]
    for tag in tags:
        cells = 8  # 6 data bits and a 1-bit black border on each side
        marker = cv2.aruco.generateImageMarker(DICTIONARY, tag.tag_id, cells)
        source = np.float32([[-0.5, -0.5], [cells - 0.5, -0.5], [cells - 0.5, cells - 0.5], [-0.5, cells - 0.5]])
        world = (tag.T_world_tag[:3, :3] @ tag_corners(tag.size_m).T).T + tag.T_world_tag[:3, 3]
        target = cv2.projectPoints(world, rvec, T_camera_world[:3, 3], K_high, D)[0].reshape(4, 2).astype(np.float32)

        center = target.mean(axis=0)
        cv2.fillConvexPoly(image, ((target - center) * 1.25 + center).astype(np.int32), 255)
        homography = cv2.getPerspectiveTransform(source, target)
        warped = cv2.warpPerspective(marker, homography, size_high, flags=cv2.INTER_NEAREST)
        mask = cv2.warpPerspective(np.full_like(marker, 255), homography, size_high, flags=cv2.INTER_NEAREST)
        image[mask > 0] = warped[mask > 0]
    return cv2.GaussianBlur(cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA), (0, 0), 0.5)


def camera_pose(xyz, roll_deg=0.0, target=(0.0, 0.0, 0.0)):
    """Optical frame (x right, y down, z forward) at xyz, looking at target."""
    eye = np.asarray(xyz, dtype=float)
    z = np.asarray(target, dtype=float) - eye
    z /= np.linalg.norm(z)
    x = np.cross([0.0, -1.0, 0.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return make_transform(np.column_stack((x, y, z)) @ rpy_deg_to_rotation([0, 0, roll_deg]), eye)


def tag(tag_id, xyz=(0, 0, 0), rpy=(0, 0, 0), size=0.10):
    return TagDefinition(tag_id, size, make_transform(rpy_deg_to_rotation(rpy), xyz))


@pytest.mark.parametrize(
    "xyz, roll",
    [
        ((0.02, 0.17, 0.45), 0),
        ((-0.20, 0.03, 0.42), 10),
        ((0.12, -0.10, 0.60), -30),
    ],
)
def test_single_tag_pose(xyz, roll):
    tags = {7: tag(7)}
    truth = camera_pose(xyz, roll)
    result = TagLocalizer(tags, K, D).localize(render(tags.values(), truth))
    assert result.status == "ok", result
    assert np.linalg.norm(result.T_world_camera[:3, 3] - truth[:3, 3]) < 0.005
    assert rotation_angle_deg(result.T_world_camera[:3, :3], truth[:3, :3]) < 1.0


def test_multiple_tags_pose():
    tags = {1: tag(1, (-0.13, 0, 0)), 2: tag(2, (0.13, 0, 0)), 3: tag(3, (0, 0.14, 0))}
    truth = camera_pose((0.05, -0.08, 0.65), 5, target=(0.0, 0.05, 0.0))
    result = TagLocalizer(tags, K, D).localize(render(tags.values(), truth))
    assert result.status == "ok", result
    assert result.used_ids == [1, 2, 3]
    assert np.linalg.norm(result.T_world_camera[:3, 3] - truth[:3, 3]) < 0.003
    assert rotation_angle_deg(result.T_world_camera[:3, :3], truth[:3, :3]) < 0.5


def test_tag_with_world_offset():
    tags = {5: tag(5, (0.4, -0.2, 0.1), (0, 0, 90))}
    offset = tags[5].T_world_tag
    truth = offset @ camera_pose((0.08, 0.05, 0.5), 10)
    result = TagLocalizer(tags, K, D).localize(render(tags.values(), truth))
    assert result.status == "ok", result
    assert np.linalg.norm(result.T_world_camera[:3, 3] - truth[:3, 3]) < 0.005


def test_unknown_tag_gives_no_pose():
    seen = {11: tag(11)}
    result = TagLocalizer({4: tag(4)}, K, D).localize(render(seen.values(), camera_pose((0.05, 0.05, 0.5))))
    assert result.T_world_camera is None
    assert result.status == "unknown_tags"
    assert result.visible_ids == [11]


def test_empty_image_gives_no_pose():
    result = TagLocalizer({4: tag(4)}, K, D).localize(np.full(SIZE, 200, dtype=np.uint8))
    assert result.T_world_camera is None
    assert result.status == "no_tags"


@pytest.mark.parametrize("rpy", [(0, 0, 0), (180, 0, 0), (0, 180, 0), (0, 0, 180), (35, -70, 120), (179, 2, -91)])
def test_quaternion_matches_rotation(rpy):
    R = rpy_deg_to_rotation(rpy)
    x, y, z, w = rotation_to_quaternion(R)
    rebuilt = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    assert np.allclose(rebuilt, R, atol=1e-7)


def test_load_tag_map(tmp_path):
    path = tmp_path / "map.yaml"
    path.write_text("family: tag36h11\ntags:\n  - {id: 3, size_m: 0.08, xyz: [1, 2, 3], rpy_deg: [0, 0, 90]}\n")
    tags = load_tag_map(path)
    assert tags[3].size_m == pytest.approx(0.08)
    assert np.allclose(tags[3].T_world_tag[:3, 3], [1, 2, 3])


def test_load_tag_map_rejects_duplicate_id(tmp_path):
    path = tmp_path / "map.yaml"
    path.write_text("family: tag36h11\ntags:\n  - {id: 3, size_m: 0.08}\n  - {id: 3, size_m: 0.05}\n")
    with pytest.raises(ValueError):
        load_tag_map(path)
