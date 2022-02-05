import os
import cv2
import json
import numpy as np
from pyquaternion import Quaternion

MODELS_PATH = "../models"


def get_model_width(model_name):
    model_json_path = os.path.join(MODELS_PATH, model_name, "model.json")
    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = np.max(corners[:, 0]) - np.min(corners[:, 0])
    size_y = np.max(corners[:, 1]) - np.min(corners[:, 1])
    size_z = np.max(corners[:, 2]) - np.min(corners[:, 2])

    return size_x, size_y, size_z


def min_area_crop(depth, normalize=True):
    depth_max = depth.max()
    depth_min = depth.min()

    contour = np.where(depth < depth_max - 0.005, 255, 0).astype(np.uint8).squeeze()

    # cv2.imshow("crop_contour", crop_contour)
    contour, _ = cv2.findContours(contour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Check if there is a contour
    if len(contour) == 0:
        return 0, None

    # find the contour with the largest area
    contour = contour[np.argmax([len(x) for x in contour], axis=0)]

    # find the smallest rotated rectangle that contains the contour
    rect = cv2.minAreaRect(contour)
    angle = rect[2]
    M = cv2.getRotationMatrix2D(rect[0], angle, 1)
    # pad the image to make it square
    h, w = depth.shape
    pad = np.abs(h - w)
    if h > w:
        depth = np.pad(depth, ((0, 0), (0, pad)), mode="constant", constant_values=depth_max)
    else:
        depth = np.pad(depth, ((0, pad), (0, 0)), mode="constant", constant_values=depth_max)

    # rotate the depth by the angle
    crop = cv2.warpAffine(
        depth,
        M,
        (depth.shape[1], depth.shape[0]),
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=float(depth_max))

    box = cv2.boxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]), M))[0]
    pts[pts < 0] = 0

    crop = crop[pts[1][1]:pts[0][1], pts[1][0]:pts[2][0]]

    if normalize:
        crop -= depth_min
        crop /= depth_max - depth_min

    return angle/90*(np.pi/2), crop


def get_facing_direction(quat):
    axis_z = np.array([0, 0, 1])
    new_axis = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis, axis_z), -1.0, 1.0)) % np.pi
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return "up"
    elif angle < np.pi / 3 * 2 * 1.2:
        i = np.argmax(np.abs(new_axis[:2]))
        print(new_axis)
        if i == 0:  # x axis
            if new_axis[i] > 0:
                return "north"
            else:
                return "south"
        if i == 1:  # y axis
            if new_axis[i] > 0:
                return "west"
            else:
                return "east"
    else:
        return "down"


def get_axes(quat):
    axes = np.ones((3, 4), dtype=np.float64)
    axes[:, :3] = np.array([
        [.1, 0, 0],
        [0, .1, 0],
        [0, 0, .1]])
    axes = np.dot(quat.rotation_matrix, axes.T)[:3, :]
    return axes


def projectPoints(points, camera_pos, camera_rot, camera_matrix, dist_coeffs, camera_axis="default"):
    """Project 3D points onto camera 2D plain

    Params:
        points: 3D points in the world coordinate frame.
        camera_matrix: camera matrix.
        dist_coeffs: distortion coefficients.
        camera_pos: camera position vector in the world coordinate frame.
        camera_rot: camera rotation matrix in the world coordinate frame.
    Returns:
        2D points in the camera coordinate frame.
    """
    if camera_axis == "default":  # camera_rot uses default convention with z axis pointing out of the screen
        pass
    elif camera_axis == "robotic":  # camera_rot uses robotic axes convention, convert standard camera axes
        camera_rot = np.dot(np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64), camera_rot)
    else:
        raise ValueError("camera_axis must be 'default' or 'robotic'")
    (image_points, _) = cv2.projectPoints(points, camera_rot, camera_pos, camera_matrix, dist_coeffs)
    return image_points
