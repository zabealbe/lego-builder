#!/bin/python
import os
import cv2
import json
import copy
import numpy as np
from pyquaternion import Quaternion

MODELS_PATH = "models"


def get_model_size(model_name):
    model_json_path = os.path.join(MODELS_PATH, model_name, "model.json")
    # make path absolute
    model_json_path = os.path.abspath(model_json_path)
    # check path exists
    if not os.path.exists(model_json_path):
        raise FileNotFoundError(f"Model file {model_json_path} not found")

    model_json = json.load(open(model_json_path, "r"))
    corners = np.array(model_json["corners"])

    size_x = np.max(corners[:, 0]) - np.min(corners[:, 0])
    size_y = np.max(corners[:, 1]) - np.min(corners[:, 1])
    size_z = np.max(corners[:, 2]) - np.min(corners[:, 2])

    return size_x, size_y, size_z


def min_area_crop(depth, thresh, normalize=True):
    depth_max = depth.max()
    depth_min = depth.min()

    contour = np.where(depth < thresh, 255, 0).astype(np.uint8).squeeze()

    contour, _ = cv2.findContours(contour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Check if there is a contour
    if len(contour) == 0:
        return None, 0, None

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

    return rect, angle/90*(np.pi/2), crop


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


def rob2cam(pose):
    """
        rob2cam:
        convert robotic arm pose convention to camera pose convention
    """
    camera_pose = copy.deepcopy(pose)
    orientation = Quaternion(
        x=camera_pose.orientation.x,
        y=camera_pose.orientation.y,
        z=camera_pose.orientation.z,
        w=camera_pose.orientation.w)
    # orientation uses robotic axes convention, convert to standard camera axes
    orientation = np.dot(orientation.rotation_matrix, np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))#np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]))
    camera_pose.orientation = Quaternion(matrix=orientation)
    return camera_pose


def projectPoints(points, camera_pos, camera_rot, camera_matrix, dist_coeffs):
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
    (image_points, _) = cv2.projectPoints(points, camera_rot, camera_pos, camera_matrix, dist_coeffs)
    return image_points
