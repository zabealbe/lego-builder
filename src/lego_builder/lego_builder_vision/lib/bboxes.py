"""
    Calculate 2D bboxes as /camera would see them.
"""
import cv2
import numpy as np
import math


def calculateBBox(points):
    # find the smallest box that contains all points
    # x_min, y_min, x_max, y_max
    bbox = np.array((
        np.min(points[:, 0]),
        np.min(points[:, 1]),
        np.max(points[:, 0]),
        np.max(points[:, 1])), dtype=np.float64).reshape(-1, 2)
    return bbox


def toPixelBBox(yolo_bbox: np.array, image_width, image_height):
    bbox = np.zeros((4), dtype=np.float64)
    bbox += (-yolo_bbox[2], -yolo_bbox[3], yolo_bbox[2], yolo_bbox[3])
    bbox /= 2
    bbox += (yolo_bbox[0], yolo_bbox[1], yolo_bbox[0], yolo_bbox[1])
    bbox *= (image_width, image_height, image_width, image_height)
    return bbox.astype(dtype=np.int32)


def toYoloBBox(bbox: np.array, image_width=1, image_height=1):
    if bbox.shape != (2, 2):
        raise ValueError("bbox must be a 2x2 array")
    bbox = bbox.astype(dtype=np.float64)
    #divide x by image width
    bbox[:, 0] /= image_width
    #divide y by image height
    bbox[:, 1] /= image_height
    #get the center of the box
    center = bbox.mean(axis=0)
    #get the width and height of the box
    width, height = bbox[1, :] - bbox[0, :]
    return np.array((center[0], center[1], width, height), dtype=np.float64)


def toYoloLabels(labels, class_names):
    string = ""
    for clss, bbox in labels:
        string += f"{class_names.index(clss)} {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}\n"
    return string


def calculateBBoxYolo(points, image_width=1, image_height=1):
    bbox = calculateBBox(points)
    return toYoloBBox(bbox, image_width, image_height)
