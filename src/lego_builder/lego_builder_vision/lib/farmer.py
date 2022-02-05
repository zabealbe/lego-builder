#!/bin/python3
import os
import copy
import spawner
import random as rd

import rospy
import json
import cv2
import time
import utils

import numpy as np

import bboxes as bb
import message_filters
from pyquaternion import Quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState

MODELS_PATH = "../models"

OUTPUT_PATH = "dataset/out"


def callback_dummy(message):
    pass

msg_color = None
msg_depth = None
def callback_camera(color, depth):
    global msg_color, msg_depth
    msg_color = color
    msg_depth = depth


def view_pose(image, bboxes, axes):
    for bbox, axis in zip(bboxes, axes):
        cv2.rectangle(image,
                      (round(bbox[0]-bbox[2]/2),
                       round(bbox[1]-bbox[3]/2)),
                      (round(bbox[0]+bbox[2]/2),
                       round(bbox[1]+bbox[3]/2)),
                      (0, 255, 0), 2)
        cv2.line(image,
                 axis[0],
                 axis[1],
                 (0, 0, 255), 2)  # B G R
        cv2.line(image,
                 axis[0],
                 axis[2],
                 (0, 255, 0), 2)
        cv2.line(image,
                 axis[0],
                 axis[3],
                 (255, 0, 0), 2)

    cv2.imshow("bboxes", image)
    cv2.waitKey(1)


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
    print("orientation:", orientation)
    camera_pose.orientation = Quaternion(matrix=orientation)
    return camera_pose


def init(camera_name):
    camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    rospy.wait_for_service("/gazebo/get_model_state")
    camera_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    camera_state = camera_state_srv(camera_name, "")  # Change this to your model name
    camera_pose = rob2cam(camera_state.pose)          # Convert to camera axis convention

    # Convert camera pose to numpy
    camera_pose.position = np.array((
        abs(camera_pose.position.x),
        abs(camera_pose.position.y),
        camera_pose.position.z), dtype=np.float64)
    # Convert quaternion to pyquaternion
    camera_pose.orientation = Quaternion(
            x=camera_pose.orientation.x,
            y=camera_pose.orientation.y,
            z=camera_pose.orientation.z,
            w=camera_pose.orientation.w
        )

    # Create directory tree
    os.makedirs(f"{OUTPUT_PATH}/images", exist_ok=True)
    os.makedirs(f"{OUTPUT_PATH}/depths", exist_ok=True)
    os.makedirs(f"{OUTPUT_PATH}/labels", exist_ok=True)
    os.makedirs(f"{OUTPUT_PATH}/visual", exist_ok=True)

    # Create dataset.yml
    with open(f"{OUTPUT_PATH}/dataset.json", "w+") as f:
        f.write(json.dumps(
            {
                "camera": {
                    "pose": {
                        "position": {
                            "x": camera_pose.position[0],
                            "y": camera_pose.position[1],
                            "z": camera_pose.position[2]
                        },
                        "orientation": {
                            "x": camera_pose.orientation.x,
                            "y": camera_pose.orientation.y,
                            "z": camera_pose.orientation.z,
                            "w": camera_pose.orientation.w
                        }
                    },
                    "info": {
                        "width": camera_info.width,
                        "height": camera_info.height,
                        "distortion_model": camera_info.distortion_model,
                        "D": camera_info.D,
                        "K": camera_info.K,
                        "R": camera_info.R,
                        "P": camera_info.P
                    }
                },
                "classes": [name.replace("lego_", "") for name in spawner.get_legos(mode="name")]
            }, indent=4))
    return camera_pose, camera_info


def snapshot():
    """
        snapshot

        creates a datapoint from the current scene
    """
    msg_color_copy = copy.deepcopy(msg_color)
    msg_depth_copy = copy.deepcopy(msg_depth)
    msg_models = rospy.wait_for_message("/gazebo/model_states", ModelStates)

    bboxes = []
    axeses = []
    poses = []
    classes = []
    for (name, pose) in zip(msg_models.name, msg_models.pose):
        if "lego_" not in name:
            continue

        rot_tra = np.zeros((4, 4), dtype=np.float64)
        # Rotation matrix
        rot_tra[:3, :3] = Quaternion(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w).rotation_matrix
        # Trasform matrix
        rot_tra[:4, 3] = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            1.0)

        # Axes of the lego
        axes = np.ones((4, 4), dtype=np.float64)
        axes[:, :3] = np.array([
            [0, 0, 0],    # origin
            [.1, 0, 0],   # x
            [0, .1, 0],   # y
            [0, 0, .1]])  # z
        axes = np.dot(rot_tra, axes.T)[:3, :]
        axes_2dproj = utils.projectPoints(axes,
                             camera_pose.position, camera_pose.orientation.rotation_matrix,
                             camera_matrix, dist_coeffs).reshape(-1, 2)
        axeses.append(axes_2dproj.astype(np.int32))

        # Load bbox corners
        name = name.rsplit("_", maxsplit=1)[0]
        with open(f"{MODELS_PATH}/{name}/model.json", "r") as f:
            model_json = json.load(f)

        corners = np.ones((8, 4), dtype=np.float64)
        corners[:, :3] = np.array(model_json["corners"], dtype=np.float64).reshape((-1, 3))

        # Set bbox corners frame of reference to lego
        corners = np.dot(rot_tra, corners.T)[:3, :]
        corners_2d = utils.projectPoints(corners,
                             camera_pose.position, camera_pose.orientation.rotation_matrix,
                             camera_matrix, dist_coeffs).reshape(-1, 2)
        bbox = bb.calculateBBoxYolo(corners_2d,
            image_width=camera_view[0], image_height=camera_view[1])

        classes.append(name.replace("lego_", ""))
        bboxes.append(bbox)
        poses.append(pose)

    # save datapoint
    with open(f"{OUTPUT_PATH}/labels/{msg_depth_copy.header.stamp.to_nsec()}.json", "w+") as f:
        poses = [{
            "position": {
                "x": p.position.x,
                "y": p.position.y,
                "z": p.position.z,
            },
            "orientation": {
                "x": p.orientation.x,
                "y": p.orientation.y,
                "z": p.orientation.z,
                "w": p.orientation.w
            }
        } for p in poses]
        datapoint = [
            {
                "class": c,
                "bbox": list(b),
                "world_pose": p
            } for (c, b, p) in zip(classes, bboxes, poses)
        ]
        f.write(json.dumps(datapoint, indent=4))

    # save image
    image = CvBridge().imgmsg_to_cv2(msg_color_copy, "bgr8")
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # opencv uses BGR, convert to standard RGB
    cv2.imwrite(f"{OUTPUT_PATH}/images/{msg_color_copy.header.stamp.to_nsec()}.png", image)

    # save image containing visual information
    view_pose(image, (np.array(bboxes)*640).astype(np.int32), axeses)
    cv2.imwrite(f"{OUTPUT_PATH}/visual/{msg_color_copy.header.stamp.to_nsec()}.png", image)

    # save depths data
    depth = CvBridge().imgmsg_to_cv2(msg_depth_copy, "32FC1")
    cv2.imwrite(f"{OUTPUT_PATH}/visual/{msg_color_copy.header.stamp.to_nsec()}_depth.png", depth*255)
    cv2.imshow("depths", depth)
    cv2.waitKey(1)
    # save depths data as numpy array
    depth_array = np.array(depth, dtype=np.float32)
    np.save(f"{OUTPUT_PATH}/depths/{msg_depth_copy.header.stamp.to_nsec()}.npy", depth_array)


if __name__ == '__main__':
    print("Initializing ROS node")
    rospy.init_node("farmer")

    sub_model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback_dummy, queue_size=1)

    sub_image_color = message_filters.Subscriber("/camera/color/image_raw", Image)
    sub_image_depth = message_filters.Subscriber("/camera/depth/image_raw", Image)

    # Synchronize the two topics together
    ts = message_filters.TimeSynchronizer([sub_image_color, sub_image_depth], 10)  # exact sync
    ts.registerCallback(callback_camera)

    # Initialize
    # + Create output directory
    # + Obtain camera parameters and pose
    camera_pose, camera_info = init(camera_name="kinect")
    camera_view = (camera_info.width, camera_info.height)
    camera_matrix = np.array(camera_info.K).reshape((3, 3))
    dist_coeffs = np.array(camera_info.D)

    # Wait for first image
    print("Waiting for first image")
    rospy.wait_for_message("/camera/color/image_raw", Image)
    rospy.wait_for_message("/camera/depth/image_raw", Image)

    print("Starting main loop")
    for i in range(0, 100_000):
        spawner.spawn_legos(rd.randint(1, 8), distribution="lay")
        time.sleep(.5)
        snapshot()
        print(f"Dataset size: {i} samples")
        spawner.delete_models("lego_")
