import copy
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def get_model_state(quat):
    axis_z = np.array([0, 0, 1])
    new_axis = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis, axis_z), -1.0, 1.0)) % np.pi
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return "up"
    elif angle < np.pi / 3 * 2:
        return "side"
    else:
        return "down"


def get_approach_angle(model_quat, model_state):
    if model_state == "up":
        return model_quat.yaw_pitch_roll[0] - math.pi/2
    elif model_state == "side":
        axis_x = np.array([1, 0, 0])
        axis_y = np.array([0, 1, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1]))
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0)
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0)
        return math.atan2(det, dot)
    elif model_state == "down":
        return -model_quat.yaw_pitch_roll[0] - math.pi/2
    else:
        raise ValueError(f"Invalid model state {model_state}")


def get_approach_quat(model_state, approach_angle):
    quat = PyQuaternion(vector=(0, 0, 1))

    yaw_angle = -math.pi/2

    if model_state == "up":
        pitch_angle = math.pi
    elif model_state == "side":
        if approach_angle < 0:
            pitch_angle = math.pi/2 + 0.2
        else:
            pitch_angle = math.pi - 0.2
    elif model_state == "down":
        pitch_angle = math.pi/2 + 0.2
    else:
        raise ValueError(f"Invalid model state {model_state}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle) * quat

    return quat


def get_target_quat(model_state, approach_angle):
    quat = PyQuaternion(vector=(0, 0, 1))

    pitch_angle = 0
    yaw_angle = -math.pi/2

    new_model_state = "up"

    if model_state == "up":
        pitch_angle = math.pi
    elif model_state == "side":
        if approach_angle < 0:
            pitch_angle = math.pi + 0.2
        else:
            pitch_angle = -math.pi/2 - 0.2
    elif model_state == "down":
        pitch_angle = math.pi/2 + 0.2
        yaw_angle = math.pi/2
    else:
        raise ValueError(f"Invalid model state {model_state}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle) * quat

    return quat, new_model_state


def straighten(model_pose):
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)
    model_state = get_model_state(model_quat)
    approach_angle = get_approach_angle(model_quat, model_state)

    print(f"Approaching at {approach_angle:.2f} deg")

    approach_quat = get_approach_quat(model_state, approach_angle)

    target_quat = copy.deepcopy(approach_quat)
    if model_state == "side":
        if approach_angle < 0:
            pitch_angle = math.pi/2
        else:
            pitch_angle = -math.pi/2
        target_quat = target_quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    elif model_state == "down":  # Add a yaw rotation of 180 deg
        yaw_angle = math.pi
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)

    return approach_quat, target_quat


rospy.init_node('insert_object', log_level=rospy.INFO)
rospy.wait_for_service('gazebo/spawn_sdf_model')
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


while True:
    models = rospy.wait_for_message("gazebo/model_states", ModelStates)
    lego, pose = [(n, p) for n, p in zip(models.name, models.pose) if "lego_X1-Y2-Z2-T" in n][0]
    approach_quat, target_quat, _ = straighten(pose)

    steps = 100
    for i in range(0, steps):
        grip = PyQuaternion.slerp(approach_quat, target_quat, i/steps)

        state_msg = ModelState()
        state_msg.model_name = "lego_X1-Y2-Z2-CHAMFER"
        state_msg.pose.position.x = 0.5585
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.790502
        state_msg.pose.orientation.x = grip.x
        state_msg.pose.orientation.y = grip.y
        state_msg.pose.orientation.z = grip.z
        state_msg.pose.orientation.w = grip.w

        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)

        except Exception as e:
            print(e)

        time.sleep(0.01)
