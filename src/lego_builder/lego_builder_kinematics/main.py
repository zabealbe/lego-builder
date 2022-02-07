#!/usr/bin/python3

import os
import math
import copy
import json
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "X1-Y2-Z1": {
        "home": [0.072761, -0.802368, 0.775888]
    },
    "X2-Y2-Z2": {
        "home": [0.376376, -0.802140, 0.775888]
    },
    "X1-Y3-Z2": {
        "home": [0.375306, -0.640330, 0.775888]
    },
    "X1-Y2-Z2": {
        "home": [-0.081469, -0.798799, 0.775888]
    },
    "X1-Y2-Z2-CHAMFER": {
        "home": [-0.237561, -0.801300, 0.775888]
    },
    "X1-Y4-Z2": {
        "home": [0.378296, -0.186005, 0.775888]
    },
    "X1-Y1-Z2": {
        "home": [0.224808, -0.809098, 0.775888]
    },
    "X1-Y2-Z2-TWINFILLET": {
        "home": [-0.380065, -0.797871, 0.775888]
    },
    "X1-Y3-Z2-FILLET": {
        "home": [0.378370, -0.491671, 0.775888]
    },
    "X1-Y4-Z1": {
        "home": [0.372466, -0.338593, 0.775888]
    },
    "X2-Y2-Z2-FILLET": {
        "home": [0.218573, -0.194531, 0.775888]
    }
}

for model, info in MODELS_INFO.items():
    model_json_path = os.path.join(PKG_PATH, "..", "models", f"lego_{model}", "model.json")
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

    MODELS_INFO[model]["size"] = (size_x, size_y, size_z)

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.2, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10


"""
def send_joints(x, y, z, quat, blocking=True, duration=1.0):
    th_res = get_Joints(x, y, z, quat.rotation_matrix)

    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory = copy.deepcopy(DEFAULT_JOINT_TRAJECTORY)
    goal.trajectory.header.stamp = rospy.Time.now()

    #goal.goal_tolerance = [DEFAULT_PATH_TOLERANCE]
    goal.goal_time_tolerance = rospy.Duration(3)

    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = th_res[:, 5]  # selecting 5th kinematic solution
    point.time_from_start = rospy.Duration(duration)
    # Set the points to the trajectory
    goal.trajectory.points = [point]
    # Publish the message
    action_trajectory.send_goal(goal)
    if blocking:  # Wait for result
        action_trajectory.wait_for_result()

    return action_trajectory.get_result()
"""


def get_gazebo_model_name(vision_model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for model_name, model_pose in zip(models.name, models.pose):
        if vision_model_name not in model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return model_name
    raise ValueError(f"Model {vision_model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def get_legos_pos(vision=False):
    #get legos position reading vision topic
    if vision:
        legos = rospy.wait_for_message("/estimated_model_states", ModelStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        legos = ModelStates()

        for name, pose in zip(models.name, models.pose):
            if "lego_" not in name:
                continue
            name = name.replace("lego_", "").rsplit("_", maxsplit=1)[0]

            legos.name.append(name)
            legos.pose.append(pose)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]


def straighten(model_pose, model_name):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)

    """
        Calculate approach quaternion and target quaternion
    """

    facing_direction = get_facing_direction(model_quat)
    approach_angle = get_approach_angle(model_quat, facing_direction)

    print(f"Lego is facing {facing_direction}")
    print(f"Approaching at {approach_angle:.2f} deg")

    # Calculate approach quat
    approach_quat = get_approach_quat(facing_direction, approach_angle)

    # Calculate target quat

    if facing_direction == "side":
        target_quat = DEFAULT_QUAT
        pitch_angle = -math.pi/2 + 0.2

        if abs(approach_angle) < math.pi/2:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        else:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
        target_quat = PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat
    elif facing_direction == "down":  # Add a yaw rotation of 180 deg
        target_quat = copy.deepcopy(approach_quat)
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)
        #target_quat = PyQuaternion(axis=(0, 1, 0), angle=0.2) * target_quat
    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)

    """
        Grip the model
    """
    # Get above the object
    controller.move_to(x, y, target_quat=approach_quat)

    # Lower down and grip
    controller.move_to(x, y, z, approach_quat)
    close_gripper(model_name)

    """
        Straighten model if needed
    """
    if facing_direction != "up":
        controller.move_to(x, y, z+0.02, target_quat, z_raise=0.2)
        open_gripper(model_name)

        # Re grip the model
        controller.move_to(x, y, z, DEFAULT_QUAT, z_raise=0.2)
        close_gripper(model_name)


def close_gripper(model_name):
    set_gripper(0.1)

    # Create dynamic joint
    if model_name is not None:
        req = AttachRequest()
        req.model_name_1 = model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(model_name=None):
    set_gripper(0.0)

    # Destroy dynamic joint
    if model_name is not None:
        req = AttachRequest()
        req.model_name_1 = model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)


def set_model_fixed(model_name):

    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("SETTING {} TO STATIC".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == "up":
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == "side":
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi/2:
            yaw_angle = math.pi/2
        else:
            yaw_angle = -math.pi/2
    elif facing_direction == "down":
        pitch_angle = - math.pi/2 + 0.2
        yaw_angle = -math.pi / 2
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat


def get_facing_direction(quat):
    axis_z = np.array([0, 0, 1])
    new_axis = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis, axis_z), -1.0, 1.0))
    # get if model is facing up, down or sideways
    if angle < math.pi / 3:
        return "up"
    elif angle < math.pi / 3 * 2 * 1.2:
        return "side"
    else:
        return "down"


def get_approach_angle(model_quat, facing_direction):#get gripper approach angle
    if facing_direction == "up":
        return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
    elif facing_direction == "side":
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == "down":
        return -(model_quat.yaw_pitch_roll[0] + math.pi) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def get_lego_pos_by_name(name):
    legos=get_legos_pos(vision=True)
    for lego in legos:
        if(name == lego[0]):
            return lego


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = 1  # -1.0  # Do not limit the effort
    action_gripper.send_goal_and_wait(goal)

    return action_gripper.get_result()


if __name__ == "__main__":
    print("Initializing kinematics node")
    rospy.init_node("send_joints")

    controller = ArmController()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for gripper controller action")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    print("Waiting for models detection")
    rospy.sleep(0.5)
    legos = get_legos_pos(vision=True)
    legos.sort(reverse=False, key=lambda a: (a[1].position.x, a[1].position.y))

    for model_name, model_pose in legos:
        open_gripper()
        try:
            model_home = MODELS_INFO[model_name]["home"]
            model_size = MODELS_INFO[model_name]["size"]
        except ValueError as e:
            print(f"Model name {model_name} was not recognized!")

        # Get actual model_name at model xyz coordinates
        try:
            gazebo_model_name = get_gazebo_model_name(model_name, model_pose)
        except ValueError as e:
            print(e)
            continue

        # Straighten lego
        straighten(model_pose, gazebo_model_name)
        controller.move(dz=0.15)

        """
            Go to destination
        """
        x, y, z = model_home
        z += model_size[2] / 2
        print(f"Moving model {model_name} to {x} {y} {z}")

        controller.move_to(x, y, target_quat=DEFAULT_QUAT)
        # Lower the object and release
        controller.move_to(x, y, z)
        set_model_fixed(gazebo_model_name)
        open_gripper(gazebo_model_name)
        controller.move(dz=0.15)

        # increment z in order to stack lego correctly
        MODELS_INFO[model_name]["home"][2] += model_size[2]
    print("Moving to DEFAULT_POS")
    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    rospy.sleep(0.4)
