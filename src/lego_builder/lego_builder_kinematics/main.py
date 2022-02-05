#!/usr/bin/python3
import math
import copy
import actionlib
import gazebo_msgs.srv
import control_msgs.msg
import trajectory_msgs.msg
from gazebo_msgs.msg import ModelStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from joint_controller import get_Joints
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

lego_info = np.zeros( (11,5) )
#in [x,0], x defines the piece of lego in alphabetic order
#in [1,x], with x=0 it defines X target position of lego numb 1
#          with x=1 it defines Y target position of lego numb 1
#          with x=2 it defines current Z target position of lego numb 1
#          with x=3 it defines height of lego numb 1

#HARDCODED POSITIONS FOR LEGOS
# lego_X1-Y1-Z2

MODELS = ["lego_X1-Y2-Z1", "lego_X2-Y2-Z2", "lego_X1-Y3-Z2", "lego_X1-Y2-Z2", "lego_X1-Y2-Z2-CHAMFER", "lego_X1-Y4-Z2", "lego_X1-Y1-Z2", "lego_X1-Y2-Z2-TWINFILLET", "lego_X1-Y3-Z2-FILLET", "lego_X1-Y4-Z1", "lego_X2-Y2-Z2-FILLET"]

# lego_X1-Y2-Z1
lego_info[0,0]=0.072761
lego_info[0,1]=-0.802368
lego_info[0,2]=0.81
lego_info[0,3]=0.057
lego_info[0,4]=0.2
#lego_X2-Y2-Z2
lego_info[1,0]=0.376376
lego_info[1,1]=-0.802140
lego_info[1,2]=0.82
lego_info[1,3]=0.038
lego_info[1,4]=0.1
# lego_X1-Y3-Z2
lego_info[2,0]= 0.375306
lego_info[2,1] = -0.640330
lego_info[2,2]=0.82
lego_info[2,3]=0.057
lego_info[2,4]=0.2
# lego_X1-Y2-Z2
lego_info[3,0]= -0.081469
lego_info[3,1] = -0.798799
lego_info[3,2]=0.82
lego_info[3,3]=0.057
lego_info[3,4]=0.0
# lego_X1-Y2-Z2-CHAMFER
lego_info[4,0]= -0.237561
lego_info[4,1] = -0.801300
lego_info[4,2]=0.82
lego_info[4,3]=0.057
lego_info[4,4]=0.2
# lego_X1-Y4-Z2
lego_info[5,0]= 0.378296
lego_info[5,1] = -0.186005
lego_info[5,2]=0.82
lego_info[5,3]=0.057
lego_info[5,4]=0.2
# lego_X1-Y1-Z2
lego_info[6,0]= 0.224808
lego_info[6,1] = -0.809098
lego_info[6,2]=0.815
lego_info[6,3]=0.057
lego_info[6,4]=0.2
# lego_X1-Y2-Z2-TWINFILLET
lego_info[7,0]= -0.380065
lego_info[7,1] = -0.797871
lego_info[7,2]=0.82
lego_info[7,3]=0.057
lego_info[7,4]=0.2
# lego_X1-Y3-Z2-FILLET
lego_info[8,0]= 0.378370
lego_info[8,1] = -0.491671
lego_info[8,2]=0.82
lego_info[8,3]=0.057
lego_info[8,4]=0.2
 # lego_X1-Y4-Z1
lego_info[9,0]= 0.372466
lego_info[9,1] = -0.338593
lego_info[9,2]=0.82
lego_info[9,3]=0.057
lego_info[9,4]=0.2
# lego_X2-Y2-Z2-FILLET
lego_info[10,0]= 0.218573
lego_info[10,1] = -0.194531
lego_info[10,2]=0.82
lego_info[10,3]=0.057
lego_info[10,4]=0.1

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
# Resting position of the end effector
DEFAULT_POS  = (-0.3, 0, 1)

DEFAULT_JOINT_TRAJECTORY = trajectory_msgs.msg.JointTrajectory()
DEFAULT_JOINT_TRAJECTORY.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint","elbow_joint", "wrist_1_joint", "wrist_2_joint","wrist_3_joint"]

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10


def send_joints(x, y, z, quat, blocking=True, duration=1.0):
    th_res = get_Joints(x, y, z, quat.rotation_matrix)

    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory = copy.deepcopy(DEFAULT_JOINT_TRAJECTORY)
    goal.trajectory.header.stamp = rospy.Time.now()

    print(DEFAULT_PATH_TOLERANCE)
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


def get_gazebo_model_name(vision_model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = ModelStates()
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
        legos = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]


def move_to(start_pos, start_quat, target_pos, target_quat, z_raise=0.0):
    """
    Move the end effector to target_pos with target_quat as orientation

    :param start_pos:
    :param start_quat:
    :param target_pos:
    :param target_quat:
    :param z_raise:
    :return:
    """
    def smooth(percent_value, period=math.pi):
        return (1-math.cos(percent_value*period))/2


    steps = 15
    step = 1 / steps

    x, y, z = start_pos
    dx, dy, dz = target_pos[0]-x, target_pos[1]-y, target_pos[2]-z

    for i in np.arange(0, 1+step, step):
        i_2 = smooth(i, 2*math.pi)  # from 0 to 1 to 0
        i = smooth(i)  # from 0 to 1

        grip = PyQuaternion.slerp(start_quat, target_quat, i)

        send_joints(
            x+i*dx,
            y+i*dy,
            z+i*dz+i_2*z_raise,
            grip,
            blocking=False, duration=0.5)
        rospy.sleep(0.1)
    send_joints(
        x + i * dx,
        y + i * dy,
        z + i * dz,
        grip,
        blocking=True, duration=0.5)


def straighten(model_pose, model_name, lego_type):
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
    send_joints(x, y, z+0.2, approach_quat)

    # Lower down and grip
    send_joints(x, y, z, approach_quat, duration=2)
    close_gripper(model_name)

    """
        Straighten model if needed
    """
    if facing_direction != "up":
        move_to((x, y, z), approach_quat, (x, y, z), target_quat, z_raise=0.2)
        rospy.sleep(0.7)
        open_gripper(model_name)

        # Re grip the model
        move_to((x, y, z), target_quat, (x, y, z), DEFAULT_QUAT, z_raise=0.2)
        rospy.sleep(0.7)
        close_gripper(model_name)


def close_gripper(model_name):
    rospy.loginfo("Attaching object and gripper")

    """
        Create dynamic joint
    """
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "robot"
    req.link_name_2 = "wrist_3_link"
    attach_srv.call(req)

    set_gripper(0.8)


def open_gripper(model_name):

    rospy.loginfo("Detaching object and gripper")

    """
        Destroy dynamic joint
    """
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "robot"
    req.link_name_2 = "wrist_3_link"
    detach_srv.call(req)

    set_gripper(0.0)


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
    angle = np.arccos(np.clip(np.dot(new_axis, axis_z), -1.0, 1.0)) % math.pi
    # get if model is facing up, down or sideways
    if angle < math.pi / 3:
        return "up"
    elif angle < math.pi / 3 * 2 * 1.1:
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
        return -model_quat.yaw_pitch_roll[0]
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def get_lego_pos_by_name(name):
    legos=ModelStates()
    legos=get_legos_pos()
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
    # Create an action client for the joint trajector
    action_trajectory = actionlib.SimpleActionClient(
        "/trajectory_controller/follow_joint_trajectory",
        control_msgs.msg.FollowJointTrajectoryAction,
    )
    print("Waiting for joint controller action")
    action_trajectory.wait_for_server()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for gripper controller action")
    action_gripper.wait_for_server()

    #setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    #setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    legos = get_legos_pos()
    legos.sort(reverse=False , key=lambda a: (a[1].position.x, a[1].position.y))
    #choose destination
    model_index = -1
    #print(legos)

    for model_name, model_pose in legos:
        if("lego_" not in model_name):
            continue

        try:
            # TODO: temporary fix for gazebo model_states
            fixed_model_name = model_name.rsplit("_", maxsplit=1)[0]

            model_index = MODELS.index(fixed_model_name)
        except ValueError as e:
            print(f"Model name {model_name} was not recognized!")

        print(f"Moving model {model_name}")

        # Get actual model_name at model xyz coordinates
        try:
            model_name = get_gazebo_model_name(model_name, model_pose)
        except ValueError as e:
            print(e)
            continue

        # Straighten lego
        straighten(model_pose, model_name, model_index)

        """
            Go to destination
        """
        print("MODEL ", model_index)
        target_pos=lego_info[model_index,0], lego_info[model_index,1], lego_info[model_index,2]
        rospy.sleep(0.1)
        move_to(DEFAULT_POS, DEFAULT_QUAT, (target_pos[0], target_pos[1], DEFAULT_POS[2]), DEFAULT_QUAT)
        # Lower the object and release
        move_to(target_pos, DEFAULT_QUAT, target_pos, DEFAULT_QUAT)
        # set_model_fixed(model_name)
        open_gripper(model_name)

        # increment z in order to stack lego correctly
        lego_info[model_index, 2] += lego_info[model_index, 3]

        #rospy.wait_for_service("/gazebo/reset_world")
        #reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        #reset_world()
        #previous_name=name
        #rospy.sleep(0.1)
    print("Moving to DEFAULT_POS")
    send_joints(*DEFAULT_POS, DEFAULT_QUAT)
    rospy.sleep(0.4)
