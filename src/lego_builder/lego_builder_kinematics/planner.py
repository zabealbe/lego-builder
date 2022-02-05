#!/usr/bin/python3

# libraries import

from fileinput import close
from sunau import AUDIO_FILE_ENCODING_FLOAT
from typing import final
from unittest.mock import DEFAULT
import joint_controller
import os
import sys
import time
import rospy
import math
import copy
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from rospy.core import xmlrpcapi
from std_msgs.msg import Float64
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import rospy
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from math import pi as pi
from joint_controller import get_Joints
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_srvs.srv import Empty
from control_msgs.msg import JointTrajectoryControllerState

SAFE_Z_OFFSET = 0.15
gripper_closure= '0.1'
lego_info = np.zeros( (11,5) )
#HARDCODED POSITIONS FOR LEGOS
# in [x,0], x defines the piece of lego in alphabetic order
# in [1,x], with x=0 it defines X target position of lego numb 1
#          with x=1 it defines Y target position of lego numb 1
#          with x=2 it defines current Z target position of lego numb 1
#          with x=3 it defines height of lego numb 1
#          with x=4 it defines the fixed gripper closure for that block
    
    
MODELS = ["lego_X1-Y2-Z1", "lego_X2-Y2-Z2", "lego_X1-Y3-Z2", "lego_X1-Y2-Z2", "lego_X1-Y2-Z2-CHAMFER", "lego_X1-Y4-Z2", "lego_X1-Y1-Z2", "lego_X1-Y2-Z2-TWINFILLET", "lego_X1-Y3-Z2-FILLET", "lego_X1-Y4-Z1", "lego_X2-Y2-Z2-FILLET"]

# lego_X1-Y2-Z1
lego_info[0,0]=0.072761
lego_info[0,1]=-0.802368
lego_info[0,2]=0.81
lego_info[0,3]=0.057
lego_info[0,4]=0.3
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
lego_info[3,4]=0.1
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
lego_info[6,2]=0.83
lego_info[6,3]=0.057
lego_info[6,4]=0.35
# lego_X1-Y2-Z2-TWINFILLET
lego_info[7,0]= -0.380065
lego_info[7,1] = -0.797871
lego_info[7,2]=0.82
lego_info[7,3]=0.057
lego_info[7,4]=0.2
# lego_X1-Y3-Z2-FILLET
lego_info[8,0]= 0.378370
lego_info[8,1] = -0.491671
lego_info[8,2]=0.80
lego_info[8,3]=0.057
lego_info[8,4]=0.2
 # lego_X1-Y4-Z1
lego_info[9,0]= 0.372466
lego_info[9,1] = -0.338593
lego_info[9,2]=0.80
lego_info[9,3]=0.057
lego_info[9,4]=0.3
# lego_X2-Y2-Z2-FILLET
lego_info[10,0]= 0.218573
lego_info[10,1] = -0.194531
lego_info[10,2]=0.82
lego_info[10,3]=0.057
lego_info[10,4]=0.1

#definition of default quaternion for rotation
DEFAULT_QUAT = PyQuaternion(axis=(1, 0, 0), angle=math.pi)
#definition of default position
DEFAULT_POS  = (0.58, 0, 1)
#############################
#Send_joints: fuction to send the robotic arm in a defined position
#takes in input the values of x,y,z, a quaternion, the number of loops and the delay
#############################
def Send_joints(x,y,z,quat,loop_times=1,delay=0.05): #x,y,z and orientation of lego block
    i=0
    rate = rospy.Rate(10)
    th_res=get_Joints(x,y,z,quat.rotation_matrix)

    while not rospy.is_shutdown() and i<loop_times:
        pts = JointTrajectoryPoint()
        index = 5
        pts.positions = [ th_res[0,index], th_res[1,index], th_res[2,index], th_res[3,index], th_res[4,index],th_res[5,index]]
        pts.time_from_start = rospy.Duration(0.3)
    # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
    # Publish the message
        pub.publish(traj)
        i=i+1
        rospy.sleep(delay)
    ''' try:
    catch					
    except rospy.ROSInterruptException:
        pass'''
#############################
#check_position_loop: function that is used to verify that the robotic arm has reached the desired position
#when the difference between the robot joints positions is lower than epsilon the loop will end allowing code to continue execution
#############################
def check_position_loop():
    epsilon=0.1
    while(1):
        valid=True
        current_pos = rospy.wait_for_message('/trajectory_controller/state', JointTrajectoryControllerState, timeout=None)
        for i in range(0,6) :
            if (abs(current_pos.actual.positions[i]-current_pos.desired.positions[i])>epsilon):
                valid=False
                break
        if(valid == True):
            break;        
    time.sleep(0.02)
#############################
#move_smooth: function to move a piece from a place to an other, there are 4 possible modes of moving an object, each one of these has their particular interpolation function
# in order to have the movement as smooth as possible according to the type of actions that the robot has to do:
''' 
    MODES:
        0: 
        1:
        2:
        3: 
'''
#############################
def move_smooth(point1,point2,initial_quat,final_quat,steps=15,mode=0): #make progress slow at the beginning and at the end  
    print('mode: '+str(mode))
    step = 1 / steps
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    dz = point1[2] - point2[2]
    '''
    def z_path(percent, z_raise):
        return math.sin(percent*math.pi)*z_raise
    '''
    def z_path(percent_value, period=math.pi):
        return (1-math.cos(percent_value*period))/2
    if (mode == 0):
        for i in np.arange(0, 1+step, step):
            interpolation = z_path(i,pi)
            grip = PyQuaternion.slerp(initial_quat, final_quat, interpolation)
            increment = z_path(i)
            Send_joints(point1[0]-dx*increment,point1[1]-dy*increment,point1[2]-dz*increment,grip, 10, 0.008)
            #check_position_loop()
    elif (mode==1):
        for i in np.arange(0, 1+step, step):
            interpolation = z_path(i,pi)
            grip = PyQuaternion.slerp(initial_quat, final_quat, interpolation)
            increment = z_path(i,2*pi)
            Send_joints(point1[0],point1[1],point1[2]+increment*0.15,grip, 10, 0.008)    
            check_position_loop()
    elif (mode==2):
        for i in np.arange(0, 1+step, step):
            interpolation = z_path(i,pi)
            grip = PyQuaternion.slerp(initial_quat, final_quat, interpolation)
            increment = z_path(i,2*pi)
            Send_joints(point1[0],point1[1],point1[2]+increment*0.1, grip, 10, 0.0008)
            check_position_loop()
    else:
        for i in np.arange(0, 1+step, step):
            d = (math.cos(i*math.pi)+1)/2
            Send_joints(point1[0]-dx*d,point1[1]-dy*d,point1[2]-dz*d,final_quat,10, 0.008)
            check_position_loop()
    check_position_loop()
    rospy.sleep(0.3)
############################
#get_gazebo_model_name: function to get the Gazebo model name from the vision name and position
#in order to do that we check wich of the gazebo models is located into a square of side epsilon centered in vision_model_pose
#############################
def get_gazebo_model_name(vision_model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = ModelStates()
    models = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None)
    epsilon = 0.05
    for model_name, model_pose in zip(models.name, models.pose):
        if vision_model_name not in model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return model_name
    raise ValueError(f"Model {vision_model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")
#############################
#get_lego_pos: function to decide where to take the lego names and position based on the variable vision
#############################
def get_legos_pos():
    #get legos position reading vision topic
    vision=False
    if vision:
        legos = rospy.wait_for_message("/estimated_model_states", ModelStates, timeout=None)
    else:   
        legos = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]
#############################
#rotate_downside_lego: function used to rotate a downside lego in order to make its upside rotation easiera and safer
#############################
def rotate_downside_lego(x,y,z,approach_angle,model_name):
    '''
    Rotate z axis of a downside lego in order to facilitate rotation upside
    '''
    initial_quat = DEFAULT_QUAT
    initial_quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+pi) * initial_quat
    final_quat = initial_quat
    initial_point=np.array([x,y,z+SAFE_Z_OFFSET])
    final_point=np.array([x,y,z])

    #send robot above object
    Send_joints(x,y,z+SAFE_Z_OFFSET,initial_quat)
    check_position_loop()
    #send robot to model
    move_smooth(initial_point,final_point,initial_quat,initial_quat)
    close_gripper(model_name)
    initial_point=final_point
    final_quat = PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) * DEFAULT_QUAT

    #re-orient model
    move_smooth(final_point,final_point,initial_quat,final_quat,mode=1,steps=30)
    rospy.sleep(0.25)
    open_gripper(model_name)
    rospy.sleep(0.25)
    initial_point=final_point
    final_point=[x,y,z+SAFE_Z_OFFSET]
    initial_quat=final_quat
    final_quat=DEFAULT_QUAT
    #move_smooth(initial_point,final_point,initial_quat,final_quat,mode=0,steps=30)
#############################
#straighten: general function to rotate a block, it calculates the approach quaternion based on the variable facing_direction that indicates how the block is placed
#############################
def straighten(model_pose, model_name, lego_type):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z -0.015
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
        if approach_angle < 0:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        else:
            target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
        target_quat =  PyQuaternion(axis=(0, 1, 0), angle=pitch_angle) * target_quat
    elif facing_direction == "down":  # Add a yaw rotation of 180 deg
        rotate_downside_lego(x,y,z,approach_angle,model_name)
        #target_quat = copy.deepcopy(approach_quat)
        target_quat=DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 1, 0), angle=math.pi/2)
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2)
        target_quat = PyQuaternion(axis=(0, 1, 0), angle=0.15) * target_quat
        approach_quat = copy.deepcopy(target_quat)
        approach_quat = approach_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi)

    else:
        target_quat = DEFAULT_QUAT
        target_quat = target_quat * PyQuaternion(axis=(0, 0, 1), angle=-math.pi/2)
    
    """
        Grip the model
    """
    # go above object
    '''
    Send_joints(x,y,z+SAFE_Z_OFFSET,approach_quat)
    check_position_loop()
    '''
    initial_point=np.array([x,y,z+SAFE_Z_OFFSET])
    final_point=np.array([x,y,z+0.15])
    move_smooth(initial_point,final_point,DEFAULT_QUAT,DEFAULT_QUAT,mode=0,steps=25)
    
    check_position_loop()
    # Grip the model
    initial_point=[x,y,z+SAFE_Z_OFFSET]
    final_point=[x,y,z+0.015]
    move_smooth(initial_point,final_point,approach_quat,approach_quat,mode=0,steps=30)
    close_gripper(model_name)
    # return above previous object position to avoid collision with other models
    """
        Straighten model if needed
    """
    if facing_direction != "up":
        initial_point=final_point
        move_smooth(initial_point,initial_point,approach_quat,target_quat,mode=1,steps=30)
        open_gripper(model_name)
        if(facing_direction=='down'):
            final_quat = DEFAULT_QUAT
            final_quat = final_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) 
        else:
            final_quat = DEFAULT_QUAT
            #final_quat = final_quat * PyQuaternion(axis=(0, 0, 1), angle=math.pi/2) 
        # Re grip the model
        initial_point=[x,y,z]
        move_smooth(initial_point,initial_point,target_quat,final_quat,mode=2,steps=25)
#############################
#move_to_destination: function that moves a block from the initial pose to the final pose while doing an interpolation of points in order to have slow start and end
#############################
def move_to_destination(model_pose, model_name, model_index):
    close_gripper(model_name)
    x_dest=lego_info[model_index,0]
    y_dest=lego_info[model_index,1]
    z_dest=lego_info[model_index,2]-0.02
    X=model_pose.position.x
    Y=model_pose.position.y
    Z=model_pose.position.z
    #move_smooth(initial_point,final_point,DEFAULT_QUAT,DEFAULT_QUAT,mode=0)
    Send_joints(X,Y,Z+SAFE_Z_OFFSET,DEFAULT_QUAT)
    check_position_loop()
    initial_point=[X,Y,Z+SAFE_Z_OFFSET]
    final_point=[x_dest,y_dest,z_dest+SAFE_Z_OFFSET]
    move_smooth(initial_point,final_point,DEFAULT_QUAT,DEFAULT_QUAT,mode=0)
    initial_point=final_point
    final_point=[x_dest,y_dest,z_dest+0.025]
    move_smooth(initial_point,final_point,DEFAULT_QUAT,DEFAULT_QUAT,mode=0)
    #Send_joints(x_dest,y_dest,z_dest+SAFE_Z_OFFSET, DEFAULT_QUAT, 10) 
    '''
    initial_point==final_point
    final_point=([x_dest,y_dest,z_dest])
    move_smooth(initial_point,final_point,DEFAULT_QUAT,DEFAULT_QUAT,mode=3)
    '''
    #set_model_fixed(model_name)
    open_gripper(model_name)
    final_quat = DEFAULT_QUAT * PyQuaternion(axis=(0, 0, 1), angle=pi/2)
    Send_joints(x_dest,y_dest,z_dest+SAFE_Z_OFFSET, DEFAULT_QUAT, 10)    
    check_position_loop()
#############################
#close_gripper: function that closes the gripper on the blocks and creates the dynamic link between the gripper and the block named
# by model_name. If models have _* inside their names, the functions renames them removing extra characters
#############################
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
    rospy.sleep(0.1)
    #close gripper
    model_name = model_name.replace("_0", "").replace("_1", "").replace("_2", "")
    model_index = MODELS.index(model_name)
    set_gripper(lego_info[model_index, 4])
    rospy.sleep(0.3)
#############################
#open_gripper: function that opens the gripper after it has been closed. It also destroys the dynamic link created between the gripper and the block labeled as model_name
#############################
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
    rospy.sleep(0.2)
#############################
#set_model_fixed: function that is used to edit the properties of a block in order to make it stay in position and not to be afflicted from collisions. 
# to do this we create a link between the block and the ground plane, and we edit the propery of the name via the setstatic service
#############################
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
#############################
#get_approach_quat: function that chooses the right values of the approach quaternion based on the facing direction and the approach angle
#############################
def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    yaw_angle = -pi/2
    if facing_direction == "up":
        pitch_angle = 0
        yaw_angle = 0
        roll_angle =0
    elif facing_direction == "side":
        roll_angle =0
        pitch_angle = - 0.2
        if approach_angle < 0:
            yaw_angle+= math.pi
        else:
            yaw_angle += 0
    elif facing_direction == "down":
        yaw_angle+= math.pi
        pitch_angle = -math.pi/2 - 0.2
        roll_angle =pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle) 
    quat = quat * PyQuaternion(axis=(1, 0, 0), angle=roll_angle)   
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat
#############################
#get_facing_direction: based on the input quaternion it returns the facing direction of the block
#############################
def get_facing_direction(quat):
    axis_z = np.array([0, 0, 1])
    new_axis = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis, axis_z), -1.0, 1.0)) % np.pi
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return "up"
    elif angle < np.pi / 3 * 2 * 1.1:
        return "side"
    else:
        return "down"
#############################
#get_approach_angle: returns the value of the approach angle
#############################
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
        print("DOWN")
        return -model_quat.yaw_pitch_roll[0] 
    else:
        raise ValueError(f"Invalid model state {facing_direction}")
#############################
#get_lego_pos_by_name: given the name of a block it returns the Gazebo position of the block
#############################
def get_lego_pos_by_name(name):
    legos=ModelStates()
    legos=get_legos_pos()
    for lego in legos:
        if(name == lego[0]):
            return lego
#############################
#set_gripper: given a value between 0 and 0.8 this function makes the gripper reach that position
#############################
def set_gripper(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = 0.01   # Do not limit the effort
    client.send_goal(goal)
    #client.wait_for_result()
    #return client.get_result()
#############################
#main: main function where services and variables are declared and where worplan is developed
#############################
if __name__ == "__main__":
    rospy.init_node("send_joints")
    pub = rospy.Publisher("/trajectory_controller/command",JointTrajectory,queue_size=10)
    
    setstatic_srv = rospy.ServiceProxy('/link_attacher_node/setstatic', SetStatic)
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()
    
    traj = JointTrajectory()
    traj.header = Header()
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
    #name=sys.argv[1]
    
    legos = get_legos_pos()
    #print(legos)
    legos.sort(reverse=False , key=lambda a: (a[1].position.x, a[1].position.y))
    #choose destination
    model_index = -1
    #print(legos)

    for model_name, model_pose in legos:

        if("lego_" not in model_name):
            continue

        try:
            # TODO: temporary fix for gazebo model_states
            fixed_model_name = model_name.replace("_0", "").replace("_1", "").replace("_2", "")

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
        open_gripper(model_name)

        print("Manipulating MODEL: ",model_name)
        """
            Straghten models
        """
        straighten(model_pose, model_name, model_index)

        """
            Go to destination
        """
        move_to_destination(model_pose, model_name, model_index)

        #update final z destination in order to stack models correctly
        lego_info[model_index,2]=lego_info[model_index,2]+lego_info[model_index,3]-0.02