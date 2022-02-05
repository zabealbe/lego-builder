#!/usr/bin/python3
import joint_controller
import os
import sys
import time
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion
import numpy as np
from math import pi as pi
sleep_time=0
SAFE_Z_OFFSET = 0.08
gripper_closure= '0.1'
#print(sys.argv)


#########################################


def callback(data):
    for i in range(len(data.name)):
        if("lego_" not in data.name[i]):
            continue
        rospy.loginfo("name: "+str(data.name[i]))
        rospy.loginfo("posizione.x: "+str(data.pose[i].position.x))
        rospy.loginfo("posizione.y: "+str(data.pose[i].position.y))
        rospy.loginfo("posizione.z: "+str(data.pose[i].position.z))
        rospy.loginfo("orientamento.x: "+str(data.pose[i].orientation.x))
        rospy.loginfo("orientamento.y: "+str(data.pose[i].orientation.y))
        rospy.loginfo("orientamento.z: "+str(data.pose[i].orientation.z))
        rospy.loginfo("orientamento.w: "+str(data.pose[i].orientation.w))
        rospy.loginfo("linear_twist.x: "+str(data.twist[i].linear.x))
        rospy.loginfo("linear_twist.y: "+str(data.twist[i].linear.y))
        rospy.loginfo("linear_twist.z: "+str(data.twist[i].linear.z))
        rospy.loginfo("angular_twist.x: "+str(data.twist[i].angular.x))
        rospy.loginfo("angular_twist.y: "+str(data.twist[i].angular.y))
        rospy.loginfo("angular_twist.z: "+str(data.twist[i].angular.z))
        print('...........................')
    
    
    
def get_legos_pos():
    rospy.init_node('get_legos_position', anonymous=True)
    #rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    legos = ModelStates()
    legos = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=None)
    callback(legos)
    return legos

def get_lego_pos_by_name(name):
    x=0
    y=0
    z=0
    R=0
    P=0
    Y=0
    legos=ModelStates()
    legos=get_legos_pos()
    for i in range(len(legos.name)):
        if(name == legos.name[i]):
            x=legos.pose[i].position.x 
            y=legos.pose[i].position.y
            z=legos.pose[i].position.z
            Y, P, R = Quaternion ( x=legos.pose[i].orientation.x, y=legos.pose[i].orientation.y, z=legos.pose[i].orientation.z, w=legos.pose[i].orientation.w).yaw_pitch_roll
            
    return x,y,z,R,P,Y




def move_robot():
    #STATIC DECLARATION OF FINAL STANDINGS
    lego_info = np.zeros( (12,4) )
    #in [x,0], x defines the piece of lego in alphabetic order
    #in [1,x], with x=0 it defines X target position of lego numb 1
    #          with x=1 it defines Y target position of lego numb 1
    #          with x=2 it defines current Z target position of lego numb 1
    #          with x=3 it defines height of lego numb 1
    
    #HARDCODED POSITIONS FOR LEGOS
    # lego_X1-Y1-Z2
    lego_info[0,0]=0.792540
    lego_info[0,1]=0.085486
    lego_info[0,2]=0.60
    lego_info[0,3]=0.057
    #lego_X1-Y2-Z1
    lego_info[1,0]=0.638743
    lego_info[1,1]=0.396839
    lego_info[1,2]=0.60
    lego_info[1,3]=0.038
    # lego_X1-Y2-Z2
    lego_info[2,0]= 0.482974
    lego_info[2,1] = 0.392723
    lego_info[2,2]=0.60
    lego_info[2,3]=0.057
     # lego_X1-Y2-Z2-CHAMFER
    lego_info[3,0]= 0.339595
    lego_info[3,1] = 0.392723
    lego_info[3,2]=0.60
    lego_info[3,3]=0.057
     # lego_X1-Y2-Z2-FILLET
    lego_info[4,0]= 0.179393
    lego_info[4,1] = 0.392723
    lego_info[4,2]=0.60
    lego_info[4,3]=0.057
     # lego_X2-Y2-Z2-FILLET
    lego_info[5,0]= 0.179193
    lego_info[5,1] = -0.365227
    lego_info[5,2]=0.60
    lego_info[5,3]=0.057
     # lego_X2-Y2-Z2
    lego_info[6,0]= 0.335391
    lego_info[6,1] = -0.359541
    lego_info[6,2]=0.60
    lego_info[6,3]=0.057
     # lego_X1-Y4-Z2
    lego_info[7,0]= 0.485200
    lego_info[7,1] = -0.368766
    lego_info[7,2]=0.60
    lego_info[7,3]=0.057
     # lego_X1-Y4-Z1
    lego_info[8,0]= 0.635625
    lego_info[8,1] = -0.378159
    lego_info[8,2]=0.60
    lego_info[8,3]=0.057
     # lego_X1-Y3-Z2-FILLET
    lego_info[9,0]= 0.792054
    lego_info[9,1] = -0.374025
    lego_info[9,2]=0.60
    lego_info[9,3]=0.057
     # lego_X1-Y3-Z2
    lego_info[10,0]= 0.793135
    lego_info[10,1] = -0.079446
    lego_info[10,2]=0.60
    lego_info[10,3]=0.057


    '''
    #SET NAME OF OBJECT TO ATTACH INTO TARGET STANDING ()
    previous_name='cafe_table_texture'
    '''
    #OPEN GRIPPER
    cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/send_gripper.py --value 0.0'
    print('apro gripper')
    os.system(cmd)
    #z_target = z_target_pos
    #LOOP FOR LEGO POSITIONS
    legos = ModelStates()
    legos = get_legos_pos() 
    for i in range(len(legos.name)):
        if("lego_" not in legos.name[i]):
            continue
        name=legos.name[i]
        x=legos.pose[i].position.x 
        y=legos.pose[i].position.y
        #z=legos.pose[i].position.z
        Y, P, R = Quaternion ( x=legos.pose[i].orientation.x, y=legos.pose[i].orientation.y, z=legos.pose[i].orientation.z, w=legos.pose[i].orientation.w).yaw_pitch_roll
        print("quaternione")
        print(-R,-P,-Y)
        if (name == 'lego_X1-Y2-Z1' or name == 'lego_X1-Y4-Z1'):
            z=0.57
        else:
            z=0.58
        if(R<1 and R>-1): #OGGETTO IN POSIZIONE UP
            R=0
            P=0
        else:		#OGGETTO IN POSIZIONE DOWN
            #INVOCO SCRIPT PER GIRARE OGGETTO
            print("before passing")
            print(R,P,Y)           
            if(R<-3 or R>3):
                orientamento='down'
            else:    
                orientamento='side'
            
            print("orientamento" + str(orientamento))
            cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/rotate.py '+str(x)+' '+str(y)+' '+str(z)+' '+str(R)+' '+str(P)+' '+str(Y)+' '+name +' '+orientamento
            os.system(cmd)            
            #RICALCOLO POSIZIONE OGGETTO DOPO ROTAZIONE
            x,y,f,R,P,Y = get_lego_pos_by_name(name)
        
        #GO INTO SAFE POSITION ABOVE OBJECT
        z = z + SAFE_Z_OFFSET
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/joint_controller.py '+' '+str(x)+' '+str(y)+' '+str(z)+' '+str(Y)+' '+str(0)+' '+str(0)
        print('muovo robot verso safe position')
        os.system(cmd)
        
        #PICK UP OBJECT
        z = z - SAFE_Z_OFFSET
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/joint_controller.py '+' '+str(x)+' '+str(y)+' '+str(z)+' '+str(Y)+' '+str(0)+' '+str(0)
        print('muovo robot verso oggetto')
        os.system(cmd)
        print('chiudo gripper')

        time.sleep(1)
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/send_gripper.py --value '+ gripper_closure
        os.system(cmd)
        print('attacco oggetto a gripper ')
        time.sleep(0.5)
        cmd = 'python3 $HOME/utecrobotics/src/gazebo_ros_link_attacher/scripts/attach.py ' + name
        os.system(cmd)


        #RETURN TO SAFE POSITION
        print('muovo robot verso safe position')
        z = z + SAFE_Z_OFFSET+0.03
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/joint_controller.py '+' '+str(x)+' '+str(y)+' '+str(z)+' '+str(Y)+' '+str(0)+' '+str(0)
        os.system(cmd)


        #SEND OBJECT TO DESTINATION
	    #choose destination
        lego_considered = -1
        if 'lego_X1-Y1-Z2' in name :
            lego_considered = 0
        if 'lego_X1-Y2-Z1' in name:
            lego_considered = 1
        if 'lego_X1-Y2-Z2' in name and( name!= 'lego_X1-Y2-Z2-CHAMFER' and name !='lego_X1-Y2-Z2-TWINFILLET' ):
            lego_considered = 2
        if 'lego_X1-Y2-Z2-CHAMFER' in name and( name!= 'lego_X1-Y2-Z2' and name !='lego_X1-Y2-Z2-TWINFILLET' ):
            lego_considered = 3
        if 'lego_X1-Y2-Z2-TWINFILLET' in name and( name!= 'lego_X1-Y2-Z2-CHAMFER' and name !='lego_X1-Y2-Z2' ):
            lego_considered = 4
        if 'lego_X2-Y2-Z2-FILLET' in name:
            lego_considered = 5
        if 'lego_X2-Y2-Z2' in name and (name!='lego_X2-Y2-Z2-FILLET'): 
            lego_considered = 6
        if 'lego_X1-Y4-Z2' in name:
            lego_considered = 7
        if 'lego_X1-Y4-Z1' in name:
            lego_considered = 8
        if 'lego_X1-Y3-Z2-FILLET' in name and (name!='lego_X1-Y3-Z2'):
            lego_considered = 9
        if 'lego_X1-Y3-Z2' in name and (name!='lego_X1-Y3-Z2-FILLET'):
            lego_considered = 10
       



            
        print('type detected: '+str(lego_considered))
        #choose right target position based on lego type detected
        x_target_pos = lego_info[lego_considered,0]
        y_target_pos = lego_info [lego_considered,1]
        z_target = lego_info [lego_considered,2]
        #update current height of pile of lego
        lego_info[lego_considered,2]+=lego_info[lego_considered,3]
        print(lego_info[lego_considered,2])
        #MOVE TO TARGET POSITION
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/joint_controller.py '+' '+str(x_target_pos)+' '+str(y_target_pos)+' '+str(z_target+SAFE_Z_OFFSET)+' '+str(0)+' '+str(0)+' '+str(0)
        print('muovo robot verso destinazione')
        os.system(cmd)
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/joint_controller.py '+' '+str(x_target_pos)+' '+str(y_target_pos)+' '+str(z_target)+' '+str(0)+' '+str(0)+' '+str(0)
        os.system(cmd)
	

        #DROP OBJECT
        time.sleep(sleep_time+0.5)
        cmd = 'python3 $HOME/utecrobotics/src/gazebo_ros_link_attacher/scripts/detach.py ' + name
        os.system(cmd)
        cmd = 'python3 $HOME/utecrobotics/src/ur5/ur5_gazebo/scripts/send_gripper.py --value 0.0'
        print('rilascio oggetto')
        os.system(cmd)
        
        '''
        name2= name + '_0'
        cmd = 'python3 $HOME/utecrobotics/src/gazebo_ros_link_attacher/scripts/attach2.py ' + name + ' ' + previous_name
        previous_name=name
        os.system(cmd)
        '''
        

if __name__ == '__main__':
    move_robot()
