#!/usr/bin/python
import os
import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

BATCH_SIZE = 20

# Gaussian distribution
POS_AVG = Point(x = 0.79536, y = -0.083997, z = 1)  # Average   (Mu)
POS_DEV = 0.08                                      # Deviation (Sigma)

rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 1    # Pose vector
initial_pose.position.y = 1
initial_pose.position.z = 1
initial_pose.orientation.x = 0 # Orientation quaternion
initial_pose.orientation.y = 0
initial_pose.orientation.z = 0
initial_pose.orientation.w = 0

models = [f"../.gazebo/models/{x}" for x in os.listdir("../.gazebo/models") if "lego_" in x]

models_batch = []
for _ in range(0, BATCH_SIZE):
    models_batch.append((
        random.choice(models),
        Pose(
            position = Point(
                x = random.gauss(POS_AVG.x, POS_DEV),
                y = random.gauss(POS_AVG.y, POS_DEV),
                z = random.gauss(POS_AVG.z, POS_DEV)),
            orientation = Quaternion(
                x = random.uniform(0, 3),
                y = random.uniform(0, 3),
                z = random.uniform(0, 3),
                w = 0))
        ))

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

for i, (m, pose) in enumerate(models_batch):
    f = open(f"{m}/model.sdf",'r')
    sdff = f.read()

    spawn_model_prox(f"{m.split('/')[-1]}_{i}", sdff, "robotos_name_space", pose, "world")
    print(f"Spawned {m} as {m.split('/')[-1]}_{i}")
