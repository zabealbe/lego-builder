#!/bin/python3

import os
import sys
import math
import rospy


import random
from pyquaternion import Quaternion as PyQuaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates

import time
random.seed(time.time())

TABLE_CENTER = Point(x=0, y=-0.558, z=0.80)

spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
delete_model_prox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

BBOX_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.4">
  <model name="{name}">
  <static>true</static>
  <link name="{name}">
  <visual name="{name}_top">
    <pose>0 {-height/2} 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{width} {stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_right">
    <pose>{width/2} 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{stroke} {height} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_bottom">
    <pose>0 {height/2} 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{width} {stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_left">
    <pose>{-width/2} 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{stroke} {height} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
  </link>
  </model>
</sdf>\n"""


def spawn_bboxes(pose):
    point = pose.position
    # generate 11 different colours
    colours = [
        (0.0, 0.0, 0.0, 1),
        (1.0, 0.0, 0.0, 1),
        (0.0, 1.0, 0.0, 1),
        (0.0, 0.0, 1.0, 1),
        (1.0, 1.0, 0.0, 1),
        (0.0, 1.0, 1.0, 1),
        (1.0, 0.0, 1.0, 1),
        (1.0, 1.0, 1.0, 1),
        (1.0, 0.5, 0.0, 1),
        (0.5, 1.0, 0.0, 1),
        (0.0, 0.5, 1.0, 1)
    ]
    # generate random sizes
    sizes = [(random.uniform(0.03, 0.1), random.uniform(0.03, 0.1)) for _ in range(11)]
    for i, (colour, size) in enumerate(zip(colours, sizes)):
        pose.position.x += 0.1
        spawn_bbox(*size, pose, colour)


def spawn_bbox(width, height, pose, colour=(0, 0, 0, 1)):
    name = f"bbox_{random.randint(0, 100000)}"
    stroke = 0.01
    rgba = " ".join([str(x) for x in colour])

    bbox_sdf = f"""<?xml version="1.0"?>
<sdf version="1.4">
  <model name="{name}">
  <static>true</static>
  <link name="{name}">
  <visual name="{name}_top">
    <pose>0 {-height/2} 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{width + stroke} {stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_right">
    <pose>{width/2} 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{stroke} {height + stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_bottom">
    <pose>0 {height/2} 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{width + stroke} {stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
    <visual name="{name}_left">
    <pose>{-width/2} 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>{stroke} {height + stroke} 0.0002</size>
      </box>
    </geometry>
    <material>
      <ambient>{rgba}</ambient>
      <diffuse>{rgba}</diffuse>
      <specular>{rgba}</specular>
      <emissive>{rgba}</emissive>
    </material>
  </visual>
  </link>
  </model>
</sdf>\n"""
    spawn_model_prox(name, bbox_sdf, "", pose, "world")


def spawn_checkerboard():
    checkerboard_path = "../models/checkerboard/checkerboard.sdf"
    pose = Pose(
        TABLE_CENTER,
        Quaternion(x=0, y=0, z=0, w=0))

    with open(checkerboard_path, "r") as f:
        spawn_model_prox("checkerboard", f.read(), "", pose, "world")


def get_legos(models_path="../models", mode="path"):
    models = [f"{x}" for x in os.listdir(models_path) if "lego_" in x]
    if mode == "path":
        models = [f"{models_path}/{x}" for x in models]
    elif mode != "name":
        raise ValueError("mode must be either 'path' or 'name'")
    models.sort()  # sort for consistency
    return models


def spawn_model(sdf_path, pose, name, color=(0, 0, 0, 1)):
    rgba = " ".join([str(x) for x in color]) + " 1"

    f = open(sdf_path, 'r')
    sdff = f.read()
    sdff = sdff.replace("<ambient>0.3 0.3 0.3 1</ambient>", f"<ambient>{rgba}</ambient>")

    spawn_model_prox(name, sdff, "", pose, "world")


def quat_from_axis_angle(axis, angle):
    quat = PyQuaternion(axis=axis, angle=angle)
    # do a swing twist
    return quat


def quat_from_random():
    quat = PyQuaternion.random()
    return quat


def random_pose(pose_mean, pose_var, distribution="gaussian"):
    if distribution == "gaussian":
        return Pose(
            Point(
                x=random.gauss(pose_mean.position.x, pose_var.position.x),
                y=random.gauss(pose_mean.position.y, pose_var.position.y),
                z=random.gauss(pose_mean.position.z, pose_var.position.z)),
            Quaternion(
                x=random.gauss(pose_mean.orientation.x, pose_var.orientation.x),
                y=random.gauss(pose_mean.orientation.y, pose_var.orientation.y),
                z=random.gauss(pose_mean.orientation.z, pose_var.orientation.z),
                w=random.gauss(pose_mean.orientation.w, pose_var.orientation.w)))
    elif distribution == "uniform":
        quat = quat_from_random()
        return Pose(
            Point(
                x=random.uniform(pose_mean.position.x - pose_var.position.x, pose_mean.position.x + pose_var.position.x),
                y=random.uniform(pose_mean.position.y - pose_var.position.y, pose_mean.position.y + pose_var.position.y),
                z=random.uniform(pose_mean.position.z - pose_var.position.z, pose_mean.position.z + pose_var.position.z)),
            quat)
    elif distribution == "lay":  # idk just make em lay on the table
        quat = PyQuaternion(axis=(0, 0, 1), angle=random.uniform(0, 2 * math.pi))
        quat *= random.choice([
            PyQuaternion(axis=(0, 1, 0), angle=0),          # UP
            PyQuaternion(axis=(0, 1, 0), angle=math.pi/2),  # LONG SIDE
            PyQuaternion(axis=(0, 1, 0), angle=math.pi),    # DOWN
            PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)   # SHORT SIDE
        ])

        return Pose(
            Point(
                x=random.uniform(pose_mean.position.x - pose_var.position.x, pose_mean.position.x + pose_var.position.x),
                y=random.uniform(pose_mean.position.y - pose_var.position.y, pose_mean.position.y + pose_var.position.y),
                z=random.uniform(pose_mean.position.z - pose_var.position.z, pose_mean.position.z + pose_var.position.z)),
            quat)
    else:
        raise ValueError("distribution must be either 'gaussian' or 'uniform'")


def random_lego(pose_mean, pose_var):
    lego = random.choice(get_legos())

    position = Point(
            x=random.uniform(pose_mean.position.x - pose_var.position.x, pose_mean.position.x + pose_var.position.x),
            y=random.uniform(pose_mean.position.y - pose_var.position.y, pose_mean.position.y + pose_var.position.y),
            z=random.uniform(pose_mean.position.z - pose_var.position.z, pose_mean.position.z + pose_var.position.z))

    quat_up         = PyQuaternion(axis=(0, 1, 0), angle=0)
    quat_down       = PyQuaternion(axis=(0, 1, 0), angle=math.pi)
    quat_side_short = PyQuaternion(axis=(1, 0, 0), angle=-math.pi / 2)
    quat_side_long  = PyQuaternion(axis=(0, 1, 0), angle=math.pi / 2)

    quat = random.choice([quat_up, quat_down, quat_side_short, quat_side_long])

    # adjust to a stable pose
    if quat == quat_up:
        pass
        #if lego.split("/")[-1] == "lego_X1-Y2-Z2-TWINFILLET":
        #    quat = PyQuaternion(axis=(1, 0, 0), angle=2.145364)
    elif quat == quat_side_long:
        if lego.split("/")[-1] == "lego_X2-Y2-Z2":
            position.z += 0.01
        elif lego.split("/")[-1] == "lego_X2-Y2-Z2-FILLET":
            position.z += 0.01
    elif quat == quat_side_short:
        if lego.split("/")[-1] == "lego_X1-Y4-Z1":
            position.z += 0.040
        elif lego.split("/")[-1] == "lego_X1-Y4-Z2":
            position.z += 0.040
        elif lego.split("/")[-1] == "lego_X1-Y2-Z2":
            position.z += 0.01
        elif lego.split("/")[-1] == "lego_X1-Y2-Z1":
            position.z += 0.01
        elif lego.split("/")[-1] == "lego_X1-Y3-Z1":
            position.z += 0.02
        elif lego.split("/")[-1] == "lego_X1-Y3-Z2":
            position.z += 0.02
        elif lego.split("/")[-1] == "lego_X1-Y2-Z2-CHAMFER":
            position.z += 0.01
        elif lego.split("/")[-1] == "lego_X2-Y2-Z2":
            position.z += 0.01
        elif lego.split("/")[-1] == "lego_X2-Y2-Z2-FILLET":
            position.z += 0.005
        elif lego.split("/")[-1] == "lego_X1-Y2-Z2-TWINFILLET":
            quat = PyQuaternion(axis=(1, 0, 0), angle=2.145364)
        if lego.split("/")[-1] == "lego_X1-Y3-Z2-FILLET":
            position.z += 0.025
    elif quat == quat_down:
        if lego.split("/")[-1] == "lego_X1-Y2-Z2-CHAMFER":
            quat = PyQuaternion(axis=(1, 0, 0), angle=2.359470)
            position.z -= 0.01
        elif lego.split("/")[-1] == "lego_X1-Y3-Z2-FILLET":
            quat = PyQuaternion(axis=(1, 0, 0), angle=2.645406)
        elif lego.split("/")[-1] == "lego_X2-Y2-Z2-FILLET":
            quat = PyQuaternion(axis=(1, 0, 0), angle=2.496794)
        position.z += 0.0025
    quat = PyQuaternion(axis=(0, 0, 1), angle=random.uniform(0, 2 * math.pi)) * quat

    pose = Pose(position, quat)

    return lego, pose


def spawn_legos(count=10, distribution="gaussian"):
    pos_avg = Pose(TABLE_CENTER, Quaternion(x=0, y=0, z=0, w=1))
    pos_dev = Pose(Point(x=0.4, y=0.4, z=0.0), Quaternion(x=0, y=0, z=0, w=1))

    models_batch = []
    for i in range(0, count):
        models_batch.append(random_lego(pos_avg, pos_dev))

    for i, (m, pose) in enumerate(models_batch):
        color = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        spawn_model(os.path.join(m, "model.sdf"), pose, f"{os.path.basename(m)}_{i}", color)


def delete_models(match):
    # get all models in gazebo
    models = rospy.wait_for_message("gazebo/model_states", ModelStates)
    for m in models.name:
        if match not in m:
            continue
        try:
            delete_model_prox(m)
        except Exception:
            print(f"Couldn't delete {m}")


if __name__ == "__main__":
    rospy.init_node('insert_object', log_level=rospy.INFO)
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    if len(sys.argv) < 2:
        print("Usage:")
        print("\tpython3 spawner.py <object>")
        print("\t\t<object> can be: c, l")
        print("\t\tc: spawn a checkerboard")
        print("\t\tl: spawn a batch of legos")
        print("\t\tb: spawn a batch of bboxes")
        exit(1)
    elif sys.argv[1][0] == "l":
        if len(sys.argv[1]) == 2:
            if sys.argv[1][1] == "d":
                delete_models("lego_")
            elif sys.argv[1][1] == "l":  # make em lay
                spawn_legos(distribution="lay")
        else:
            spawn_legos(distribution="uniform")
    elif sys.argv[1][0] == "c":
        spawn_checkerboard()
    elif sys.argv[1][0] == "b":
        if len(sys.argv[1]) > 1:
            if sys.argv[1][1] == "b":
                spawn_bboxes(Pose(Point(x=TABLE_CENTER.x-0.5, y=TABLE_CENTER.y, z=TABLE_CENTER.z), Quaternion(x=0, y=0, z=0, w=0)))
            if sys.argv[1][1] == "d":
                delete_models("bbox_")
        else:
            spawn_bbox(0.1, 0.1, Pose(TABLE_CENTER, Quaternion(x=0, y=0, z=0, w=0)))
    else:
        print("Invalid argument")
