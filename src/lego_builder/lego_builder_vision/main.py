#! /usr/bin/env python3

import os
import cv2
import math
import torch
import warnings
import numpy as np
from cv_bridge import CvBridge
from pyquaternion import Quaternion as PyQuaternion

# import the necessary ROS packages
import message_filters
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point

# import the custom packages
import my_utils
from yofo.model import Yofo

PKG_PATH = os.path.dirname(os.path.abspath(__file__))
MODELS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")

MODELS = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET"]

POSES = ["UP", "DOWN", "NORTH", "SOUTH", "EAST", "WEST"]


def init(camera_name):
    """
    Initialize the camera parameters
    """
    from sensor_msgs.msg import Image, CameraInfo
    from gazebo_msgs.srv import GetModelState
    camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    rospy.wait_for_service("/gazebo/get_model_state")
    camera_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    camera_state = camera_state_srv("kinect", "")  # Change this to your model name
    camera_pose = my_utils.rob2cam(camera_state.pose)  # Convert to camera axis convention

    # Build camera reference frame
    camera_frame = np.zeros((4, 4), dtype=np.float64)
    camera_frame[:, 3] = [camera_pose.position.x, camera_pose.position.y, camera_pose.position.z, 1]
    camera_frame[:3, :3] = camera_pose.orientation.rotation_matrix

    # Convert camera pose to numpy
    camera_pose.position = np.array((
        abs(camera_pose.position.x),
        abs(camera_pose.position.y),
        camera_pose.position.z), dtype=np.float64)
    # Convert quaternion to pyquaternion
    camera_pose.orientation = PyQuaternion(
        x=camera_pose.orientation.x,
        y=camera_pose.orientation.y,
        z=camera_pose.orientation.z,
        w=camera_pose.orientation.w
    )
    camera_view = (camera_info.width, camera_info.height)
    camera_matrix = np.array(camera_info.K).reshape((3, 3))
    dist_coeffs = np.array(camera_info.D)

    return camera_frame, camera_view, camera_matrix, dist_coeffs


def detect_yofo(image):
    pred = yofo(image).squeeze()
    clss = pred.argmax(dim=0, keepdim=True).flatten().cpu().item()
    conf = pred[clss].cpu().item()
    #clss = pred.sum(axis=0).argmax(dim=0, keepdim=True).flatten().cpu()
    #conf = pred.sum(axis=0)[clss].cpu().item()
    return clss, conf


def detect_yolo(image):
    pred = yolo(image).pandas()
    return pred


###############################################################################
# Callback
###############################################################################
def camera_callback(image_color, image_depth, model_infos):
    global image_view
    image_color = CvBridge().imgmsg_to_cv2(image_color, "bgr8")
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")

    # Convert depth map to rgb grayscale
    image_depth = -depth + depth.max()
    image_depth *= 255 / 0.20
    image_depth = cv2.cvtColor(image_depth, cv2.COLOR_GRAY2RGB)

    # CREATE THE ESTIMATED MODEL STATES MESSAGE
    estimated_model_states = ModelStates()

    res = detect_yolo(image_depth)
    for x1, y1, x2, y2, clss_conf, clss_id, clss_nm in res.xyxy[0].to_numpy()[:, :7]:
        if clss_conf < 0.65:
            continue

        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # crop the depth image from the bounding box
        depth_crop = depth[y1:y2, x1:x2]
        depth_crop_max = depth_crop.max()

        model_info = model_infos[clss_id]

        # Find the contour of the object
        thresh = depth_crop_max - 0.005
        rect, angle, depth_crop = my_utils.min_area_crop(depth_crop, thresh)
        if depth_crop is None:  # Object was not found
            continue

        # Resize the cropped depth image to a 32x32 image
        depth_crop = cv2.resize(depth_crop, (32, 32), interpolation=cv2.INTER_NEAREST)
        depth_crop = np.expand_dims(depth_crop, axis=(0, 3))

        # Infer pose using custom model
        pose_id, pose_conf = detect_yofo(torch.from_numpy(depth_crop))

        # Find the x-y-z coordinates of the object
        model_xyz = np.array(
            [int(rect[0][0] + x1),
             int(rect[0][1] + y1),
             depth_crop_max,
             1],
            dtype=np.float32)
        model_xyz[:2] /= image_color.shape[1], image_color.shape[0]
        model_xyz[:2] -= 0.5
        #model_xyz[:2] -= model_xyz[:2]*2*0.02  # correct for perspective
        model_xyz[:2] *= (0.900, 0.900)
        model_xyz = np.dot(camera_frame, model_xyz)

        # Get the camera facing axis of the object
        axis = "x"  # axis facing the camera

        # Calculate xy-plane rotation mod 180°
        rotZ = - rect[2] / 90 * (math.pi/2)
        horizontal = rect[1][0] < rect[1][1]  # rect long side is along x-axis

        # Calculate the object quaternion
        if pose_id == 0:    # UP
            rotZ = rotZ if horizontal else rotZ + math.pi / 2
            quat = PyQuaternion(axis=(0, 0, 1), angle=0)        # z-axis along world-frame z-axis
        elif pose_id == 1:  # DOWN
            rotZ = rotZ if horizontal else rotZ + math.pi / 2
            quat = PyQuaternion(axis=(0, 1, 0), angle=math.pi)  # z-axis along world-frame negative z-axis
        elif pose_id == 2:  # NORTH
            quat = PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)
            rotZ += 0
        elif pose_id == 3:  # SOUTH
            quat = PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)
            rotZ += math.pi
        elif pose_id == 4:  # EAST
            quat = PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)
            rotZ += -math.pi/2
        elif pose_id == 5:  # WEST
            quat = PyQuaternion(axis=(1, 0, 0), angle=math.pi/2)
            rotZ += math.pi/2
        else:
            raise ValueError("Unknown pose id: {}".format(pose_id))
        if axis == "x":
            quat *= PyQuaternion(axis=(0, 0, 1), angle=math.pi / 2)
        quat = PyQuaternion(axis=(0, 0, 1), angle=rotZ) * quat  # apply x-y world plane rotation

        # Store the estimated pose
        estimated_model_states.name.append(model_info["name"])
        estimated_model_states.pose.append(Pose(
            Point(*model_xyz[:3]),
            quat
        ))

        """
            VISUALIZATION
        """
        rot_tra = np.zeros((4, 4), dtype=np.float64)
        # Rotation matrix
        rot_tra[:3, :3] = quat.rotation_matrix
        # Trasform matrix
        rot_tra[:4, 3] = (*model_xyz[:3], 1.0)

        # Axes of the model
        axes = np.ones((4, 4), dtype=np.float64)
        axes[:, :3] = np.array([
            [0, 0, 0],    # origin
            [.1, 0, 0],   # x
            [0, .1, 0],   # y
            [0, 0, .1]])  # z
        axes = np.dot(rot_tra, axes.T)[:3, :]
        axes_2dproj = my_utils.projectPoints(
            axes,
            abs(camera_frame[:3, 3]), camera_frame[:3, :3],
            camera_matrix, dist_coeffs).reshape(-1, 2).astype(np.int32)

        cv2.line(image_color,
                 axes_2dproj[0],
                 axes_2dproj[1],
                 (0, 0, 255), 2)  # B G R
        cv2.line(image_color,
                 axes_2dproj[0],
                 axes_2dproj[2],
                 (0, 255, 0), 2)
        cv2.line(image_color,
                 axes_2dproj[0],
                 axes_2dproj[3],
                 (255, 0, 0), 2)

        # visualize the yolo bounding box
        # cv2.rectangle(image_show, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # visualize the rotated bounding box
        box = np.int0(cv2.boxPoints(rect) + (x1, y1))
        cv2.drawContours(image_color, [box], 0, (0, 0, 255), 2)

        # visualize the pose
        text = f"{POSES[pose_id]} {pose_conf:.2f}"
        cv2.putText(image_color, text, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # visualize the class
        text = f"{model_info['name']} {clss_conf:.2f}"
        cv2.putText(image_color, text, (x1, y2+10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)

    # Send the estimated model poses
    pub_model_states.publish(estimated_model_states)
    image_view = image_color


if __name__ == "__main__":
    # Ignore warnings due to YOLOv5 spamming the console when running on CPU
    warnings.simplefilter("ignore")

    # Setting default device for pytorch
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.set_grad_enabled(False)

    print("Initializing vision node")
    rospy.init_node("ros_yolo")

    print("  Loading camera sensor parameters")
    camera_frame, camera_view, camera_matrix, dist_coeffs = init("kinect")

    print("  Loading pytorch models")
    yolo = torch.hub.load(
        f"{PKG_PATH}/yolo/yolov5",
        "custom",
        path=f"{PKG_PATH}/yolo/best.20epoch.pt",
        force_reload=True,
        device=device,
        source="local").eval()
    print(f"  + Loaded YOLO on {device}")

    yofo = Yofo().eval()
    yofo.load_state_dict(torch.load(f"{PKG_PATH}/yofo/last.pt", map_location=device))
    print(f"  + Loaded YOFO on {device}")

    print(f"  Loading 3D models dimensions")
    model_infos = []
    for m in MODELS:
        model_infos.append({
            "name": m,
            "size": (0, 0, 0)#my_utils.get_model_size(m)
        })

    print(f"  Initializing ROS publisher and ROS subscriber")
    pub_model_states = rospy.Publisher("estimated_model_states", ModelStates, queue_size=1)

    sub_image_color = message_filters.Subscriber("/camera/color/image_raw", Image)
    sub_image_depth = message_filters.Subscriber("/camera/depth/image_raw", Image)
    ts = message_filters.TimeSynchronizer([sub_image_color, sub_image_depth], 1, reset=True)  # exact sync
    ts.registerCallback(camera_callback, model_infos)

    rate = rospy.Rate(1)
    image_view = np.zeros((camera_view[1], camera_view[0], 3), dtype=np.uint8)
    print(f"Starting main loop...")
    # Visualize results from camera_callback
    #rospy.spin()
    while not rospy.is_shutdown():
        cv2.imshow("Predictions", image_view)
        cv2.waitKey(1)
        rate.sleep()
