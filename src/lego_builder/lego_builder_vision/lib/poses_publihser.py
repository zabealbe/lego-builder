import copy

import rospy
from gazebo_msgs.msg import ModelStates


def callback(message):
    iter = copy.deepcopy(message.name)
    count = 0
    for i in range(0, len(iter)):
        j = i - count
        if "lego_" not in message.name[j]:
            del message.name[j]
            del message.pose[j]
            del message.twist[j]
            count += 1
    publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    publisher = rospy.Publisher("/lego_builder/lego_states", ModelStates, queue_size=1)

    print("Started publishing on /lego_builder/lego_states")
    rospy.spin()