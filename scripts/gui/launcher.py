import rospy
import os
import rospy
import rospkg
import subprocess
import roslaunch
import rosbag
from std_srvs.srv import Trigger, TriggerResponse

def start_noisy_node(odom, odom_noised, set_variance):
    """
    Does work as well from service/topic callbacks directly using rosrun
    """
    package = 'noisy'
    node_name = 'noisier.py'
    command = "rosrun {0} {1} _odom:={2} _odom_noised:={3} _set_variance:={4}".format(package, node_name, odom, odom_noised, set_variance)
    # command = "rosrun {0} {1} _odom:={2} _imu:={3} _odom_noised:={4} _set_variance:={5}".format(package, node_name, odom, imu, odom_noised, set_variance)

    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")

def start_node(odom, imu, odom_noised):
    """
    Does work as well from service/topic callbacks directly using rosrun
    """
    package = 'noisy'
    node_name = 'noisier.py'
    command = "rosrun {0} {1} _odom:={2} _imu:={3} _odom_noised:={4}".format(package, node_name, odom, imu, odom_noised)

    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")

def start_launch(launch_file):
    """
    Does work as well from service/topic callbacks using launch files
    """
    command = "roslaunch {0}".format(launch_file)

    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


def list_of_odometry_topics(bagfile):
    odom_topics = []
    imu_topics = []
    with rosbag.Bag(bagfile, 'r') as bag:
        info = bag.get_type_and_topic_info()
        for topic in info.topics:
                # print(topic, info.topics[topic].msg_type)
                if (info.topics[topic].msg_type == "nav_msgs/Odometry"):
                    odom_topics.append(topic)
    # print("the topics:", odom_topics)
    return odom_topics, imu_topics

def service_callback():
    start_launch("/home/mzhobro/evaluation/src/noisy/launch/bag1.launch")
    start_noisy_node("/husky_velocity_controller/odom", "/imu/data", "odometry/noised")
    start_noisy_node("/husky_velocity_controller/odom", "/imu/data", "odometry/noised2")

if __name__ == '__main__':
    rospy.init_node('test1', anonymous=True)
    service_callback()
    print(list_of_odometry_topics("/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test1.bag"))
    rospy.spin()