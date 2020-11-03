#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

class Noisier():
    def __init__(self):
        self.means = np.zeros(12)
        self.variances = np.zeros(12)
        self.initialize()

    def initialize(self):
        ## ROS NODE
        rospy.init_node('noisier', anonymous=True)
        odom_topic = rospy.get_param('~odom', "eha")
        # imu_topic_list = rospy.get_param('~imu', [])
        odom_noised_topic = rospy.get_param('~odom_noised', "")
        set_variance_service = rospy.get_param('~set_variance', "set_variance")
        rospy.loginfo("PARAMS: "+ odom_topic + ", " + odom_noised_topic)

        rospy.Subscriber(odom_topic, Odometry, self.callback_odom)
        rospy.Subscriber(set_variance_service, Float64MultiArray, self.handle_set_variance)
        print(odom_topic, odom_noised_topic)
        self.pub = rospy.Publisher(odom_noised_topic, Odometry, queue_size=10)
        rospy.spin()

    def create_noise(self):
        return np.random.normal(self.means, self.variances)

    def callback_odom(self, data):
        noise = self.create_noise()

        # position
        data.pose.pose.position.x += noise[0]
        data.pose.pose.position.y += noise[1]
        data.pose.pose.position.z += noise[2]

        # orientation
        quat = [data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        rpy = euler_from_quaternion(quat,'szyx')
        print("quat1", quat)
        rpy += noise[3:6]*0.2
        quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rzyx')
        print("quat2", quat)
        data.pose.pose.orientation.x = quat[0]
        data.pose.pose.orientation.y = quat[1]
        data.pose.pose.orientation.z = quat[2]
        data.pose.pose.orientation.w = quat[3]

        # linear velocity
        data.twist.twist.linear.x += noise[6]
        data.twist.twist.linear.y += noise[7]
        data.twist.twist.linear.z += noise[8]

        # angular velocity
        data.twist.twist.angular.x += noise[9]
        data.twist.twist.angular.y += noise[10]
        data.twist.twist.angular.z += noise[11]

        new_cov_pose = [x for x in data.pose.covariance]
        new_cov_twist = [x for x in data.twist.covariance]
        # Covariances
        for i in range(6):
            new_cov_pose[i + i*6] += self.variances[i]
            new_cov_twist[i + i*6] += self.variances[i+6]
        data.pose.covariance = new_cov_pose
        data.twist.covariance = new_cov_twist

        self.pub.publish(data)

    def handle_set_variance(self, msg):
        # variances are in the interval 0-2 for x,y,z
        # and 0-1 for the rest (can be changed in main.py)
        self.variances = [v for v in msg.data]


if __name__ == '__main__':
    myno = Noisier()
    myno.initialize()