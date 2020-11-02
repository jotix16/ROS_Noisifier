#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler, quaternion_multiply

class Noisier():
    def __init__(self):
        ## CONSTANTS
        # position
        self.variance_position = 0.5 # meters
        #orientation
        self.variance_orient = np.pi/10 # rad
        # linear velocity
        self.variance_lin_vel = 0.3 # m/s
        # angularvelocity
        self.variance_angular_vel = np.pi/10 # rad/s

        self.means = np.zeros(12)
        self.variances = np.array([self.variance_position, self.variance_position, self.variance_position,
                                self.variance_orient, self.variance_orient, self.variance_orient,
                                self.variance_lin_vel, self.variance_lin_vel, self.variance_lin_vel,
                                self.variance_angular_vel, self.variance_angular_vel, self.variance_angular_vel])
        self.initialize()

    def initialize(self):
        ## ROS NODE
        rospy.init_node('noisier', anonymous=True)

        odom_topic = rospy.get_param('~odom', "eha")
        imu_topic = rospy.get_param('~imu', "")
        odom_noised_topic = rospy.get_param('~odom_noised', "")
        rospy.loginfo("PARAMS: "+ odom_topic + ", " + odom_noised_topic)

        rospy.Subscriber(odom_topic, Odometry, self.callback_odom)
        rospy.Subscriber('set_variance', Float64MultiArray, self.handle_set_variance)
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
        # quat = [data.pose.pose.orientation.x,
        #         data.pose.pose.orientation.y,
        #         data.pose.pose.orientation.z,
        #         data.pose.pose.orientation.w]

        # q = quaternion_from_euler(noise[3], noise[4], noise[5], 'rzyx')
        # quat = quaternion_multiply(quat , q)

        # data.pose.pose.orientation.x = quat[0]
        # data.pose.pose.orientation.y = quat[1]
        # data.pose.pose.orientation.z = quat[2]
        # data.pose.pose.orientation.w = quat[3]

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
        self.variances = [v for v in msg.data]
        # print(self.variances)


if __name__ == '__main__':
    myno = Noisier()
    myno.initialize()