#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import *
import numpy as np

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform, Quaternion
import std_msgs.msg
from geometry_msgs.msg import Point
import tf

# import csv



class Controller:
    # initialization method
    def __init__(self):
        # Drone state

        self.commandMultiDoFPub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

        self.ALT_SP = 1.0
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self.inactive = True

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_quat[0] = msg.pose.orientation.x
        self.local_quat[1] = msg.pose.orientation.y
        self.local_quat[2] = msg.pose.orientation.z
        self.local_quat[3] = msg.pose.orientation.w

        orientation_list = [self.local_quat[0], self.local_quat[1], self.local_quat[2], self.local_quat[3]]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.inactive = False

        # print("hi")

    def multiDoFPub(self):

        self.traj = MultiDOFJointTrajectory()
        self.des_z = 1.2

        # header = std_msgs.msg.Header()
        # header.stamp = rospy.Time()

        now = rospy.Time.now()
        self.traj.header.stamp = now

        # transforms =Transform(translation=Point(desired_x, desired_y, desired_z), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
        transforms =Transform(translation=Point(self.des_x, self.des_y, self.des_z), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))

        velocities =Twist()
        accelerations=Twist()
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(2))

        self.traj.points.append(point)
        # self.traj.header.stamp.secs = int(time)
        # self.traj.header.stamp.nsecs = (time%1)*1000000000
        self.commandMultiDoFPub.publish(self.traj)

    def transformation(self):
        dx = -0.15
        self.des_x = self.local_pos.x + dx*np.cos(self.yaw)
        self.des_y = self.local_pos.y + dx*np.sin(self.yaw)

        print(self.des_x, self.des_y)
        # rospy.shutdown()


# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    # Subscribe to drone's local position
    rospy.Subscriber('vrpn_client_node/RigidBody002/pose', PoseStamped, cnt.posCb)
    while (cnt.inactive):
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        
        cnt.transformation()
        cnt.multiDoFPub()
        rate.sleep()
    



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass