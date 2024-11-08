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



class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        # self.x = None
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.roll, self.pitch, self.yaw = 0,0,0


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


# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    # Subscribe to drone's local position
    rospy.Subscriber('/vrpn_client_node/RigidBody002/pose', PoseStamped, cnt.posCb)


    tx = 0.3 #10 cm
    ty = 0.0

    # ROS main loop
    while not rospy.is_shutdown():

        old_pos = np.array([cnt.local_pos.x, cnt.local_pos.y, cnt.local_pos.z])
        x_new = cnt.local_pos.x + tx*np.cos(cnt.yaw) - ty*np.sin(cnt.yaw)
        y_new = cnt.local_pos.y + tx*np.sin(cnt.yaw) + ty*np.cos(cnt.yaw)

        print(old_pos)
        print(x_new, y_new)
        print("----------")
      
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass