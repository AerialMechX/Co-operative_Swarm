#!/usr/bin/env python3
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from geometry_msgs.msg import Quaternion
import tf.transformations as transformations
from std_msgs.msg import Float32

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self, uav):
        rospy.wait_for_service(uav + 'mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(uav + 'mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

    def setArm(self, uav):
        rospy.wait_for_service(uav + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(uav + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self, uav):
        rospy.wait_for_service(uav + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(uav + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setOffboardMode(self, uav):
        rospy.wait_for_service(uav + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(uav + 'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service set_mode call failed: %s. Offboard Mode could not be set." % e)

class Controller:
    # initialization method
    def __init__(self):
        # Drone states
        self.states = [State() for _ in range(5)]
        # Instantiate setpoints for each drone
        self.sps = [PoseStamped() for _ in range(5)]

        # Initial values for setpoints
        positions = [
            (0.0, 3.0), (-3.0, 3.0), (3.0, 3.0), (-3.0, -3.0), (3.0, -3.0)
        ]
        for i, sp in enumerate(self.sps):
            sp.pose.position.x = positions[i][0]
            sp.pose.position.y = positions[i][1]
            sp.pose.position.z = 3.0

        self.yaw_angles = [Float32() for _ in range(5)]
        for yaw_angle in self.yaw_angles:
            yaw_angle.data = 0.0

    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*quaternion)

    def yawAngle(self, msg, index):
        self.yaw_angles[index].data = msg.data
        self.yaw_angles[index].data = np.deg2rad(self.yaw_angles[index].data)
        q = self.euler_to_quaternion(0, 0, self.yaw_angles[index].data)

        self.sps[index].pose.orientation.x = q.x
        self.sps[index].pose.orientation.y = q.y
        self.sps[index].pose.orientation.z = q.z
        self.sps[index].pose.orientation.w = q.w

    def newPoseCB(self, msg, index):
        self.sps[index].pose.position.x = msg.pose.position.x
        self.sps[index].pose.position.y = msg.pose.position.y
        self.sps[index].pose.position.z = msg.pose.position.z

    def stateCb(self, msg, index):
        self.states[index] = msg

# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  # flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    uav_ns = [f"uav{i}/" for i in range(5)]  # Namespaces for 5 drones

    # Subscribers and Publishers for each drone
    sp_pubs = []
    for i in range(5):
        rospy.Subscriber(uav_ns[i] + 'mavros/state', State, cnt.stateCb, i)
        rospy.Subscriber(uav_ns[i] + 'new_pose', PoseStamped, cnt.newPoseCB, i)
        rospy.Subscriber(uav_ns[i] + 'yaw_in_deg', Float32, cnt.yawAngle, i)
        sp_pubs.append(rospy.Publisher(uav_ns[i] + 'mavros/setpoint_position/local', PoseStamped, queue_size=10))

    print("ARMING")
    while not all([state.armed for state in cnt.states]):
        for i in range(5):
            modes.setArm(uav_ns[i])  # Pass UAV namespace to setArm
        rate.sleep()

    k = 0
    while k < 20:
        for i in range(5):
            sp_pubs[i].publish(cnt.sps[i])
        rate.sleep()
        k = k + 1

    for i in range(5):
        modes.setOffboardMode(uav_ns[i])  # Pass UAV namespace to setOffboardMode
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():
        for i in range(5):
            sp_pubs[i].publish(cnt.sps[i])
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
