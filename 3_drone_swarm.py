#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np, math


class FCUModes:
    def set_arm(self, uav_ns):
        rospy.wait_for_service(uav_ns + 'mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy(uav_ns + 'mavros/cmd/arming', CommandBool)
            arm_service(True)
        except rospy.ServiceException as e:
            rospy.logerr(f"Arming failed: {e}")

    def set_offboard_mode(self, uav_ns):
        rospy.wait_for_service(uav_ns + 'mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy(uav_ns + 'mavros/set_mode', SetMode)
            set_mode_service(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            rospy.logerr(f"Set mode failed: {e}")

class DroneController:
    def __init__(self):
        self.states = [State() for _ in range(3)]
        self.setpoints = [PoseStamped() for _ in range(3)]

        # Initial positions: all drones take off to 3 meters
        self.positions = [(0.0, -0.3, 3.0), (0.0, 0.3, 3.0), (0.6*math.cos(math.radians(30)), 0.0, 3.0)]
        for i in range(3):
            self.setpoints[i].pose.position.x = self.positions[i][0]
            self.setpoints[i].pose.position.y = self.positions[i][1]
            self.setpoints[i].pose.position.z = self.positions[i][2]

    def state_cb(self, msg, index):
        self.states[index] = msg

    def takeoff(self, pub_list):
        for i in range(3):
            pub_list[i].publish(self.setpoints[i])


def main(): 
    rospy.init_node('drone_takeoff', anonymous=True)
    modes = FCUModes()
    controller = DroneController()
    rate = rospy.Rate(20)

    uav_ns_list = [f"uav{i}/" for i in range(3)]

    # Publishers and subscribers for each drone
    sp_pubs = []
    for i in range(3):
        rospy.Subscriber(uav_ns_list[i] + 'mavros/state', State, controller.state_cb, i)
        sp_pubs.append(rospy.Publisher(uav_ns_list[i] + 'mavros/setpoint_position/local', PoseStamped, queue_size=10))

    # Wait for all drones to arm
    while not all([state.armed for state in controller.states]):
        for i in range(3):
            modes.set_arm(uav_ns_list[i])
        rate.sleep()

    # Publish initial setpoints for takeoff
    takeoff_start = rospy.Time.now()
    while (rospy.Time.now() - takeoff_start).to_sec() < 3.0:
        controller.takeoff(sp_pubs)
        rate.sleep()

    # Set all drones to OFFBOARD mode
    for i in range(3):
        modes.set_offboard_mode(uav_ns_list[i])

    rospy.loginfo("Drones armed and in OFFBOARD mode")

    # Keep publishing setpoints to maintain hover
    while not rospy.is_shutdown():
        controller.takeoff(sp_pubs)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
