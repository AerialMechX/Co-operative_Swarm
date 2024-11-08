#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import numpy as np
import tf.transformations as transformations
from std_msgs.msg import Float32


class fcuModes:
    def setArm(self, uav):
        rospy.wait_for_service(uav + 'mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(uav + 'mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setOffboardMode(self, uav):
        rospy.wait_for_service(uav + 'mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(uav + 'mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service set_mode call failed: %s. Offboard Mode could not be set." % e)


class Controller:
    def __init__(self):
        # Drone states and setpoints
        self.states = [State() for _ in range(5)]
        self.sps = [PoseStamped() for _ in range(5)]

        # Initial positions (set all to 3 meters altitude)
        self.positions = [(0.0, 3.0, 3.0), (-3.0, 3.0, 3.0), (3.0, 3.0, 3.0), (-3.0, -3.0, 3.0), (3.0, -3.0, 3.0)]
        for i, sp in enumerate(self.sps):
            sp.pose.position.x = self.positions[i][0]
            sp.pose.position.y = self.positions[i][1]
            sp.pose.position.z = self.positions[i][2]  # Set initial altitude

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

    # Function to plan smooth position changes
    def smooth_position_change(self, target_positions):
        step_size = 0.05  # Small incremental step
        for i in range(5):
            current_x = self.sps[i].pose.position.x
            current_y = self.sps[i].pose.position.y
            target_x, target_y, target_z = target_positions[i]

            # Move in small increments towards the target position
            diff_x = target_x - current_x
            diff_y = target_y - current_y

            self.sps[i].pose.position.x += np.sign(diff_x) * min(abs(diff_x), step_size)
            self.sps[i].pose.position.y += np.sign(diff_y) * min(abs(diff_y), step_size)

        return all(
            np.isclose(self.sps[i].pose.position.x, target_positions[i][0], atol=step_size) and
            np.isclose(self.sps[i].pose.position.y, target_positions[i][1], atol=step_size)
            for i in range(5)
        )


def main():
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()
    cnt = Controller()
    rate = rospy.Rate(30)

    uav_ns = [f"uav{i}/" for i in range(5)]  # Namespaces for 5 drones

    # Subscribers and Publishers
    sp_pubs = []
    for i in range(5):
        rospy.Subscriber(uav_ns[i] + 'mavros/state', State, cnt.stateCb, i)
        sp_pubs.append(rospy.Publisher(uav_ns[i] + 'mavros/setpoint_position/local', PoseStamped, queue_size=10))

    print("ARMING")
    while not all([state.armed for state in cnt.states]):
        for i in range(5):
            modes.setArm(uav_ns[i])
        rate.sleep()

    # Publish initial setpoints for 3 seconds (hover)
    hover_start_time = rospy.Time.now()
    while (rospy.Time.now() - hover_start_time).to_sec() < 3.0:
        for i in range(5):
            sp_pubs[i].publish(cnt.sps[i])
        rate.sleep()

    # Transition to OFFBOARD mode
    for i in range(5):
        modes.setOffboardMode(uav_ns[i])
    print("---------")
    print("OFFBOARD")
    print("---------")

    # Main loop for switching positions
    target_positions = cnt.positions[:]
    last_switch_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Hover for 3 seconds after each switch
        if (current_time - last_switch_time).to_sec() >= 3.0:
            # Rotate positions smoothly
            cnt.smooth_position_change(target_positions)
            last_switch_time = current_time

        # Publish the new setpoints
        for i in range(5):
            sp_pubs[i].publish(cnt.sps[i])

        # Check if all drones reached their positions, then rotate
        if cnt.smooth_position_change(target_positions):
            # Rotate the target positions (circular shift)
            target_positions = target_positions[-1:] + target_positions[:-1]

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
