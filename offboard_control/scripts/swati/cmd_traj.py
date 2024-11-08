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

import csv



class Controller:
    # initialization method
    def __init__(self):
        # Drone state

        self.commandMultiDoFPub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
        self.cmd_enable_ = False

    def odomCb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.q0 = msg.pose.pose.orientation.w
        self.q1 = msg.pose.pose.orientation.x
        self.q2 = msg.pose.pose.orientation.y
        self.q3 = msg.pose.pose.orientation.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z

    def multiDoFPub(self, desired_x, desired_y, desired_z,time):

        self.traj = MultiDOFJointTrajectory()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()

        transforms =Transform(translation=Point(desired_x, desired_y, desired_z), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))

        velocities =Twist()
        accelerations=Twist()
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time(2))

        self.traj.points.append(point)
        self.traj.header.stamp.secs = int(time)
        self.traj.header.stamp.nsecs = (time%1)*1000000000
        self.commandMultiDoFPub.publish(self.traj)

    def enablePub(self, msg):
        self.cmd_enable_ = True


# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    # Subscribe to drone's local position
    # rospy.Subscriber('/vrpn_client_node/RigidBody002/pose', PoseStamped, cnt.odomCb)
    rospy.Subscriber('mavros/setpoint_attitude/thrust', Thrust, cnt.enablePub)


    # command_publisher = rospy.Publisher('/mavros/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

    with open('mobilerobotpose-vrpn_client_node-RigidBody002-pose.csv') as csv_file:
        print("file read")
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        abs_time = []
        time_ = []
        pos_x_ = []
        pos_y_ = []
        for row in csv_reader:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                # print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
                # print(row)
                time_.append(float(row[2])+float(row[3])/1000000000)
                pos_x_.append(float(row[5]))
                pos_y_.append(float(row[6]))
                # abs_time.append(float(row[0]))
                line_count += 1
        # print(f'Processed {line_count} lines.')

    pos_x = np.array(pos_x_)
    pos_y = np.array(pos_y_)
    time = np.array(time_)
    time = time-time[0]

    # print(np.max(pos_x))
    offset_x = (np.max(pos_x) + np.min(pos_x))/2
    offset_y = (np.max(pos_y) + np.min(pos_y))/2
    pos_x = (pos_x - offset_x)/1.5 
    pos_y = (pos_y - offset_y)/1.5
    print((pos_x[0], pos_y[0]))

    sleep_times_ = [time[i+1] - time[i] for i in range(len(time)-1)]
    sleep_times_.append(0.1)

    print((pos_x[899], pos_y[899]))

    sleep_times = np.array(sleep_times_)

    # print((np.max(pos_x), np.min(pos_x)))
    # print((np.max(pos_y), np.min(pos_y)))
    while cnt.cmd_enable_ == False:
        rospy.sleep(0.01)
        # print(cnt.cmd_enable_)
    # cnt.multiDoFPub(pos_x[0], pos_y[0], 1.2, time[0])
    rospy.sleep(4.0)       

    # ROS main loop
    for i in range(900):
        cnt.multiDoFPub(pos_x[i], pos_y[i], 1.2, time[i])
        rospy.sleep(sleep_times[i])
        rospy.sleep(0.02)
    rospy.sleep(1.0)
    cnt.multiDOFPub(pos_x[899], pos_y[899], 1.2, time[899])


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass