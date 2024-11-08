#!/usr/bin/env python3
import sys
# ROS python API
import rospy

from sensor_msgs.msg import Imu
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Vector3
import tf.transformations as transformations
from tf.transformations import *
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from std_msgs.msg import Float32

from gazebo_msgs.msg import ModelStates

import numpy as np
from tf.transformations import *

from msg_check.msg import PlotDataMsg 


# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
           


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.yaw_angle = Float32()
        self.yaw_angle.data = 0.0

        self.imu = Imu()
        
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 1.0
        self.sp.pose.position.z = self.ALT_SP

        self.local_pos = PoseStamped()
        self.local_vel = TwistStamped()

        self.errInt = np.zeros(3)
        self.att_cmd = PoseStamped()

        # Publishers
        self.att_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
    def posCb(self, msg):
        self.local_pos.pose.position.x = msg.pose.position.x
        self.local_pos.pose.position.y = msg.pose.position.y
        self.local_pos.pose.position.z = msg.pose.position.z
        # self.local_pos.pose.orientation.x = msg.pose.orientation.x
        # self.local_pos.pose.orientation.y = msg.pose.orientation.y
        # self.local_pos.pose.orientation.z = msg.pose.orientation.z
        # self.local_pos.pose.orientation.w = msg.pose.orientation.w

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def orien(self,msg):
        self.imu.orientation.w = msg.orientation.w
        self.imu.orientation.x = msg.orientation.x
        self.imu.orientation.y = msg.orientation.y
        self.imu.orientation.z = msg.orientation.z

    def yawAngle(self,msg):
        self.yaw_angle.data = msg.data
        self.yaw_angle.data = np.deg2rad(self.yaw_angle.data)

    def pub_att(self):
        self.sp_pub.publish(self.local_pos)
        self.att_pub.publish(self.imu)

# Main function
def main(argv):
   
    rospy.init_node('setpoint_node_swarm', anonymous=True)

    modes = fcuModes()  #flight modes
    cnt = Controller()  #controller object
    rate = rospy.Rate(30)

    rospy.Subscriber('/drone1/mavros/state', State, cnt.stateCb)

    rospy.Subscriber('/drone1/mavros/local_position/pose', PoseStamped, cnt.posCb)
    # rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    rospy.Subscriber('/drone1/mavros/imu/data', Imu, cnt.orien)

    print("ARMING")

    while not cnt.state.armed:
        modes.setArm()
        cnt.armed = True
        rate.sleep()

    cnt.armed = True
    k=0

    while k<20:
        cnt.sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():

#--------------------------------------------
        cnt.pub_att()
        rate.sleep()
#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass

