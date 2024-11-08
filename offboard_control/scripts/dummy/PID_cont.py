#!/usr/bin/env python
import sys
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *

import numpy as np
from tf.transformations import *

# Kpos = np.array([-2, -2, -3])
# Kvel = np.array([-2, -2, -3])


Kpos = np.array([-2, -2, -4])
Kvel = np.array([-2, -2, -4])
Kint = np.array([-0.01, -0.01, -0.1])
mass = 1.6
errInt = np.zeros(3)
max_acc = 4

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
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        
        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone

        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 1.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Gains
        self.Kpos = np.array([-2, -2, -4])
        self.Kvel = np.array([-2, -2, -3])
        self.Kint = np.array([-0.01, -0.01, -0.1])

        # Time parameters
        # self.

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_quat[0] = msg.pose.orientation.x
        self.local_quat[1] = msg.pose.orientation.y
        self.local_quat[2] = msg.pose.orientation.z
        self.local_quat[3] = msg.pose.orientation.w

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.x
        self.sp.pose.position.y = self.local_pos.y
        # self.sp.position.z = self.local_pos.z

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

    def newPoseCB(self, msg):
        if(self.sp.pose.position != msg.pose.position):
            print("New pose received")
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
    
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w
    def a_des():
        pass

    def acc2quat():
        pass

    def geo_con():
        pass

    def pub_att():
        pass


def a_des(currPos, desPos, currVel, pre_time):
    # Kpos = np.array([-3.0,-3.0,-4])
    # Kvel = np.array([-1.5,-1.5,-2])
    global errInt


    gravity = np.array([0, 0, 9.8])
    desVel = np.array([0.0, 0, 0])
    now = rospy.get_time()
    dt = now - pre_time
    pre_time = now
    if dt>0.034:
        dt = 0.034
    # print(dt)


    errPos = (currPos - desPos) 
    errVel = (currVel - desVel)
    # print(errInt[2])
    # print(errPos[2])
    des_a = Kpos*errPos + Kvel*errVel + Kint*errInt
    errInt = errInt + errPos*dt
    if(np.linalg.norm(des_a) > max_acc):
        des_a = (max_acc/np.linalg.norm(des_a))*des_a
    des_a = des_a + gravity


    # print(des_a)
    # print("---------")now = rospy.Time.now()

    return pre_time, des_a

def acc2quaternion(des_a, des_yaw):

    proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
    if np.linalg.norm(des_a) == 0.0:
        zb_des = np.array([0,0,1])
    else:    
        zb_des = des_a / np.linalg.norm(des_a)
    yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
    xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
    
    rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))


    # print(rotmat)
    # print(des_quat)
    # print("--------")
    return rotmat

def geometric_attcontroller(r_cur, des_a):
    # x_fac, y_fac, z_fac = 0.2, 0.2, 0.1
    x_fac, y_fac, z_fac = 1, 1, 1

    norm_thrust_const = 0.06
    r_des = acc2quaternion(des_a, 0)
    rot_44 = np.vstack((np.hstack((r_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))

    des_quat = quaternion_from_matrix(rot_44)


    # err_matrix = 0.5 * ((np.transpose(r_des)).dot(r_cur) - (np.transpose(r_cur)).dot(r_des))
    # err = np.array([err_matrix[2,1], err_matrix[0,2], err_matrix[1,0]])

    # bd_rtX = (2.0/x_fac)*err[0]
    # bd_rtY = (2.0/y_fac)*err[1]
    # bd_rtZ = (2.0/z_fac)*err[2]

    zb = r_des[:,2]
    # zbd = r_cur[0:3:,2]
    thrust = norm_thrust_const * des_a.dot(zb)
    thrust = np.maximum(0.0, np.minimum(thrust, 0.8))

    return des_quat, thrust



# Main function
def main(argv):
    
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Setpoint publisher    
    movement_cmd = AttitudeTarget()
    thrust_cmd = Thrust()
    orientation_cmd = PoseStamped()
    bodyrate_thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    orientation_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
    thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)

    print("ARMING")
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    k=0
    while k<20:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # x_fac = float(argv[0])
    # y_fac = float(argv[1])
    # z_fac = float(argv[2])
    x_fac = 1
    y_fac = 1
    z_fac = 1
    des_orientation = np.array([0,0,0,1])

    des_a = np.array([0, 0, 0])
    pre_time = rospy.get_time()


    # ROS main loop
    while not rospy.is_shutdown():
        # r_des = quaternion_matrix(des_orientation)[:3,:3]
        # r_cur = quaternion_matrix(cnt.local_quat)[:3,:3]

#--------------------------------------------
        currPos = np.array([cnt.x, cnt.y, cnt.z])
        desPos = np.array([cnt.sp.pose.position.x, cnt.sp.pose.position.y, cnt.sp.pose.position.z])
        currVel = np.array([cnt.vx, cnt.vy, cnt.vz])

        r_cur = quaternion_matrix(cnt.local_quat)[:3,:3]
        pre_time, des_a = a_des(currPos, desPos, currVel, pre_time)
        quat_des, thrust = geometric_attcontroller(r_cur, des_a)
        now = rospy.Time.now()
        orientation_cmd.header.stamp = now
        orientation_cmd.pose.orientation.x = quat_des[0]
        orientation_cmd.pose.orientation.y = quat_des[1]
        orientation_cmd.pose.orientation.z = quat_des[2]
        orientation_cmd.pose.orientation.w = quat_des[3]
        thrust_cmd.header.stamp = now
        thrust_cmd.thrust = thrust
        # print(thrust)

        # movement_cmd.body_rate.x = bd_rtX
        # movement_cmd.body_rate.y = bd_rtY
        # movement_cmd.body_rate.z = bd_rtZ
        # movement_cmd.thrust = thrust



        thrust_pub.publish(thrust_cmd)
        orientation_pub.publish(orientation_cmd)
        rate.sleep()
        

#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
