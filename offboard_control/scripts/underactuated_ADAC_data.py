#!/usr/bin/env python
import sys
# ROS python API
import rospy
from sensor_msgs.msg import Imu
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from std_msgs.msg import Float32
from msg_check.msg import PlotDataMsg
# from scipy import linalg as la

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetModelState

import numpy as np
from tf.transformations import *
#import RPi.GPIO as GPIO
import time

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
        self.start_pendulum = Float32()
        self.drone = Point(0.0, 0.0, 0.0)
        self.pendulum = Point(0.0, 0.0, 0.0)

        self.payload_state = PoseStamped()
        self.payload_vel  = TwistStamped()
        # set the flag to use position setpoints and yaw angle
       
        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone

        # initial values for setpoints
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        self.imu = Imu()
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.start_pendulum.data = 0.0
        self.ALT_SP = 0.8
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.array([0.01,0.01,0.01])
        self.errInt_a = np.zeros(3)
        self.errInt_u = np.zeros(2)

        self.desAcc = np.zeros(3)
        self.rho =  0.0
        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()

        # Control parameters
        self.M = 0.95
        self.m = 0.01
        self.l = 0.3

        self.curAccPrev = np.array([0.,0.,0.])
        self.thrust_prev =np.array([0.0,0.0,0.0])

        # self.KP_a = np.array([2.5, 2.5, 8.0])
        # self.KD_a = np.array([1.25, 1.25, 4.0])
        # self.KP_u = np.array([[0.2, 0.0],[0.0, 0.2],[0.0,0.0]])
        # self.KD_u = np.array([[0.02, 0],[0.0, 0.02],[0.000,0.000]])

        self.Phi = np.array([1.5, 1.5, 1.0])

        self.KP_a = np.array([3.0, 3.0, 4.0])
        self.KD_a = np.array([1.0, 1.0, 2.0])

        self.KP_u = np.array([[0.43, 0.0],[0.0,0.43],[0.0,0.0]])
        self.KD_u = np.array([[0.029, 0],[0.0, 0.029],[00000,0.000]])

        self.Kp0 = 0.01
        self.Kp1 = 0.1
        self.neta_0 = 0.9
        self.neta_1 = 0.9
        self.beta = 4.0

        self.gamma = 0.1
        self.gamma_0 = 10.0
        self.gamma_1 = 1.0
        self.gamma_2 = 0.001
        self.gamma_bar = 0.05

        self.epsilon = 1.0

        self.norm_thrust_const = 0.060
        self.max_th = 12.0
        self.max_throttle = 0.96
        self.gravity = np.array([0, 0, 9.8])
        self.pre_time = rospy.get_time()    
        self.data_out = PlotDataMsg()

        # Publishers
        self.att_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)
        self.armed = False


    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        # self.data_out.sq = self.sp.pose.position
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        # print(self.desVel)
        # print('----------------------------------------')
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])


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

    ##Start Pendulum Feedback
    def startPendulum(self, msg):
        self.start_pendulum.data = msg.data 

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.x
        self.sp.pose.position.y = self.local_pos.y
        # self.sp.position.z = self.local_pos.z

    def odomCb(self, msg):
        self.cur_pose.pose.position.x = msg.pose.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.pose.position.z

        self.cur_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.cur_pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.cur_pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.cur_pose.pose.orientation.z = msg.pose.pose.orientation.z

        self.cur_vel.twist.linear.x = msg.twist.twist.linear.x
        self.cur_vel.twist.linear.y = msg.twist.twist.linear.y
        self.cur_vel.twist.linear.z = msg.twist.twist.linear.z

        self.cur_vel.twist.angular.x = msg.twist.twist.angular.x
        self.cur_vel.twist.angular.y = msg.twist.twist.angular.y
        self.cur_vel.twist.angular.z = msg.twist.twist.angular.z

    def accCB(self,msg):
        self.imu.orientation.w = msg.orientation.w
        self.imu.orientation.x = msg.orientation.x
        self.imu.orientation.y = msg.orientation.y
        self.imu.orientation.z = msg.orientation.z

        self.imu.angular_velocity.x = msg.angular_velocity.x
        self.imu.angular_velocity.y = msg.angular_velocity.y
        self.imu.angular_velocity.z = msg.angular_velocity.z

        self.imu.linear_acceleration.x = msg.linear_acceleration.x
        self.imu.linear_acceleration.y = msg.linear_acceleration.y
        self.imu.linear_acceleration.z = msg.linear_acceleration.z

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

    def Vector1Arrays(self, vector):
        return np.array([vector.x, vector.y])

    def vector2Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z])

    def vector3Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z , vector.w])

    def array2Vector3(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]

    def array2Vector4(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]
        vector.z = array[2]
        vector.w = array[3]

    def array2Vector2(self, array, vector):
        vector.x = array[0]
        vector.y = array[1]

    def sigmoid(self, s, v):
        if np.absolute(s) > v:
            return s/np.absolute(s)
        else:
            return s/v

    def drone_states(self, msg):
        self.drone.x = msg.pose.position.x
        self.drone.y = msg.pose.position.y
        self.drone.z = msg.pose.position.z

    def pendulum_states(self, msg):
        self.pendulum.x = msg.pose.position.x
        self.pendulum.y = msg.pose.position.y
        self.pendulum.z = msg.pose.position.z

        x = np.round(-np.round(self.drone.x, 2) + np.round(self.pendulum.x, 2), 2)
        y = np.round(-np.round(self.drone.y, 2) + np.round(self.pendulum.y, 2), 2)
        z = np.round(-np.round(self.drone.z, 2) + np.round(self.pendulum.z, 2), 2)

        self.roll = (np.arctan2(x, z)) #theta
        self.pitch = (np.arctan2(y, z))  #phi

        # print(self.alpha_ang_, self.beta_ang_)

        if self.roll <= np.pi and self.roll >= 0.0:
            # print("Condition 1: ",np.rad2deg(self.roll))
            self.roll = - self.roll + np.pi
            self.roll = -self.roll
            # print("Condition 1: ",np.rad2deg(self.roll))

        elif self.roll >= -np.pi and self.roll <= 0.0:
            # print("Condition 2: ",np.rad2deg(self.roll))
            self.roll =  self.roll + np.pi
            # print("Condition 2: ",np.rad2deg(self.roll))

        if self.pitch <= np.pi and self.pitch >= 0.0:
            # print("Condition 1: ",np.rad2deg(self.roll))
            self.pitch = - self.pitch + np.pi
            self.pitch = -self.pitch
            # print("Condition 1: ",np.rad2deg(self.roll))

        elif self.pitch >= -np.pi and self.pitch <= 0.0:
            # print("Condition 2: ",np.rad2deg(self.roll))
            self.pitch =  self.pitch + np.pi
            # print("Condition 2: ",np.rad2deg(self.roll))

        self.roll = -self.roll
        self.pitch = -self.pitch
        # print(np.rad2deg(self.roll), np.rad2deg(self.pitch))
        #roll = alpha = Fx
        #pitch = beta = Fy

        self.alpha_ang = self.roll #theta
        self.beta_ang = self.pitch #phi


    def th_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04
        if dt == 0.0:
            dt = 0.01

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)
        curAcc = self.vector2Arrays(self.imu.linear_acceleration)
        curor = self.vector3Arrays(self.cur_pose.pose.orientation)

        errPos_a = curPos - desPos
        print(errPos_a)
        errVel_a = curVel - self.desVel

        des_alpha = 0.0
        des_beta = 0.0
        alpha = self.start_pendulum.data*self.alpha_ang   # Fx
        beta = self.start_pendulum.data*self.beta_ang    # Fy
        # print(self.alpha_ang,self.beta_ang)
        

        self.data_out.cur_alpha = alpha
        self.data_out.cur_beta = beta
        self.data_out.des_alpha = des_alpha
        self.data_out.des_beta = des_beta

        
        # print(alpha, beta)
        err_alpha = alpha - des_alpha
        err_beta = beta - des_beta


        errPos_u = np.array([err_alpha, err_beta])
        errVel_u = np.array([err_alpha/dt, err_beta/dt])

        self.array2Vector2(errPos_u, self.data_out.poserr_u) #Underactuated Position Error
        self.array2Vector2(errVel_u, self.data_out.velerr_u) #Underactuated Velocity Error
        self.array2Vector3(errPos_a, self.data_out.poserr_a) #Actuated Position Error
        self.array2Vector3(errVel_a, self.data_out.velerr_a) #Actuated Position Error

        # alpha = 0.0
        # beta = 0.0
        # err_alpha = alpha - des_alpha
        # err_beta = beta - des_beta
        print(err_alpha, err_beta)
        print("------------")

        # errPos_u = np.array([err_alpha, err_beta])
        # errVel_u = np.array([err_alpha/dt, err_beta/dt])

        Ma_bar = np.array([[ self.M+self.m,  -0.000462531479, 0.00308354319],
                           [ -0.000462531479,  self.M +self.m, -0.0013068977],
                           [  0.00308354319, -0.0013068977,  self.M+self.m]])


        xi_a = np.concatenate((errPos_a, errVel_a),axis = 0)
        xi_u = np.concatenate((errPos_u, errVel_u),axis = 0)
        xi = np.concatenate((errPos_a, errPos_u, errVel_a, errVel_u),axis = 0)

        xi_aNorm = np.linalg.norm(xi_a)
        xi_uNorm = np.linalg.norm(xi_u)
        xi_Norm = np.linalg.norm(xi)

        
        # define r and r_norm
        Oa = np.zeros((3,3))
        Ia = np.eye(3)
        Ba = np.vstack((Oa,Ia))
        Ba = np.array(Ba.T)

        r = errVel_a + np.multiply(self.Phi, errPos_a)
        # print(r)
        r_norm = np.linalg.norm(r)
        # print(xi_uNorm, xi_aNorm, xi_Norm)

        if xi_uNorm > 10:
            xi_uNorm = 10
        if xi_Norm > 10:
            xi_Norm = 10

        if self.armed:
            Kp0_dot = (r_norm + xi_uNorm) - self.neta_0*self.Kp0*self.beta*xi_uNorm

            # Kp1_dot = ((r_norm + xi_uNorm) - self.neta_1*self.Kp1*self.beta*xi_uNorm)*xi_Norm
            Kp1_dot = ((r_norm + xi_uNorm) - self.neta_1*self.Kp1*self.beta*xi_uNorm)

            gamma_dot = ( -1*self.gamma*(self.gamma_0 + self.gamma_1*xi_uNorm + self.gamma_2*xi_Norm**4)
                         + (r_norm + xi_uNorm) + (self.gamma_0*self.gamma_bar))

            self.Kp0 = self.Kp0 + Kp0_dot*dt
            self.Kp1 = self.Kp1 + (Kp1_dot*dt)
            self.gamma = (self.gamma + gamma_dot*dt)

            self.rho = self.Kp0 + self.Kp1*xi_Norm + self.gamma
            # self.rho = self.Kp0 + self.Kp1*xi_Norm

            # print(xi_Norm)
            # print(self.Kp0, self.Kp1*xi_Norm, self.gamma)
            # print(r, r_norm, self.rho)
            self.rho = self.rho


            self.data_out.gamma = self.gamma
            self.data_out.rho = self.rho
            self.data_out.Kp0 = self.Kp0
            self.data_out.Kp1 = self.Kp1

            
            if r_norm >= self.epsilon: 
                # del_Tau = (self.rho/r_norm)*r
                del_Tau = np.multiply(self.rho, r)/r_norm

            if r_norm < self.epsilon:
                # del_Tau = (self.rho/self.epsilon)*r
                del_Tau = np.multiply(self.rho, r)/self.epsilon

            # print(del_Tau)
            # del_Tau = np.array([0,0,0])
            
            nu = (self.desAcc 
                  - np.multiply(self.KD_a, errVel_a) 
                  - np.multiply(self.KP_a, errPos_a) 
                  - np.dot(self.KD_u, errVel_u) 
                  - np.dot(self.KP_u, errPos_u) 
                  - np.multiply(del_Tau, np.array([0.8, 0.8, 0.8]))
                  # - np.multiply(del_Tau, np.array([0.25, 0.25, 1.0]))
                  + (self.M + self.m)*(np.array([0.0,0.0,9.8])))

            ha_bar_hat =  self.thrust_prev - Ma_bar.dot(self.curAccPrev)
            # print(ha_bar_hat)
            # ha_bar_hat = np.multiply(ha_bar_hat, np.array([0.0,0.0,1]))
            ha_bar_hat = np.multiply(ha_bar_hat, np.array([0.0, 0.0, 1.0]))
            des_th =  Ma_bar.dot(nu) + ha_bar_hat

            
            # print("Cur Acc",curAcc)
            
        if np.linalg.norm(des_th) > self.max_th:
            # print("In Loop")
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th
            # self.curAccPrev = np.multiply(curAcc,np.array([0.0,0.0,0.98]))
            # print(curAcc)
            # self.thrust_prev = des_th

        self.curAccPrev = np.multiply(curAcc,np.array([0.0,0.0,0.1]))
        self.thrust_prev = np.multiply(des_th,np.array([0.0,0.0,0.1]))


        # print(des_th)

        self.array2Vector3(curPos, self.data_out.curpos) #Current Pose
        self.array2Vector3(desPos, self.data_out.despos) #Desired Pose
        self.array2Vector4(curor , self.data_out.curor)  #Current Orientation
        self.array2Vector3(curVel, self.data_out.curvel) #Current Velocity
        self.array2Vector3(self.desVel, self.data_out.desvel) #Desired Velocity

        self.array2Vector3(self.desAcc, self.data_out.desacc) #Desired Acceleration
        self.array2Vector3(curAcc, self.data_out.curacc) #Desired Acceleration
        return des_th

    def acc2quat(self, des_th, des_yaw):
        des_th = des_th[0:3]
        proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
        if np.linalg.norm(des_th) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_th / np.linalg.norm(des_th)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat

    def geo_con(self):
        des_th = self.th_des()  
        des_th = des_th[0:3]  
        r_des = self.acc2quat(des_th, 0.0)
        rot_44 = np.vstack((np.hstack((r_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))
        quat_des = quaternion_from_matrix(rot_44)
       
        zb = r_des[:,2]
        thrust = self.norm_thrust_const * des_th.dot(zb)
        # print(thrust)
        
        thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))
        self.data_out.thrust = thrust
        now = rospy.Time.now()
        self.att_cmd.header.stamp = now
        self.thrust_cmd.header.stamp = now
        self.data_out.header.stamp = now
        self.att_cmd.pose.orientation.x = quat_des[0]
        self.att_cmd.pose.orientation.y = quat_des[1]
        self.att_cmd.pose.orientation.z = quat_des[2]
        self.att_cmd.pose.orientation.w = quat_des[3]
        self.thrust_cmd.thrust = thrust
        # self.data_out.orientation = self.att_cmd.pose.orientation
        self.data_out.desor = self.att_cmd.pose.orientation #Desired Orientation
    
    def pub_att(self):
        self.geo_con()
        self.thrust_pub.publish(self.thrust_cmd)
        self.att_pub.publish(self.att_cmd)
        self.data_pub.publish(self.data_out)
        # print("")


def main(argv):
   
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(60)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('mavros/imu/data', Imu, cnt.accCB)
    # Subscribe to payload position
    # rospy.Subscriber('/gazebo/link_states', LinkStates, cnt.payload_states)

    # Subscribe the raw marker data
    rospy.Subscriber('/vrpn_client_node/RigidBody/pose', PoseStamped, cnt.drone_states)
    rospy.Subscriber('/vrpn_client_node/RigidBody002/pose', PoseStamped, cnt.pendulum_states)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    rospy.Subscriber('start_pendulum',Float32,cnt.startPendulum)
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("ARMING")
    while not cnt.state.armed:
        modes.setArm()
        cnt.armed = True
        rate.sleep()

    cnt.armed = True
    k=0
    while k<20:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # ROS main loop
    while not rospy.is_shutdown():
        # r_des = quaternion_matrix(des_orientation)[:3,:3]
        # r_cur = quaternion_matrix(cnt.local_quat)[:3,:3]

#--------------------------------------------
        cnt.pub_att()
        rate.sleep()
       

#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
