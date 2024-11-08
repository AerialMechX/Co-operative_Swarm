#!/usr/bin/env python
import sys
# ROS python API
import rospy

from std_msgs.msg import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Accel, Vector3
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
from msg_check.msg import PlotDataMsg
from scipy import linalg as la

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from tf import *


import numpy as np
import tf
import RPi.GPIO as GPIO
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
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        self.drone = Point(0.0, 0.0, 0.0)
        self.pendulum = Point(0.0, 0.0, 0.0)

        self.payload_state = PoseStamped()
        self.payload_vel  = TwistStamped()
        # set the flag to use position setpoints and yaw angle
       
        # Step size for position update
        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0
        self.flag_0 = 0
        self.flag_1 = 0
        self.flag_2 = 0
        self.sleep_0 = 0
        self.sleep_1 = 0
        self.sleep_2 = 0
        self.sleep_3 = 0



        # A Message for the current local position of the drone

        # initial values for setpoints
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        self.acc = Accel()
        self.imu = Imu()
        self.sp.pose.position.x = 1.0
        self.sp.pose.position.y = 0.0
        self.desAngPos = Vector3()
        self.curAngPos = Vector3()
        self.ALT_SP = 0.91
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])

        self.desVel = np.zeros(3)
        self.desAcc = np.zeros(3)

        self.armConfig_data = 1.0

        self.desPos_m1, self.desVel_m1, self.desAcc_m1 = np.deg2rad(0), 0.1, 0.0
        self.desPos_m2, self.desVel_m2, self.desAcc_m2 = np.deg2rad(90), 0.1, 0.0

        self.M1_pos_err, self.M1_vel_err, self.M2_pos_err, self.M2_vel_err = 0, 0, 0, 0

        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()

        #Drone
        self.phi = np.array([1.1,1.1,1.5])
        self.Lambd = np.array([1.8,1.8,1.4])
        self.Kd = np.array([1.0,1.0,0.4])

        self.M = 0.825

        self.rho = 2.0
        self.Kp0, self.Kp1, self.Kp2, self.Kp3 = 0.1, 0.1, 0.1, 0.1
        self.alpha_0, self.alpha_1, self.alpha_2, self.alpha_3  = 5.0, 5.0, 10.0, 10.0 
        self.zeta = 0.1
        self.epsilon_bar = 0.01  

        # Motor 1
        self.position_msg = JointState()
        self.position_msg.position.append(0)
        self.position_msg.position.append(0)  

        self.desPos_m1, self.desVel_m1 = (0), 0.0 #conf1
        self.desPos_m2, self.desVel_m2 = (-90), 0.0


        self.norm_thrust_const = 0.06
        self.max_th = 16.0
        self.max_throttle = 0.96
        self.gravity = np.array([0, 0, 9.8])
        self.pre_time = rospy.get_time()    
        self.data_out = PlotDataMsg()

        # Publishers
        self.att_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)

        self.positionM_pub = rospy.Publisher('/goal_dynamixel_position', JointState, queue_size=10)
        self.armConfig_pub = rospy.Publisher('/motor_conf', Float64, queue_size=10)

        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)
        self.armed = False
        self.pin_1 = 16
        # self.pin_2 = 18
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_1, GPIO.OUT)
        # GPIO.setup(self.pin_2, GPIO.OUT)


    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        # self.sp.pose.position.z = pt.transforms[0].translation.z
        self.sp.pose.position.z = 0.91
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])
                   
        self.desAngVel = np.array([pt.velocities[0].angular.x, pt.velocities[0].angular.y, pt.velocities[0].angular.z])
        self.desAngAcc = np.array([pt.accelerations[0].angular.x, pt.accelerations[0].angular.y, pt.accelerations[0].angular.z])
        
        # print(pt.transforms[0].translation.x)

        if (pt.transforms[0].translation.x < -0.49 and pt.transforms[0].translation.x > -0.51) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 ) \
        and self.flag_0 == 1 and self.sleep_0 == 0:
            print("Conf3 at",pt.transforms[0].translation.x)
            self.armConfig_data = 3.0
            self.sleep_0 = 1


        if (pt.transforms[0].translation.x < 0.91 and pt.transforms[0].translation.x > 0.89) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 )\
         and self.flag_1 == 0 and self.sleep_1 == 0:
            print("Conf4 at",pt.transforms[0].translation.x)
            self.armConfig_data = 4.0
            self.sleep_1 = 1
            self.sleep_3 = 1
            GPIO.output(self.pin_1, False)
            print("EM ON")

        if (pt.transforms[0].translation.x < -0.99 and pt.transforms[0].translation.x > -1.0) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 ) and self.sleep_2 == 0:
            self.flag_1 = 1 
            self.flag_0 = 1        

            # GPIO.output(self.pin_1, True) 
            print("Picking payload at",pt.transforms[0].translation.x)
            self.sleep_2 = 1


        if (pt.transforms[0].translation.x < 1.0 and pt.transforms[0].translation.x > 0.99) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 )and self.sleep_3 == 1:         
            GPIO.output(self.pin_1, True)
            print("Dropping payload at",pt.transforms[0].translation.x)
            # print("Electro Magnet OFF")
            time.sleep(0.05)
            self.sleep_3 = 0




    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_quat[0] = msg.pose.orientation.x
        self.local_quat[1] = msg.pose.orientation.y
        self.local_quat[2] = msg.pose.orientation.z
        self.local_quat[3] = msg.pose.orientation.w
    
    def armError_poseCB(self,msg):
        self.M1_pos_err = msg.data[0]
        self.M1_vel_err = msg.data[1]
        self.M2_pos_err = msg.data[2]
        self.M2_vel_err = msg.data[3]

        # print(self.M1_pos_err, self.M1_vel_err, self.M2_pos_err, self.M2_vel_err)



    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

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

    def accAng(self,msg):
        self.acc.linear_acc.x = msg.linear.x
        self.acc.linear_acc.y = msg.linear.y
        self.acc.linear_acc.z = msg.linear.z

        self.acc.angular_acc.x = msg.angular.x
        self.acc.angular_acc.y = msg.angular.y
        self.acc.angular_acc.z = msg.angular.z


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

    def sigmoid(self, s, v):
        if np.absolute(s) > v:
            return s/np.absolute(s)
        else:
            return s/v

    def pi_2_pi(self, angle):
        while(angle > np.pi):
            angle = angle - 2.0 * np.pi
        while(angle < -np.pi):
            angle = angle + 2.0 * np.pi
        return angle

    def motor_poseCB(self, msg):

        self.desPos_m1 = msg.data[0]
        self.desVel_m1 = msg.data[1]

        self.desPos_m2 = msg.data[2]
        self.desVel_m2 = msg.data[3]



    def joint_states(self, msg):
        ind_m1 = msg.name.index('id_1')
        ind_m2 = msg.name.index('id_2')

        self.joint1Pos = (msg.position[ind_m1])
        self.joint1Vel = msg.velocity[ind_m1]
        # self.joint1Effort = msg.effort[ind_m1]

        self.joint2Pos = (msg.position[ind_m2])
        self.joint2Vel = msg.velocity[ind_m2]
        # self.joint2Effort = msg.effort[ind_m2]

        # print(np.rad2deg(self.pi_2_pi(self.joint1Pos)), np.rad2deg(self.pi_2_pi(self.joint2Pos)))
        # print(self.joint1Vel, self.joint2Vel)
        # print("--- CALLBACK ---")

    def motor_pos_control(self):
        self.m1_position = np.deg2rad(self.desPos_m1)
        self.m2_position = np.deg2rad(self.desPos_m2)

        # print(self.desPos_m1, self.desPos_m2)


    def th_des(self):


        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04
        if dt == 0.0:
            dt = 0.01

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)

        desPos = self.vector2Arrays(self.sp.pose.position)
        desVel = self.desVel

        curAcc = self.vector2Arrays(self.imu.linear_acceleration)
        desAcc = self.desAcc
        desAcc_norm = np.linalg.norm(desAcc)

        curor = self.vector3Arrays(self.cur_pose.pose.orientation)

        errPos = curPos - desPos
        errVel = curVel - desVel

        self.array2Vector3(errPos, self.data_out.poserr) #Actuated Position Error
        self.array2Vector3(errVel, self.data_out.velerr)

        arm_pos_err_M1 = self.M1_pos_err
        arm_pos_err_M2 = self.M2_pos_err
        arm_vel_err_M1 = self.M1_vel_err
        arm_vel_err_M2 = self.M2_vel_err


        self.data_out.M1_pos_err = arm_pos_err_M1
        self.data_out.M2_pos_err = arm_pos_err_M2
        self.data_out.M1_vel_err = arm_vel_err_M1
        self.data_out.M2_vel_err = arm_vel_err_M2

        print(errPos)
        print("-------")

        r = errVel + np.multiply(self.phi,errPos)
        r_norm = np.linalg.norm(r)

        xi = np.concatenate((errPos, errVel),axis = 0)
        xi_Norm = np.linalg.norm(xi)

        # print(np.rad2deg(self.joint1Pos), np.rad2deg(self.joint1Pos))

        if self.armed:
            # self.motor_command()

            Kp0_dot = r_norm - self.alpha_0*self.Kp0
            Kp1_dot = r_norm*xi_Norm - self.alpha_1*self.Kp1
            Kp2_dot = r_norm*xi_Norm*xi_Norm - self.alpha_2*self.Kp2

            Kp3_dot = r_norm*desAcc_norm - self.alpha_3*self.Kp3
            zeta_dot = -(1 - self.Kp3*desAcc_norm*r_norm)*self.zeta + self.epsilon_bar

            self.Kp0 = self.Kp0 + Kp0_dot*dt
            # print(self.Kp0)
            self.Kp1 = self.Kp1 + Kp1_dot*dt
            # print(self.Kp1)
            self.Kp2 = self.Kp2 + Kp2_dot*dt
            # print(self.Kp2)
            self.Kp3 = self.Kp3 + Kp3_dot*dt
            # print(self.Kp3)
            self.zeta = self.zeta + zeta_dot*dt
            # print(self.zeta)

            self.rho = (self.Kp0 + self.Kp1*xi_Norm + self.Kp2*xi_Norm*xi_Norm
                   + self.Kp3*desAcc_norm + self.zeta)
            # print(self.rho)

            del_Tau = np.multiply(self.rho, r)/r_norm
            # print(del_Tau)

            des_th =  (self.M*desAcc
                  - np.multiply(self.Lambd, r)
                  - self.M*np.multiply(self.Kd, errVel)
                  - np.multiply(del_Tau, np.array([1.0, 1.0, 1.0]))
                  + self.M*self.gravity)
            # print(des_th)
            # print("------------------")

        if np.linalg.norm(des_th) > self.max_th:
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th

        self.array2Vector3(curPos, self.data_out.curpos) #Current Pose
        self.array2Vector3(desPos, self.data_out.despos) #Desired Pose
        self.array2Vector4(curor , self.data_out.curor)  #Current Orientation
        self.array2Vector3(curVel, self.data_out.curvel) #Current Velocity
        self.array2Vector3(self.desVel, self.data_out.desvel) #Desired Velocity

        self.array2Vector3(self.desAcc, self.data_out.desacc) #Desired Acceleration
        self.array2Vector3(curAcc, self.data_out.curacc) #Desired Acceleration

        return des_th

    def motor_command(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04
        if dt == 0.0:
            dt = 0.01

        curPos_m1, curVel_m1 = self.joint1Pos, self.joint1Vel
        curPos_m2, curVel_m2 = self.joint2Pos, self.joint2Vel

        desPos_m1, desVel_m1 = self.desPos_m1, self.desVel_m1
        desPos_m2, desVel_m2 = self.desPos_m2, self.desVel_m2

        desAcc_m1, desAcc_m2 = self.desAcc_m1, self.desAcc_m2
        desAcc_m1_norm = np.linalg.norm(desAcc_m1)
        desAcc_m2_norm = np.linalg.norm(desAcc_m2)

        errPos_m1 = curPos_m1 - desPos_m1
        errVel_m1 = curVel_m1 - desVel_m1

        errPos_m2 = curPos_m2 - desPos_m2
        errVel_m2 = curVel_m2 - desVel_m2

        r_m1 = errVel_m1 + np.multiply(self.phi_m1,errPos_m1)
        r_m1_norm = np.linalg.norm(r_m1)

        r_m2 = errVel_m2 + np.multiply(self.phi_m2, errPos_m2)
        r_m2_norm = np.linalg.norm(r_m2)

        xi_m1 = np.array([errPos_m1, errVel_m1])
        xi_m1_Norm = np.linalg.norm(xi_m1)

        xi_m2 = np.array([errPos_m2, errVel_m2])
        xi_m2_Norm = np.linalg.norm(xi_m2)

        if self.armed:
            # Motor 1
            Kp0_m1_dot = r_m1_norm - self.alpha_m1_0*self.Kp0_m1
            Kp1_m1_dot = r_m1_norm*xi_m1_Norm - self.alpha_m1_1*self.Kp1_m1
            Kp2_m1_dot = r_m1_norm*xi_m1_Norm*xi_m1_Norm - self.alpha_m1_2*self.Kp2_m1
            Kp3_m1_dot = r_m1_norm*desAcc_m1_norm - self.alpha_m1_3*self.Kp3_m1
            zeta_m1_dot = -(1 - self.Kp3_m1*desAcc_m1_norm*r_m1_norm)*self.zeta_m1 + self.epsilon_bar_m1

            self.Kp0_m1 = self.Kp0_m1 + Kp0_m1_dot*dt
            # print(self.Kp0)
            self.Kp1_m1 = self.Kp1_m1 + Kp1_m1_dot*dt
            # print(self.Kp1)
            self.Kp2_m1 = self.Kp2_m1 + Kp2_m1_dot*dt
            # print(self.Kp2)
            self.Kp3_m1 = self.Kp3_m1 + Kp3_m1_dot*dt
            # print(self.Kp3)
            self.zeta_m1 = self.zeta_m1 + zeta_m1_dot*dt
            # print(self.zeta)

            self.rho_m1 = (self.Kp0_m1 + self.Kp1_m1*xi_m1_Norm + self.Kp2_m1*xi_m1_Norm*xi_m1_Norm
                   + self.Kp3_m1*desAcc_m1_norm + self.zeta_m1)

            del_Tau_m1 = np.multiply(self.rho_m1, r_m1)/r_m1_norm
            # print(del_Tau_m1)

            print(self.Lambd_m1*r_m1, self.J1*self.Kd_m1*errVel_m1, del_Tau_m1)

            des_th_m1 =  (self.J1*desAcc_m1
                  - self.Lambd_m1*r_m1
                  - self.J1*self.Kd_m1*errVel_m1
                  - del_Tau_m1)

            # Motor 2
            Kp0_m2_dot = r_m2_norm - self.alpha_m2_0*self.Kp0_m2
            Kp1_m2_dot = r_m2_norm*xi_m2_Norm - self.alpha_m2_1*self.Kp1_m2
            Kp2_m2_dot = r_m2_norm*xi_m2_Norm*xi_m2_Norm - self.alpha_m2_2*self.Kp2_m2
            Kp3_m2_dot = r_m2_norm*desAcc_m2_norm - self.alpha_m2_3*self.Kp3_m2
            zeta_m2_dot = -(1 - self.Kp3_m2*desAcc_m2_norm*r_m2_norm)*self.zeta_m2 + self.epsilon_bar_m2

            self.Kp0_m2 = self.Kp0_m2 + Kp0_m2_dot*dt
            # print(self.Kp0)
            self.Kp1_m2 = self.Kp1_m2 + Kp1_m2_dot*dt
            # print(self.Kp1)
            self.Kp2_m2 = self.Kp2_m2 + Kp2_m2_dot*dt
            # print(self.Kp2)
            self.Kp3_m2 = self.Kp3_m2 + Kp3_m2_dot*dt
            # print(self.Kp3)
            self.zeta_m2 = self.zeta_m2 + zeta_m2_dot*dt
            # print(self.zeta)

            self.rho_m2 = (self.Kp0_m2 + self.Kp1_m2*xi_m2_Norm + self.Kp2_m2*xi_m2_Norm*xi_m2_Norm
                   + self.Kp3_m2*desAcc_m2_norm + self.zeta_m2)

            del_Tau_m2 = np.multiply(self.rho_m2, r_m2)/r_m2_norm
            # print(del_Tau_m2)

            des_th_m2 = (self.J2*desAcc_m2
                  - self.Lambd_m2*r_m2
                  - self.J1*self.Kd_m1*errVel_m1
                  - del_Tau_m1)

        self.m1_torque = des_th_m1*0.2
        self.m2_torque = des_th_m2*0.2

        return des_th_m1, des_th_m2


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
        quat_des = tf.transformations.quaternion_from_matrix(rot_44)
       
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
        # self.data_out.desor = 
        self.att_cmd.pose.orientation #Desired Orientation
        self.data_out.desor = self.att_cmd.pose.orientation #Desired Orientation


    def pub_att(self):
        self.geo_con()
        # self.motor_command()
        self.thrust_pub.publish(self.thrust_cmd)
        self.att_pub.publish(self.att_cmd)

        self.motor_pos_control()
        self.position_msg.position[0] = -self.m1_position
        self.position_msg.position[1] = -self.m2_position
        self.positionM_pub.publish(self.position_msg)

        self.armConfig_pub.publish(self.armConfig_data)
        self.data_pub.publish(self.data_out)
       


def main(argv):
   
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('mavros/imu/data', Imu, cnt.accCB)

    # Subscribe the raw marker data
    motors_states = rospy.wait_for_message('/joint_states', JointState)
    cnt.joint_states(motors_states)    
    rospy.Subscriber('/joint_states', JointState, cnt.joint_states)

    rospy.Subscriber('/traj_pos', Float64MultiArray, cnt.motor_poseCB)
    rospy.Subscriber('/arm_error_data', Float64MultiArray, cnt.armError_poseCB)


    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
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
