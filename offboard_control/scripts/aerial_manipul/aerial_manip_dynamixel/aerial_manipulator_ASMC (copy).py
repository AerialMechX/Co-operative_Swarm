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

import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

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

class dynamixtest:

    def shutdown(self):
        print('shutdown')
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        self.write_to_motor(0)

    def __init__(self,id):
        self.M1_POS = 0
        self.M1_VEL = 0

        self.ADDR_TORQUE_ENABLE      = 64

        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132
        self.GOAL_POSITION           = 0
        self.PRESENT_POSITION        = 0
        self.GOAL_POSITION_ANGLE = 0

        self.ADDR_GOAL_CURRENT       = 102
        self.ADDR_PRESENT_CURRENT    = 126
        self.GOAL_CURRENT            = 0 #-1193 to +1193
        self.PRESENT_CURRENT         = 0

        self.ADDR_GOAL_VELOCITY      = 104
        self.ADDR_PRESENT_VELOCITY   = 128
        self.GOAL_VELOCITY           = 0
        self.PRESENT_VELOCITY        = 0

        self.ADDR_OPERATING_MODE     = 11
        self.OPERATING_MODE          = 0 #Current Mode 
        #self.OPERATING_MODE          = 3 #Position mode  

        self.PROTOCOL_VERSION        = 2.0

        self.DXL_ID                  = id                
        self.BAUDRATE                = 57600            
        self.DEVICENAME              = '/dev/ttyUSB0' 
        self.TORQUE_ENABLE           = 1

        self.kp = 0.01
        self.kd = 0.1

        self.error = 0
        self.previous_error=0

        self.pub = rospy.Publisher('error', Int32, queue_size=10)
        self.pub2 = rospy.Publisher('zero', Int32, queue_size=10)

        self.sub2 = rospy.Subscriber("dynamixel"+str(self.DXL_ID)+"_control", Float64MultiArray, self.data_set1)

        self.start_val = 0
        self.init_motor_connection()


    def data_set1(self,data):
        self.M1_POS = data.data[0]
        self.M1_VEL = data.data[1]
        self.GOAL_POSITION_ANGLE = self.M1_POS
        self.GOAL_VELOCITY = self.M1_VEL

    def init_motor_connection(self):
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            quit()

        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")

    def map(self,x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
        #return x - 32768
        #return x

    def get_motor_data(self):
        self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        #self.PRESENT_CURRENT, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_CURRENT)
        
        #self.PRESENT_POSITION = self.map(self.PRESENT_POSITION,0,65536,-32768,32768)
        #if self.DXL_ID == 1:
            #print(self.PRESENT_VELOCITY*0.229)

    def write_to_motor(self,current):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, current)

    def pid(self):
        self.GOAL_POSITION = self.GOAL_POSITION_ANGLE * (1024/90) + 1024
        self.error = self.GOAL_POSITION - self.PRESENT_POSITION
        
        print(  (self.PRESENT_POSITION/(1024/90)) - (self.GOAL_POSITION/(1024/90))  )
        
        self.pub.publish(self.error)
        self.pub2.publish(0)
        p = self.error * self.kp
        d = (self.error - self.previous_error) * self.kd
        #d = (self.PRESENT_VELOCITY-self.GOAL_VELOCITY) * self.kd

        pd = int(p+d)

        if pd > 1193:
            pd = 1193
        if pd < -1193:
            pd = -1193 

        #print(self.PRESENT_POSITION,self.error,self.M1_POS,self.M1_VEL)

        self.write_to_motor(pd)
        self.previous_error = self.error


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
        self.ALT_SP = 0.8
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
        self.Kp0 = np.array([1.0, 1.0, 1.0])
        self.Kp1 = np.array([2.0, 2.0, 1.0])
        self.Lam = np.array([2.2, 2.2, 5.0]) #Change 1
        self.Phi = np.array([1.5, 1.5, 1.1])
        self.M = 0.4 #Change 2
        self.alpha_0 = np.array([1,1,1])
        self.alpha_1 = np.array([3,3,3])
        self.alpha_m = 0.05 #Change 3
        self.v = 0.1  

        # Motor 1
        self.position_msg = JointState()
        self.position_msg.position.append(0)
        self.position_msg.position.append(0)  

        # self.desPos_m1, self.desVel_m1 = (-120), 0.0 #conf1 (90 degree == 100 deg)
        # self.desPos_m2, self.desVel_m2 = (0), 0.0          

        # self.desPos_m1, self.desVel_m1 = (-105), 0.0 #conf2
        # self.desPos_m2, self.desVel_m2 = (-60), 0.0

        # self.desPos_m1, self.desVel_m1 = (-110), 0.0 #conf3
        # self.desPos_m2, self.desVel_m2 = (-80), 0.0

        # self.desPos_m1, self.desVel_m1 = (170), 0.0 #conf3
        # self.desPos_m2, self.desVel_m2 = (00), 0.0

        # # Gripper Code Config 1 takeoff Default
        self.desPos_m1, self.desPos_m2 = 90, 0

        # # Gripper Code Config 2
        # self.desPos_m1, self.desPos_m2 = -100, -40


        # # Gripper Code Config 3
        # self.desPos_m1, self.desPos_m2 = -120, 0



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
        self.sp.pose.position.z = 0.8
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])
                   
        self.desAngVel = np.array([pt.velocities[0].angular.x, pt.velocities[0].angular.y, pt.velocities[0].angular.z])
        self.desAngAcc = np.array([pt.accelerations[0].angular.x, pt.accelerations[0].angular.y, pt.accelerations[0].angular.z])
        
        # print(pt.transforms[0].translation.x)

        # if (pt.transforms[0].translation.x < -0.49 and pt.transforms[0].translation.x > -0.51) \
        # and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 ) \
        # and self.flag_0 == 1 and self.sleep_0 == 0:
        #     print("Conf3 at",pt.transforms[0].translation.x)


        if (pt.transforms[0].translation.x < 1.0 and pt.transforms[0].translation.x > 0.99) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 )and self.sleep_3 == 1:         
            GPIO.output(self.pin_1, True)
            print("Takeoff at",pt.transforms[0].translation.x)
            # Gripper Code Config 1 takeoff
            self.desPos_m1, self.desPos_m2 = 90, 0


        if (pt.transforms[0].translation.x < 0.01 and pt.transforms[0].translation.x > -0.01) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 )\
         and self.flag_1 == 0 and self.sleep_1 == 0:
            print("Change Config : ",pt.transforms[0].translation.x)
            # GPIO.output(self.pin_1, False)
            # print("EM ON")
            # Gripper Code Config 2
            self.desPos_m1, self.desPos_m2 = -90, -40



        if (pt.transforms[0].translation.x < -0.99 and pt.transforms[0].translation.x > -1.0) \
        and (pt.transforms[0].translation.y > -0.01 and pt.transforms[0].translation.y < 0.01 ) and self.sleep_2 == 0:
      
            # GPIO.output(self.pin_1, True) 
            print("Land",pt.transforms[0].translation.x)
            # Gripper Code Config 3
            self.desPos_m1, self.desPos_m2 = -90, 0



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


    # def th_des(self):


        # 


    def th_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)

        errPos = curPos - desPos
        errVel = curVel - self.desVel
        sv = errVel + np.multiply(self.Phi, errPos)
        #print(errPos)
        #print("------------------")

        if self.armed:
            self.Kp0 += (sv - np.multiply(self.alpha_0, self.Kp0))*dt
            self.Kp1 += (sv - np.multiply(self.alpha_1, self.Kp1))*dt
            self.Kp0 = np.maximum(self.Kp0, 0.0001*np.ones(3))
            self.Kp1 = np.maximum(self.Kp1, 0.0001*np.ones(3))
            self.M += (-sv[2] - self.alpha_m*self.M)*dt
            self.M = np.maximum(self.M, 0.1)
            # print(self.M)

        Rho = self.Kp0 + self.Kp1*errPos

        delTau = np.zeros(3)
        delTau[0] = Rho[0]*self.sigmoid(sv[0],self.v)
        delTau[1] = Rho[1]*self.sigmoid(sv[1],self.v)
        delTau[2] = Rho[2]*self.sigmoid(sv[2],self.v)

        des_th = -np.multiply(self.Lam, sv) - delTau + self.M*self.gravity

        # self.array2Vector3(sv, self.data_out.sp)
        # self.array2Vector3(self.Kp0, self.data_out.Kp_hat)
        # self.array2Vector3(errPos, self.data_out.position_error)
        # self.array2Vector3(errVel, self.data_out.velocity_error)
        # self.array2Vector3(delTau, self.data_out.delTau_p)
        # self.array2Vector3(Rho, self.data_out.rho_p)
        # self.data_out.M_hat = self.M


        if np.linalg.norm(des_th) > self.max_th:
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th

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

            #print(self.Lambd_m1*r_m1, self.J1*self.Kd_m1*errVel_m1, del_Tau_m1)

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
    # motors_states = rospy.wait_for_message('/joint_states', JointState)
    # cnt.joint_states(motors_states)    
    # rospy.Subscriber('/joint_states', JointState, cnt.joint_states)

    rospy.Subscriber('/traj_pos', Float64MultiArray, cnt.motor_poseCB)
    rospy.Subscriber('/arm_error_data', Float64MultiArray, cnt.armError_poseCB)


    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # print("ARMING")
    # while not cnt.state.armed:
    #     modes.setArm()
    #     cnt.armed = True
    #     rate.sleep()

    # cnt.armed = True
    # k=0
    # while k<20:
    #     sp_pub.publish(cnt.sp)
    #     rate.sleep()
    #     k = k + 1

    # modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    obj = dynamixtest(1)
    #obj1 = dynamixtest(2)





    # ROS main loop
    while not rospy.is_shutdown():
        # r_des = quaternion_matrix(des_orientation)[:3,:3]
        # r_cur = quaternion_matrix(cnt.local_quat)[:3,:3]

#--------------------------------------------
        cnt.pub_att()
        rate.sleep()

        obj.pid()
        #obj1.pid()
        obj.get_motor_data()
        #obj1.get_motor_data()
    obj1.shutdown()
    obj.shutdown()
#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
