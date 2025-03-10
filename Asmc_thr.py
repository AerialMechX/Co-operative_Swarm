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
# from scipy import linalg as la

# import dynamixel_sdk
# from dynamixel_sdk import *
# from dynamixel_sdk_examples.srv import *
# from dynamixel_sdk_examples.msg import *

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from tf import *


import numpy as np
import tf
# import RPi.GPIO as GPIO
import time
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

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
        

    def __init__(self,id):

        self.PRESENT_POSITION        = 0

        # self.OPERATING_MODE          = 3 # CURRENT/TORQUE MODE = 0; POSITION MODE = 3; PWM MODE = 16

        self.OPERATING_MODE          = 5 # Current based position control

        # Control table address
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_PRESENT_POSITION   = 132
        self.ADDR_OPERATING_MODE     = 11
        self.ADDR_GOAL_PWM           = 100
        self.ADDR_PWM_LIMIT          = 36
        self.ADDR_GOAL_CURRENT       = 102
        self.ADDR_CURRENT_LIMIT      = 38
        self.ADDR_PRESENT_CURRENT    = 126
        self.ADDR_PRESENT_PWM        = 124

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        # DXL_ID                      = 1                 # Dynamixel ID : 1
        self.DXL_ID                  = id                
        print("DXL_ID",self.DXL_ID)

        self.BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        # DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MAXIMUM_POSITION_VALUE  = 10000*(1024/90)            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.init_motor_connection()



    def init_motor_connection(self):  #do not change 
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            getch()
            quit()


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("TORQUE_DISABLE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE))

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("OPERATING_MODE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE))        

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("TORQUE_ENABLE", self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE))

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            getch()
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")


    def get_motor_data(self):
        self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        # self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)


    def set_goal_pos_callback(self, position):
        pos_ticks = int(round(position * (4095 / 360)))
        # print("Position in ticks : ", pos_ticks)
        if 0 <= pos_ticks <= 4095:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, pos_ticks)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[RxPacketError] The data value exceeds the limit value!")


    def set_goal_curr_callback(self, curr_ticks):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, curr_ticks)

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

        self.arm_flag = 1


        # A Message for the current local position of the drone

        # initial values for setpoints
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        self.acc = Accel()
        self.imu = Imu()
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.desAngPos = Vector3()
        self.curAngPos = Vector3()
        self.ALT_SP = 0.80
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

        #Current (8/4/24)
        self.Kp0 = np.array([1.0, 1.0, 1.0])
        self.Kp1 = np.array([2.0, 2.0, 1.0])
        self.Lam = np.array([2.2, 2.2, 3.5]) #Change 1
        self.Phi = np.array([1.5, 1.5, 0.8])
        self.M = 0.45 #Change 2
        self.alpha_0 = np.array([1,1,1])
        self.alpha_1 = np.array([3,3,3])
        self.alpha_m = 0.05 #Change 3
        self.v = 0.1  

        # Defualt2 (23/10/23)
        # self.Kp0 = np.array([1.0, 1.0, 1.0])
        # self.Kp1 = np.array([2.0, 2.0, 1.0])
        # self.Lam = np.array([2.0, 2.0, 4.5]) #Change 1    #[1.75, 1.75, 4]
        # self.Phi = np.array([1.5, 1.5, 1.2])
        # self.M = 0.4 #Change 2
        # self.alpha_0 = np.array([1,1,1])
        # self.alpha_1 = np.array([3,3,3])
        # self.alpha_m = 0.05 #Change 3
        # self.v = 0.1  

        #Default values (previous expt.)
        # self.Kp0 = np.array([1.0, 1.0, 1.0])
        # self.Kp1 = np.array([2.0, 2.0, 1.0])
        # self.Lam = np.array([2.2, 2.2, 5.0]) #Change 1
        # self.Phi = np.array([1.5, 1.5, 1.1])
        # self.M = 0.4 #Change 2
        # self.alpha_0 = np.array([1,1,1])
        # self.alpha_1 = np.array([3,3,3])
        # self.alpha_m = 0.05 #Change 3
        # self.v = 0.1  

        # Motor 1
        self.position_msg = JointState()
        self.position_msg.position.append(0)
        self.position_msg.position.append(0)  

        # # Gripper Code Config 1 takeoff Default
        self.desPos_m1, self.desPos_m2 = 90, 0

        self.flag = 0

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

        self.motot1_pub = rospy.Publisher('/dynamixel1_cont', Float32, queue_size=10)
        self.motot2_pub = rospy.Publisher('/dynamixel2_cont', Float32, queue_size=10)

        self.gripper_pub = rospy.Publisher('/toggle_gripper', Int32, queue_size=10)
        #rosrun rosserial_arduino serial_node.py /dev/ttyUSB1 

        self.armed = False
        

    def grab(self):
        
        dynamixtest(0).set_goal_pos_callback(225) #motor 1 first
        dynamixtest(2).set_goal_pos_callback(75) #link 2 motor
        dynamixtest(1).set_goal_pos_callback(176) #link 3
        time.sleep(1)
        dynamixtest(3).set_goal_pos_callback(180) #gripper

        self.arm_flag = 0

    def multiDoFCb(self, msg):
        # print("multiDoFCb")
        pt = msg.points[0]
        #print(pt.transforms[0].translation.x)
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        self.sp.pose.position.z = self.ALT_SP
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])
                   
        self.desAngVel = np.array([pt.velocities[0].angular.x, pt.velocities[0].angular.y, pt.velocities[0].angular.z])
        self.desAngAcc = np.array([pt.accelerations[0].angular.x, pt.accelerations[0].angular.y, pt.accelerations[0].angular.z])
        
              
        # # # (-0.5, -0.45)(Idle position)

        # if ( pt.transforms[0].translation.x > -0.5 - 0.01 and pt.transforms[0].translation.x < -0.5 + 0.01 ) \
        # and (pt.transforms[0].translation.y > -0.45 - 0.01 and pt.transforms[0].translation.y <  -0.45 + 0.01):
        #     print("1. (-0.5, -0.45) reached")

        #     dynamixel_obj0.set_goal_pos_callback(210) #motor 1 first
        #     dynamixel_obj1.set_goal_pos_callback(280) #link 2
        #     # dynamixel_obj2.set_goal_pos_callback(307) #gripper


    # # arm down
    # dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
    # dynamixel_obj1.set_goal_pos_callback(245) #link 2
    # # dynamixel_obj2.set_goal_pos_callback(300) #gripper
    
    # time.sleep(2)
    
    # # arm up
    # dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
    # dynamixel_obj1.set_goal_pos_callback(265) #link 2
    # dynamixel_obj2.set_goal_pos_callback(20) #gripper open
        

        # # (0, 0)(hammering position)
        # if ( pt.transforms[0].translation.x > -0.01 and pt.transforms[0].translation.x < + 0.01):
        #     if self.arm_flag == 1:
        #         dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
        #         dynamixel_obj1.set_goal_pos_callback(265) #link 2
        #         # dynamixel_obj2.set_goal_pos_callback(300) #gripper Open


        # # (-0.8, 0)(gripping at position 1)
        # if ( pt.transforms[0].translation.x > -0.81 - 0.01 and pt.transforms[0].translation.x < -0.81 + 0.01):
        #     if self.arm_flag == 1:
                
        #         print("Gripping Position reached")
                
        #         # time.sleep(1)
                
        #         dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
        #         dynamixel_obj1.set_goal_pos_callback(265) #link 2
                
        #         time.sleep(1)
                
        #         dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
        #         dynamixel_obj1.set_goal_pos_callback(225) #link 2
                
        #         time.sleep(1)

        #         dynamixel_obj0.set_goal_pos_callback(225) #motor 1 first
        #         dynamixel_obj1.set_goal_pos_callback(255) #link 2
        #         self.arm_flag = 0
                
        #         # time.sleep(0.25)
        #         # dynamixel_obj1.set_goal_pos_callback(265) #link 2



 
        # # (drop reached)
        # if ( pt.transforms[0].translation.x > 0.78 - 0.01 and pt.transforms[0].translation.x < 0.78 + 0.01)\
        #     and self.arm_flag == 0:
            
        #     # dynamixel_obj0.set_goal_pos_callback(245) #motor 1 first
        #     # dynamixel_obj1.set_goal_pos_callback(225) #link 2
        #     dynamixel_obj2.set_goal_pos_callback(10) #gripper Open
            
            


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

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)

        curor = self.vector3Arrays(self.cur_pose.pose.orientation)
        curAcc = self.vector2Arrays(self.imu.linear_acceleration)

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

        # self.array2Vector3(sv, self.data_out.sp) #Check this
        # self.array2Vector3(self.Kp0, self.data_out.Kp_hat)
        # self.array2Vector3(errPos, self.data_out.position_error)
        # self.array2Vector3(errVel, self.data_out.velocity_error)
        # self.array2Vector3(delTau, self.data_out.delTau_p)
        # self.array2Vector3(Rho, self.data_out.rho_p)
        # self.data_out.M_hat = self.M

        ## Storing to data_out

        self.array2Vector3(curPos, self.data_out.curpos)
        self.array2Vector3(desPos, self.data_out.despos)
        
        self.array2Vector3(curVel, self.data_out.curvel)
        self.array2Vector3(self.desVel, self.data_out.desvel)

        self.array2Vector3(self.desAcc, self.data_out.desacc) #Desired Acceleration
        self.array2Vector3(curAcc, self.data_out.curacc) #Desired Acceleration

        self.array2Vector3(errPos, self.data_out.poserr)
        self.array2Vector3(errVel, self.data_out.velerr)


        self.array2Vector4(curor , self.data_out.curor)  #Current Orientation

        ## DesOr is in Geocon


        if np.linalg.norm(des_th) > self.max_th:
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th

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
    print("All Good")

    # global dynamixel_obj1
    # global dynamixel_obj2


    # dynamixel_obj1 = dynamixtest(1)
    # time.sleep(0.5)
    # dynamixel_obj2 = dynamixtest(2)
    # time.sleep(0.5)

    
    # posd = 90
    # posd2 = 270
    # dynamixel_obj1.get_motor_data()
    # dynamixel_obj2.get_motor_data()


    # dynamixel_obj1.set_goal_pos_callback(200)
    # dynamixel_obj2.set_goal_pos_callback(270)

    # # dynamixel_obj1.set_goal_curr_callback(int(tau1))
    # # dynamixel_obj2.set_goal_curr_callback(int(tau2))


    # dynamixel_obj1.set_goal_curr_callback(int(120))
    # dynamixel_obj2.set_goal_curr_callback(int(90))

    
    
    

    # time.sleep(1)
    
    
    # dynamixel_obj0.get_motor_data()
    # print("Position ", dynamixel_obj0.DXL_ID ," : ",dynamixel_obj0.PRESENT_POSITION*90/1024)
    # dynamixel_obj1.get_motor_data()
    # print("Position ", dynamixel_obj1.DXL_ID ," : ",dynamixel_obj1.PRESENT_POSITION*90/1024)
    # dynamixel_obj2.get_motor_data()
    # print("Position ", dynamixel_obj2.DXL_ID ," : ",dynamixel_obj2.PRESENT_POSITION*90/1024)

    # dynamixel_obj2.get_motor_data()
    # print("Position ", dynamixel_obj2.DXL_ID ," : ",dynamixel_obj2.PRESENT_POSITION*90/1024)
    
    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('mavros/imu/data', Imu, cnt.accCB)

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

# --------------------------------------------
        cnt.pub_att()
        rate.sleep()

        # ESC_ASCII_VALUE = 27 # ESC ASCII VALUE
        # if  getch() == chr(ESC_ASCII_VALUE):
        #     break
        # else:
        #     pass
# --------------------------------------------  


if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass