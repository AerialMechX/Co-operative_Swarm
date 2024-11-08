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
from msg_check.msg import PlotDataMsg
#from scipy import linalg as la



import numpy as np
from tf.transformations import *
import RPi.GPIO as GPIO
import time

# Kpos = np.array([-2, -2, -3])
# Kvel = np.array([-2, -2, -3])
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
        except rospy.ServiceException,e:
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
        except rospy.ServiceException,e:
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
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        self.imu = Imu()
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 0.5
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.zeros(3)
        self.errInt = np.zeros(3)
        self.desAcc = np.zeros(3)
        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()

        # Control parameters
        self.Kp = np.array([10, 10, 11.5])
        self.Kd = np.array([5, 5, 6]) #Stability
        self.M = [1.4,1.4,1.4]
        #self.M = 1.23
        self.alpha = np.array([2, 2, 3]) #Useless
        self.Phi = np.array([1.5, 1.5, 1.0])  # Sliding Variable #Useless
        self.epsilon = 0.0005
        self.beta_0 = 0.0001 # Adaptive
        self.beta_1 = 0.0001 # Adaptive
        self.g = 1# Between 1 2 3 Useless
        self.count = 0   
        #self.P = [[ 0.70833333 ,-0.5       ],[-0.5  ,       2.16666667]] #8,3

        self.curAccPrev = 0.
        self.s_prev = 0.
        self.errPosPrev = 0.
        self.errVelPrev = 0.
        self.torque_prev = np.array([0.1,0.1,10.5])


        self.norm_thrust_const = 0.059
        # self.norm_thrust_const = 0.058
        self.max_th = 22
        self.max_throttle = 0.9
        self.gravity = np.array([0, 0, 9.8])
        self.pre_time = rospy.get_time()    
        self.data_out = PlotDataMsg()
        #self.data_out.sq.z = self.ALT_SP


        # Publi0her0
        self.att_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)
        self.armed = False
        self.pin_1 = 16
        self.pin_2 = 18
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_1, GPIO.OUT)
        GPIO.setup(self.pin_2, GPIO.OUT)
        self.magnet_ON = False


        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks



    # def multiDoFCb(self, msg):

    def multiDoFCb(self, msg):
        
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        #self.data_out.sq = self.sp.pose.position
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])
        #self.array2Vector3(pt.accelerations[0].linear, self.data_out.DesAcc)
        DesAcc_x = pt.accelerations[0].linear.x
        DesAcc_y = pt.accelerations[0].linear.y
        DesAcc_z = pt.accelerations[0].linear.z

        self.data_out.DesAcc_x = DesAcc_x
        self.data_out.DesAcc_y = DesAcc_y
        self.data_out.DesAcc_z = DesAcc_z
        #print("Translation",pt.transforms[0].translation)
        if (pt.transforms[0].translation.x < 0.51 and pt.transforms[0].translation.x > 0.49):
            self.count = 1
        if (self.count == 1):
            if ((pt.transforms[0].translation.x < -4.99 and pt.transforms[0].translation.x > -5.01) 
            and (pt.transforms[0].translation.y > -5.01 and pt.transforms[0].translation.y < -4.99 ) and (self.magnet_ON == False)):            
                GPIO.output(self.pin_1, True)
                time.sleep(0.05)
                GPIO.output(self.pin_2, True)
                time.sleep(0.05)
                print("Magnet off")
                self.magnet_ON = True


            # self.switch = 2.0
            # print(self.switch)



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
        self.cur_pose.pose.position.x = msg.pose.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.pose.position.z
        #print(self.cur_pose.pose.position.z)
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




    def th_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        #if dt > 0.04:
            #dt = 0.04

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)
        curAcc = self.vector2Arrays(self.imu.linear_acceleration)

        #curor = self.vector3Arrays(self.cur_pose.pose.orientation)
        curor_w = self.cur_pose.pose.orientation.w
        curor_x = self.cur_pose.pose.orientation.x
        curor_y = self.cur_pose.pose.orientation.y
        curor_z = self.cur_pose.pose.orientation.z

    
        CurAcc_x = self.imu.linear_acceleration.x
        CurAcc_y = self.imu.linear_acceleration.y
        CurAcc_z = self.imu.linear_acceleration.z


        self.data_out.curor_w = curor_w
        self.data_out.curor_x = curor_x
        self.data_out.curor_y = curor_y
        self.data_out.curor_z = curor_z

        self.data_out.CurAcc_x = CurAcc_x
        self.data_out.CurAcc_y = CurAcc_y
        self.data_out.CurAcc_z = CurAcc_z


        self.array2Vector3(curPos, self.data_out.curpos)
        self.array2Vector3(desPos, self.data_out.despos)
        
        

        #self.array2Vector4(curor , self.data_out.curor)

        # kp = 6
        # kd = 3

        errPos = (desPos - curPos)
        print("PosErr",errPos)
        errVel = (self.desVel - curVel)
        print("VelErr",errVel)
        print("---------------------")
        
        E = np.array([errPos,errVel])
        #print("Error",E)
        # A = np.array([[0,1],[-kp,-kd]])
        # Q = [[-1 ,-2],[-2 ,-5]]
        #P = la.solve_continuous_lyapunov(A, Q)
        #print(P)
        #P = [[ 0.88888889 ,-0.5       ],[-0.5       ,  1.83333333]]
        #P = [[ 0.70833333 ,-0.5       ],[-0.5  ,       2.16666667]] # Working 668 333
        #P = [[ 0.6 ,-0.5],[-0.5 , 2.5]] # Not Working but violent 6610 333
        #P = [[ 0.52777778 ,-0.5       ],[-0.5      ,   2.83333333]]
        #P = [[ 0.45555556 ,-0.5       ],[-0.5   ,      3.33333333]]
        #P = [[ 0.51666667 ,-0.5       ],[-0.5 ,        1.7       ]]
        #P = [[ 0.725 ,-0.5  ],[-0.5 ,  1.3  ]]
        #P = [[ 0.6 ,-0.5],[-0.5,  2.5]]


        B = np.array([0,1])
        #s= B.dot(self.P).dot(E)
        s =  - errVel - np.multiply(self.Phi, errPos)

        

        if self.armed:
            
            self.Beta_0_prev = self.beta_0
            self.Beta_1_prev = self.beta_1
            beta_0_prev = self.Beta_0_prev            
            beta_1_prev = self.Beta_1_prev  

            self.s_prev =   self.Kp*self.errPosPrev + self.Kd*self.errVelPrev

            beta_0_dot = 0
            beta_1_dot = 0
            s_norm = np.linalg.norm(s)
            s_prev_norm = np.linalg.norm(self.s_prev)
            E_norm = np.linalg.norm(E)
        
            if s_norm > s_prev_norm: 
                beta_0_dot = self.g*s_norm
                beta_1_dot = self.g*np.linalg.norm(E)*s_norm
            elif s_norm <= s_prev_norm:
                beta_0_dot = -self.g*s_norm
                beta_1_dot = -self.g*E_norm*s_norm

        
            beta_0 = beta_0_prev + beta_0_dot*dt
            beta_1 = beta_1_prev + beta_1_dot*dt
            # beta_1 = 0
            self.c = beta_0 + beta_1 * E_norm
            self.du = (self.c)*(s/np.sqrt(np.square(s_norm)+np.square(self.epsilon)))
            self.u = self.desAcc - self.Kd*errVel - self.Kp*errPos + self.alpha*self.du# - self.gravity
            #print(self.c , self.du,self.u)
            self.data_out.beta_0 = beta_0
            self.data_out.beta_1 = beta_1
            self.data_out.c = self.c

            des_th = self.torque_prev - np.array(np.multiply(self.M,np.array(self.curAccPrev).T) + np.multiply(self.M,np.array(self.u).T)).T 
            #print("des_th",des_th )
            #self.curAccPrev = curAcc
            self.errPosPrev = errPos
            self.errVelPrev = errVel
            #self.torque_prev = des_th

            if np.linalg.norm(des_th) > self.max_th:
                des_th = (self.max_th/np.linalg.norm(des_th))*des_th
                # print("LOOP")
                #print("Des_th",des_th)
                self.torque_prev = des_th
                self.curAccPrev = curAcc
                #self.errPosPrev = errPos
                # self.errVelPrev = errVel

        #self.array2Vector3(sv, self.data_out.sp)
        self.array2Vector3(errPos, self.data_out.position_error)
        self.array2Vector3(errVel, self.data_out.velocity_error)
        return des_th

    def acc2quat(self, des_th, des_yaw):
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
        des_th= self.th_des()    
        r_des = self.acc2quat(des_th, 0.0)
        rot_44 = np.vstack((np.hstack((r_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))

        quat_des = quaternion_from_matrix(rot_44)
       
        zb = r_des[:,2]
        thrust = self.norm_thrust_const * des_th.dot(zb)
        self.data_out.thrust = thrust
        
        thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))

        now = rospy.Time.now()
        self.att_cmd.header.stamp = now
        self.thrust_cmd.header.stamp = now
        self.data_out.header.stamp = now
        self.att_cmd.pose.orientation.x = quat_des[0]
        self.att_cmd.pose.orientation.y = quat_des[1]
        self.att_cmd.pose.orientation.z = quat_des[2]
        self.att_cmd.pose.orientation.w = quat_des[3]
        self.thrust_cmd.thrust = thrust
        # print(thrust)
        # print(quat_des)
        self.data_out.orientation = self.att_cmd.pose.orientation
        '''delta_orient = self.att_cmd.pose.orientation - curorien
                                self.delta_or = delta_orient'''

    def pub_att(self):
        self.geo_con()
        self.thrust_pub.publish(self.thrust_cmd)
        self.att_pub.publish(self.att_cmd)
        self.data_pub.publish(self.data_out)

# Main function
def main(argv):
   
    rospy.init_node('setpoint_node', anonymous=True)
    #rospy.init_node('demo_detach_links')
    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('mavros/imu/data', Imu, cnt.accCB)


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
