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

    def setTakeoff(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 + '/mavros/cmd/takeoff' and uavn2 + '/mavros/cmd/takeoff')
        try:
            takeoffService1 = rospy.ServiceProxy(uavn1 + '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService2 = rospy.ServiceProxy(uavn2 + '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)

            takeoffService1(altitude = 3)
            takeoffService2(altitude = 3)

        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def setArm(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 + '/mavros/cmd/arming' and uavn2 + '/uav1/mavros/cmd/arming')
        try:
            armService1 = rospy.ServiceProxy(uavn1 + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService1(True)

            armService2 = rospy.ServiceProxy(uavn2 + '/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2(True)

        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self, uavn1, uavn2):
        rospy.wait_for_service((uavn1 + 'mavros/cmd/arming') or (uavn2 + '/uav1/mavros/cmd/arming'))
        try:
            armService1 = rospy.ServiceProxy(uavn1 + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService1(False)

            armService2 = rospy.ServiceProxy(uavn2 + 'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService2(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 + 'mavros/set_mode' and uavn2 +'/uav1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy(uavn1 + + '/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='STABILIZED')

            flightModeService2 = rospy.ServiceProxy(uavn1 +'/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2(custom_mode='STABILIZED')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 +'/mavros/set_mode'  and uavn2 +'/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy(uavn1 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='OFFBOARD')

            flightModeService2 = rospy.ServiceProxy(uavn2 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2(custom_mode='OFFBOARD')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 +'/mavros/set_mode'  and uavn2 + '/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy(uavn1 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='ALTCTL')

            flightModeService2 = rospy.ServiceProxy(uavn2+'/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2(custom_mode='ALTCTL')

        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 +'/mavros/set_mode'  and uavn2 + '/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy(uavn1 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='POSCTL')

            flightModeService2 = rospy.ServiceProxy(uavn2 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self, uavn1, uavn2):
        rospy.wait_for_service(uavn1 +'/mavros/set_mode'  and uavn2 +'/uav1/mavros/set_mode')
        try:
            flightModeService1 = rospy.ServiceProxy(uavn1 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService1(custom_mode='AUTO.LAND')

            flightModeService2 = rospy.ServiceProxy(uavn2 +'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService2(custom_mode='AUTO.LAND')

        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
           


class Controller:
    # initialization method
    def __init__(self):



        self.uavn1 = '/uav0'
        self.uavn2 = '/uav0'
        self.drone_num = 0
        # Drone state
        self.state0 = State()
        self.state1 = State()
        # Instantiate a setpoints message
        self.sp0 = PoseStamped()
        self.sp1 = PoseStamped()
        # set the flag to use position setpoints and yaw angle
        self.yaw_angle0 = Float32()
        self.yaw_angle0.data = 0.0

        self.imu0= Imu()
        self.imu1= Imu()
        
        self.sp0.pose.position.x = 0.0
        self.sp0.pose.position.y = 0.0
        self.ALT_SP0 = 3.0
        
        self.sp1.pose.position.x = -3.0
        self.sp1.pose.position.y = 3.0
        self.ALT_SP1 = 3.0

        self.sp0.pose.position.z = self.ALT_SP0
        self.sp1.pose.position.z = self.ALT_SP1

        self.local_pos0 = PoseStamped()
        self.local_vel0 = TwistStamped()

        self.local_pos1 = PoseStamped()
        self.local_vel1 = TwistStamped()

        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.zeros(3)
        self.desAcc = np.zeros(3)

        self.errInt = np.zeros(3)
        self.att_cmd = PoseStamped()
        self.thrust_cmd = Thrust()
        
# all data has been collected on this value
        self.Kpos_ = np.array([4, 4, 12])
        self.Kvel_ = np.array([3, 3, 3])
        self.Kint_ = np.array([0.2, 0.2, 3.0])

        self.norm_thrust_const = 0.056
        self.max_th = 36.0
        self.max_throttle = 0.96

        self.gravity = np.array([0, 0, 9.8])
        self.pre_time = rospy.get_time()

        self.armed = False

        self.command = AttitudeTarget()
        self.collective_thrust = 0.0

        # Publishers
        self.att_pub = rospy.Publisher(self.uavn1 +'/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub = rospy.Publisher(self.uavn1 +'/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.body_rate = rospy.Publisher(self.uavn1 +'/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.att_pub1 = rospy.Publisher(self.uavn2 +'/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_pub1 = rospy.Publisher(self.uavn2 +'/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.body_rate1 = rospy.Publisher(self.uavn2 +'/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.data_pub = rospy.Publisher('/data_out', PlotDataMsg, queue_size=10)

        self.data_out = PlotDataMsg()


    def base_link_pos(self, msg0, msg1):
        idx = msg0.name.index('iris0')
        idx = msg1.name.index('iris1')
        iris_pose0 = msg0.pose[idx]
        iris_twist0 = msg0.twist[idx]

        iris_pose1 = msg1.pose[idx]
        iris_twist1 = msg1.twist[idx]

# taking positipon from iris gazebo for second drone 

        self.local_pos0.pose.position.x = iris_pose0.position.x
        self.local_pos0.pose.position.y = iris_pose0.position.y
        self.local_pos0.pose.position.z = iris_pose0.position.z

        self.local_pos0.pose.orientation.x = iris_pose0.orientation.x
        self.local_pos0.pose.orientation.y = iris_pose0.orientation.y
        self.local_pos0.pose.orientation.z = iris_pose0.orientation.z
        self.local_pos0.pose.orientation.w = iris_pose0.orientation.w

        self.local_vel0.twist.linear.x = iris_twist0.linear.x
        self.local_vel0.twist.linear.y = iris_twist0.linear.y
        self.local_vel0.twist.linear.z = iris_twist0.linear.z

        self.local_vel0.twist.angular.x = iris_twist0.angular.x
        self.local_vel0.twist.angular.y = iris_twist0.angular.y
        self.local_vel0.twist.angular.z = iris_twist0.angular.z

# taking positipon from iris gazebo for second drone 

        self.local_pos1.pose.position.x = iris_pose1.position.x
        self.local_pos1.pose.position.y = iris_pose1.position.y
        self.local_pos1.pose.position.z = iris_pose1.position.z

        self.local_pos1.pose.orientation.x = iris_pose1.orientation.x
        self.local_pos1.pose.orientation.y = iris_pose1.orientation.y
        self.local_pos1.pose.orientation.z = iris_pose1.orientation.z
        self.local_pos1.pose.orientation.w = iris_pose1.orientation.w

        self.local_vel1.twist.linear.x = iris_twist1.linear.x
        self.local_vel1.twist.linear.y = iris_twist1.linear.y
        self.local_vel1.twist.linear.z = iris_twist1.linear.z

        self.local_vel1.twist.angular.x = iris_twist1.angular.x
        self.local_vel1.twist.angular.y = iris_twist1.angular.y
        self.local_vel1.twist.angular.z = iris_twist1.angular.z

    def posCb(self, msg):
        self.local_pos0.pose.position.x = msg.pose.position.x
        self.local_pos0.pose.position.y = msg.pose.position.y
        self.local_pos0.pose.position.z = msg.pose.position.z
        self.local_pos0.pose.orientation.x = msg.pose.orientation.x
        self.local_pos0.pose.orientation.y = msg.pose.orientation.y
        self.local_pos0.pose.orientation.z = msg.pose.orientation.z
        self.local_pos0.pose.orientation.w = msg.pose.orientation.w


        self.local_pos1.pose.position.x = msg.pose.position.x
        self.local_pos1.pose.position.y = msg.pose.position.y
        self.local_pos1.pose.position.z = msg.pose.position.z
        self.local_pos1.pose.orientation.x = msg.pose.orientation.x
        self.local_pos1.pose.orientation.y = msg.pose.orientation.y
        self.local_pos1.pose.orientation.z = msg.pose.orientation.z
        self.local_pos1.pose.orientation.w = msg.pose.orientation.w

        print()

    def velCb(self, msg):
        self.local_vel0.twist.linear.x = msg.twist.linear.x
        self.local_vel0.twist.linear.y = msg.twist.linear.y
        self.local_vel0.twist.linear.z = msg.twist.linear.z

        self.local_vel0.twist.angular.x = msg.twist.angular.x
        self.local_vel0.twist.angular.y = msg.twist.angular.y
        self.local_vel0.twist.angular.z = msg.twist.angular.z


        self.local_vel1.twist.linear.x = msg.twist.linear.x
        self.local_vel1.twist.linear.y = msg.twist.linear.y
        self.local_vel1.twist.linear.z = msg.twist.linear.z

        self.local_vel1.twist.angular.x = msg.twist.angular.x
        self.local_vel1.twist.angular.y = msg.twist.angular.y
        self.local_vel1.twist.angular.z = msg.twist.angular.z


    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        self.desAcc = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])


    ## Drone State callback
    def stateCb0(self, msg):
        self.state0 = msg

    def stateCb1(self, msg):
        self.state1 = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp0.pose.position.x = self.local_pos0.pose.position.x
        self.sp0.pose.position.y = self.local_pos0.pose.position.y
        self.sp0.pose.position.z = self.local_pos0.pose.position.z

        self.sp0.pose.position.x = self.local_pos1.pose.position.x
        self.sp0.pose.position.y = self.local_pos1.pose.position.y
        self.sp0.pose.position.z = self.local_pos1.pose.position.z

    def accCB1(self,msg):
        self.imu0.orientation.w = msg.orientation.w
        self.imu0.orientation.x = msg.orientation.x
        self.imu0.orientation.y = msg.orientation.y
        self.imu0.orientation.z = msg.orientation.z

        self.imu0.angular_velocity.x = msg.angular_velocity.x
        self.imu0.angular_velocity.y = msg.angular_velocity.y
        self.imu0.angular_velocity.z = msg.angular_velocity.z

        self.imu0.linear_acceleration.x = msg.linear_acceleration.x
        self.imu0.linear_acceleration.y = msg.linear_acceleration.y
        self.imu0.linear_acceleration.z = msg.linear_acceleration.z

    def accCB2(self,msg):
        self.imu1.orientation.w = msg.orientation.w
        self.imu1.orientation.x = msg.orientation.x
        self.imu1.orientation.y = msg.orientation.y
        self.imu1.orientation.z = msg.orientation.z

        self.imu1.angular_velocity.x = msg.angular_velocity.x
        self.imu1.angular_velocity.y = msg.angular_velocity.y
        self.imu1.angular_velocity.z = msg.angular_velocity.z

        self.imu1.linear_acceleration.x = msg.linear_acceleration.x
        self.imu1.linear_acceleration.y = msg.linear_acceleration.y
        self.imu1.linear_acceleration.z = msg.linear_acceleration.z


    def newPoseCB(self, msg):
        self.sp0.pose.position.x = msg.pose.position.x        
        self.sp0.pose.position.y = msg.pose.position.y
        self.sp0.pose.position.z = msg.pose.position.z
   
        self.sp0.pose.orientation.x = msg.pose.orientation.x
        self.sp0.pose.orientation.y = msg.pose.orientation.y
        self.sp0.pose.orientation.z = msg.pose.orientation.z
        self.sp0.pose.orientation.w = msg.pose.orientation.w

    # def newPoseCB1(self, msg):
    #     if(self.sp.pose.position != msg.pose.position):
    #         print("New pose received")
    #     self.sp.pose.position.x = msg.pose.position.x        
    #     self.sp.pose.position.y = msg.pose.position.y
    #     self.sp.pose.position.z = msg.pose.position.z
   
    #     self.sp.pose.orientation.x = msg.pose.orientation.x
    #     self.sp.pose.orientation.y = msg.pose.orientation.y
    #     self.sp.pose.orientation.z = msg.pose.orientation.z
    #     self.sp.pose.orientation.w = msg.pose.orientation.w

    def yawAngle(self,msg):
        self.yaw_angle0.data = msg.data
        self.yaw_angle0.data = np.deg2rad(self.yaw_angle0.data)

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


    def a_des(self):
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.03:
            dt = 0.03

        curPos0 = self.vector2Arrays(self.local_pos0.pose.position)

        curPos1 = self.vector2Arrays(self.local_pos1.pose.position)
        desPos = self.vector2Arrays(self.sp0.pose.position)
        curVel = self.vector2Arrays(self.local_vel0.twist.linear)
        curAcc = self.vector2Arrays(self.imu0.linear_acceleration) - self.gravity

        errPos0 = curPos0 - desPos
        errPos1 = curPos1 - desPos
        errVel = curVel - self.desVel
        errAcc = curAcc - self.desAcc


#  DATA COLLECTION PURPOSE ONLY
        self.data_out.x_curr = curPos0[0]
        self.data_out.y_curr = curPos0[1]
        self.data_out.z_curr = curPos0[2]
        self.data_out.x_dot_curr = curVel[0]
        self.data_out.y_dot_curr = curVel[1]
        self.data_out.z_dot_curr = curVel[2]
        self.data_out.x_ddot_curr = curAcc[0]
        self.data_out.y_ddot_curr = curAcc[1]
        self.data_out.z_ddot_curr = curAcc[2]
        self.data_out.x_des = desPos[0]
        self.data_out.y_des = desPos[1]
        self.data_out.z_des = desPos[2]
        self.data_out.x_dot_des = self.desVel[0]
        self.data_out.y_dot_des = self.desVel[1]
        self.data_out.z_dot_des = self.desVel[2]
        self.data_out.x_ddot_des = self.desAcc[0]
        self.data_out.y_ddot_des = self.desAcc[1]
        self.data_out.z_ddot_des = self.desAcc[2]
        self.data_out.x_err = errPos0[0]
        self.data_out.y_err = errPos0[1]
        self.data_out.z_err = errPos0[2]
        self.data_out.x_dot_err = errVel[0]
        self.data_out.y_dot_err = errVel[1]
        self.data_out.z_dot_err = errVel[2]
        self.data_out.x_ddot_err = errAcc[0]
        self.data_out.y_ddot_err = errAcc[1]
        self.data_out.z_ddot_err = errAcc[2]

        if self.armed:
            self.errInt += errPos0*dt
            self.errInt += errPos1*dt

        des_a = -(self.Kpos_*errPos0 + self.Kvel_*errVel + self.Kint_*self.errInt) + self.gravity

        # print(errPos)
        # print(des_a)
        print('--------------')

        if np.linalg.norm(des_a) > self.max_th:
            des_a = (self.max_th/np.linalg.norm(des_a))*des_a

        return des_a 


    def geo_con_(self):
        des_a = self.a_des()

        pose = transformations.quaternion_matrix(  
                np.array([self.local_pos0.pose.orientation.x, 
                             self.local_pos0.pose.orientation.y, 
                             self.local_pos0.pose.orientation.z, 
                             self.local_pos0.pose.orientation.w]))  #4*4 matrix
        pose_temp1 = np.delete(pose, -1, axis=1)
        rot_curr = np.delete(pose_temp1, -1, axis=0)   #3*3 current rotation matrix
        zb_curr = rot_curr[:,2]
        thrust = self.norm_thrust_const * des_a.dot(zb_curr)
        self.collective_thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))

        self.data_out.tau_x = des_a[0]
        self.data_out.tau_y = des_a[1]
        self.data_out.tau_z = des_a[2]
        self.data_out.thrust = des_a.dot(zb_curr)
        self.data_out.collective_thrust = thrust
        # ----------------------------------------------- # 

        # Current Euler orientation and 
        orientation_q = self.local_pos0.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll_curr, pitch_curr, yaw_curr) = euler_from_quaternion (orientation_list)

        #---------------------------------------------#
        rot_des = self.acc2quat(des_a, self.yaw_angle.data)   #desired yaw = 0 Intially
        rot_44 = np.vstack((np.hstack((rot_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))

        quat_des = quaternion_from_matrix(rot_44)
        orientation_list = [quat_des[0], quat_des[1], quat_des[2], quat_des[3]]
        (roll_des, pitch_des, yaw_des) = euler_from_quaternion (orientation_list)

        # -------------------------------------------------#

        # DATA COLLECTION PUROSE ONLY
        angle_error_matrix = 0.5* (np.dot(np.transpose(rot_des), rot_curr) -
                                    np.dot(np.transpose(rot_curr), rot_des) ) #skew matrix
        roll_x_err = -angle_error_matrix[1,2]
        pitch_y_err = angle_error_matrix[0,2]   
        yaw_z_err = -angle_error_matrix[0,1]



        self.data_out.roll_dot_curr = self.local_vel0.twist.angular.x
        self.data_out.pitch_dot_curr = self.local_vel0.twist.angular.y
        self.data_out.yaw_dot_curr = self.local_vel0.twist.angular.z
        self.data_out.roll_err = roll_x_err
        self.data_out.pitch_err = pitch_y_err
        self.data_out.yaw_err = yaw_z_err
        self.data_out.roll_curr = roll_curr
        self.data_out.pitch_curr = pitch_curr
        self.data_out.yaw_curr = yaw_curr
        self.data_out.roll_des = roll_des
        self.data_out.pitch_des = pitch_des
        self.data_out.yaw_des = yaw_des


    # -------------------------------
        # Control MODE 1
        now = rospy.Time.now()
        self.att_cmd.header.stamp = now
        self.thrust_cmd.header.stamp = now
        self.data_out.header.stamp = now
        self.att_cmd.pose.orientation.x = quat_des[0]
        self.att_cmd.pose.orientation.y = quat_des[1]
        self.att_cmd.pose.orientation.z = quat_des[2]
        self.att_cmd.pose.orientation.w = quat_des[3]
        self.thrust_cmd.thrust = self.collective_thrust


    def acc2quat(self,des_a, des_yaw):
        proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
        if np.linalg.norm(des_a) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_a / np.linalg.norm(des_a)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat

    def pub_att(self):
        self.geo_con_()
        self.thrust_pub.publish(self.thrust_cmd)
        self.att_pub.publish(self.att_cmd)
        self.data_pub.publish(self.data_out)

        self.thrust_pub1.publish(self.thrust_cmd)
        self.att_pub1.publish(self.att_cmd)
        # self.data_pub1.publish(self.data_out)

# Main function
def main(self, argv):
    uavn1 = '/uav1'
    uavn2 = '/uav2'
    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()  #flight modes
    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.base_link_pos)
    rospy.Subscriber(uavn1 +'/mavros/state', State, cnt.stateCb)

    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    # rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    rospy.Subscriber(uavn1 +'/mavros/imu/data', Imu, cnt.accCB1)
    rospy.Subscriber(uavn2 +'/mavros/imu/data', Imu, cnt.accCB2)

    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('yaw_in_deg',Float32,cnt.yawAngle)

    sp_pub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    sp_pub1 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("ARMING")
    print("done")
    while not cnt.state.armed:
        modes.setArm()
        cnt.armed = True
        rate.sleep()

    cnt.armed = True
    k=0

    

    while k<20:
        sp_pub.publish(cnt.sp)
        sp_pub1.publish(cnt.sp)
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

