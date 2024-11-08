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
        # self.att_pub = rospy.Publisher(self.uavn1 +'/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        # self.thrust_pub = rospy.Publisher(self.uavn1 +'/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        # self.body_rate = rospy.Publisher(self.uavn1 +'/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.pose_pub0 = rospy.Publisher("/drone0_poseintion", PoseStamped, queue_size=10)
        self.pose_pub1 = rospy.Publisher("/drone1_poseintion", PoseStamped, queue_size=10)



    def base_link_pos(self, msg0):
        idx = msg0.name.index('iris0')
        idx1 = msg0.name.index('iris1')
        iris_pose0 = msg0.pose[idx]
        iris_twist0 = msg0.twist[idx]

        iris_pose1 = msg0.pose[idx1]
        iris_twist1 = msg0.twist[idx1]

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
    
    def pose_publisher(self):

        # self.local_pos0.header = rospy.Time.now()
        # self.local_pos1.header = rospy.Time.now()

        self.pose_pub0.publish(self.local_pos0)
        self.pose_pub1.publish(self.local_pos1)





# Main function
def main(argv):

    uavn2 = '/uav2'
    uavn1 = '/uav2'
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.base_link_pos)
    # rospy.Subscriber(uavn1 +'/mavros/state', State, cnt.stateCb)

    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    # rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)
    # rospy.Subscriber(uavn1 +'/mavros/imu/data', Imu, cnt.accCB1)
    # rospy.Subscriber(uavn2 +'/mavros/imu/data', Imu, cnt.accCB2)

    # rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    # rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    # rospy.Subscriber('yaw_in_deg',Float32,cnt.yawAngle)

    # sp_pub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # sp_pub1 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # ROS main loop
    while not rospy.is_shutdown():

#--------------------------------------------
        cnt.pose_publisher()
        rate.sleep()
#--------------------------------------------  

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass

