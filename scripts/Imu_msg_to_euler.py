#!/usr/bin/env python
# header:
#   seq: 174
#   stamp:
#     secs: 1655999161
#     nsecs: 157929897
#   frame_id: ''
# name: [r_hip_yaw_joint, r_hip_roll_joint, r_hip_pitch_joint, r_knee_pitch_joint, r_ankle_pitch_joint,
#   r_ankle_roll_joint, l_hip_yaw_joint, l_hip_roll_joint, l_hip_pitch_joint, l_knee_pitch_joint,
#   l_ankle_pitch_joint, l_ankle_roll_joint]
# position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# velocity: []
# effort: []

    # pub_joint_origin_imu = nh_.advertise<sensor_msgs::Imu>("origin_Imu_pub",1);
    # pub_joint_r_shank_imu = nh_.advertise<sensor_msgs::Imu>("r_shank_Imu_pub",1);
    # pub_joint_l_shank_imu = nh_.advertise<sensor_msgs::Imu>("l_shank_Imu_pub",1);
    # pub_joint_r_thigh_imu = nh_.advertise<sensor_msgs::Imu>("r_thingh_Imu_pub",1);
    # pub_joint_l_thigh_imu = nh_.advertise<sensor_msgs::Imu>("l_thingh_Imu_pub",1);

# !/usr/bin/env python
import math
from re import X
import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from can_imu_lws.msg import IMU_Euler_msg


euler_OR_pub = None
euler_RT_pub = None
euler_LT_pub = None
euler_RS_pub = None
euler_LS_pub = None



def euler_to_radian(get_euler):
    radians = math.pi / 180 * get_euler
    return radians


def o_imu_callback(data):
    X = data.orientation.x
    Y = data.orientation.y
    Z = data.orientation.Z
    W = data.orientation.w
    (r, p, y) = euler_from_quaternion(X, Y, Z, W)
    IMU_Euler_msg


   


def listener():
    global joint_state_pub
    rospy.init_node('imu_to_jointState_node', anonymous=True)
    euler_OR_pub = rospy.Publisher('o_euler', IMU_Euler_msg, queue_size=10)
    o_imu_euler_sub = rospy.Subscriber('origin_Imu_pub', Imu, o_imu_callback)
   
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
