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

# !/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from can_imu_lws.msg import IMU_Euler_msg


origin_imu_id = 0x54
right_thigh_id = 0x55
left_thigh_id = 0x53
right_shank_id = 0x52
left_shank_id = 0x51

joint_state_pub = None
joint_position_list = [0, 0, 0,
                       0,
                       0, 0,
                       0, 0, 0,
                       0,
                       0, 0]
# Yaw Roll Pitch
origin_imu_id_current_list = [0, 0, 0]
right_thigh_imu_current_list = [0, 0, 0]
left_thigh_imu_current_list = [0, 0, 0]
right_shank_imu_current_list = [0, 0, 0]
left_shank_imu_current_list = [0, 0, 0]

can_id_exit_list = [0, 0, 0, 0, 0, 0,0]

# imu_current_list = [origin_imu_id_current_list, right_thigh_imu_current_list, right_shank_imu_current_list, left_thigh_imu_current_list, left_shank_imu_current_list]


def euler_to_radian(get_euler):
    radians = math.pi / 180 * get_euler
    return radians


def euler_callback(data):
    global joint_position_list
    global joint_state_pub
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.get_rostime()
    joint_state_msg.header.frame_id = ''
    joint_state_msg.name = ['r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint',
                            'r_knee_pitch_joint',
                            'r_ankle_pitch_joint', 'r_ankle_roll_joint',
                            'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint',
                            'l_knee_pitch_joint',
                            'l_ankle_pitch_joint', 'l_ankle_roll_joint']
    joint_state_msg.position = joint_position_list
    can_id_flag = data.imu_can_id - 0x50
    can_id_exit_list[can_id_flag] = 1
    if data.imu_can_id == origin_imu_id:
        origin_imu_id_current_list = [euler_to_radian(data.Yaw), euler_to_radian(data.Roll), euler_to_radian(data.Pitch)]
    elif data.imu_can_id == right_thigh_id:
        right_thigh_imu_current_list = [euler_to_radian(data.Yaw), euler_to_radian(data.Roll), euler_to_radian(data.Pitch)]
    elif data.imu_can_id == left_thigh_id:
        left_thigh_imu_current_list = [euler_to_radian(data.Yaw), euler_to_radian(data.Roll), euler_to_radian(data.Pitch)]
    elif data.imu_can_id == right_shank_id:
        right_shank_imu_current_list = [euler_to_radian(data.Yaw), euler_to_radian(data.Roll), euler_to_radian(data.Pitch)]
    elif data.imu_can_id == left_shank_id:
        left_shank_imu_current_list = [euler_to_radian(data.Yaw), euler_to_radian(data.Roll), euler_to_radian(data.Pitch)]

    # right
    if can_id_exit_list[origin_imu_id - 0x50] != 0 and can_id_exit_list[right_thigh_id - 0x50] != 0:
        joint_position_list[0] = right_thigh_imu_current_list[0] - origin_imu_id_current_list[0]
        # leg rise
        joint_position_list[1] = right_thigh_imu_current_list[2] - origin_imu_id_current_list[2]
        joint_position_list[2] = right_thigh_imu_current_list[1] - origin_imu_id_current_list[1]
    else:
        joint_position_list[0] = 0
        joint_position_list[1] = 0
        joint_position_list[2] = 0

    if can_id_exit_list[right_shank_id - 0x50] != 0 and can_id_exit_list[right_thigh_id - 0x50] != 0:
        joint_position_list[3] = right_shank_imu_current_list[2] - right_thigh_imu_current_list[2]
    else:
        joint_position_list[3] = 0


    # left
    if can_id_exit_list[origin_imu_id - 0x50] != 0 and can_id_exit_list[left_thigh_id - 0x50] != 0:
        joint_position_list[6] = left_thigh_imu_current_list[0] - origin_imu_id_current_list[0]
        # leg rise
        joint_position_list[7] = left_thigh_imu_current_list[2] - origin_imu_id_current_list[2]
        joint_position_list[8] = left_thigh_imu_current_list[1] - origin_imu_id_current_list[1]
    else:
        joint_position_list[6] = 0
        joint_position_list[7] = 0
        joint_position_list[8] = 0

    if can_id_exit_list[left_shank_id - 0x50] != 0 and can_id_exit_list[left_thigh_id - 0x50] != 0:
        joint_position_list[9] = left_shank_imu_current_list[2] - right_thigh_imu_current_list[2]   
    else:
        joint_position_list[9] = 0
    joint_state_pub.publish(joint_state_msg)
    # print(joint_position_list[0])
    print(joint_position_list[1])


def listener():
    global joint_state_pub
    rospy.init_node('imu_to_jointState_node', anonymous=True)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    imu_euler_sub = rospy.Subscriber('imu_euler_pub', IMU_Euler_msg, euler_callback)
   
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInternalException:
        pass
