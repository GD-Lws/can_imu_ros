#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

from IMU_HWT901B import HWT901B_CAN
from can_imu_lws.msg import IMU_Euler

def talker():
    imu_msg_pub = rospy.Publisher('ros_imu_pub', Imu, queue_size=10)
    imu_euler_pub = rospy.Publisher('imu_euler_pub', IMU_Euler, queue_size=10)
    imu_node = rospy.init_node('can_imu_publisher_node', anonymous=True)
    imu_can = HWT901B_CAN(win_linux=0, imu_msg_publisher=imu_msg_pub, imu_euler_publisher= imu_euler_pub, pub_mode=0)
    imu_can.can_init()
    imu_can.can_channel_open()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        imu_can.can_receive_msg_2()
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
