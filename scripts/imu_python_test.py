#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu


def talker():
    imu_pub = rospy.Publisher('ros_imu', Imu, queue_size=10)
    rospy.init_node('ros_imu_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        stamp = rospy.get_rostime()
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "base_link"

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_pub.publish(imu_msg)
	rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInternalException:
        pass
