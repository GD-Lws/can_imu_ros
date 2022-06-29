import datetime
import math
import struct
import time

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from beginner_tutorials.msg import IMU_Euler
from Can_Derive import Can_Derive


# 16 to ieee float
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def receive_data_pro(data_L, data_H):
    PN_flag = -1
    if data_H > 127:
        PN_flag = 1
    ans = data_L + data_H * 256
    if PN_flag == 1:
        return 65535 - ans + 1
    else:
        return ans


def imu_data_process(receive_date, index, result_date, data_type):
    WxL = receive_date[2]
    WxH = receive_date[3]
    Wx = receive_data_pro(WxL, WxH)
    WyL = receive_date[4]
    WyH = receive_date[5]
    Wy = receive_data_pro(WyL, WyH)
    WzL = receive_date[6]
    WzH = receive_date[7]
    Wz = receive_data_pro(WzL, WzH)
    if data_type == 1:
        # linear_acceleration
        [result_date[index], result_date[index + 1], result_date[index + 2]] = [
            hex_to_short(receive_date)[i] / 32768.0 * 16 * 9.8 for i in range(1, 4)]
        print(str(datetime.datetime.now()), "  ", index, " linear_acceleration: ")
    elif data_type == 2:
        # angular_velocity
        [result_date[index], result_date[index + 1], result_date[index + 2]] = [
            hex_to_short(receive_date)[i] / 32768.0 * 2000 * math.pi / 180 for i in range(1, 4)]
        print(str(datetime.datetime.now()), "  ", index, " angular_velocity: ")
    elif data_type == 3:
        # euler
        [result_date[index], result_date[index + 1], result_date[index + 2]] = [
            hex_to_short(receive_date)[i] / 32768.0 * 180 for i in range(1, 4)]
        print(str(datetime.datetime.now()), "  ", index, " euler: ")
    else:
        print("ERROR DATA_TYPE")
    print(result_date[index], " ", result_date[index + 1], " ", result_date[index + 2])
    print("\n")


# imu_publisher=rospy.Publisher('ros_imu', Imu, queue_size=10)
class HWT901B_CAN(Can_Derive):
    def __init__(self, win_linux=0, imu_num=10, imu_msg_publisher = rospy.Publisher('ros_imu_pub', Imu, queue_size=10), 
    imu_euler_publisher = rospy.Publisher('imu_euler_pub',IMU_Euler, queue_size=10), pub_mode = 0):
        Can_Derive.__init__(self, win_linux=win_linux)
        self.__imu_num = imu_num
        self.__imu_msg_publisher = imu_msg_publisher
        self.__imu_euler_publisher = imu_euler_publisher
        self.__can_id_list = []
        self.__imu_date_list = []
        self.__pub_mode = pub_mode
        for index in range(0, imu_num):
            self.__can_id_list.append(0)
            for index_0 in range(0, 9):
                self.__imu_date_list.append(0)

    def change_mode(self,mode):
        self.__pub_mode = mode

    # receive_date decode
    # 0x51 AC 0x52 EC 0x53 AG
    def receiving_msg_processing(self, vci_can_obj):
        print("can_id: ", vci_can_obj.ID)
        print("data_array: ")
        print(list(vci_can_obj.Data))
        temp_can_id = vci_can_obj.ID - 0x50
        if temp_can_id + 1 > self.__imu_num:
            self.__imu_num = temp_can_id + 1
            while len(self.__can_id_list) < self.__imu_num:
                self.__can_id_list.append(0)
            while len(self.__imu_date_list) < self.__imu_num * 9:
                self.__imu_date_list.append(0)
            print("Add new can_id")
            print("Now can_id_list len", len(self.__can_id_list))

        elif self.__can_id_list[temp_can_id] == 3:
            # pub imu_msg
            if self.__pub_mode == 1 or self.__pub_mode == 2:
                stamp = rospy.get_rostime()
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = str(vci_can_obj.ID)
                qua = quaternion_from_euler(self.__imu_date_list[temp_can_id * 9 + 6 + 0], self.__imu_date_list[temp_can_id * 9 + 6 + 1], self.__imu_date_list[temp_can_id * 9 + 6 + 2])
                
                imu_msg.orientation.x = qua[0]
                imu_msg.orientation.y = qua[1]
                imu_msg.orientation.z = qua[2]
                imu_msg.orientation.w = qua[3]
                
                imu_msg.angular_velocity.x = self.__imu_date_list[temp_can_id * 9 + 3]
                imu_msg.angular_velocity.y = self.__imu_date_list[temp_can_id * 9 + 4]
                imu_msg.angular_velocity.z = self.__imu_date_list[temp_can_id * 9 + 5]
                
                imu_msg.linear_acceleration.x = self.__imu_date_list[temp_can_id * 9 + 0]
                imu_msg.linear_acceleration.y = self.__imu_date_list[temp_can_id * 9 + 1]
                imu_msg.linear_acceleration.z = self.__imu_date_list[temp_can_id * 9 + 2]
                print("finish one imu pub")
                self.__imu_msg_publisher.publish(imu_msg)
            self.__can_id_list[temp_can_id] = 0

        if vci_can_obj.Data[1] == 0x51:
            # linear_acceleration
            imu_data_process(list(vci_can_obj.Data), temp_can_id * 9, self.__imu_date_list, vci_can_obj.Data[1] - 0x50)
            if self.__can_id_list[temp_can_id] == 0:
                self.__can_id_list[temp_can_id] = 1
        elif vci_can_obj.Data[1] == 0x52:
            # angular_velocity
            imu_data_process(list(vci_can_obj.Data), temp_can_id * 9 + 3, self.__imu_date_list,
                             vci_can_obj.Data[1] - 0x50)
            if self.__can_id_list[temp_can_id] == 1:
                self.__can_id_list[temp_can_id] = 2
        elif vci_can_obj.Data[1] == 0x53:
            # euler
            imu_data_process(list(vci_can_obj.Data), temp_can_id * 9 + 6, self.__imu_date_list,
                             vci_can_obj.Data[1] - 0x50)
            if self.__pub_mode == 0 or self.__pub_mode == 2:
                imu_euler = IMU_Euler()
                imu_euler.imu_can_id = vci_can_obj.ID
                imu_euler.Roll = self.__imu_date_list[temp_can_id * 9 + 6 + 0] 
                imu_euler.Pitch = self.__imu_date_list[temp_can_id * 9 + 6 + 1]
                imu_euler.Yaw = self.__imu_date_list[temp_can_id * 9 + 6 + 2]
                self.__imu_euler_publisher.publish(imu_euler)

            if self.__can_id_list[temp_can_id] == 2:
                self.__can_id_list[temp_can_id] = 3
    


if __name__ == "__main__":
    imu_receive = HWT901B_CAN()
    imu_receive.can_init()
    imu_receive.can_channel_open()
    for i in range(0, 100):
        imu_receive.can_receive_msg_2()
    imu_receive.can_close()

