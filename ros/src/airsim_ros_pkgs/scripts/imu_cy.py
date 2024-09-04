import rospy
from sensor_msgs.msg import Imu
import threading
import airsim
import re
import signal

ip = '192.168.1.51'
path = 'd/AirSim/ros'
client = airsim.MultirotorClient(ip)
client.confirmConnection()
client.enableApiControl(False)
previous_imu_data_x = None
previous_imu_data_y = None
previous_imu_data_z = None
exit_event = threading.Event()
imu_threads = []

def imu_data_thread(publisher, sensor_name, vehicle_name):
    global previous_imu_data_x
    global previous_imu_data_y
    global previous_imu_data_z
    global exit_event
    rate = rospy.Rate(500)
    while not rospy.is_shutdown() and not exit_event.is_set():
        try:
            imu_data = client.getImuData(sensor_name, vehicle_name)
            if (imu_data.linear_acceleration.z_val == previous_imu_data_z or imu_data.linear_acceleration.y_val ==
                    previous_imu_data_y or imu_data.linear_acceleration.x_val == previous_imu_data_x):
                continue
            imu_msg = get_imu_msg_from_airsim(imu_data)
            imu_msg.header.frame_id = vehicle_name
            publisher.publish(imu_msg)
            previous_imu_data_z = imu_data.linear_acceleration.z_val
            previous_imu_data_y = imu_data.linear_acceleration.y_val
            previous_imu_data_x = imu_data.linear_acceleration.x_val
            rate.sleep()
        except Exception as e:
            rospy.logerr("Error in imu_data_thread: %s", str(e))
            exit_event.set()
        except KeyboardInterrupt:  # 添加这一行来捕获键盘中断
            rospy.loginfo("KeyboardInterrupt received, shutting down...")
            exit_event.set()


def get_imu_msg_from_airsim(imu_data):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
    imu_msg.orientation.x = imu_data.orientation.x_val
    imu_msg.orientation.y = imu_data.orientation.y_val
    imu_msg.orientation.z = imu_data.orientation.z_val
    imu_msg.orientation.w = imu_data.orientation.w_val
    imu_msg.linear_acceleration_covariance[0] = -1
    imu_msg.angular_velocity_covariance[0] = -1
    imu_msg.orientation_covariance[0] = -1
    return imu_msg


def signal_handler(signal, frame):
    global client
    global exit_event
    exit_event.set()
    for thread in imu_threads:
        thread.join()
    if exit_event.is_set():
        client = not client
        rospy.signal_shutdown("Exiting imu_cy.py")


def main():
    global vehicle_name
    rospy.init_node("Imu_publisher_node")
    imu_publisher = rospy.Publisher("imu_topic", Imu, queue_size=250)
    signal.signal(signal.SIGINT, signal_handler)
    sensor_names = ["Imu"]  # 修改为包含传感器名称的列表
    root = "/mnt/"
    file1 = root + path + "/src/airsim_ros_pkgs/scripts/device_name1.txt"
    file2 = root + path + "/src/airsim_ros_pkgs/scripts/device_name2.txt"
    file3 = root + path + "/src/airsim_ros_pkgs/scripts/device_name3.txt"
    with open(file1, "r+", encoding='utf-8') as f1:
        file_content1 = f1.read()
        if re.search(r'\b{}\b'.format(ip), file_content1):
            device_name = file_content1.split('(')[0].strip()
            f1.seek(0)  # 将文件指针移动到文件开头
            f1.truncate()  # 清空文件内容
    f1.close()
    with open(file2, "r+", encoding='utf-8') as f2:
        file_content2 = f2.read()
        if re.search(r'\b{}\b'.format(ip), file_content2):
            device_name = file_content2.split('(')[0].strip()
            f2.seek(0)  # 将文件指针移动到文件开头
            f2.truncate()  # 清空文件内容
    f2.close()
    with open(file3, "r+", encoding='utf-8') as f3:
        file_content3 = f3.read()
        if re.search(r'\b{}\b'.format(ip), file_content3):
            device_name = file_content3.split('(')[0].strip()
            f3.seek(0)  # 将文件指针移动到文件开头
            f3.truncate()  # 清空文件内容
    f3.close()
    vehicle_name = device_name

    for sensor_name in sensor_names:
        class SensorPublisher:
            def __init__(self, publisher, sensor_name):
                self.publisher = publisher
                self.sensor_name = sensor_name

            def publish(self, imu_msg):
                self.publisher.publish(imu_msg)

        imu_publisher_instance = SensorPublisher(imu_publisher, sensor_name)
        imu_thread = threading.Thread(target=imu_data_thread, args=(imu_publisher_instance, sensor_name, vehicle_name))
        imu_thread.start()
        imu_threads.append(imu_thread)




if __name__ == '__main__':
    main()
