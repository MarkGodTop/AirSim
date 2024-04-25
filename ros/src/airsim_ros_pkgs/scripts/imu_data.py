import rospy
from numpy import double
from sensor_msgs.msg import Imu
import threading
import airsim
import numpy as np
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import QuaternionStamped

client = airsim.MultirotorClient(ip="192.168.1.51")
client.confirmConnection()
client.enableApiControl(True)
previous_imu_data = None


def imu_data_thread(publisher, sensor_name, vehicle_name):
    global previous_imu_data
    rate = rospy.Rate(210)
    # 创建卡尔曼滤波器对象
    kf = KalmanFilter(dim_x=7, dim_z=7)
    kf.x = np.array([0., 0., 0., 0., 0., 0., 0.])  # 初始状态向量：[x, y, z]
    kf.P *= 1000.  # 初始状态协方差矩阵
    kf.R = 1  # 观测噪声协方差
    kf.Q = np.array([[0., 0., 0., 0., 0., 0., 0.],
                     [0., 0., 0., 0., 0., 0., 0.],
                     [0., 0., 1., 0., 0., 0., 0.],
                     [0., 0., 0., 1., 0., 0., 0.],
                     [0., 0., 0., 0., 1., 0., 0.],
                     [0., 0., 0., 0., 0., 1., 0.],
                     [0., 0., 0., 0., 0., 0., 1.]])  # 过程噪声协方差矩阵
    kf.H = np.array([[1., 0., 0., 0., 0., 0., 0.],
                     [0., 1., 0., 0., 0., 0., 0.],
                     [0., 0., 1., 0., 0., 0., 0.],
                     [0., 0., 0., 1., 0., 0., 0.],
                     [0., 0., 0., 0., 1., 0., 0.],
                     [0., 0., 0., 0., 0., 1., 0.],
                     [0., 0., 0., 0., 0., 0., 1.]])  # 观测矩阵
    kf_gyro = KalmanFilter(dim_x=3, dim_z=3)
    kf_gyro.x = np.array([0., 0., 0.])  # 初始状态向量：[x, y, z]
    kf_gyro.P *= 1000.  # 初始状态协方差矩阵
    kf_gyro.R = 1  # 观测噪声协方差
    kf_gyro.Q = np.array([[0., 0., 0.],
                          [0., 0., 0.],
                          [0., 0., 1.]])  # 过程噪声协方差矩阵
    kf_gyro.H = np.array([[1., 0., 0.],
                          [0., 1., 0.],
                          [0., 0., 1.]])  # 观测矩阵
    while not rospy.is_shutdown():
        imu_data = get_imu_data(sensor_name, vehicle_name)
        x = imu_data.linear_acceleration.x_val  # 当前加速度观测值（x方向）
        y = imu_data.linear_acceleration.y_val  # 当前加速度观测值（y方向）
        z = imu_data.linear_acceleration.z_val  # 当前加速度观测值（z方向）
        x_gyro = imu_data.angular_velocity.x_val  # 当前角速度观测值（x方向）
        y_gyro = imu_data.angular_velocity.y_val  # 当前角速度观测值（y方向）
        z_gyro = imu_data.angular_velocity.z_val  # 当前角速度观测值（z方向）
        quat = imu_data.orientation  # 当前四元数观测值
        kf.predict()
        kf.update(np.array([[x], [y], [z], [quat.x_val], [quat.y_val], [quat.z_val], [quat.w_val]]))  # 更新状态向量和状态协方差矩阵
        filtered_x = kf.x[0]  # 滤波后的x加速度值
        filtered_y = kf.x[1]  # 滤波后的y加速度值
        filtered_z = kf.x[2]  # 滤波后的z加速度值
        filtered_quat = QuaternionStamped()
        filtered_quat.quaternion.x = kf.x[3]  # 滤波后的x角度值
        filtered_quat.quaternion.y = kf.x[4]  # 滤波后的y角度值
        filtered_quat.quaternion.z = kf.x[5]  # 滤波后的z角度值
        filtered_quat.quaternion.z = kf.x[6]  # 滤波后的w角度值
        kf_gyro.predict()
        kf_gyro.update(np.array([[x_gyro], [y_gyro], [z_gyro]]))  # 更新角速度状态向量和状态协方差矩阵
        filtered_x_gyro = kf_gyro.x[0]  # 滤波后的x角速度值
        filtered_y_gyro = kf_gyro.x[1]  # 滤波后的y角速度值
        filtered_z_gyro = kf_gyro.x[2]  # 滤波后的z角速度值
        if filtered_z == imu_data.linear_acceleration.z_val:
            continue
        imu_msg = get_imu_msg_from_airsim(imu_data)
        imu_msg.linear_acceleration.x = float(filtered_x)
        imu_msg.linear_acceleration.y = float(filtered_y)
        imu_msg.linear_acceleration.z = float(filtered_z)
        imu_msg.angular_velocity.x = float(filtered_x_gyro)
        imu_msg.angular_velocity.y = float(filtered_y_gyro)
        imu_msg.angular_velocity.z = float(filtered_z_gyro)
        imu_msg.orientation.x = float(filtered_quat.quaternion.x)
        imu_msg.orientation.y = float(filtered_quat.quaternion.y)
        imu_msg.orientation.z = float(filtered_quat.quaternion.z)
        imu_msg.orientation.w = float(filtered_quat.quaternion.w)
        imu_msg.header.frame_id = vehicle_name
        publisher.publish(imu_msg)
        previous_imu_data = filtered_z
        rate.sleep()


def get_imu_data(sensor_name, vehicle_name):
    imu_data = client.getImuData(sensor_name, vehicle_name)
    return imu_data


def get_imu_msg_from_airsim(imu_data):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.linear_acceleration.x = double(imu_data.linear_acceleration.x_val)
    imu_msg.linear_acceleration.y = double(imu_data.linear_acceleration.y_val)
    imu_msg.linear_acceleration.z = double(imu_data.linear_acceleration.z_val)
    imu_msg.angular_velocity.x = double(imu_data.angular_velocity.x_val)
    imu_msg.angular_velocity.y = double(imu_data.angular_velocity.y_val)
    imu_msg.angular_velocity.z = double(imu_data.angular_velocity.z_val)
    imu_msg.orientation.x = double(imu_data.orientation.x_val)
    imu_msg.orientation.y = double(imu_data.orientation.y_val)
    imu_msg.orientation.z = double(imu_data.orientation.z_val)
    imu_msg.orientation.w = double(imu_data.orientation.w_val)
    imu_msg.linear_acceleration_covariance[0] = -1
    imu_msg.angular_velocity_covariance[0] = -1
    imu_msg.orientation_covariance[0] = -1
    return imu_msg


def main():
    rospy.init_node("imu_publisher_node")
    imu_publisher = rospy.Publisher("imu_topic", Imu, queue_size=250)
    sensor_names = ["Imu"]  # 修改为包含传感器名称的列表
    vehicle_name = "car_1"
    imu_threads = []
    for sensor_name in sensor_names:
        class SensorPublisher:
            def __init__(self, publisher, sensor_name):
                self.publisher = publisher
                self.sensor_name = sensor_name

            def publish(self, imu_msg):
                self.publisher.publish(imu_msg)

        imu_publisher_instance = SensorPublisher(imu_publisher, sensor_name)
        imu_thread = threading.Thread(target=imu_data_thread,
                                      args=(imu_publisher_instance, sensor_name, vehicle_name))
        imu_thread.start()
        imu_threads.append(imu_thread)
    for thread in imu_threads:
        thread.join()


if __name__ == '__main__':
    main()
