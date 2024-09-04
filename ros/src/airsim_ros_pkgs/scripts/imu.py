import threading
import airsim
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

should_publish = True
client = airsim.MultirotorClient(ip="169.254.102.220")
client.confirmConnection()
client.enableApiControl(True)  # 允许使用API控制


def lidar_data_thread(publisher, vehicle_name):
    rate = rospy.Rate(10)  # 设置线程的频率，例如10 Hz
    while should_publish:
        lidar_data = get_lidar_data(publisher.sensor_name, vehicle_name)
        lidar_msg = get_lidar_msg_from_airsim(lidar_data)
        lidar_msg.header.frame_id = vehicle_name
        publisher.publish(lidar_msg)
        rate.sleep()


def get_lidar_data(sensor_name, vehicle_name):
    lidar_data = client.getLidarData(sensor_name, vehicle_name)
    return lidar_data


def get_lidar_msg_from_airsim(lidar_data):
    lidar_msg = PointCloud2()  # 创建一个PointCloud2消息对象
    lidar_msg.header = Header()
    lidar_msg.header.stamp = rospy.Time.now()
    lidar_msg.header.frame_id = 'lidar_frame'
    lidar_msg.height = 1
    lidar_msg.width = len(lidar_data.point_cloud)
    lidar_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    lidar_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    lidar_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    lidar_msg.fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
    lidar_msg.is_bigendian = False
    lidar_msg.point_step = 16
    lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
    pc_data = []
    for point in lidar_data.point_cloud:
        if isinstance(point, tuple) and len(point) == 4:
            pc_data.extend(point)
        else:
            # 添加异常处理逻辑
            continue
    lidar_msg.data = np.array(pc_data, dtype=np.float32).tostring()
    return lidar_msg


def main():
    rospy.init_node("lidar_publisher_node")
    vehicle_name = "drone_1"  # 替换为你的车辆名称
    lidar_publisher = rospy.Publisher("lidar_topic", PointCloud2, queue_size=300)  # 添加雷达发布器
    lidar_sensor_name = "LidarSensor1"

    class LidarPublisher:
        def __init__(self):
            self.publisher = lidar_publisher
            self.sensor_name = lidar_sensor_name

        def publish(self, lidar_msg):
            self.publisher.publish(lidar_msg)

    lidar_publisher_instance = LidarPublisher()  # 创建Lidar发布实例
    lidar_thread = threading.Thread(target=lidar_data_thread, args=(lidar_publisher_instance, vehicle_name))
    lidar_thread.start()
    lidar_thread.join()


if __name__ == '__main__':
    main()
