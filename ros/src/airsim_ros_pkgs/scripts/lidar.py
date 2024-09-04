import rospy
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, TimeSynchronizer


def callback(point_cloud):
    # 计算点云数量
    num_points = point_cloud.height * point_cloud.width
    rospy.loginfo(f"Number of points in point cloud: {num_points}")


rospy.init_node('point_cloud_subscriber')

# 创建ROS消息订阅器
point_cloud_sub = Subscriber('/airsim_node/car_2/lidar/LidarSensor1', PointCloud2)

# 创建时间同步器
ts = TimeSynchronizer([point_cloud_sub], 10)
ts.registerCallback(callback)
rospy.spin()
