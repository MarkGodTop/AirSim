import rospy
from sensor_msgs.msg import Imu


def imu_callback(msg):
    global unique_data, duplicate_data
    imu_data = (
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z
    )
    if imu_data in unique_data:
        duplicate_data.append(msg)
    else:
        unique_data.add(imu_data)


rospy.init_node('imu_duplicate_checker')
duplicate_data = []
unique_data = set()
rospy.Subscriber('/airsim_node/car_1/imu/Imu', Imu, imu_callback)
rospy.spin()
if duplicate_data:
    rospy.loginfo("重复的IMU数据：")
    for imu_msg in duplicate_data:
        rospy.loginfo(imu_msg)
else:
    rospy.loginfo("没有重复的IMU数据")
