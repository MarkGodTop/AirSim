import json
import os
import roslaunch
import xml.etree.ElementTree as ET
import signal


json_file_path = './settings.json'
launch_file_path = './src/airsim_ros_pkgs/launch/airsim_node.launch'

def signal_handler(sig, frame):
    print("Caught SIGINT, shutting down roslaunch and exiting.")
    tracking_launch.shutdown()
    exit(0)
# Read and parse the JSON file
with open(json_file_path, 'r') as json_file:
    json_content = json.load(json_file)

# Read the launch file 
with open(launch_file_path, 'r') as launch_file:
    launch_content = launch_file.read()

# 从字符串解析XML内容
root = ET.fromstring(launch_content)

# 初始化变量
lidar_update_rate = None
img_update_rate = None
# 遍历launch文件中的所有param元素
for param in root.findall('.//param'):
    # 检查param的name属性是否是'update_lidar_every_n_sec'
    if param.get('name') == 'update_lidar_every_n_sec':
        # 获取对应的value属性
        lidar_update_rate = param.get('value')
    if param.get('name') == 'update_airsim_img_response_every_n_sec':
        img_update_rate = param.get('value')
modified_launch_content = launch_content
if 'LidarSensor1' in json_content['Vehicles']['drone_1']['Sensors']:
    search_str_lidar = f'name="update_lidar_every_n_sec" type="double" value="{lidar_update_rate}"'
    replace_str_lidar = f'name="update_lidar_every_n_sec" type="double" value="0.1"'
    modified_launch_content = modified_launch_content.replace(search_str_lidar, replace_str_lidar)
else:
    search_str_lidar = f'name="update_lidar_every_n_sec" type="double" value="{lidar_update_rate}"'
    replace_str_lidar = f'name="update_lidar_every_n_sec" type="double" value="0"'
    modified_launch_content = modified_launch_content.replace(search_str_lidar, replace_str_lidar)
if 'Cameras' in json_content['Vehicles']['drone_1']:
    search_str_img = f'name="update_airsim_img_response_every_n_sec" type="double" value="{img_update_rate}"'
    replace_str_img = f'name="update_airsim_img_response_every_n_sec" type="double" value="0.05"'
    modified_launch_content = modified_launch_content.replace(search_str_img, replace_str_img)
else:
    search_str_img = f'name="update_airsim_img_response_every_n_sec" type="double" value="{img_update_rate}"'
    replace_str_img = f'name="update_airsim_img_response_every_n_sec" type="double" value="0"'
    modified_launch_content = modified_launch_content.replace(search_str_img, replace_str_img)

with open(launch_file_path, 'w') as modified_launch_file:
    modified_launch_file.write(modified_launch_content)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
                uuid, [launch_file_path])
tracking_launch.start()
signal.signal(signal.SIGINT, signal_handler)
signal.pause()