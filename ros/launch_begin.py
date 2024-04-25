import json
from time import sleep
import roslaunch
import websocket
import rospy
from websocket import WebSocketApp
import os
import re
import subprocess
# 用于保存启动的进程对象
launch_process = None
kill_launch = None
ip = '192.168.1.51'


def on_message(ws, message):
    global launch_process
    global kill_launch
    print("Received message:", message)
    data = json.loads(message)

    # 根据解析后的字典对象内容判断是启动还是停止 launch 文件

    if 'message' in data:
        # if re.search(r'\b{}\b'.format(ip), data['message']):
        #     if re.search(r'1\b', data['message'].split('(')[0]):
        #         with open("./src/airsim_ros_pkgs/scripts/device_name1.txt", "w", encoding='utf-8') as f1:
        #             f1.write(data['message'])
        #         f1.close()
        #     elif re.search(r'2\b', data['message'].split('(')[0]):
        #         with open("./src/airsim_ros_pkgs/scripts/device_name2.txt", "w", encoding='utf-8') as f2:
        #             f2.write(data['message'])
        #         f2.close()
        #     elif re.search(r'3\b', data['message'].split('(')[0]):
        #         with open("./src/airsim_ros_pkgs/scripts/device_name3.txt", "w", encoding='utf-8') as f3:
        #             f3.write(data['message'])
        #         f3.close()
        if data['message'] == 'ready':
            os.system("rosnode kill -a")

        if data['message'] == 'join':
            os.system("ps aux | grep airsim_node.launch")
            # 杀死这些进程
            if kill_launch is not None:
                os.system("kill -9 " + str(kill_launch.pid))
                kill_launch = None
                print("Launch file has been terminated.")
        if data['message'] == 'begin':
            # 如果已经启动了 launch 文件，则不进行重复启动
            if launch_process is not None:
                print("Launch file is already running.")
                return
            # 启动 launch 文件
            try:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                tracking_launch = roslaunch.parent.ROSLaunchParent(
                    uuid, ["./src/airsim_ros_pkgs/launch/airsim_node.launch"])
                tracking_launch.start()
                launch_process = tracking_launch
                kill_launch = tracking_launch
                sleep(3)
                signal_data = {'message': 'launch_ok'}
                ws.send(json.dumps(signal_data))
            except roslaunch.RLException as e:
                print("ROS launch 异常:", str(e))
            except rospy.ROSInterruptException as e:
                print("ROS 中断异常:", str(e))
            except Exception as e:
                print("其他异常:", str(e))
        if data['message'] == 'end' or data['message'] == 'reset':
            # 如果已经启动了 launch 文件，则终止它
            if launch_process is not None:
                launch_process.shutdown()
                launch_process = None
                print("Launch file has been terminated.")
            else:
                print("No launch file is currently running.")


def on_error(ws, error):
    print(error)


def on_close(ws):
    print("WebSocket connection closed")


def on_open(ws):
    print("WebSocket connection opened")


websocket_url = 'ws://192.168.1.51:8010/ws/send/signal/'

ws = WebSocketApp(websocket_url,
                  on_open=on_open,
                  on_message=on_message,
                  on_error=on_error,
                  on_close=on_close)
ws.run_forever()
