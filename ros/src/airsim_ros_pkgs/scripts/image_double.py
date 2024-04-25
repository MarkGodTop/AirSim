import rospy
import cv2
import threading
import airsim
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np


class CameraCapture:
    def __init__(self, camera_name):
        self.camera_name = camera_name
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher("/airsim_node/drone_1/front_center_custom/Scene/image_raw", Image,
                                               queue_size=10)
        self.client = airsim.MultirotorClient(ip="192.168.1.46")
        self.client.confirmConnection()
        self.lock = threading.Lock()
        self.last_capture_time = time.time()
        self.frame_count = 0
        self.thread1 = 0
        self.thread2 = 0

    def capture_image(self):
        while not rospy.is_shutdown():
            with self.lock:
                response = self.client.simGetImage(self.camera_name, airsim.ImageType.Scene)
            if response is not None:
                img_data = response
                cv_image = np.frombuffer(img_data, np.uint8)
                cv_image = cv2.imdecode(cv_image, cv2.IMREAD_COLOR)
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
                self.image_publisher.publish(ros_image)
                self.frame_count += 1
                if threading.current_thread().name == 'Thread-4':  # 判断当前线程
                    self.thread1 += 1
                elif threading.current_thread().name == 'Thread-5':
                    self.thread2 += 1
                current_time = time.time()
                elapsed_time = current_time - self.last_capture_time
                if elapsed_time > 1.0:  # Calculate frame rate every 1 second
                    frame_rate = self.frame_count / elapsed_time
                    frame_rate_thread1 = self.thread1 / elapsed_time
                    frame_rate_thread2 = self.thread2 / elapsed_time
                    rospy.loginfo("Camera {}: Total Frame rate: {:.2f} fps".format(self.camera_name, frame_rate))
                    rospy.loginfo(
                        "Camera {}: Thread 1 Frame rate: {:.2f} fps".format(self.camera_name, frame_rate_thread1))
                    rospy.loginfo(
                        "Camera {}: Thread 2 Frame rate: {:.2f} fps".format(self.camera_name, frame_rate_thread2))
                    self.last_capture_time = current_time
                    self.frame_count = 0
                    self.thread1 = 0
                    self.thread2 = 0


def main():
    rospy.init_node('airsim_camera_capture')
    camera_name = "front_center_custom"
    capture_obj = CameraCapture(camera_name)

    thread1 = threading.Thread(target=capture_obj.capture_image)
    thread2 = threading.Thread(target=capture_obj.capture_image)

    thread1.start()
    thread2.start()

    rospy.spin()

    thread1.join()
    thread2.join()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
