#!/usr/bin/env python3
import rospy
import cv2
import time
import threading
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False

        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self):
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.opened = True
        except Exception as e:
            rospy.logerr("Failed to open camera: %s", e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            rospy.logerr("Failed to close camera: %s", e)

    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        self.frame = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                    else:
                        self.frame = None
                        self.cap.release()
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    self.cap.release()
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap
                else:
                    time.sleep(0.01)
            except Exception as e:
                rospy.logerr("Error capturing frame: %s", e)
                time.sleep(0.01)

class CameraROSNode:
    def __init__(self):
        rospy.init_node("camera_node", anonymous=True)
        self.camera = Camera(resolution=(640, 480))
        self.camera.camera_open()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.capture_sub = rospy.Subscriber("/camera/capture", Empty, self.capture_callback)
        self.save_dir = rospy.get_param("~save_dir", "captured_images")
        os.makedirs(self.save_dir, exist_ok=True)
        self.img_counter = 0

    def capture_callback(self, msg):
        if self.camera.frame is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}_{self.img_counter}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, self.camera.frame)
            rospy.loginfo("Captured image saved to: %s", filepath)
            self.img_counter += 1
        else:
            rospy.logwarn("No frame available for capture!")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz publish rate
        while not rospy.is_shutdown():
            if self.camera.frame is not None:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(self.camera.frame, "bgr8")
                    self.publisher.publish(img_msg)
                except Exception as e:
                    rospy.logerr("Error converting frame: %s", e)
            rate.sleep()

    def shutdown(self):
        self.camera.camera_close()

if __name__ == '__main__':
    node = CameraROSNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()

