#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import traceback

class ImageStreamClient:
    def __init__(self):
        self.bridge = CvBridge()
        # Replace with the actual ROS2 machine IP or use '127.0.0.1' if on the same machine.
        self.server_ip = '10.220.86.145'
        self.server_port = 6000
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.loginfo("Image stream client started.")

    def image_callback(self, msg):
        try:
            print("in image callback")
            # Convert ROS Image message to an OpenCV image.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Encode the image as JPEG.
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if not ret:
                rospy.logerr("Failed to encode image")
                return
            jpeg_bytes = jpeg.tobytes()
            length = len(jpeg_bytes)
            rospy.loginfo("Encoding image, size: {} bytes".format(length))

            # Create and use a socket without the 'with' statement.
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                rospy.loginfo("Connecting to {}:{}".format(self.server_ip, self.server_port))
                s.connect((self.server_ip, self.server_port))
                # Send a 4-byte length prefix (big-endian) followed by the JPEG image data.
                s.sendall(struct.pack('!I', length))
                s.sendall(jpeg_bytes)
                rospy.loginfo("Sent image of length {} bytes".format(length))
            except Exception as e:
                rospy.logerr("Socket error: {}".format(e))
            finally:
                s.close()
        except Exception:
            rospy.logerr("Error in image callback: %s" % traceback.format_exc())

if __name__ == '__main__':
    rospy.init_node('image_stream_client')
    client = ImageStreamClient()
    rospy.spin()
