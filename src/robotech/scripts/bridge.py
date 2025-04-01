#!/usr/bin/env python

import rospy
import socket
import struct
from std_msgs.msg import Int32, Float32MultiArray

class Ros1TcpClient(object):
    """
    A simple class-based ROS1 TCP client that connects to a ROS2 server,
    sends an integer, and awaits two floats in response.
    """

    def __init__(self, host="10.220.86.145", port=5000):
        """
        Initialize the client with default or user-specified host and port.
        Create the subscriber and publisher.
        """
        self.host = host
        self.port = port
        self.box_id = None

        # Subscriber: listens for an Int32 on /box_needed
        self.box_needed_sub = rospy.Subscriber('/box_detection_needed', Int32, self.box_needed_callback)

        # Publisher: publishes a 2-element float array to /box_needed_reply
        self.angle_publisher = rospy.Publisher('/box_detection_reply', Float32MultiArray, queue_size=10)

    def box_needed_callback(self, msg):
        """Callback triggered when a new Int32 is published to /box_needed."""
        self.box_id = msg.data
        rospy.loginfo("Received box_id: {}".format(self.box_id))
        self.run()

    def run(self):
        """
        Establish a TCP connection to the ROS2 server, send self.box_id,
        receive two floats, and publish them on /box_needed_reply.
        """
        try:
            # Create a TCP socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            rospy.loginfo("Connecting to server at {}:{}".format(self.host, self.port))
            s.connect((self.host, self.port))

            # Send an integer value (4 bytes, signed)
            s.sendall(struct.pack('i', self.box_id))
            rospy.loginfo("Sent int value: {}".format(self.box_id))

            # Wait for the response of two floats (4 bytes each, total 8 bytes)
            data = s.recv(8)
            if len(data) < 8:
                rospy.logwarn("Received incomplete data from the server.")
            else:
                # Unpack the two floats
                float1, float2 = struct.unpack('ff', data)
                rospy.loginfo("Received float values: {}, {}".format(float1, float2))

                # Publish the float values as a Float32MultiArray
                float_array = Float32MultiArray()
                float_array.data = [float1, float2]
                self.angle_publisher.publish(float_array)

            s.close()

        except Exception as e:
            rospy.logerr("Error in client run(): {}".format(e))

def main():
    rospy.init_node('dummy_ros1_client', anonymous=True)
    client = Ros1TcpClient(host="10.220.86.145", port=5000)

    # Keep the node alive so the subscriber callback can be triggered
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python
# import rospy
# import socket
# import struct

# def ros1_client():
#     rospy.init_node('dummy_ros1_client', anonymous=True)
#     host = "10.220.86.145"  # IP/hostname of the ROS2 server machine
#     port = 5000            # Must match the port used by the ROS2 server

#     try:
#         # Create a TCP/IP socket
#         s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         rospy.loginfo("Connecting to server at {}:{}".format(host, port))
#         s.connect((host, port))

#         # Send an integer value (4 bytes, signed)
#         integer_value = 42
#         s.sendall(struct.pack('i', integer_value))
#         rospy.loginfo("Sent int value: {}".format(integer_value))

#         # Wait for the response of two floats (4 bytes each, total 8 bytes)
#         data = s.recv(8)
#         if len(data) < 8:
#             rospy.logwarn("Received incomplete data from the server.")
#         else:
#             # Unpack the two floats
#             float1, float2 = struct.unpack('ff', data)
#             rospy.loginfo("Received float values: {}, {}".format(float1, float2))

#         s.close()
#     except Exception as e:
#         rospy.logerr("Error in client: {}".format(e))

# if __name__ == '__main__':
#     try:
#         ros1_client()
#     except rospy.ROSInterruptException:
#         pass



# #!/usr/bin/env python
# import rospy
# import socket
# import struct
# def ros1_client():
#     rospy.init_node('dummy_ros1_client', anonymous=True)
#     host = "10.220.86.145"  # Adjust this if the ROS2 server is on a different machine.
#     port = 5000         # Must match the port used by the ROS2 server.
#     try:
#         # Create a TCP/IP socket.
#         s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         rospy.loginfo("Connecting to server at {}:{}".format(host, port))
#         s.connect((host, port))
#         # Send a boolean value.
#         # Here, True is sent as a single byte (value 1).
#         bool_value = True
#         s.sendall(struct.pack('B', 1 if bool_value else 0))
#         rospy.loginfo("Sent bool value: {}".format(bool_value))
#         # Wait for the response of two floats (each float is 4 bytes, total 8 bytes).
#         data = s.recv(8)
#         if len(data) < 8:
#             rospy.logwarn("Received incomplete data from the server.")
#         else:
#             # Unpack the two floats.
#             float1, float2 = struct.unpack('ff', data)
#             rospy.loginfo("Received float values: {}, {}".format(float1, float2))
#         s.close()
#     except Exception as e:
#         rospy.logerr("Error in client: {}".format(e))
# if __name__ == '__main__':
#     try:
#         ros1_client()
#     except rospy.ROSInterruptException:
#         pass 

# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import socket
# import struct
# class ImageStreamClient:
#     def __init__(self):
#         self.bridge = CvBridge()
#         # Set the IP address of the ROS2 machine.
#         self.server_ip = "10.220.86.145"  # Change this to the actual IP of your ROS2 node
#         self.server_port = 6000             # Must match the port used by the ROS2 node
#         rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
#         rospy.loginfo("Image stream client started.")
#     def image_callback(self, msg):
#         try:
#             # Convert the ROS Image message to a CV2 image.
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#             # Encode the image as JPEG.
#             ret, jpeg = cv2.imencode('.jpg', cv_image)
#             if not ret:
#                 rospy.logerr("Failed to encode image")
#                 return
#             jpeg_bytes = jpeg.tobytes()
#             # Create a TCP socket and send the data.
#             with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#                 s.connect((self.server_ip, self.server_port))
#                 # First, send a 4-byte length prefix (big-endian).
#                 length = len(jpeg_bytes)
#                 s.sendall(struct.pack('!I', length))
#                 # Now send the JPEG image bytes.
#                 s.sendall(jpeg_bytes)
#                 rospy.loginfo("Sent image of length %d bytes" % length)
#         except Exception as e:
#             rospy.logerr("Error in image callback: %s" % e)
# if __name__ == '__main__':
#     rospy.init_node('image_stream_client')
#     client = ImageStreamClient()
#     rospy.spin()

# #!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import socket
# import struct
# import threading

# class ImageStreamClient:
#     def __init__(self):
#         # Initialize ROS node and subscriber
#         rospy.init_node('image_stream_client', anonymous=True)
#         self.bridge = CvBridge()
#         self.server_ip = "10.220.86.145"  # Change this to your server's IP address
#         self.server_port = 6000          # Must match the server's port
#         self.sock = None                 # TCP socket for persistent connection
#         self.connection_lock = threading.Lock()  # Lock to manage socket access

#         # Subscribe to the image topic
#         rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
#         rospy.loginfo("Image stream client started.")
        
#         # Attempt initial connection to the server
#         self.connect_to_server()

#     def connect_to_server(self):
#         """Establish a TCP connection to the server."""
#         with self.connection_lock:
#             if self.sock:
#                 self.sock.close()
#             try:
#                 self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                 self.sock.connect((self.server_ip, self.server_port))
#                 rospy.loginfo("Connected to server at %s:%d", self.server_ip, self.server_port)
#             except socket.error as e:
#                 rospy.logerr("Failed to connect to server: %s", e)
#                 self.sock = None

#     def image_callback(self, msg):
#         """Callback function for the image topic."""
#         with self.connection_lock:
#             if not self.sock:  # Reconnect if no active socket
#                 rospy.logwarn("No active connection. Attempting to reconnect...")
#                 self.connect_to_server()
#                 if not self.sock:
#                     return  # Exit if reconnection fails

#             try:
#                 # Convert ROS Image message to OpenCV image
#                 cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                
#                 # Encode the image as JPEG
#                 ret, jpeg = cv2.imencode('.jpg', cv_image)
#                 if not ret:
#                     rospy.logerr("Failed to encode image")
#                     return
                
#                 jpeg_bytes = jpeg.tobytes()
                
#                 # Send the length of the JPEG data followed by the data itself
#                 length = len(jpeg_bytes)
#                 self.sock.sendall(struct.pack('!I', length))  # Send length as 4-byte integer (big-endian)
#                 self.sock.sendall(jpeg_bytes)  # Send JPEG data
                
#                 rospy.loginfo("Sent image of length %d bytes", length)
#             except socket.error as e:
#                 rospy.logerr("Socket error: %s", e)
#                 if self.sock:
#                     self.sock.close()
#                     self.sock = None  # Mark socket as closed for reconnection
#             except Exception as e:
#                 rospy.logerr("Error in image callback: %s", e)

# if __name__ == '__main__':
#     try:
#         client = ImageStreamClient()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
