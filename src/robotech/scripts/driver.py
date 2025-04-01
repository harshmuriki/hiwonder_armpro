#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Bool, Float32MultiArray
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from database import Database

class DriverNode(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('driver_node', anonymous=True)

        self.db = Database()

        # Subscribers
        # self.sub_string = rospy.Subscriber('/user_input', String, self.string_callback)
        self.sub_int = rospy.Subscriber('/user_input', Int32, self.full_sequence_callback)

        # Publisher
        # self.pub_string = rospy.Publisher('/output_string', String, queue_size=10)

        # Create a service client for std_srvs/SetBool on service name '/example_service'
        # # You can replace 'SetBool' with any custom or other standard service type.
        # service_name = '/example_service'
        # rospy.loginfo("Waiting for service '{}'...".format(service_name))
        # rospy.wait_for_service(service_name)
        # self.set_bool_client = rospy.ServiceProxy(service_name, SetBool)
        # rospy.loginfo("Service '{}' is now available.".format(service_name))
        self.box_detection_needed_pub = rospy.Publisher('/box_detection_needed', Int32, queue_size=10)
        self.box_det_reply_sub  = rospy.Subscriber('/box_detection_reply', Float32MultiArray, self.box_detection_reply_callback)

        self.navigate_pub = rospy.Publisher('/compartment_id', Int32, queue_size=10)
        self.navigate_sub = rospy.Subscriber('/compartment_reached', Bool, self.navigation_status_callback)

        self.angle_pub = rospy.Publisher('/servo_angles', Float32MultiArray, queue_size=10)
        # self.angle_reached = rospy.Subscriber('/angle_reached', Bool, self.angle_reached_callback)

        # Example of calling the service once at startup
        # self.call_set_bool_service(True)

    def navigation_status_callback(self, msg):
        self.navigation_done = True

    
    def box_detection_reply_callback(self, msg):
        """ Handle incoming box detection reply. """
        rospy.loginfo("Received box detection reply: {}".format(msg.data))
        # Example action: republish on /output_string
        self.box_detection_reply_horz_angle = msg.data[0]
        self.box_detection_reply_vert_angle = msg.data[1]
        
        self.box_reply_got = True

    def string_callback(self, msg):
        """ Handle incoming string data from /input_string. """
        rospy.loginfo("Received String: {}".format(msg.data))
        # Example action: republish on /output_string
        self.pub_string.publish("Echo: " + msg.data)

    def full_sequence_callback(self, msg):
        """ Handle incoming integer data from /input_int. """
        self.wait_more = False
        rospy.loginfo("box_id: {}".format(msg.data))
        self.box_id = int(msg.data)
        # Call the database function to get the bin number
        bin_no = self.db.box_id_to_bin_no(self.box_id)
        if bin_no == -1:
            rospy.logerr("Box ID {} not found in database.".format(self.box_id))
            return
        rospy.loginfo("Navigating to bin: {}".format(bin_no))
        # call navigation function here
        self.navigate_to_bin(bin_no)
        # call the bin_no service here
        horz_angle,vert_angle = self.box_detection(bin_no)
        if horz_angle==0.0 and vert_angle==0.0:
            rospy.loginfo("Box not detected")
            self.send_angle(-1010, bin_no+1+0.1)
            self.wait_more = True
            # self.navigate_to_bin(bin_no+1)
            horz_angle,vert_angle = self.box_detection(self.box_id)
        
        if horz_angle==0.0 and vert_angle==0.0:
            rospy.loginfo("Box not detected")
            # done
            return
        
        # send angle. 
        self.send_angle(horz_angle, vert_angle)

    def navigate_to_bin(self, bin_no):
        self.navigation_done = False
        self.navigate_pub.publish(bin_no)
        while not self.navigation_done:
            rospy.loginfo("Waiting for navigation to bin {}...".format(bin_no))
            rospy.sleep(0.1)
        rospy.loginfo("Navigation to bin {} completed.".format(bin_no))
    

            
    def send_angle(self, horz_angle, vert_angle):
        angle_msg = Float32MultiArray()
        angle_msg.data = [horz_angle, vert_angle]
        self.angle_pub.publish(angle_msg)
        rospy.loginfo("Sent angles: Horizontal: {}, Vertical: {}".format(horz_angle, vert_angle))
        # Wait for the angles to be reached
        rospy.sleep(5)
        if self.wait_more:
            self.wait_more = False
            rospy.sleep(5)

    def box_detection(self, bin_no):
        self.box_reply_got = False
        self.box_detection_needed_pub.publish(bin_no)
        while not self.box_reply_got:
            rospy.loginfo("Waiting for box detection reply...")
            rospy.sleep(0.1)
        rospy.loginfo("Box detection completed. Horizontal angle: {}, Vertical angle: {}".format(
            self.box_detection_reply_horz_angle, self.box_detection_reply_vert_angle
        ))
        return (self.box_detection_reply_horz_angle, self.box_detection_reply_vert_angle)

    # def navigate_to_bin(self, bin_no):
    #     rospy.logwarn("navigate_to_bin not implemented")

    # def call_set_bool_service(self, flag_value):
    #     """ Example method to call the /example_service with a bool. """
    #     try:
    #         request = SetBoolRequest()
    #         request.data = flag_value
    #         rospy.loginfo("Calling /example_service with data = {}".format(flag_value))
    #         response = self.set_bool_client(request)
    #         rospy.loginfo("Service response: success={} message='{}'".format(
    #             response.success, response.message
    #         ))
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: {}".format(e))

    def spin(self):
        """ Keep python from exiting until this node is stopped. """
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DriverNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
