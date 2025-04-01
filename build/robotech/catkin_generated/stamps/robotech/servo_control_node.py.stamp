#!/usr/bin/python3
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

class ServoController:
    def __init__(self):
        rospy.init_node('servo_control', log_level=rospy.DEBUG)
        self.pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', 
                                 MultiRawIdPosDur, queue_size=1)
        
    def set_servo_position(self, servo_id, position, duration_ms):
        """
        Set servo position with safety checks
        :param servo_id: Servo ID (1-253)
        :param position: Target position (0-1000 for most HiWonder servos)
        :param duration_ms: Movement duration in milliseconds (20-30000)
        """
        msg = MultiRawIdPosDur()
        
        # Create servo command with input validation
        if 0 <= position <= 1000 and 20 <= duration_ms <= 30000:
            msg.id_pos_dur_list = [RawIdPosDur(
                id=servo_id,
                pos=int(position),
                duration=int(duration_ms)
            )]
            self.pub.publish(msg)
            rospy.loginfo(f"Moving servo {servo_id} to {position} in {duration_ms}ms")
        else:
            rospy.logerr("Invalid parameters! Position (0-1000), Duration (20-30000)ms")

if __name__ == '__main__':
    controller = ServoController()
    
    # Wait for publisher registration
    rospy.sleep(1)
    
    # Example: Move servo ID 3 to position 500 over 800ms
    try:
        controller.set_servo_position(3, 500, 800)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
