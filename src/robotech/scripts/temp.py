#!/usr/bin/python3
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from chassis_control.msg import SetVelocity

class IntegratedController:
    def __init__(self):
        # Initialize single node for both controllers
        rospy.init_node('integrated_controller', log_level=rospy.INFO)
        
        # Initialize robot movement publisher
        self.robot_pub = rospy.Publisher('/chassis_control/set_velocity', 
                                       SetVelocity, queue_size=1)
        
        # Initialize servo controller publisher
        self.servo_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur',
                                       MultiRawIdPosDur, queue_size=1)

    def move_robot(self, velocity, direction, angular):
        """Control robot chassis movement"""
        msg = SetVelocity()
        msg.velocity = float(velocity)
        msg.direction = float(direction)
        msg.angular = float(angular)
        self.robot_pub.publish(msg)
        rospy.loginfo(f"Robot moving: v={velocity}, dir={direction}°, ang={angular}rad/s")
        rospy.sleep(1)  # Ensure message transmission

    def set_servo_position(self, data=[], level=None):
        """Control servo position and movement speed"""
        # Data: servo_id, position, duration_ms
        if level == "DOWN":
            data = [
                (1, 90, 2000),  # Example servo data
                (2, 500, 2000),
                (3, 500, 2000),
                (4, 504, 2000),
                (5, 150, 2000),
                (6, 500, 2000)
            ]
        elif level == "UP":
            data = [
                (1, 90, 2000),  # Example servo data
                (2, 500, 2000),
                (3, 115, 2000),
                (4, 498, 2000),
                (5, 506, 2000),
                (6, 500, 2000)
            ]
        elif level == "RESET":
            data = [
                (1, 0, 2000),  # Example servo data
                (2, 0, 2000),
                (3, 0, 2000),
                (4, 0, 2000),
                (5, 0, 2000),
                (6, 0, 2000)
            ]
            
        for servo_id, position, duration_ms in data:
            if 0 <= position <= 1000 and 20 <= duration_ms <= 30000:
                msg = MultiRawIdPosDur()
                msg.id_pos_dur_list = [RawIdPosDur(
                    id=servo_id,
                    position=int(position),
                    duration=int(duration_ms)
                )]
                self.servo_pub.publish(msg)
                rospy.loginfo(f"Servo {servo_id} moving to {position} in {duration_ms}ms")
                rospy.sleep(0.1)  # Ensure message transmission
            else:
                rospy.logerr("Invalid servo params! Position (0-1000), Duration (20-30000)ms")

if __name__ == '__main__':
    controller = IntegratedController()
    
    # Wait for publishers to register
    rospy.sleep(0.5)
    
    # Compartment 1
    # Front: s=50; 90, t=5
    # 1.1 s=50, t=9
    # 1.2 s=50, t=3.5
    
    # Compartment 2
    # 1.2 - 2.1 s=50, t=9
    # 2.1 s=50, t=9
    # 2.2 s=50, t=2.5
    
    # Compartment 3
    # 2.2 - 3.1 s=50, t=9
    # 3.1 s=50, t=8.5
    # 3.2 s=50, t=2.5
    
    try:
        id = 11
        # Example sequence: Move robot and adjust servo simultaneously
        # controller.move_robot(50.0, 90.0, 0.0)  # Move forward
        # controller.move_robot(50.0, 0.0, 0.0)
        # # controller.set_servo_position(level="UP")  # Adjust servo
        
        # rospy.sleep(0.5)  # Maintain movement for 2 seconds
        
        # # controller.set_servo_position(level="RESET")  # Adjust servo down
        # # Stop robot and return servo to initial position
        # controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        # # controller.set_servo_position(level="BELOW")  # Return servo
        
        # rospy.loginfo("Operation completed")
        
        # From a topic, get the compartment number from 1-12
        # If id is 3/7/11
        # Im at home rn
        if id <= 4:
            # Go to the 1st compartment
            # Move forward
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            controller.move_robot(50.0, 0.0, 0.0)
            rospy.sleep(7.5)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            if id == 3:
                # Go to the lower compartment
                controller.set_servo_position(level="DOWN")
                controller.move_robot(50.0, 90.0, 0.0)
                rospy.sleep(3.0)
            else:
                # Go to the upper compartment
                controller.set_servo_position(level="UP")
                controller.move_robot(50.0, 90.0, 0.0)  # Move forward
                rospy.sleep(5.0)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            print("Done with compartment 1")
            
            # Send the images and "yes" to a rostopic
            
            # Go to the next img?????
        # Compartment 1
        # Front: s=50; 90, t=5
        # 1.1 s=50, t=9
        # 1.2 s=50, t=3.5
        
        # Compartment 2
        # 1.2 - 2.1 s=50, t=9
        # 2.1 s=50, t=9
        # 2.2 s=50, t=2.5
        elif 5 <= id <= 8:
            # Go to the 2nd compartment
            # Move forward
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            controller.move_robot(50.0, 0.0, 0.0) #1st ckpt
            rospy.sleep(20)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            
            if id == 7:
                # Go to the lower compartment
                controller.set_servo_position(level="DOWN")
                controller.move_robot(50.0, 90.0, 0.0)
                rospy.sleep(3.0)
            else:
                # Go to the upper compartment
                controller.set_servo_position(level="UP")
                controller.move_robot(50.0, 90.0, 0.0)  # Move forward
                rospy.sleep(5.0)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            print("Done with compartment 2")
            # At the 1st img
            # Send the images and "yes" to a rostopic
            
            
        # Compartment 3
        # 2.2 - 3.1 s=50, t=9
        # 3.1 s=50, t=8.5
        # 3.2 s=50, t=2.5
            
        elif 9 <= id <= 12:
            # Go to the 3rd compartment
            
            # Move forward
            controller.move_robot(0.0, 0.0, 0.0)
            controller.move_robot(50.0, 0.0, 0.0)
            rospy.sleep(32)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            
            if id == 11:
                # Go to the lower compartment
                controller.set_servo_position(level="DOWN")
                controller.move_robot(50.0, 90.0, 0.0)
                rospy.sleep(3.0)
            else:
                # Go to the upper compartment
                controller.set_servo_position(level="UP")
                controller.move_robot(50.0, 90.0, 0.0)
                rospy.sleep(5.0)
            controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
            print("Done with compartment 3")
            
            # At the 1st img
            # Send the images and "yes" to a rostopic

    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.signal_shutdown("Mission complete")
