#!/usr/bin/python3
import rospy
import numpy as np
import threading
from std_msgs.msg import Bool, Int32, Float32MultiArray
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from chassis_control.msg import SetVelocity

class IntegratedController:
    def __init__(self):
        rospy.init_node('integrated_controller', log_level=rospy.INFO)
        
        # Initialize communication components
        self.robot_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.status_pub = rospy.Publisher('/compartment_reached', Bool, queue_size=1)
        
        # Subscribers
        self.id_sub = rospy.Subscriber('/compartment_id', Int32, self.id_callback)
        self.angles_sub = rospy.Subscriber('/servo_angles', Float32MultiArray, self.angles_callback)
        
        # State management
        self.current_id = None
        self.servo_config = np.zeros(6)  # Stores angles for 6 servos
        self.servo_lock = threading.Lock()
        self.operation_active = False
        
        # Initialize servo positions
        self.horizontal_angle = 500  # Default horizontal servo position
        self.vertical_angle = 500    # Default vertical servo position

    def id_callback(self, msg):
        """Handle compartment ID updates"""
        if 1 <= msg.data <= 12:
            self.current_id = msg.data
            rospy.loginfo(f"Received new compartment ID: {self.current_id}")
        else:
            rospy.logwarn(f"Ignoring invalid compartment ID: {msg.data}")

    def angles_callback(self, msg):
        """
        Callback function for receiving horizontal and vertical angles.
        :param msg: Float32MultiArray with two elements [horizontal_angle, vertical_angle]
        """
        if len(msg.data) == 2:
            
            if msg.data[0] < -1000:
                rospy.logwarn("Going to the next half of the compartment cause the box wasn't found in the first half")
                self.current_id = int(msg.data[1])
                print(f"Going to the next half of the compartment cause the box wasn't found in the first half: {self.current_id}")
                self.horizontal_angle = -1
                self.process_compartment()
                
            else:
                rospy.loginfo(f"Received angles: Horizontal={msg.data[0]}, Vertical={msg.data[1]}")
                # Extract horizontal and vertical angles from the message
                self.horizontal_angle = int(max(0, min(1000, msg.data[0])))  # Clamp between 0 and 1000
                self.vertical_angle = int(max(0, min(1000, msg.data[1])))    # Clamp between 0 and 1000
                
                # Move servos to the new positions
                self.move_servos(self.horizontal_angle, self.vertical_angle)
        else:
            rospy.logerr("Invalid angle data received. Expected two values (horizontal and vertical).")

    def move_servos(self, horizontal, vertical):
        """
        Publish commands to move the horizontal and vertical servos.
        :param horizontal: Target position for the horizontal servo (0-1000).
        :param vertical: Target position for the vertical servo (0-1000).
        """
        msg = MultiRawIdPosDur()
        
        # Add commands for both servos
        msg.id_pos_dur_list.append(RawIdPosDur(id=6, position=horizontal, duration=500))  # Horizontal servo (ID 1)
        msg.id_pos_dur_list.append(RawIdPosDur(id=4, position=vertical, duration=500))    # Vertical servo (ID 2)
        
        # Publish the message
        self.servo_pub.publish(msg)
        rospy.loginfo(f"Moved servos: Horizontal={horizontal}, Vertical={vertical}")

    def publish_status(self, success):
        """Publish operation status"""
        status = Bool()
        status.data = success
        self.status_pub.publish(status)
        rospy.loginfo("Published status: %s", success)

    def move_robot(self, velocity, direction, angular, duration=0.1):
        """Execute robot movement with optional timed duration"""
        try:
            msg = SetVelocity()
            msg.velocity = float(velocity)
            msg.direction = float(direction)
            msg.angular = float(angular)
            
            self.robot_pub.publish(msg)
            rospy.loginfo(f"Robot moving: v={velocity}, dir={direction}°, ang={angular}rad/s")
            rospy.sleep(duration)
                
        except Exception as e:
            rospy.logerr(f"Movement error: {str(e)}")
            self.stop_robot()

    def stop_robot(self):
        """Immediately stop robot movement"""
        self.robot_pub.publish(SetVelocity())
        rospy.loginfo("Robot stopped")

    def update_servos(self, duration_ms=1000):
        """Update all servos to current configuration"""
        with self.servo_lock:
            msg = MultiRawIdPosDur()
            msg.id_pos_dur_list = [
                RawIdPosDur(
                    id=i+1,
                    position=int(self.servo_config[i]),
                    duration=int(duration_ms)
                ) for i in range(6)
            ]
            self.servo_pub.publish(msg)
            rospy.loginfo("Servos updated to: %s", self.servo_config)
            rospy.sleep(0.2)  # Allow for message transmission

    def process_compartment(self):
        """Main processing pipeline for compartments"""
        try:
            if self.current_id is None:
                return

            rospy.loginfo(f"Starting processing for compartment {self.current_id}")
            self.operation_active = True
            
            if self.horizontal_angle == -1:
                # Common initial movement
                self.move_robot(50.0, 0.0, 0.0, 2.5)
                self.horizontal_angle = None
            else:
                # Compartment-specific handling
                if self.current_id <= 4:
                    self.handle_compartment1()
                elif 5 <= self.current_id <= 8:
                    self.handle_compartment2()
                elif 9 <= self.current_id <= 12:
                    self.handle_compartment3()

            # Final status update
            self.publish_status(True)
            self.operation_active = False

        except Exception as e:
            rospy.logerr(f"Compartment processing failed: {str(e)}")
            self.publish_status(False)
        finally:
            self.current_id = None

    def handle_compartment1(self):
        """Processing logic for first set of compartments"""
        # Preset positions
        preset = "DOWN" if self.current_id == 3 else "UP"
        with self.servo_lock:
            self.servo_config = np.array([
                [90, 500, 500, 504, 150, 500],  # DOWN
                [90, 500, 115, 498, 506, 500]   # UP
            ][preset == "UP"])
        
        self.update_servos()
        # self.move_robot(50.0, 90.0, 0.0, 5.0 if preset == "UP" else 3.0)
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        controller.move_robot(50.0, 0.0, 0.0, 3.5) # going right
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        if self.current_id == 3 or self.current_id == 1:
            if self.current_id  == 1:
                controller.move_robot(50.0, 90.0, 0.0, 6.5)  # Move forward
            # Go to the lower compartment
            # controller.move_robot(50.0, 90.0, 0.0, 0.0)
            else:
                controller.move_robot(50.0, 90.0, 0.0, 3)  # Move forward
        # else:
            # # Go to the upper compartment
            # controller.move_robot(50.0, 0.0, 0.0, 2.5)
            # controller.move_robot(50.0, 90.0, 0.0, 5.0)  # Move forward
        controller.move_robot(0.0, 0.0, 0.0, 0.0)  # Stop robot

    def handle_compartment2(self):
        """Processing logic for second set of compartments"""
        preset = "DOWN" if self.current_id == 7 else "UP"
        with self.servo_lock:
            self.servo_config = np.array([
                [90, 500, 500, 504, 150, 500],  # DOWN
                [90, 500, 115, 498, 506, 500]   # UP
            ][preset == "UP"])
        
        self.update_servos()
        # self.move_robot(50.0, 90.0, 0.0, 5.0 if preset == "UP" else 3.0)
        # Move forward
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        controller.move_robot(70.0, 0.0, 0, 22.5) #1st ckpt
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        
        if self.current_id  == 7 or self.current_id  == 5:
            # Go to the lower compartment
            # controller.move_robot(50.0, 90.0, 0.0, 0.0
            if self.current_id  == 5:
                controller.move_robot(50.0, 90.0, 0.0, 5.0)  # Move forward
            else:
                controller.move_robot(50.0, 90.0, 0.0, 1.5)  # Move forward
        else:
            # Go to the upper compartment
            controller.move_robot(50.0, 0.0, 0.0, 2.5)
            controller.move_robot(50.0, 90.0, 0.0, 5.0)  # Move forward
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot

    def handle_compartment3(self):
        """Processing logic for third set of compartments"""
        preset = "DOWN" if self.current_id == 11 else "UP"
        with self.servo_lock:
            self.servo_config = np.array([
                [90, 500, 500, 504, 150, 500],  # DOWN
                [90, 500, 115, 498, 506, 500]   # UP
            ][preset == "UP"])
        
        self.update_servos()
        # self.move_robot(50.0, 90.0, 0.0, 5.0 if preset == "UP" else 3.0)
        
        # Move forward
        controller.move_robot(0.0, 0.0, 0.0)
        controller.move_robot(70.0, 0.0, 0.0, 40)
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot
        
        if self.current_id  == 11 or self.current_id  == 9:
            # Go to the lower compartment
            # controller.move_robot(50.0, 90.0, 0.0, 0.0)
            if self.current_id  == 9:
                controller.move_robot(50.0, 90.0, 0.0, 5.0)  # Move forward
            else:
                controller.move_robot(50.0, 90.0, 0.0, 1.5)  # Move forward
        else:
            # Go to the upper compartment
            controller.move_robot(50.0, 0.0, 0.0, 2.5)
            controller.move_robot(50.0, 90.0, 0.0, 5.0)
        controller.move_robot(0.0, 0.0, 0.0)  # Stop robot

    def execute_sequence(self):
        """Main execution loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.current_id is not None and not self.operation_active:
                self.process_compartment()
            rate.sleep()
            rospy.loginfo("Waiting for compartment ID...")

if __name__ == '__main__':
    controller = IntegratedController()
    rospy.loginfo("Integrated controller initialized")
    
    try:
        controller.execute_sequence()
        # controller.move_robot(50.0, 0.0, 0.0, 10) 
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C detected, shutting down...")
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.stop_robot()
        rospy.loginfo("System shutdown complete")

