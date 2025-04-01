#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def main():
    rospy.init_node('box_id_input_node')
    pub = rospy.Publisher('/user_input', Int32, queue_size=10)

    rospy.loginfo("Node started. Enter integer box IDs in the terminal, or 'q' to quit.")

    while not rospy.is_shutdown():
        # Python 2 uses raw_input()
        user_input = raw_input("Enter a box ID (or 'q' to quit): ")

        if user_input.lower() == 'q':
            rospy.loginfo("Exiting node.")
            break

        # Attempt to parse an integer
        try:
            box_id = int(user_input)
            msg = Int32()
            msg.data = box_id
            pub.publish(msg)
            rospy.loginfo("Published box_id: %d" % box_id)
        except ValueError:
            rospy.logwarn("Invalid input. Please enter an integer or 'q' to quit.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# #!/usr/bin/env python

# import rospy
# import tkinter as tk
# from std_msgs.msg import Int32, Bool

# class ButtonGridGUI(object):
#     def __init__(self):
#         # Create the main window
#         self.root = tk.Tk()
#         self.root.title("Button Grid GUI")

#         # Publisher: Integer messages for clicked buttons
#         self.pub_pressed = rospy.Publisher('/button_pressed', Int32, queue_size=10)

#         # Subscriber: Watches a bool topic to enable/disable all buttons
#         rospy.Subscriber('/enable_buttons', Bool, self.enable_buttons_callback)

#         # Create a list to store references to all buttons
#         self.buttons = []

#         # 
#         rows = 5
#         cols = 13
#         for r in range(rows):
#             for c in range(cols):
#                 # Assign an integer index for each button (0..64)
#                 index = r * cols + c
#                 btn_text = f"Btn {index}"  # or any string you prefer
#                 btn = tk.Button(
#                     self.root,
#                     text=btn_text,
#                     command=lambda idx=index: self.on_button_click(idx)
#                 )
#                 btn.grid(row=r, column=c)
#                 self.buttons.append(btn)

#     def on_button_click(self, index):
#         """ Callback for when a button is clicked. """
#         rospy.loginfo(f"Button {index} clicked.")
#         # Publish the integer for the pressed button
#         self.pub_pressed.publish(index)

#         # Disable all buttons immediately after a click
#         self.disable_all_buttons()

#     def enable_buttons_callback(self, msg):
#         """
#         Callback for bool topic.
#         If True  enable buttons, otherwise disable.
#         """
#         if msg.data:
#             self.enable_all_buttons()
#         else:
#             self.disable_all_buttons()

#     def enable_all_buttons(self):
#         for button in self.buttons:
#             button.config(state=tk.NORMAL)

#     def disable_all_buttons(self):
#         for button in self.buttons:
#             button.config(state=tk.DISABLED)

#     def main_loop(self):
#         """
#         Because Tkinter has its own event loop and rospy uses spin,
#         we can't just call rospy.spin(). Instead, we periodically
#         check if ROS is still running.
#         """
#         self.periodic_check()
#         self.root.mainloop()

#     def periodic_check(self):
#         # If ROS is not shutting down, schedule another check
#         if not rospy.is_shutdown():
#             # Check again after 100 ms
#             self.root.after(100, self.periodic_check)
#         else:
#             # If ROS is shutting down, close the Tk window
#             self.root.quit()

# def main():
#     rospy.init_node('button_grid_gui', anonymous=True)
#     gui = ButtonGridGUI()
#     gui.main_loop()

# if __name__ == '__main__':
#     main()
    