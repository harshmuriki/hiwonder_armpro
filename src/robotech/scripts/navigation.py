# To move servos: rostopic pub /servo_controllers/port_id_1/multi_id_pos_dur hiwonder_servo_msgs/MultiRawIdPosDur
# "{id_pos_dur_list: [{id: 3, position: 500, duration: 800}]}" -1

# To move the robot: rostopic pub /chassis_control/set_velocity chassis_control/SetVelocity "{velocity: 10.0, direction: 0.0, angular: 0.0}"

# 6 different x, y, z hardcode positions for each of the bins

# Service call to be at the pos

# input base_angle, up-down movement

# move the arm to the position

# Point to the box

# Bring the arm back


rostopic pub /chassis_control/set_velocity chassis_control/SetVelocity "{velocity: 0.0, direction: 0.0, angular: 0.0}"