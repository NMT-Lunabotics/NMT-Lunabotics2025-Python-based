int8 blocking_id    # Takes an id and stop all commands expect those produced by the blocking id, -1 resets the blocker for command type
string command      # The type of command being sent
int32[] data        # The command or data that is sent whcih the robot runs
string text         # The string to send to the system



# M: [float velocity (-1, 1), float angular_velocity (-1, 1)] 
# A: [arm_act_max_pos, arm_act_min_pos, bucket_act_max_pos, bucket_act_min_pos, arm_velocity, bucket_velocity]