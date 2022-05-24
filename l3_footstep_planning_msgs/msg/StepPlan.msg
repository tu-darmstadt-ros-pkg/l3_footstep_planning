std_msgs/Header header

std_msgs/String parameter_set_name  # name of parameter set to be used, keep empty to use default parameter set

l3_msgs/Foothold[] start  # start configuration used by planner
l3_msgs/Foothold[] goal   # goal configuration used by planner (must not be equal to reached goal)

l3_msgs/StepQueue plan     # resulting step plan
