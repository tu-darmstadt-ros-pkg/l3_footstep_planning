std_msgs/Header header
l3_msgs/FootStepData[] visited_steps
l3_msgs/BaseStepData[] visited_bases
l3_msgs/Foothold[] last_visited_state
l3_msgs/Step last_visited_step
l3_msgs/StepQueue current_step_plan
bool forward_search                   # indicates search direction
