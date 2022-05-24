std_msgs/Header header

l3_msgs/Foothold[] start_footholds   # if empty, the planner will determine by robot pose
l3_msgs/Foothold[] goal_footholds    # used by 2D/3D planning mode

l3_msgs/FloatingBase[] start_floating_bases         # floating start base data
l3_msgs/FloatingBase[] goal_floating_bases          # floating goal base data

uint32 AUTO_START_FOOT_IDX  = 2147483647
uint32 start_foot_idx # foot index of first leg to move
int32 start_step_idx  # index (offset) of first step in plan

# used by pattern generator
l3_footstep_planning_msgs/PatternParameters pattern_parameters

uint8 PLANNING_MODE_2D      = 0
uint8 PLANNING_MODE_3D      = 1
uint8 PLANNING_MODE_PATTERN = 2
uint8 planning_mode

float32 initial_eps                 # initial eps to use, set 0 for default

float32 max_planning_time           # maximum planning time given in second, set 0 for default value

std_msgs/String parameter_set_name  # name of parameter set to be used, keep empty to use default parameter set

uint8[] data # contains robot specific data
