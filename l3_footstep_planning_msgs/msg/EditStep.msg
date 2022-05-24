l3_msgs/Step step # modified/new/deleted step

# this will ignore the "lock" of step
uint8 EDIT_MODE_REMOVE     = 1
uint8 EDIT_MODE_2D         = 2  # planner will update z, roll and pitch
uint8 EDIT_MODE_3D         = 3  # planner will update roll and pitch
uint8 EDIT_MODE_FULL       = 4  # planner will update anything
uint8 plan_mode
