uint8 UPDATE_MODE_Z                 =  1    # updates only z position of step
uint8 UPDATE_MODE_3D                =  2    # updates z, roll and pitch of step
uint8 UPDATE_MODE_MOVE_TO_VALID     =  4    # moves pose to avoid collision
uint8 UPDATE_MODE_COST              =  8    # recomputes cost
uint8 UPDATE_MODE_CHECK_VALIDITY    = 16    # check if step is reachable from predecessor (only UpdateStepPlan)
uint8 UPDATE_MODE_CHECK_COLLISION   = 32    # check if step is colliding
uint8 UPDATE_MODE_REPLAN            = 64    # replans entire plan

uint8 mode # modes are used as bit flags and may be combined freely
