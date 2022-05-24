std_msgs/Header header
geometry_msgs/Pose pose         # Target center pose between both (parallel) feet on the ground. Transformation between pelvis and ground can be triggered with FLAG_ROBOT_FRAME.

uint8 FLAG_CURRENT      =  1    # returns current feet pose (ground truth); this will make the generator to ignore all other flags and given pose
uint8 FLAG_CURRENT_Z    =  2    # uses z position of current feet pose (ground truth)
uint8 FLAG_PELVIS_FRAME =  4    # given pose is in pelvis frame which will be transformed to a ground level
uint8 FLAG_3D           =  8    # generates full 3D pose (6 DoF) based on terrain
uint8 flags                     # As default (flags = 0) the generator returns 2D pose by projection
