uint8 steps # number of steps which should be executed

float32 step_distance_forward   # [m]
float32 step_distance_sideward  # [m]
float32 turn_angle              # [rad]
float32 dz

bool close_step
bool ignore_robot_model

bool use_terrain_model

bool override
float32 roll
float32 pitch

# modes
uint8 FORWARD                   =  1
uint8 BACKWARD                  =  2
uint8 STRAFE_LEFT               =  3
uint8 STRAFE_RIGHT              =  4
uint8 ROTATE_LEFT               =  5
uint8 ROTATE_RIGHT              =  6
uint8 SAMPLING                  =  7
uint8 STEP_UP                   =  8
uint8 STEP_DOWN                 =  9
uint8 STEP_OVER                 = 10
uint8 FEET_REALIGN              = 11 # realign feet relative to robot pose
uint8 WIDE_STANCE               = 12 # special mode for wide stance
uint8 mode

uint8[] data # contains robot specific data
