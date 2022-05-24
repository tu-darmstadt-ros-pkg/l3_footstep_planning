# Service messages for pattern generator
bool enable                 # enables pattern generator
bool simulation_mode        # lets the pattern generator ignore robot feedback and continuously generate steps
bool joystick_mode          # enables support for joystick input

# parameters for pattern generator
geometry_msgs/Twist cmd
float32 foot_seperation

std_msgs/String parameter_set_name # name of parameter set to be used, keep empty to use default parameter set

uint8[] data # contains robot specific data
