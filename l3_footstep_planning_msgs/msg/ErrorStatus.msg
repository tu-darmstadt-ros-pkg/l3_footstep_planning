uint32 NO_ERROR                     =    0
uint32 ERR_UNKNOWN                  =    1
uint32 ERR_NO_SOLUTION              =    2
uint32 ERR_INVALID_START            =    4
uint32 ERR_INVALID_GOAL             =    8
uint32 ERR_INVALID_GRID_MAP         =   16
uint32 ERR_INVALID_TERRAIN_MODEL    =   32
uint32 ERR_INVALID_STEP             =   64
uint32 ERR_INCONSISTENT_STEP_PLAN   =  128
uint32 ERR_INVALID_PARAMETERS       =  256
uint32 ERR_NO_PLUGIN_AVAILABLE      =  512
uint32 ERR_INCONSISTENT_REQUEST     = 1024

uint32 error # binary encoded
string error_msg

uint32 NO_WARNING                   =    0
uint32 WARN_UNKNOWN                 =    1
uint32 WARN_INCOMPLETE_STEP_PLAN    =    2
uint32 WARN_INVALID_STEP_PLAN       =    4
uint32 WARN_NO_TERRAIN_DATA         =    8

uint32 warning # binary encoded
string warning_msg
