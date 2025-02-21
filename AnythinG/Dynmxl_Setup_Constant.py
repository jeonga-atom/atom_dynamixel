ATOM_DEVICENAME         = '/dev/ttyACM0'    # Port name
ATOM_PROTOCOL_VERSION   = 2.0               # Dynamixel Protocol Version
ATOM_BAUDRATE           = 1000000           # Baudrate

# Dynamixel Address and Length values
ADDR_TORQUE_VALUE       = 64
LEN_TORQUE_VALUE        = 1
TORQUE_DISABLE          = 0
TORQUE_ENABLE           = 1

ADDR_GOAL_POSITION      = 116
LEN_GOAL_POSITION       = 4
ADDR_PRESENT_POSITION   = 132
LEN_PRESENT_POSITION    = 4

ADDR_GOAL_VELOCITY      = 104
LEN_GOAL_VELOCITY       = 4
ADDR_PRESENT_VELOCITY   = 128
LEN_PRESENT_VELOCITY    = 4


# Our Team, ATOM's Manual Constant
Dynmxl_init_Value = False


WHEELBASE               = 0.3               # meter(Length)
WHEELTREAD              = 0.2               # meter(Width)
HALF_TREAD              = WHEELTREAD / 2    # meter(Half width)
FORWARD_POS             = 2048              # unit(value depends on robot)
MAXIMUM_STEERING_ANG    = 45                # degree(steering wheel limit angle)

DIR_FORWARD             = 0
DIR_LEFT                = -1
DIR_RIGHT               = 1
DIR_STOP                = 100

DXL_ID_ALL              = [1, 2, 3, 4, 5, 6] # Dynamixel's ID
DXL_ID_POS              = [5, 6]
DXL_ID_VEL              = [1, 2, 3, 4]