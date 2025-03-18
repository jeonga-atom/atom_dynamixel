ATOM_DEVICENAME          = '/dev/ttyACM0'    # Port name
ATOM_PROTOCOL_VERSION    = 2.0               # Dynamixel Protocol Version
ATOM_BAUDRATE            = 1000000           # Baudrate

# Dynamixel Address and Length values
ADDR_TORQUE_VALUE        = 64
LEN_TORQUE_VALUE         = 1
TORQUE_DISABLE           = 0
TORQUE_ENABLE            = 1

OP_VALUE_POSITION        = 3
ADDR_GOAL_POSITION       = 116
LEN_GOAL_POSITION        = 4
ADDR_PRESENT_POSITION    = 132
LEN_PRESENT_POSITION     = 4

OP_VALUE_VELOCITY        = 1
ADDR_GOAL_VELOCITY       = 104
LEN_GOAL_VELOCITY        = 4
ADDR_PRESENT_VELOCITY    = 128
LEN_PRESENT_VELOCITY     = 4


# Our Team, ATOM's Manual Constant
DYNMXL_ENABLED_STATE     = False

DXL_POS_THRESHOLD        = 2
DXL_VEL_LIMIT            = 190

LIMIT_DISTANCE           = 0.4

WHEELBASE                = 0.3                   # meter(Length)
WHEELTREAD               = 0.2                   # meter(Width)
HALF_TREAD               = WHEELTREAD / 2        # meter(Half width)
LIST_WHEELBASE           = [WHEELBASE]
LIST_WHEELTREAD          = [WHEELTREAD]
LIST_HALF_TREAD          = [HALF_TREAD]

INIT_DXL_POS             = 2048
FORWARD_POS              = 2048                  # unit(value depends on robot)
MAXIMUM_STEERING_ANG     = 45                    # degree(steering wheel limit angle)
MAXIMUM_GIMBAL_ANG       = 45

DIR_FORWARD              = 0                     # Predefine car's direction
DIR_LEFT                 = -1
DIR_RIGHT                = 1
DIR_STOP                 = 100                   # Value doesn't matter, but set it 100 as you can.

# Dynamixel's ID
DXL_ID_ALL               = [1, 2, 3, 4, 5, 6, 7, 8]
DXL_ID_POS               = [5, 6, 7, 8]
DXL_ID_VEL               = [1, 2, 3, 4]

DXL_INIT_SETUP           = [[0x01, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_VELOCITY], [12, 1, 101], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, 0], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x02, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000001], [11, 1, OP_VALUE_VELOCITY], [12, 1, 101], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, 0], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x03, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_VELOCITY], [12, 1, 101], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, 0], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x04, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000001], [11, 1, OP_VALUE_VELOCITY], [12, 1, 101], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, 0], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x05, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_POSITION], [12, 1, 105], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_POSITION, LEN_GOAL_POSITION, FORWARD_POS], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x06, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_POSITION], [12, 1, 105], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_POSITION, LEN_GOAL_POSITION, FORWARD_POS], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]], 

                            [0x07, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_POSITION], [12, 1, 107], 
                                 [108, 4, 60], [112, 4, 350], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_POSITION, LEN_GOAL_POSITION, FORWARD_POS], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]],

                            [0x08, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
                                 [8, 1, 0x03], [9, 1, 250], [10, 1, 0b00000000], [11, 1, OP_VALUE_POSITION], [12, 1, 107], 
                                 [108, 4, 100], [112, 4, 350], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_ENABLE], [ADDR_GOAL_POSITION, LEN_GOAL_POSITION, FORWARD_POS], 
                                 [ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 0]]]]
#'''