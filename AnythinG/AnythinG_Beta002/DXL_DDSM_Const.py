# AnythinG_Beta002.py

ATOM_DXL_DEVICENAME         = '/dev/ttyACM0'    # Port name
ATOM_DXL_PROTOCOL_VER       = 2.0               # Dynamixel Protocol Version
ATOM_DXL_BAUDRATE           = 1000000           # Baudrate

ATOM_DDSM_USB_PORT_LIST     = [1, 2, 3, 4]      # list of the number of x in 'ttyACM(x)' (port number)

# Dynamixel Address and Length values
ADDR_TORQUE_VALUE           = 64
LEN_TORQUE_VALUE            = 1
TORQUE_DISABLE              = 0
TORQUE_ENABLE               = 1

OP_VALUE_POSITION           = 3
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4

OP_VALUE_VELOCITY           = 1
ADDR_GOAL_VELOCITY          = 104
LEN_GOAL_VELOCITY           = 4
ADDR_PRESENT_VELOCITY       = 128
LEN_PRESENT_VELOCITY        = 4


# DDSM values ##### From DDSM_ctrl_class_wDXL.py
# 모터 모드값 문자화(상수) 정의
DDSM_MODE_CURRENT           = 0x01 # 전류 모드
DDSM_MODE_VELOCITY          = 0x02 # 속도 모드
DDSM_MODE_POSITION          = 0x03 # 위치(각도) 모드

# 모터 설정값 범위 상수 정의
DDSM_CURRENT_MAX_VALUE      = 8 # 8암페어
DDSM_VELOCITY_MAX_VALUE     = 330 # 330RPM
DDSM_POSITION_MAX_VALUE     = 360 # 360도
DDSM_ENCODER_RESOLUTION     = 32767 # 모터가 송수신하는 데이터 분해능의 역수(2의 16제곱, 음수 범위 포함)


# Our Team, ATOM's Manual Constant
DXL_POS_THRESHOLD           = 2
DDSM_MAX_VEL_LIMIT          = 100

LIMIT_DISTANCE              = 0.4

WHEELBASE                   = 0.450                     # meter(Length)
WHEELTREAD                  = 0.347                     # meter(Width)
HALF_TREAD                  = WHEELTREAD / 2        # meter(Half width)
LIST_WHEELBASE              = [WHEELBASE]
LIST_WHEELTREAD             = [WHEELTREAD]
LIST_HALF_TREAD             = [HALF_TREAD]

INIT_DXL_POS                = 2048
FORWARD_POS                 = 2048                  # unit(value depends on robot)
MAXIMUM_STEERING_ANG        = 45                    # degree(steering wheel limit angle)
MAXIMUM_GIMBAL_ANG          = 45

DIR_FORWARD                 = 0                     # Predefine car's direction
DIR_LEFT                    = -1
DIR_RIGHT                   = 1
DIR_STOP                    = 100                   # Value doesn't matter, but set it 100 as you can.

# Dynamixel's ID
# DXL_ID_ALL                  = [5, 6, 7, 8]
DXL_ID_POS                  = [5, 6, 7, 8]          # DXLs' ID
DDSM_ID_VEL                 = [1, 2, 3, 4]          # DDSMs' ID(Not port number)

DDSM_INIT_SETUP             = [[1, ]]

DXL_INIT_SETUP              = [[0x05, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
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
'''

                               [0x01, [[ADDR_TORQUE_VALUE, LEN_TORQUE_VALUE, TORQUE_DISABLE], [65, 1, 1], 
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

                               

#'''