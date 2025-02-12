# 250211 - 2개는 각도(위치) 제어 / 4개 속도 제어


import os
import math

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():        
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_XM430_OPERATING_MODE       = 11
ADDR_XM430_HOMING_OFFSET        = 20
ADDR_XM430_TORQUE_ENABLE        = 64               # Control table address is different in Dynamixel model
ADDR_XM430_GOAL_VELOCITY        = 104
ADDR_XM430_GOAL_POSITION        = 116
ADDR_XM430_PRESENT_VELOCITY     = 128
ADDR_XM430_PRESENT_POSITION     = 132

# Data Byte Length
LEN_XM430_OPERATING_MODE        = 1
LEN_XM430_HOMING_OFFSET         = 4
LEN_XM430_GOAL_VELOCITY         = 4
LEN_XM430_PRESENT_VELOCITY      = 4
LEN_XM430_GOAL_POSITION         = 4
LEN_XM430_PRESENT_POSITION      = 4

# Protocol version
PROTOCOL_VERSION                = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
THE_NUMBER_OF_DXL               = 6
DXL1_ID                         = 1                 # Dynamixel#1 ID : 1
DXL2_ID                         = 2                 # Dynamixel#1 ID : 2
DXL3_ID                         = 3                 # Dynamixel#1 ID : 3
DXL4_ID                         = 4                 # Dynamixel#1 ID : 4
DXL5_ID                         = 5                 # Dynamixel#1 ID : 5
DXL6_ID                         = 6                 # Dynamixel#1 ID : 6
BAUDRATE                        = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                      = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

OP_MODE_CURRENT                 = 0
OP_MODE_VELOCITY                = 1
OP_MODE_POSITION                = 3
OP_MODE_EXTENDED_POS            = 4
OP_MODE_CUR_BASED_POS           = 5
# OP_MODE_PWM                     = 16

TORQUE_ENABLE                   = 1                 # Value for enabling the torque
TORQUE_DISABLE                  = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 1536
DXL_MINIMUM_POSITION_VALUE      = 2560
DXL_MOVING_STATUS_THRESHOLD     = 5                 # Dynamixel moving status threshold

DXL_DEFAULT_GO_STRAIGHT_POS     = 2048              # 차량 직진 시 다이나믹셀 기본 위치(각도)값

MOBILITY_WHEELBASE              = 200               # 휠베이스(전후간 차축 수평 거리)
MOBILITY_TREAD                  = 150               # 차륜 거리(윤거, 양쪽 차륜 수평 거리)




# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)


# Initialize GroupSyncWrite instance
groupSyncWrite_GoalP = GroupSyncWrite(portHandler, packetHandler, ADDR_XM430_GOAL_POSITION, LEN_XM430_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead_PresP = GroupSyncRead(portHandler, packetHandler, ADDR_XM430_PRESENT_POSITION, LEN_XM430_PRESENT_POSITION)

# Initialize GroupSyncWrite instance for Goal Velocity
groupSyncWrite_GoalV = GroupSyncWrite(portHandler, packetHandler, ADDR_XM430_GOAL_VELOCITY, LEN_XM430_GOAL_VELOCITY)

# Initialize GroupSyncRead instace for Present Velocity
groupSyncRead_PresV = GroupSyncRead(portHandler, packetHandler, ADDR_XM430_PRESENT_VELOCITY, LEN_XM430_PRESENT_VELOCITY)



def set_DXL_Torque(DXL_ID: int | list[int], Torque_value: int):
    # Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_XM430_TORQUE_ENABLE, Torque_value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    return 0



def exchange_value_of_OP_mode(OP_mode_value):
    if type(OP_mode_value) == type(1):
        if OP_mode_value == 0:
            dxl_present_OP_printS = "Current control"
        elif OP_mode_value == 1:
            dxl_present_OP_printS = "Velocity control"
        elif OP_mode_value == 3:
            dxl_present_OP_printS = "Position control"
        elif OP_mode_value == 4:
            dxl_present_OP_printS = "Current-based Position control"
        elif OP_mode_value == 5:
            dxl_present_OP_printS = "Extended Position control"
        elif OP_mode_value == 16:
            dxl_present_OP_printS = "PWM control"
        else:
            print("Wrong(Unknown) OP mode value number.")
            quit()
        return dxl_present_OP_printS

    elif type(OP_mode_value) == type("1"):
        if OP_mode_value == "Current":
            dxl_present_OP_printN = 0
        elif OP_mode_value == "Velocity":
            dxl_present_OP_printN = 1
        elif OP_mode_value == "Position":
            dxl_present_OP_printN = 3
        elif OP_mode_value == "Current-based Position":
            dxl_present_OP_printN = 4
        elif OP_mode_value == "Extended Position":
            dxl_present_OP_printN = 5
        elif OP_mode_value == "PWM":
            dxl_present_OP_printN = 16
        else:
            print("Wrong(Unknown) OP mode value string.")
            quit()
        return dxl_present_OP_printN
    
    else:
        print("Wrong(Unknown) OP mode value.")
        quit()



def print_OP_mode(ID_):

    dxl_present_OP_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID_, ADDR_XM430_OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if dxl_comm_result == COMM_SUCCESS:
        dxl_present_OP_print = exchange_value_of_OP_mode(OP_mode_value= dxl_present_OP_mode)

        print("Present Operating mode of Dynamixel ID %03d: %s" % (ID_, dxl_present_OP_print))
    else:
        print("Present Operating mode of Dynamixel ID %03d: Failed to load" % (ID_))



    return dxl_present_OP_print



def change_OP_mode(ID_c, Mode_set_to = None):
    
    while True:

        # Disable Dynamixel#1 Torque
        set_DXL_Torque(DXL_ID= ID_c, Torque_value= TORQUE_DISABLE)
        '''
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            '''

        
        if not Mode_set_to:
            dxl_present_OP_value = print_OP_mode(ID_= ID_c)

            print("Press a key from the list below to change the operating mode of Dynamixel ID %03d! (or press Enter to pass!)" % ID_c)
            print("C(c): Current\t\tV(v): Velocity\t\tP(p): Position\t\tE(e): Extended Position\t\tB(b): Current-based Position")
            mode_key = getch()

            if mode_key in [chr(0x50), chr(0x70)]: # P, p key
                print("Dynamixel ID %03d will change to Position mode." % ID_c)
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                    quit()
                break

            elif mode_key in [chr(0x56), chr(0x76)]: # V, v key
                print("Dynamixel ID %03d will change to Velocity mode." % ID_c)
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_VELOCITY)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                break

            elif mode_key in [chr(0x42), chr(0x62)]: # B, b key
                print("Dynamixel ID %03d will change to Current-based Position mode." % ID_c)
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_CUR_BASED_POS)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                break

            elif mode_key in [chr(0x45), chr(0x65)]: # E, e key
                print("Dynamixel ID %03d will change to Extended Position mode." % ID_c)
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_EXTENDED_POS)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                break

            elif mode_key in [chr(0x43), chr(0x63)]: # C, c key
                print("Dynamixel ID %03d will change to Current mode." % ID_c)
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_CURRENT)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                break

            elif mode_key == chr(0x0D): # Enter key
                print("Dynamixel ID %03d will keep the OP mode( %s )." % (ID_c, dxl_present_OP_value))
                break

            else: # 나머지 경우
                print("You pressed the wrong key. Please try again.")

        else:
            dxl_present_OP_value = Mode_set_to
            print("Dynamixel ID %03d will be set to %s mode automatically by code(or user)." % (ID_c, dxl_present_OP_value))
            OP_MODE_value = exchange_value_of_OP_mode(OP_mode_value= Mode_set_to)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_c, ADDR_XM430_OPERATING_MODE, OP_MODE_value)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            break

    
    print_OP_mode(ID_= ID_c)
    print("\n")

    return dxl_present_OP_value



def Ackermann_steering_Radius(standard_Angle_in4096: int):
    '''차량의 회전 중심점부터 차량 차체의 중심선까지의 거리를 계산하는 함수. standard_Angle_in4096는 전진 방향을 다이나믹셀 내장값 기준 2048로 보고, 그 값보다 크면 '''
    if not (standard_Angle_in4096 % 1024 == 0):
        MOBILITY_center_Radius = ((200 / math.tan((standard_Angle_in4096 * (360 / 4096)) / (180 / math.pi))) 
                                  + (MOBILITY_TREAD / 2)) # R: 차량 중심선 기준의 회전 반경
    
    else: # 값이 -2048(2048), -1024, 0, 1024, 2048 일 때, if문 첫 줄 계산이 불가함. 그런 경우 본 else문 실행.
        if standard_Angle_in4096 == 0:
            pass


def allocate_value_into_byte_array(Target_value):
    # Allocate value into byte array
    param_value = [DXL_LOBYTE(DXL_LOWORD(Target_value)), DXL_HIBYTE(DXL_LOWORD(Target_value)), DXL_LOBYTE(DXL_HIWORD(Target_value)), DXL_HIBYTE(DXL_HIWORD(Target_value))]

    return param_value



def gBulkWrite(ID_list: list[int], ADDR_list: list[int], addr_LEN_list: list[int], PARAM_list: list):

    # Clear Bulkwrite parameter storage
    groupBulkWrite.clearParam()

    if not(len(ID_list) == len(ADDR_list) and len(ID_list) == len(addr_LEN_list) and len(ID_list) == len(addr_LEN_list) and len(ID_list) == len(PARAM_list)):
        quit(f"Failed to do groupBulkWrite function(different length)\nID list: {len(ID_list):>3}\tAddress list: {len(ADDR_list):>3}\tLength of address list: {len(addr_LEN_list):>3}\tParameter list: {len(PARAM_list):>3}")

    for repeat_index in range(len(ID_list)):
        dxl_addparam_result = groupBulkWrite.addParam(ID_list[repeat_index], ADDR_list[repeat_index], addr_LEN_list[repeat_index], PARAM_list[repeat_index])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % ID_list[repeat_index])
            quit()

    # Bulkwrite
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear Bulkwrite parameter storage
    groupBulkWrite.clearParam()

    return 0


#===== ===== ===== ===== ===== 함수 끝 ===== ===== ===== ===== =====#


list_of_DXL = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID]

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
    Enable_openPort = True
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    Enable_setBaudrate = True
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

print("\n")


execute_control = Enable_openPort and Enable_setBaudrate

if execute_control:

    change_OP_mode(ID_c= DXL1_ID, Mode_set_to= "Velocity")
    change_OP_mode(ID_c= DXL2_ID, Mode_set_to= "Velocity")
    change_OP_mode(ID_c= DXL3_ID, Mode_set_to= "Velocity")
    change_OP_mode(ID_c= DXL4_ID, Mode_set_to= "Velocity")
    change_OP_mode(ID_c= DXL5_ID, Mode_set_to= "Position")
    change_OP_mode(ID_c= DXL6_ID, Mode_set_to= "Position")

    for ID_index in list_of_DXL:
        set_DXL_Torque(DXL_ID= ID_index, Torque_value= TORQUE_ENABLE)
        
    '''
    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        '''


'''
# 이 부분은 모터의 동작을 위해 토크를 켜는 과정임.
# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)
'''

Target_Position = DXL_DEFAULT_GO_STRAIGHT_POS
Target_Velocity = 0
Ackermann_Pos = 0
Ackermann_Vel = 0

print("Press any key to continue! (or press Backspace to quit!)")

while execute_control:

    auto_straight = False

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    controlling_key = getch()

    if controlling_key == chr(0x7F): # Backspace key
        print("\nQuit!\n")
        break


    if controlling_key == chr(0x77): # w key(+Velocity↑)
        Target_Velocity += 4
        if Target_Position > DXL_DEFAULT_GO_STRAIGHT_POS:
            Target_Position -= 4
        elif Target_Position < DXL_DEFAULT_GO_STRAIGHT_POS:
            Target_Position += 4
        else:
            pass

    elif controlling_key == chr(0x57): # W key(Steer wheels to go straight)
        Target_Position = DXL_DEFAULT_GO_STRAIGHT_POS

    elif controlling_key == chr(0x73): # s key(-Velocity↓)
        Target_Velocity -= 4

    elif controlling_key == chr(0x61): # a key(Turn Left)
        Target_Position -= 16

    elif controlling_key == chr(0x64): # d key(Turn Right)
        Target_Position += 16

    elif controlling_key == chr(0x71): # q key(Turn Left & +Velocity)
        Target_Position -= 8
        Target_Velocity += 2

    elif controlling_key == chr(0x65): # e key(Turn Right & +Velocity)
        Target_Position += 8
        Target_Velocity += 2

    elif controlling_key == chr(0x7A): # z key(Turn Left & -Velocity)
        Target_Position -= 8
        Target_Velocity -= 2
        
    elif controlling_key == chr(0x63): # c key(Turn Right & -Velocity)
        Target_Position += 8
        Target_Velocity -= 2

    elif controlling_key == chr(0x58): # X key(Stop smoothly)
        if Target_Velocity < 0:
            Target_Velocity += 2
        elif Target_Velocity > 0:
            Target_Velocity -= 2
        else:
            pass
        
    elif controlling_key == chr(0x78): # x key(EMERGENCY STOP)
        Target_Velocity = 0

    else:
        if Target_Position > DXL_DEFAULT_GO_STRAIGHT_POS:
            Target_Position -= 2
        elif Target_Position < DXL_DEFAULT_GO_STRAIGHT_POS:
            Target_Position += 2
        else:
            pass
        

    print(f"Target Position: {Target_Position:>-5}\tTarget Velocity: {Target_Velocity:>4}", end= "\r")

    bytes_Position = allocate_value_into_byte_array(Target_value= Target_Position)
    bytes_Velocity = allocate_value_into_byte_array(Target_value= Target_Velocity)


    gBulkWrite(ID_list= list_of_DXL,
               ADDR_list= [ADDR_XM430_GOAL_VELOCITY, ADDR_XM430_GOAL_VELOCITY, ADDR_XM430_GOAL_VELOCITY, ADDR_XM430_GOAL_VELOCITY, ADDR_XM430_GOAL_POSITION, ADDR_XM430_GOAL_POSITION],
               addr_LEN_list= [LEN_XM430_GOAL_POSITION, LEN_XM430_GOAL_VELOCITY, LEN_XM430_GOAL_VELOCITY, LEN_XM430_GOAL_VELOCITY, LEN_XM430_GOAL_POSITION, LEN_XM430_GOAL_POSITION],
               PARAM_list= [bytes_Velocity, bytes_Velocity, bytes_Velocity, bytes_Velocity, bytes_Position, bytes_Position])
    
    '''
    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_XM430_GOAL_POSITION, LEN_XM430_GOAL_POSITION, bytes_Position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % DXL1_ID)
        quit()

    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_XM430_GOAL_VELOCITY, LEN_XM430_GOAL_VELOCITY, bytes_Velocity)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % DXL2_ID)
        quit()

    # Bulkwrite goal position
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear Bulkwrite parameter storage
    groupBulkWrite.clearParam()
    '''


for disable_index in list_of_DXL:
    set_DXL_Torque(DXL_ID= disable_index, Torque_value= TORQUE_DISABLE)

'''
# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_XM430_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
    '''

print("All Dynamixels are disabled.")
