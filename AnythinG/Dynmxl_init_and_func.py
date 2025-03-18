'''
본 파일은 기존의 Dynmxl_initializing.py 파일과 Dynmxl_Ackermann.py 파일, 그리고 Dynmxl_running.py 파일을 합친 것임.
'''

import math
from dynamixel_sdk import *

import Dynmxl_Setup_Constant as dxlconst


# 다이나믹셀 변수 초기화

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(dxlconst.ATOM_DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(dxlconst.ATOM_PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance for Goal Position
groupSyncWrite_POS = GroupSyncWrite(portHandler, packetHandler, dxlconst.ADDR_GOAL_POSITION, dxlconst.LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead_POS = GroupSyncRead(portHandler, packetHandler, dxlconst.ADDR_PRESENT_POSITION, dxlconst.LEN_PRESENT_POSITION)

# Initialize GroupSyncWrite instance for Goal Velocity
groupSyncWrite_VEL = GroupSyncWrite(portHandler, packetHandler, dxlconst.ADDR_GOAL_VELOCITY, dxlconst.LEN_GOAL_VELOCITY)

# Initialize GroupSyncRead instace for Present Velocity
groupSyncRead_VEL = GroupSyncRead(portHandler, packetHandler, dxlconst.ADDR_PRESENT_VELOCITY, dxlconst.LEN_PRESENT_VELOCITY)

#'''


##### ##### ##### ##### ##### 이하는 클래스 및 함수 정의 영역 ##### ##### ##### ##### #####

'''
class Dynmxl_CMD():
    # 모든 인스턴스 공통 사항
    # 내용 없음

    # 생성자 메소드 선언
    def __init__(self, motor_ID: int, motor_Mode: int, Wheel_Coordinates: list, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = HALF_TREAD):

        self.cMotor_ID = motor_ID
        self.cMotor_Mode = motor_Mode
        self.cENabled_state = TORQUE_DISABLE

        self.cWheel_Coordinates = Wheel_Coordinates
        self.cVert_L_toWheel_fromTurningPoint = Vertical_Length_to_Wheel_from_TurningPoint
        self.cHori_W_toWheel_fromCenterLine = Horizontal_Width_to_Wheel_from_CenterLine

        if motor_Mode == OP_VALUE_VELOCITY:
            self._Goal_Velocity = 0
            self._Present_Velocity = 0
        elif motor_Mode == OP_VALUE_POSITION:
            self._Goal_Position = FORWARD_POS
            self._Present_Position = FORWARD_POS
        else:
            quit(f"[Error in __init__ of class 'Dynmxl_CMD']\nDynamixel (ID: {self.cMotor_ID:02d}) received unexpected operating mode. Received value: {self.cMotor_Mode}\nThis code use only two numbers(modes). > 1: Velocity / 3: Position")
'''

##### 다이나믹셀 기본 기능 명령들 함수 #####


def Dynmxl_Torque(Motor_ID: int, ENabled_or_DISabled: int):
    '''ENabled_or_DISabled: 0(Disabled) or 1(Enabled)'''
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Motor_ID, dxlconst.ADDR_TORQUE_VALUE, ENabled_or_DISabled)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        if ENabled_or_DISabled:
            print(f"Dynamixel (ID: {Motor_ID:02d}) has been successfully connected(Enabled torque).")
        else:
            print(f"Dynamixel (ID: {Motor_ID:02d}) has been successfully disconnected(Disabled torque).")


def Dynmxl_writing(ID_: int, Addr_: int, Len_: int, Param_: int):
    if type(ID_) == type(Addr_) == type(Len_) == type(Param_) == int:
        if Len_ == 4:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID_, Addr_, Param_)
        elif Len_ == 1:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_, Addr_, Param_)
        elif Len_ == 2:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID_, Addr_, Param_)
        else:
            quit(f"[Error in Len_ of Dynmxl_writing] This code use only three types of Len_: 1, 2, 4\nLen_ value: {Len_}")

    else:
        quit(f"[Error in variables' types of Dynmxl_writing]\nTheir values and types:\nID value: {ID_} / its type: {type(ID_)}\nAddress value: {Addr_} / its type: {type(Addr_)}\nLength value: {Len_} / its type: {type(Len_)}\nParameter value: {Param_} / its type: {type(Param_)}")

    return dxl_comm_result, dxl_error

def Dynmxl_repeat_writing(ID_: list, Addr_: list, Len_: list, Param_: list):
    if len(ID_) == len(Addr_) == len(Len_) == len(Param_):
        for repeat_index in len(ID_):
            Dynmxl_writing(ID_=ID_[repeat_index], Addr_=Addr_[repeat_index], Len_=Len_[repeat_index], Param_=Param_[repeat_index])
            '''
            if Len_[repeat_index] == 4:
                packetHandler.write4ByteTxRx(portHandler, ID_, Addr_, Param_)
            elif Len_[repeat_index] == 1:
                packetHandler.write1ByteTxRx(portHandler, ID_, Addr_, Param_)
            elif Len_[repeat_index] == 2:
                packetHandler.write2ByteTxRx(portHandler, ID_, Addr_, Param_)
            else:
                quit(f"[Error in Len_ of Dynmxl_repeat_writing] This code use only three types of Len_: 1, 2, 4\nLen_ value: {Len_}")
            '''
    
    else:
        quit(f"[Error in variables' length of Dynmxl_repeat_writing]\nTheir values and types\nID value: {ID_} / its length: {len(ID_)}\nAddress value: {Addr_} / its length: {len(Addr_)}\nLength value: {Len_} / its length: {len(Len_)}\nParameter value: {Param_} / its length: {len(Param_)}")


def Dynmxl_reading(ID_: int, Addr_: int, Len_: int):
    if type(ID_) == type(Addr_) == type(Len_) == int:
        if Len_ == 4:
            dxl_present_Value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_, Addr_)
        elif Len_ == 1:
            dxl_present_Value, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID_, Addr_)
        elif Len_ == 2:
            dxl_present_Value, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID_, Addr_)
        else:
            quit(f"[Error in Len_ of Dynmxl_reading] This code use only three types of Len_: 1, 2, 4\nLen_ value: {Len_}")
    
    else:
        quit(f"[Error in variables' types of Dynmxl_reading]\nTheir values and types:\nID value: {ID_} / its type: {type(ID_)}\nAddress value: {Addr_} / its type: {type(Addr_)}\nLength value: {Len_} / its type: {type(Len_)}")

    return dxl_present_Value, dxl_comm_result, dxl_error


def Dynmxl_Read_PresentValue(ID_: int):
    Present_OP_Mode = Dynmxl_reading(ID_=ID_, Addr_=11, Len_=1)[0]
    if Present_OP_Mode == 1:
        Target_Addr = 128
    elif Present_OP_Mode == 3:
        Target_Addr = 132
    while True:
        dxl_present_Value, dxl_comm_result, dxl_error = Dynmxl_reading(ID_=ID_, Addr_=Target_Addr, Len_=4)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        if Present_OP_Mode == 1:
            print("[ID:%02d] PresVel: %4d" % (ID_, dxl_present_Value), end="\r")

            if not abs(0 - dxl_present_Value) > 0:
                print()
                break

        elif Present_OP_Mode == 3:
            print("[ID:%02d] PresPos: %4d" % (ID_, dxl_present_Value), end="\r")

            if not abs(dxlconst.FORWARD_POS - dxl_present_Value) > 0:
                print()
                break

        else:
            pass


def Dynmxl_GrpSyncWrite_POS_addParam(Motor_ID: int, Goal_POS: int):
    if dxlconst.DYNMXL_ENABLED_STATE:
        if groupSyncWrite_POS.addParam(Motor_ID, Goal_POS):
            return True
        else:
            print(f"Failed to add parameter of Position data(Value: {Goal_POS}) in Dynamixel(ID: {Motor_ID:02d}).")
            return False
    else:
        print(f"Check torque of Dynamixels. Failed to add parameter of Position data(Value: {Goal_POS}) in Dynamixel(ID: {Motor_ID:02d}).")
        return False

def Dynmxl_GrpSyncWrite_Vel_addParam(Motor_ID: int, Goal_VEL: int):
    if dxlconst.DYNMXL_ENABLED_STATE:
        groupSyncWrite_VEL.addParam(Motor_ID, Goal_VEL)
        return True
    else:
        print(f"Check torque of Dynamixels. Failed to add parameter of Velocity data(Value: {Goal_VEL}) in Dynamixel(ID: {Motor_ID:02d}).")
        return False
    

def Dynmxl_GrpSyncRead_addParam(Motor_ID: int, motor_Operating_Type):
    # Add parameter storage for Dynamixel present position or velocity value
    if motor_Operating_Type == dxlconst.OP_VALUE_POSITION:
        dxl_addparam_result = groupSyncRead_POS.addParam(Motor_ID)
    elif motor_Operating_Type == dxlconst.OP_VALUE_VELOCITY:
        dxl_addparam_result = groupSyncRead_VEL.addParam(Motor_ID)
    else:
        quit(f"[Error in motor_Operating_Type of Dynmxl_GrpSyncRead_addParam()]\nDynamixel (ID: {Motor_ID:02d}) will be received unexpected operating mode. Received value: {motor_Operating_Type}\nThis code use only two numbers(modes). > 1: Velocity / 3: Position")

    if dxl_addparam_result != True:
        print(f"Dynamixel (ID: {Motor_ID:02d}) groupSyncRead(OP Mode: {motor_Operating_Type}) addparam failed.")
        quit()

'''
def Dynmxl_GrpSyncRead_VEL_addParam(Motor_ID):
    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncRead_VEL.addParam(Motor_ID)

    if dxl_addparam_result != True:
        print(f"Dynamixel (ID: {Motor_ID:02d}) groupSyncRead(OP Mode: Velocity) addparam failed.")
        quit()
'''

def Dynmxl_GrpSyncWrite_TxPacket(motor_Operating_Type: int):
    if motor_Operating_Type == dxlconst.OP_VALUE_POSITION:
        dxl_comm_result = groupSyncWrite_POS.txPacket()

        # Clear syncwrite(POS) parameter storage
        groupSyncWrite_POS.clearParam()

    elif motor_Operating_Type == dxlconst.OP_VALUE_VELOCITY:
        dxl_comm_result = groupSyncWrite_VEL.txPacket()
        
        # Clear syncwrite(VEL) parameter storage
        groupSyncWrite_VEL.clearParam()
    
    else:
        quit(f"[Error in motor_Operating_Type of Dynmxl_GrpSyncWrite_TxPacket()]\nDynamixels will be received unexpected operating mode. Received value: {motor_Operating_Type}\nThis code use only two numbers(modes). > 1: Velocity / 3: Position")
        

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))


def Dynmxl_GrpSyncRead_txRxPacket(motor_ID: int, motor_Operating_Type: int):
    if motor_Operating_Type == dxlconst.OP_VALUE_POSITION:
        # Syncread present position
        dxl_comm_result = groupSyncRead_POS.txRxPacket()

    elif motor_Operating_Type == dxlconst.OP_VALUE_VELOCITY:
        # Syncread present velocity
        dxl_comm_result = groupSyncRead_VEL.txRxPacket()

    else:
        quit(f"[Error in motor_Operating_Type of Dynmxl_GrpSyncRead_txRxPacket()]\nDynamixel (ID: {motor_ID:02d}) will be received unexpected operating mode. Received value: {motor_Operating_Type}\nThis code use only two numbers(modes). > 1: Velocity / 3: Position")


    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    
    if motor_Operating_Type == dxlconst.OP_VALUE_POSITION:
        return groupSyncRead_POS.getData(motor_ID, dxlconst.ADDR_PRESENT_POSITION, dxlconst.LEN_PRESENT_POSITION)
    elif motor_Operating_Type == dxlconst.OP_VALUE_VELOCITY:
        return groupSyncRead_VEL.getData(motor_ID, dxlconst.ADDR_PRESENT_VELOCITY, dxlconst.LEN_PRESENT_VELOCITY)
    else:
        quit(f"[Error in motor_Operating_Type of Dynmxl_GrpSyncRead_txRxPacket()]\nDynamixel (ID: {motor_ID:02d}) will be received unexpected operating mode. Received value: {motor_Operating_Type}\nThis code use only two numbers(modes). > 1: Velocity / 3: Position")


def Dynmxl_Initialization(Setup_List: list):
    for ID_index in Setup_List:
        for Data_index in ID_index[1]:
            Dynmxl_writing(ID_=ID_index[0], Addr_=Data_index[0], Len_=Data_index[1],Param_=Data_index[2])
            
            if Data_index[0] == dxlconst.ADDR_GOAL_POSITION:
                while True:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_index[0], dxlconst.ADDR_PRESENT_POSITION)

                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                        if not Dynmxl_reading(ID_=ID_index[0], Addr_=144, Len_=2)[0]:
                            quit(f"[Error in supplying Power to Dynamixel (ID: {ID_index[0]:>03}) of Dynmxl_Initialization]\n")
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    print("[ID:%02d] GoalPos: %4d  PresPos: %4d" % (ID_index[0], Data_index[2], dxl_present_position), end="\r")

                    if not abs(Data_index[2] - dxl_present_position) > dxlconst.DXL_POS_THRESHOLD:
                        print()
                        break

            elif Data_index[0] == dxlconst.ADDR_GOAL_VELOCITY:
                while True:
                    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_index[0], dxlconst.ADDR_PRESENT_VELOCITY)

                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                        if not Dynmxl_reading(ID_=ID_index[0], Addr_=144, Len_=2)[0]:
                            quit(f"[Error in supplying Power to Dynamixel (ID: {ID_index[0]:>03}) of Dynmxl_Initialization]\n")
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))

                    print("[ID:%02d] GoalVel: %4d  PresVel: %4d" % (ID_index[0], Data_index[2], dxl_present_velocity), end="\r")

                    if not abs(Data_index[2] - dxl_present_velocity) > 0:
                        print()
                        break

            else:
                pass # Nothing to do
    
        print(f"Dynamixel No.{ID_index[0]:02d} has been successfully initialized.")
    print("\nReady to run Dynamixels!")


##### Ackermann 구동을 위한 함수 ##### Dynmxl_Ackermann.py에서 가져옴.


def Radius_calc(OuterWheel_Angle_in4096: int):
    '''
    ※ 주의 사항: 본 함수는 양 방향으로 최대 조향각이 일정 값으로 제한되어 있음.
    OuterWheel_Angle_in4096 인자는 Dynamixel의 Position Operating Mode의 작동 범위인 0 - 4095 사이의 각도값을 받아,
    '바퀴(차체)의 회전 방향(0: Forward / -1: Left / 1: Right / Other Value: STOP)'과 '차체 회전 중심점부터 차체 중심수직선까지의 거리' 2개를 반환한다.
    변수명이 OuterWheel인 이유는, Ackermann 동작 시 최소 조향각을 최소한의 오차로 구함에 있어 외측 차륜의 조향각이 내측 차륜의 조향각보다 더 작기 때문이다.
    '''
    Outer_Angle_to360 = OuterWheel_Angle_in4096 * (360 / 4096)

    if 180.0 - dxlconst.MAXIMUM_STEERING_ANG <= Outer_Angle_to360 < 180.0: # 정면 좌측부터 정면 직전
        Virtual_Radius_from_OuterLine = (dxlconst.WHEELBASE / math.tan(math.radians(180 - Outer_Angle_to360)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - dxlconst.HALF_TREAD
        Steering_Direction = dxlconst.DIR_LEFT

    elif 180.0 < Outer_Angle_to360 <= 180.0 + dxlconst.MAXIMUM_STEERING_ANG: # 정면 우측부터 정면 직전
        Virtual_Radius_from_OuterLine = (dxlconst.WHEELBASE / math.tan(math.radians(Outer_Angle_to360)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - dxlconst.HALF_TREAD
        Steering_Direction = dxlconst.DIR_RIGHT

    elif (Outer_Angle_to360 < 180.0 - dxlconst.MAXIMUM_STEERING_ANG) and not(Outer_Angle_to360 in [0.0, 90.0, 180.0, 270.0]): # 90도 배수를 제외한 나머지 좌측 방향
        Virtual_Radius_from_OuterLine = (dxlconst.WHEELBASE / math.tan(math.radians(dxlconst.MAXIMUM_STEERING_ANG)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - dxlconst.HALF_TREAD
        Steering_Direction = dxlconst.DIR_LEFT

    elif (180.0 + dxlconst.MAXIMUM_STEERING_ANG < Outer_Angle_to360) and not(Outer_Angle_to360 in [0.0, 90.0, 180.0, 270.0]): # 90도 배수를 제외한 나머지 우측 방향
        Virtual_Radius_from_OuterLine = (dxlconst.WHEELBASE / math.tan(math.radians(dxlconst.MAXIMUM_STEERING_ANG)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - dxlconst.HALF_TREAD
        Steering_Direction = dxlconst.DIR_RIGHT

    else: # 정말 나머지
        if OuterWheel_Angle_in4096 == 2048: # 정면
            Steering_Direction = dxlconst.DIR_FORWARD
            Virtual_Radius_from_OuterLine = 0
            Virtual_Radius_from_CenterLine = 0
        else: # 정면도 아닌 90도의 배수 방향(정좌측, 정우측, 정후측)
            Steering_Direction = dxlconst.DIR_STOP
            quit(f"Error of wheel steering! OuterWheel_Angle_in4096 value: {OuterWheel_Angle_in4096}")

    return Steering_Direction, round(Virtual_Radius_from_CenterLine, 6)


def Angle_calc(Wheel_Coordinates: list, Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = dxlconst.HALF_TREAD):
    '''
    Wheel_Coordinates: list[int, int], Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = HALF_TREAD

    Wheel_Coordinates(바퀴 좌표) 인자 설명: 로봇 상단(윗면)에서 로봇 정면이 위를 바라보도록 수직으로 바라보았을 때,
    차체의 회전 중심점(TurningPoint)과 차체 중심수직선(CenterLine) 교차하는 점을 영점으로 두고,
    차체의 우상단 방면을 수직좌표계와 같이 x, y 모두 양수로 보고, 해당 값을 받아서 바퀴의 위치를 대략적으로 구하는 인자이다.
    인자 자료형은 list[int, int] (예시: [1, 1] 또는 [0, -1] 등)이며, 값은 좌표에 맞게 0(축 위의 있음) 또는 ±1 이다.
    '''
    Wheel_X, Wheel_Y = Wheel_Coordinates
    # print(Wheel_X, Wheel_Y)

    if Steering_Direction == dxlconst.DIR_FORWARD:
        Steering_Angle_to4096 = 2048
    
    elif Steering_Direction == dxlconst.DIR_LEFT or dxlconst.DIR_RIGHT:
        if Wheel_Y == 0:
            Steering_Angle_to4096 = 2048
        else: # Wheel_Y == ±1:
            Steering_Alpha = math.degrees(math.atan2(Wheel_Y * Vertical_Length_to_Wheel_from_TurningPoint, (Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine)))))
            # print(Steering_Alpha)
            
            Steering_Angle_to4096 = round(dxlconst.FORWARD_POS + ((Steering_Direction * Steering_Alpha) * (4096 / 360))) # Direction numbers(Left: -1 and Right: +1) were defined in reverse, so we should minus from FORWARD_POS. But when we use Dynamixel in real, we can think and calculate direction easily.
            # print(Steering_Angle_to4096)

    else:
        quit(f"STOP because of Steering_Direction! Its value: {Steering_Direction}")

    return Steering_Angle_to4096


def Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200: int, Wheel_Coordinates: list, Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = dxlconst.HALF_TREAD):
    '''
    Target_Velocity_of_theMost_OuterWheel_in200: int, Wheel_Coordinates: list[int, int], Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = HALF_TREAD
    
    Wheel_Coordinates(바퀴 좌표) 인자 설명: 로봇 상단(윗면)에서 로봇 정면이 위를 바라보도록 수직으로 바라보았을 때,
    차체의 회전 중심점(TurningPoint)과 차체 중심수직선(CenterLine) 교차하는 점을 영점으로 두고,
    차체의 우상단 방면을 수직좌표계와 같이 x, y 모두 양수로 보고, 해당 값을 받아서 바퀴의 위치를 대략적으로 구하는 인자이다.
    인자 자료형은 list[int, int] (예시: [1, 1] 또는 [0, -1] 등)이며, 값은 좌표에 맞게 0(축 위의 있음) 또는 ±1 이다.
    '''
    if Target_Velocity_of_theMost_OuterWheel_in200 > 200:
        quit(f"Out of Velocity! Maximum Velocity is 200[unit].\nThe targeting Velocity of the most outer wheel is {Target_Velocity_of_theMost_OuterWheel_in200}[unit].")

    Wheel_X, Wheel_Y = Wheel_Coordinates
    # print(Wheel_X, Wheel_Y)

    if Steering_Direction == dxlconst.DIR_FORWARD:
        Adjusted_Wheel_Velocity = Target_Velocity_of_theMost_OuterWheel_in200
    
    elif Steering_Direction == dxlconst.DIR_LEFT or dxlconst.DIR_RIGHT:
        if Wheel_Y == 0:
            Length_from_Wheel_to_TurningPoint = Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine))
        else: # Wheel_Y == ±1:
            Length_from_Wheel_to_TurningPoint = math.sqrt((Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine)))**2 + Vertical_Length_to_Wheel_from_TurningPoint**2)
        
        Adjusted_Wheel_Velocity = Target_Velocity_of_theMost_OuterWheel_in200 * (Length_from_Wheel_to_TurningPoint / (math.sqrt((Radius_from_TurningPoint_to_CenterLine + max(dxlconst.LIST_HALF_TREAD))**2 + max(dxlconst.LIST_WHEELBASE)**2)))

    else:
        quit(f"STOP because of Steering_Direction! Its value: {Steering_Direction}")
    
    return round(Adjusted_Wheel_Velocity)


##### 다이나믹셀 통신을 위한 함수 ##### Dynmxl_running.py에서 가져옴.


# Allocate goal position value into byte array
def Allocate_DXL_data(data_value: int):
    param_value = [DXL_LOBYTE(DXL_LOWORD(data_value)), DXL_HIBYTE(DXL_LOWORD(data_value)), DXL_LOBYTE(DXL_HIWORD(data_value)), DXL_HIBYTE(DXL_HIWORD(data_value))]

    return param_value


def Dynmxl_TxRx_for_ATOM(Steering_Direction_data: int, Turning_Radius_data: float, Velocity_data: int, Gimbal_Roll_data: int = dxlconst.INIT_DXL_POS, Gimbal_Pitch_data: int = dxlconst.INIT_DXL_POS):
    groupSyncWrite_POS.clearParam()
    groupSyncWrite_VEL.clearParam()

    FrontLeft_Angle_in4096 = Angle_calc(Wheel_Coordinates=[-1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    FrontRight_Angle_in4096 = Angle_calc(Wheel_Coordinates=[1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    
    FrontLeft_Velo_in200 = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[-1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    FrontRight_Velo_in200 = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    RearLeft_Velo_in200 = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[-1, 0], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    RearRight_Velo_in200 = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[1, 0], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)

    List_of_Angle_in4096 = [FrontLeft_Angle_in4096, FrontRight_Angle_in4096, Gimbal_Roll_data, Gimbal_Pitch_data]
    List_of_Velo_in200 = [FrontLeft_Velo_in200, FrontRight_Velo_in200, RearLeft_Velo_in200, RearRight_Velo_in200]

    # print(f"{List_of_Angle_in4096}\n{List_of_Velo_in200}")

    for POS_index in range(len(dxlconst.DXL_ID_POS)):
        groupSyncWrite_POS.addParam(dxlconst.DXL_ID_POS[POS_index], Allocate_DXL_data(List_of_Angle_in4096[POS_index]))

    for VEL_index in range(len(dxlconst.DXL_ID_VEL)):
        groupSyncWrite_VEL.addParam(dxlconst.DXL_ID_VEL[VEL_index], Allocate_DXL_data((List_of_Velo_in200[VEL_index] & 0xFFFFFFFF))) # 0xFFFFFFFF makes negative value to deliver correctly.
    
    groupSyncWrite_POS.txPacket()
    groupSyncWrite_VEL.txPacket()

    groupSyncWrite_POS.clearParam()
    groupSyncWrite_VEL.clearParam()

    return [List_of_Angle_in4096, List_of_Velo_in200]


def Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096: int, Outer_Wheel_Velocity_in200: int, Gimbal_Roll_in4096: int, Gimbal_Pitch_in4096: int):
    Car_Steering_Direction, Turning_Radius = Radius_calc(OuterWheel_Angle_in4096=Outer_Wheel_Angle_in4096)
    if Outer_Wheel_Velocity_in200 >= 0:
        return Dynmxl_TxRx_for_ATOM(Steering_Direction_data=Car_Steering_Direction, Turning_Radius_data=Turning_Radius, Velocity_data=Outer_Wheel_Velocity_in200, Gimbal_Roll_data=Gimbal_Roll_in4096, Gimbal_Pitch_data=Gimbal_Pitch_in4096)
    else:
        return Dynmxl_TxRx_for_ATOM(Steering_Direction_data=Car_Steering_Direction, Turning_Radius_data=Turning_Radius, Velocity_data=0, Gimbal_Roll_data=Gimbal_Roll_in4096, Gimbal_Pitch_data=Gimbal_Pitch_in4096)


def Dynmxl_Start_process_for_ATOM():
    for repeat_ID_index in dxlconst.DXL_ID_ALL:
        Dynmxl_Torque(Motor_ID=repeat_ID_index, ENabled_or_DISabled=1)    
    Dynmxl_TxRx_for_ATOM(Steering_Direction_data=dxlconst.DIR_FORWARD, Turning_Radius_data=0, Velocity_data=0)

    dxlconst.DYNMXL_ENABLED_STATE = True

def Dynmxl_Finish_process_for_ATOM():
    Dynmxl_TxRx_for_ATOM(Steering_Direction_data=dxlconst.DIR_FORWARD, Turning_Radius_data=0, Velocity_data=0)
    for repeat_ID_index in dxlconst.DXL_ID_ALL:
        Dynmxl_Read_PresentValue(ID_=repeat_ID_index)
        Dynmxl_Torque(Motor_ID=repeat_ID_index, ENabled_or_DISabled=0)

    dxlconst.DYNMXL_ENABLED_STATE = False


##### ##### ##### ##### ##### 포트 연결 및 통신 성공 여부 확인 및 모터 구동 전 초기화 ##### ##### ##### ##### #####

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(dxlconst.ATOM_BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

'''
# FrontLeft_Vel = Dynmxl_CMD(motor_ID=1, motor_Mode=1, Wheel_Coordinates=[-1, 1], Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
# FrontRight_Vel = Dynmxl_CMD(motor_ID=2, motor_Mode=1, Wheel_Coordinates=[1, 1], Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
# RearLeft_Vel = Dynmxl_CMD(motor_ID=3, motor_Mode=1, Wheel_Coordinates=[-1, 0], Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
# RearRight_Vel = Dynmxl_CMD(motor_ID=4, motor_Mode=1, Wheel_Coordinates=[1, 0], Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)

# FrontLeft_Pos = Dynmxl_CMD(motor_ID=5, motor_Mode=3, Wheel_Coordinates=[-1, 1], Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
# FrontRight_Pos = Dynmxl_CMD(motor_ID=6, motor_Mode=3, Wheel_Coordinates=[1, 1], Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
'''

Dynmxl_Initialization(Setup_List=dxlconst.DXL_INIT_SETUP)






##### 이 코드를 테스트로 실행할 경우에 테스트 출력하는 if문임. #####

if __name__ =="__main__":
    R = Radius_calc(1537)
    A = Angle_calc(Wheel_Coordinates=[1, 1], Steering_Direction=dxlconst.DIR_LEFT, Radius_from_TurningPoint_to_CenterLine=0.2, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)
    V = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=200,Wheel_Coordinates=[1, 1], Steering_Direction=dxlconst.DIR_RIGHT, Radius_from_TurningPoint_to_CenterLine=0.2, Vertical_Length_to_Wheel_from_TurningPoint=dxlconst.WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=dxlconst.HALF_TREAD)

    print(R, A, V)