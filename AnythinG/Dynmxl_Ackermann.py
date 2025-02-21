import math
from Dynmxl_Setup_Constant import *

# Steering_Direction      = DIR_STOP               # 0: Forward / -1: Left / 1: Right / Other Value: STOP


def Radius_calc(OuterWheel_Angle_in4096: int):
    '''
    ※ 주의 사항: 본 함수는 양 방향으로 최대 조향각이 일정 값으로 제한되어 있음.
    OuterWheel_Angle_in4096 인자는 Dynamixel의 Position Operating Mode의 작동 범위인 0 - 4095 사이의 각도값을 받아,
    '바퀴(차체)의 회전 방향(0: Forward / -1: Left / 1: Right / Other Value: STOP)'과 '차체 회전 중심점부터 차체 중심수직선까지의 거리' 2개를 반환한다.
    변수명이 OuterWheel인 이유는, Ackermann 동작 시 최소 조향각을 최소한의 오차로 구함에 있어 외측 차륜의 조향각이 내측 차륜의 조향각보다 더 작기 때문이다.
    '''
    Outer_Angle_to360 = OuterWheel_Angle_in4096 * (360 / 4096)

    if 180.0 - MAXIMUM_STEERING_ANG <= Outer_Angle_to360 < 180.0: # 정면 좌측부터 정면 직전
        Virtual_Radius_from_OuterLine = (WHEELBASE / math.tan(math.radians(180 - Outer_Angle_to360)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - HALF_TREAD
        Steering_Direction = DIR_LEFT

    elif 180.0 < Outer_Angle_to360 <= 180.0 + MAXIMUM_STEERING_ANG: # 정면 우측부터 정면 직전
        Virtual_Radius_from_OuterLine = (WHEELBASE / math.tan(math.radians(Outer_Angle_to360)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - HALF_TREAD
        Steering_Direction = DIR_RIGHT

    elif (Outer_Angle_to360 < 180.0 - MAXIMUM_STEERING_ANG) and not(Outer_Angle_to360 in [0.0, 90.0, 180.0, 270.0]): # 90도 배수를 제외한 나머지 좌측 방향
        Virtual_Radius_from_OuterLine = (WHEELBASE / math.tan(math.radians(MAXIMUM_STEERING_ANG)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - HALF_TREAD
        Steering_Direction = DIR_LEFT

    elif (180.0 + MAXIMUM_STEERING_ANG < Outer_Angle_to360) and not(Outer_Angle_to360 in [0.0, 90.0, 180.0, 270.0]): # 90도 배수를 제외한 나머지 우측 방향
        Virtual_Radius_from_OuterLine = (WHEELBASE / math.tan(math.radians(MAXIMUM_STEERING_ANG)))
        Virtual_Radius_from_CenterLine = Virtual_Radius_from_OuterLine - HALF_TREAD
        Steering_Direction = DIR_RIGHT

    else: # 정말 나머지
        if OuterWheel_Angle_in4096 == 2048: # 정면
            Steering_Direction = DIR_FORWARD
            Virtual_Radius_from_OuterLine = 0
            Virtual_Radius_from_CenterLine = 0
        else: # 정면도 아닌 90도의 배수 방향(정좌측, 정우측, 정후측)
            Steering_Direction = DIR_STOP
            quit(f"Error of wheel steering! OuterWheel_Angle_in4096 value: {OuterWheel_Angle_in4096}")

    
    # if Outer_Angle_to360 % 1.0 == 0:
    #     temp_angle = round(Outer_Angle_to360)
    #     print(temp_angle)
    
    # print(Virtual_Radius_from_OuterLine)
    # print(Virtual_Radius_from_CenterLine)

    return Steering_Direction, round(Virtual_Radius_from_CenterLine, 6)


def Angle_calc(Wheel_Coordinates: list[int], Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = HALF_TREAD):
    '''
    Wheel_Coordinates(바퀴 좌표) 인자 설명: 로봇 상단(윗면)에서 로봇 정면이 위를 바라보도록 수직으로 바라보았을 때,
    차체의 회전 중심점(TurningPoint)과 차체 중심수직선(CenterLine) 교차하는 점을 영점으로 두고,
    차체의 우상단 방면을 수직좌표계와 같이 x, y 모두 양수로 보고, 해당 값을 받아서 바퀴의 위치를 대략적으로 구하는 인자이다.
    인자 자료형은 list[int] (예시: [1, 1] 또는 [0, -1] 등)이며, 값은 좌표에 맞게 0(축 위의 있음) 또는 ±1 이다.
    '''
    Wheel_X, Wheel_Y = Wheel_Coordinates
    # print(Wheel_X, Wheel_Y)

    if Steering_Direction == DIR_FORWARD:
        Steering_Angle_to4096 = 2048
    
    elif Steering_Direction == DIR_LEFT or DIR_RIGHT:
        if Wheel_Y == 0:
            Steering_Angle_to4096 = 2048
        else: # Wheel_Y == ±1:
            Steering_Alpha = math.degrees(math.atan2(Wheel_Y * Vertical_Length_to_Wheel_from_TurningPoint, (Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine)))))
            # print(Steering_Alpha)
            
            Steering_Angle_to4096 = round(FORWARD_POS + ((Steering_Direction * Steering_Alpha) * (4096 / 360))) # Direction numbers(Left: -1 and Right: +1) were defined in reverse, so we should minus from FORWARD_POS. But when we use Dynamixel in real, we can think and calculate direction easily.
            # print(Steering_Angle_to4096)

    else:
        quit(f"STOP because of Steering_Direction! Its value: {Steering_Direction}")

    return Steering_Angle_to4096


def Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200: int, Wheel_Coordinates: list[int], Steering_Direction: int, Radius_from_TurningPoint_to_CenterLine: float, Vertical_Length_to_Wheel_from_TurningPoint: float, Horizontal_Width_to_Wheel_from_CenterLine: float = HALF_TREAD):
    '''
    Wheel_Coordinates(바퀴 좌표) 인자 설명: 로봇 상단(윗면)에서 로봇 정면이 위를 바라보도록 수직으로 바라보았을 때,
    차체의 회전 중심점(TurningPoint)과 차체 중심수직선(CenterLine) 교차하는 점을 영점으로 두고,
    차체의 우상단 방면을 수직좌표계와 같이 x, y 모두 양수로 보고, 해당 값을 받아서 바퀴의 위치를 대략적으로 구하는 인자이다.
    인자 자료형은 list[int] (예시: [1, 1] 또는 [0, -1] 등)이며, 값은 좌표에 맞게 0(축 위의 있음) 또는 ±1 이다.
    '''
    if Target_Velocity_of_theMost_OuterWheel_in200 > 200:
        quit(f"Out of Velocity! Maximum Velocity is 200[unit].\nThe targeting Velocity of the most outer wheel is {Target_Velocity_of_theMost_OuterWheel_in200}[unit].")

    Wheel_X, Wheel_Y = Wheel_Coordinates
    # print(Wheel_X, Wheel_Y)

    if Steering_Direction == DIR_FORWARD:
        Adjusted_Wheel_Velocity = Target_Velocity_of_theMost_OuterWheel_in200
    
    elif Steering_Direction == DIR_LEFT or DIR_RIGHT:
        if Wheel_Y == 0:
            Length_from_Wheel_to_TurningPoint = Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine))
        else: # Wheel_Y == ±1:
            Length_from_Wheel_to_TurningPoint = math.sqrt((Radius_from_TurningPoint_to_CenterLine - (Steering_Direction * (Wheel_X * Horizontal_Width_to_Wheel_from_CenterLine)))**2 + Vertical_Length_to_Wheel_from_TurningPoint**2)
        
        Adjusted_Wheel_Velocity = Target_Velocity_of_theMost_OuterWheel_in200 * (Length_from_Wheel_to_TurningPoint / (math.sqrt((Radius_from_TurningPoint_to_CenterLine + HALF_TREAD)**2 + WHEELBASE**2)))

    else:
        quit(f"STOP because of Steering_Direction! Its value: {Steering_Direction}")
    
    return round(Adjusted_Wheel_Velocity)



if __name__ =="__main__":
    R = Radius_calc(1537)
    A = Angle_calc(Wheel_Coordinates=[1, 1], Steering_Direction=DIR_LEFT, Radius_from_TurningPoint_to_CenterLine=0.2, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    V = Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=200,Wheel_Coordinates=[1, 1], Steering_Direction=DIR_RIGHT, Radius_from_TurningPoint_to_CenterLine=0.2, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)

    print(R, A, V)