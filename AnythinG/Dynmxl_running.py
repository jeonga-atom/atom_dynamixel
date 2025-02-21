from dynamixel_sdk import *

from Dynmxl_Setup_Constant import *
from Dynmxl_initializing import *
import Dynmxl_Ackermann as dxlackm


# Allocate goal position value into byte array
def Allocate_DXL_data(data_value):
    param_value = [DXL_LOBYTE(DXL_LOWORD(data_value)), DXL_HIBYTE(DXL_LOWORD(data_value)), DXL_LOBYTE(DXL_HIWORD(data_value)), DXL_HIBYTE(DXL_HIWORD(data_value))]

    return param_value


def Dynmxl_TxRx_for_ATOM(Steering_Direction_data, Turning_Radius_data, Velocity_data):
    groupSyncWrite_POS.clearParam()
    groupSyncWrite_VEL.clearParam()

    FrontLeft_Angle_in4096 = dxlackm.Angle_calc(Wheel_Coordinates=[-1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    FrontRight_Angle_in4096 = dxlackm.Angle_calc(Wheel_Coordinates=[1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    
    FrontLeft_Velo_in200 = dxlackm.Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[-1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    FrontRight_Velo_in200 = dxlackm.Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[1, 1], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=WHEELBASE, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    RearLeft_Velo_in200 = dxlackm.Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[-1, 0], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)
    RearRight_Velo_in200 = dxlackm.Velo_calc(Target_Velocity_of_theMost_OuterWheel_in200=Velocity_data, Wheel_Coordinates=[1, 0], Steering_Direction=Steering_Direction_data, Radius_from_TurningPoint_to_CenterLine=Turning_Radius_data, Vertical_Length_to_Wheel_from_TurningPoint=0, Horizontal_Width_to_Wheel_from_CenterLine=HALF_TREAD)

    List_of_Angle_in4096 = [FrontLeft_Angle_in4096, FrontRight_Angle_in4096]
    List_of_Velo_in200 = [FrontLeft_Velo_in200, FrontRight_Velo_in200, RearLeft_Velo_in200, RearRight_Velo_in200]

    # print(f"{List_of_Angle_in4096}\n{List_of_Velo_in200}")

    for POS_index in range(len(DXL_ID_POS)):
        groupSyncWrite_POS.addParam(DXL_ID_POS[POS_index], Allocate_DXL_data(List_of_Angle_in4096[POS_index]))

    for VEL_index in range(len(DXL_ID_VEL)):
        groupSyncWrite_VEL.addParam(DXL_ID_VEL[VEL_index], Allocate_DXL_data((List_of_Velo_in200[VEL_index] & 0xFFFFFFFF))) # 0xFFFFFFFF makes negative value to deliver correctly.
    
    groupSyncWrite_POS.txPacket()
    groupSyncWrite_VEL.txPacket()

    groupSyncWrite_POS.clearParam()
    groupSyncWrite_VEL.clearParam()


def Dynmxl_process_for_ATOM(Outer_Wheel_Angle_in4096: int, Outer_Wheel_Velocity_in200: int):
    Car_Steering_Direction, Turning_Radius = dxlackm.Radius_calc(OuterWheel_Angle_in4096=Outer_Wheel_Angle_in4096)
    Dynmxl_TxRx_for_ATOM(Steering_Direction_data=Car_Steering_Direction, Turning_Radius_data=Turning_Radius, Velocity_data=Outer_Wheel_Velocity_in200)