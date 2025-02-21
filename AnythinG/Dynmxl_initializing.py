from dynamixel_sdk import *

from Dynmxl_Setup_Constant import *


# 다이나믹셀 초기화 함수 만들기(포트 연결 및 통신 성공 여부부터 해야 함. import 특징임.)

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(ATOM_DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(ATOM_PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance for Goal Position
groupSyncWrite_POS = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead_POS = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Initialize GroupSyncWrite instance for Goal Velocity
groupSyncWrite_VEL = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

# Initialize GroupSyncRead instace for Present Velocity
groupSyncRead_VEL = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)


# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(ATOM_BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()


for DXL_index in range(len(DXL_ID_ALL)):
    # Enable Dynamixels Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_ALL[DXL_index], ADDR_TORQUE_VALUE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel #%d has been successfully connected." % DXL_ID_ALL[DXL_index])