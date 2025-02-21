################################# 다이나믹셀을 초기화하는 함수 ##################################

from dynamixel_sdk import *

portHandler = PortHandler('/dev/ttyACM0')
packetHandler = PacketHandler(2.0)
portHandler.setBaudRate(1000000)
portHandler.openPort()


IDS_vel                   = [1, 2, 3, 4]
IDS_pos                   = [5, 6]


# 6. ID 1,2,3,4의 속도 초기화 (104번 주소)
for dxl_id in IDS_vel:
    packetHandler.write4ByteTxRx(portHandler, dxl_id, 104, 0)

# 7. ID 5,6의 각도 초기화 (116번 주소)
for dxl_id in IDS_pos:
    packetHandler.write4ByteTxRx(portHandler, dxl_id, 116, 2048)

# 8. 토크 끄기 (64번 주소)
for dxl_id in IDS_vel + IDS_pos:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 0)

# 9. 모드 변경 (11번 주소)
for dxl_id in IDS_vel:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, 11, 1)
for dxl_id in IDS_pos:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, 11, 3)

# 10. 토크 켜기 (64번 주소)
for dxl_id in IDS_vel + IDS_pos:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 1)

print("다이나믹셀 초기화 완료!!")