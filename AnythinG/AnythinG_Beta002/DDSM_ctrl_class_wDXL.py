# AnythinG_Beta002.py

# This code has been written since January 14, 2025. 이 코드는 2025년 01월 14일부터 작성되었습니다.
# This code is written for DDSM115 motor. 이 코드는 DDSM115 모터를 위해 작성되었습니다.
# 모터 매뉴얼은 MS Teams의 랩실 팀즈 내에서 찾거나, 다음 주소를 참고할 것. https://www.waveshare.com/wiki/DDSM115

# 한국기술교육대학교 GADGET(가제트) 파이팅!

##### ##### #####
# import sys # exit()을 사용하기 위해 import
import serial # import serial이 오류인 경우, VS code 기준으로 '보기' -> '명령 팔레트' -> '인터프리터 선택'을 들어가서, /usr/bin/python3 으로 선택해서 해보기
import struct # 음수를 포함한 10진수 -> 16진수 변환을 위해 hex()가 아닌 pack(), unpack()을 이용하기 위해 import
import math # 모터 제어 값 데이터 계산 처리(math.floor() 등)를 위해 import
# import time # 모터 움직임을 확인하기 위해 import

from DXL_DDSM_Const import *

# 클래스 선언
class DDSM115_CMD():
    # 이곳, 생성자 메소드 밖의 정의 내용들은 모든 인스턴스에 공통으로 적용되는 사항임. (주의 사항: 값을 실행 중 입력을 통해서 바꿀 경우, 모든 인스턴스에 똑같이 적용됨.)
    # 빈 칸


    # 생성자 메소드 선언
    def __init__(self, motor_USBport_num: int, motor_setting_ID: int, motor_Coordinates: list, motor_Current_Limit: int = 7, motor_Velocity_Limit: int = 100, Output_for_User: bool = False):
        # 하단의 port, baudrate 등의 pyserial options 출처: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
        self.cSerial = serial.Serial(
            port = '/dev/ttyACM' + str(motor_USBport_num), # USB 연결 시 인식되는 /dev/ttyACM 에 물리적으로 연결한 위치(순서)에 맞게 번호(motor_USBport_num) 부여할 것. 모터의 ID와는 별개의 변수이므로 유의할 것!
            baudrate = 115200,
            bytesize = serial.EIGHTBITS,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout = 1
            # 각 값은 모터 매뉴얼 9쪽을 참고함.
            # serial.EIGHTBITS 등의 우변의 내용 출처: https://pyserial.readthedocs.io/en/latest/pyserial_api.html#constants
        )

        # 이하는 변수(속성) 설정. 메소드 호출 시 코드의 복잡성을 줄이기 위해 작성함.
        self.cMotor_ID = motor_setting_ID # 모터 ID 변수
        self.cMotor_Mode = 0x02 # 모터 모드 변수(2로 초기화-속도 모드)

        self.cOutput_for_User = Output_for_User # 모터 피드백 출력 변수

        self.cTX_data = [] # TX_data를 생성된 인스턴스 각 하나마다 공통으로 전송(또는 전송 예정인) 데이터를 처리해서 return 반환 없이도 불러올 수 있도록 하기 위한 변수(자료형은 리스트 또는 튜플)
        self.cRX_data = [] # RX_data를 생성된 인스턴스 각 하나마다 공통으로 피드백 받은 데이터를 그때그때 처리해서 return 반환 없이도 불러올 수 있도록 하기 위한 변수(자료형은 리스트 또는 튜플)

        self.cCurrent_Limit = motor_Current_Limit  # 속도 제한 값 초기화(기본값 100으로 설정됨.)
        self.cVelocity_Limit = motor_Velocity_Limit  # 속도 제한 값 초기화(기본값 100으로 설정됨.)

        # 아래 3개 변수는 각 모터(즉, 각 인스턴스)의 중요 값들(토크 전류, 속도, 각도(위치))을 하나로 저장하는 변수(정수형 및 실수형으로 초기화)
        self.cMotor_Current = 0.0 # 전류는 소수점 이하 값이 가능한 데이터임
        self.cMotor_Velocity = 0 # 속도(RPM)은 정수형 데이터임
        self.cMotor_Position = 0.0 # same as Angle value / 이 클래스 코드에서 Position은 모터의 내부 영점 기준으로 절대적인 각도(위치) 값을 나타낸 것으로 소수점 이하 값이 가능한 데이터임

        self.cMotor_Coordinates = motor_Coordinates # 차량의 바퀴 좌표값
        '''
        설명: 로봇 상단(윗면)에서 로봇 정면이 위를 바라보도록 수직으로 바라보았을 때,
        차체의 회전 중심점(TurningPoint)과 차체 중심수직선(CenterLine) 교차하는 점을 영점으로 두고,
        차체의 우상단 방면을 수직좌표계와 같이 x, y 모두 양수로 보고, 해당 값을 받아서 바퀴의 위치를 대략적으로 구하는 인자이다.
        인자 자료형은 list[int, int] (예시: [1, 1] 또는 [0, -1] 등)이며, 값은 좌표에 맞게 0(축 위의 있음) 또는 ±1 이다.
        '''


    ##### 계산 처리 함수(메서드) 부분 #####
    
    # CRC8(Cyclic redundancy check)을 계산하기 위한 함수 선언
    def calc_CRC8(self, HEX_data):
        # CRC8 table 출처: https://crccalc.com/?crc=&method=CRC-8/MAXIM-DOW&datatype=ascii&outtype=hex
        CRC8_MAXIM_table = (
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
            0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
            0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
            0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
            0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
            0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
            0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
            0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
            0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
            0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
            0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
            0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
            0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
            0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
            0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
            0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
            0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
        )
        # 매뉴얼의 "CRC-8/Maxim multinomial: x8 + x5 + x4 + 1" 다항식 부분(비트 계산식, 숫자는 각 비트의 자리, +1은 ON을 의미하는 것으로 추정.)은 참고 사항으로 위의 테이블로 대체됨.
        # 세부적인 내용은 다음을 참고: https://en.wikipedia.org/wiki/Cyclic_redundancy_check

        crc_value = 0x00 # CRC8 알고리즘을 계산하기 위한 초기값

        # HEX_data로 받은 데이터(component) 갯수만큼 XOR 연산(^)을 반복하여, 사전에 계산된 CRC8_MAXIM_table에서 인덱싱하여 crc_value에 다시 대입함.
        for component in HEX_data:
            crc_value = CRC8_MAXIM_table[crc_value ^ component] # 2진수(비트) XOR 연산
        
        return crc_value # 계산 완료된 crc 값을 반환
    

    # 16진수 값(정확히는 bytes 형식의 16진수, b'\x~~' 또는 b'~' 형태. 다만, int 형식의 자료가 담긴 list 자료형도 HEX_group으로 받을 수 있음. 또한, 데이터의 길이는 상관 없음.)을 받아서 각각을 나누어서 10진수로 변환하여 리스트에 저장하는 함수
    def HEXgroup_to_DECgroup(self, HEX_group):
        DEC_group = [DEC_data for DEC_data in HEX_group] # 읽어온 바이트 자료형(bytes) 데이터(raw_RX_data)는 파이썬의 기본 기능으로 인해 10진수 정수형으로 변환됨.

        return DEC_group # 리스트 형태로 10진수 값 반환(길이 상관 없음)
    

    # 위의 16진수 값을 10진수 값으로 변환한 값들 중, 나눠져 있는 두 값[ex: 전류, 속도, 각도(또는 위치)의 High bits와 Low bits]을 하나로 합쳐서 10진수로 나타냄.(모터의 해당 피드백 값을 사용자가 알아보기 위함.)
    def combine_two_DEC_fromHEX(self, first_DEC: int, second_DEC: int):
        # 10진수로 변환된 16진수 기반 값들을 다시 16진수로 바꿔준 후[hex()], 앞에 붙는 '0x' 형식을 빼고 두자리로 만들어서, 각 2개 자리를 합친 총 4개 자리 앞에 
        temp_combined_data = '0x'+hex(first_DEC)[2:].zfill(2)+hex(second_DEC)[2:].zfill(2)
        #print(temp_combined_data)
        combined_DEC_data = struct.unpack('>h', struct.pack('>H', int(temp_combined_data, 16)))[0]
        #print(combined_DEC_data)
        
        return combined_DEC_data
        
        
    # 10진수 값을 받아서 16진수로 변환하는 함수
    def DEC_to_HEX(self, DEC_data: int):
        # 음수를 처리하기 위해 struct 모듈을 사용하여 2의 보수를 취함.
        # 이하 코드 출처: 가제트 19기 GADGET팀 코드
        temp_HEX_data = format(struct.unpack('>H', struct.pack('>h', DEC_data))[0], 'X').zfill(4)
        # DEC_data를 '>h'형식을 통해 10진수 기반 데이터를 바이너리 데이터(b'\x~~)로 처리[pack()]하고, '>H'형식을 통해 처리된 바이너리 데이터를 다시 10진수 데이터로 처리[unpack()]함.
        # ['>': 빅 엔디언 - 쉽게 얘기해서, 가장 높은 자리의 수(큰 바이트)를 가장 왼쪽(메모리의 낮은 주소)에 저장함. / 'h': 부호 있는 2바이트(16비트) 정수형으로 처리함. / 'H': 부호 없는 2바이트(16비트) 정수형으로 처리함.]
        # [pack(): 바이너리 데이터로 변환 처리. / unpack(): 10진수 데이터로 변환 처리. unpack()의 경우, 튜플 자료형으로 반환됨.]
        # format(~, 'X')는 받은 값(~)을 16진수 대문자 형식으로 표시하는 것 / .zfill(4)는 .의 앞에서 받은 값의 형식을 4자리로 고정하고 빈칸은 0으로 채워서 반환하는 것
        
        # 변환 및 처리된 16진수 값 4자리(2바이트)를 2자리(1바이트, 8비트)로 나누어 각각을 다시 10진수로 변환하여 각 자리에 저장함.
        HEX_High_data = int(temp_HEX_data[:2], 16) # High 8 bits
        HEX_Low_data = int(temp_HEX_data[2:], 16) # Low 8 bits

        return HEX_High_data, HEX_Low_data # 각 비트 값을 반환(두 개 이상의 반환값을 가지는 경우, 합쳐서 tuple 자료형으로 반환됨.)
    
    
    # calc_DEC_data_for_read 보조 함수들
    # 전류 제한 처리 함수
    def limit_current(self, current: float):

        if abs(current / DDSM_ENCODER_RESOLUTION) > self.cCurrent_Limit:
            limited_current = -self.cCurrent_Limit if current < 0 else self.cCurrent_Limit
            # print(f'Current is LIMITED to {limited_current}. Target current: {current} [A]')
            return limited_current
        
        return current
    

    # 속도 제한 처리 함수
    def limit_velocity(self, velocity: int):

        if abs(velocity) > self.cVelocity_Limit:
            limited_velocity = -self.cVelocity_Limit if velocity < 0 else self.cVelocity_Limit
            # print(f'Velocity is LIMITED to {limited_velocity}. Target velocity: {velocity} [RPM]')
            return limited_velocity
        
        return velocity
    

    # 10진수 실수(소수)형 값을 최대한 정확하게 받아서 각 모드에 맞게 값을 변환하는 함수(calc_mode: 0(기본값)인 경우에는 TX_data(모터로의 데이터 전송[각도를 예시로, 360도 -> 32767])용, 그렇지 않은 경우에는 RX_data(피드백 사용자 확인[각도를 예시로, 32767 -> 360도])용으로 쓰임.)
    def calc_DEC_data_for_read(self, data_mode: int, get_data_value: float, calc_mode: int = 0):
        
        # 모터 모드에 맞게 each_mode_value(나중에 값을 변환하는 계산식에서 사용)를 설정하는 if문
        if data_mode == 1: # Current loop mode is from -32767 to +32767 (INT16 bits) / from -8[A] to +8[A]. Minimum unit: 0.00025[A]
            each_max_value = self.CURRENT_MAX_VALUE # = 8
            temp_calc_value = self.limit_current(get_data_value) # 값에 따른 전류 제한 처리

            if calc_mode == 0: # TX_data일 때
                calc_DEC_data = math.floor(temp_calc_value / (each_max_value / DDSM_ENCODER_RESOLUTION)) # 입력 받은 전류 값(get_data_value)을, 8A(암페어)를 32767로 나눈 값으로 나누고, 그 값을 내림하여 정수로 만들어서 calc_DEC_data에 저장
            else: # RX_data일 때
                calc_DEC_data = round(temp_calc_value * (each_max_value / DDSM_ENCODER_RESOLUTION), 6) # 입력 받은 전류 값(get_data_value)을, 8A(암페어)를 32767로 나눈 값으로 '곱'하고, 그 값을 내림하여 정수로 만들어서 calc_DEC_data에 저장
        
        elif data_mode == 2: # Velocity loop mode is from -330 to +330 (INT16 bits) / from -330[RPM] to +330[RPM]. Minimum unit: 1[RPM]
            # each_max_value = DDSM_VELOCITY_MAX_VALUE # = 330, 속도는 이 줄이 의미 없음 - 32767로 처리하지 않기 때문임.
            calc_DEC_data = self.limit_velocity(round(get_data_value)) # 값에 따른 속도 제한 처리 후 바로 calc_DEC_data에 저장
        
        elif data_mode == 3: # Angle(Position) loop mode is from 0 to +32767 (Unsigned INT16 bits) / from 0[°] to 360[°]. Minimum unit: 0.01[°]
            each_max_value = DDSM_POSITION_MAX_VALUE # = 360

            if calc_mode == 0: # TX_data일 때
                calc_DEC_data = round(get_data_value / (each_max_value / DDSM_ENCODER_RESOLUTION)) # 입력 받은 각도(위치) 값(get_data_value)을, 360°(도)를 32767로 나눈 값으로 나누고, 그 값을 반올림하여 정수로 만들어서 calc_DEC_data에 저장
            else: # RX_data일 때
                calc_DEC_data = round(get_data_value * (each_max_value / DDSM_ENCODER_RESOLUTION), 2) # 입력 받은 각도(위치) 값(get_data_value)을, 360°(도)를 32767로 나눈 값으로 '곱'하고, 그 값을 반올림하여 정수로 만들어서 calc_DEC_data에 저장
        
        else:
            raise ValueError('모터의 구동 모드가 올바르지 않아, 코드를 종료합니다.')


        return calc_DEC_data
    
    ##### 모터 설정 및 구동 함수 부분 #####

    # 모터 내 버퍼 초기화 함수
    def init_motor_Buffer(self):
        self.cSerial.reset_input_buffer()
        self.cSerial.reset_output_buffer()


    # 모터로 데이터(Command Data)를 전송하는 함수
    def send_TX_data(self, TX_data):
        # 생성자로 만들어진 cSerial을 이용하여 데이터 전송. 다음을 참고함. https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial.write
        self.cSerial.write(TX_data)


    # 모터로부터 데이터(Feedback Data)를 받는 함수(본 함수는 송신 데이터 없이(함수 send_TX_data()가 사용되지 않은 경우), 단독으로 피드백 데이터를 받을 경우에 사용하기 위해 수정됨.)
    def receive_RX_data(self):
        
        ''', TX_data = None):
        # 이하 if문은 코드 작성 초기에 본 함수를 사용한 곳에서 자체적으로 TX_data를 보내지 못해 오류가 발생하는 것을 방지하기 위해 임시로 호환될 수 있게 구성함. 추후 코드 재작성 시 삭제 예정. ctrl+F 로 함수명을 찾아서 구조 고칠 것!
        if TX_data: # 초기 개발 구상과 달라지면서 발생한 함수의 호환성 문제를 위해, 기존의 TX_data가 input으로 있는 구조에서는 실행(TX_data != None) / 그렇지 않은 구조는 TX_data 없이 다른 함수인, communicate_DATA()를 사용함.
            self.send_TX_data(TX_data = TX_data) # 모터로부터 데이터를 받기 위해 정해진 형식의 데이터(매뉴얼 참고)를 보냄.'''
            
        # ID 체크를 위해 보낸 데이터에 의해 피드백 데이터가 오면 이를 읽어서 raw_RX_data에 저장.
        if self.cSerial.readable():
            raw_RX_data = self.cSerial.read(10) # read(10)은 10바이트를 읽어온다는 의미로, 해당 데이터는 모터가 바이트(bytes) 자료형(b'\x~~')으로 보냄.
            
            RX_data = self.HEXgroup_to_DECgroup(HEX_group = raw_RX_data) # 16진수를 10진수로 변환하여 RX_data로 저장.
            
            # RX_fault_value = bin(int(RX_data[8]))[2:].zfill(8) # 매뉴얼상의 fault value를 따로 받아서 저장. return 반환에서 쓸 용도였음.
            
            # print(raw_RX_data, RX_data) #출력 테스트용
            
            # RX_data가 비어 있지 않고 올바른 길이(10개 인덱스)로 존재하면 실행 
            if RX_data != [] and len(RX_data) == 10:
                if self.cOutput_for_User: # 인스턴스의 출력 여부가 True라면 실행
                    # 토크값, 속도값, 각도값을 각각 사용자가 알아볼 수 있게 변환해서 각 변수에 저장
                    motor_Torque_DEC = self.combine_two_DEC_fromHEX(RX_data[2], RX_data[3])
                    motor_Velocity_DEC = self.combine_two_DEC_fromHEX(RX_data[4], RX_data[5])
                    motor_Degree_DEC = self.combine_two_DEC_fromHEX(RX_data[6], RX_data[7])

                    #저장된 값을 이용하여 RX_data를 새로운 변수(combined_RX_data)에 재정의.(인덱스의 각 형식은 인터넷 검색으로 참고할 것.)
                    combined_RX_data = [RX_data[0], RX_data[1], 
                                        f'{self.calc_DEC_data_for_read(data_mode = 1, get_data_value = motor_Torque_DEC, calc_mode = 1):>9.6f}', 
                                        f'{self.calc_DEC_data_for_read(data_mode = 2, get_data_value = motor_Velocity_DEC, calc_mode = 1):>4d}', 
                                        f'{self.calc_DEC_data_for_read(data_mode = 3, get_data_value = motor_Degree_DEC, calc_mode = 1):7.2f}', 
                                        f'{RX_data[8]:08b}', f'{RX_data[9]:3d}']
                    # print(f'Motor {self.cMotor_ID}, Print output: {combined_RX_data}')

                    return RX_data, combined_RX_data # RX_data(기본적으로 반환하는 데이터)와 combined_RX_data를 반환
                else: # 인스턴스의 출력 여부가 False
                    return RX_data # 받은 데이터값만 반환


            # 모터로부터 받아온 값이 없어 읽을 값이 없을 때(경우1. 전원이 인가되지 않은 경우 / ...(알게 된 오류의 경우를 내용으로 추가하기))
            if RX_data == []:
                print('Check your connection with the motor(s), and Make sure the motor(s) is(are) turned on!') # 연결 여부, 전원 인가 여부 확인 안내
                raise ValueError('모터의 전원 또는 통신이 연결되어 있지 않아, 코드를 종료합니다.')
        
        else: # 통신 중에 모터로부터 값을 읽어오지 못한 경우.
            print('Failed to communicate with the motor(s)!') # 오류로 인해 시리얼 통신 중 값을 읽어오지 못할 경우에 출력.
            raise ValueError('모터와의 통신에 실패하여, 코드를 종료합니다.')
    

    # 양방향으로 데이터를 보내고 주고 받을 경우
    def communicate_DATA(self, TX_data):
        self.send_TX_data(TX_data = TX_data)
        RX_data = self.receive_RX_data()

        return RX_data
    

    # 기본 구동시 받을 수 있는 피드백 외의 다른 형식의 피드백(이하, ofdb)을 받게 하는 함수(매뉴얼상의 11쪽 상단, 2. Obtain other feedback 참고.)
    def get_other_Feedback_data(self, motor_ID: int):
        
        temp_TX_data_value = motor_ID, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 # ofdb을 받기 위한 데이터를 보내기 전, CRC8 값 이전까지 임시 저장
        feedback_CRC8 = self.calc_CRC8(HEX_data = temp_TX_data_value) # CRC8 값을 임시 저장한 값을 통해 계산
        
        TX_data_value = motor_ID, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, feedback_CRC8 # 계산한 CRC8 값을 포함하여 보낼 데이터 변수에 저장
        
        RX_data = self.communicate_DATA(TX_data = TX_data_value) # 데이터를 보내고 반환되는 피드백 데이터를 저장
        # print(RX_data) #출력 테스트용
        
        return RX_data # ofdb 값 반환
    

    # ofdb 값 중 권선 온도(또는 모터 고정자 온도. 쉽게 얘기해서 그냥 모터 온도로 생각하면 됨.)의 값을 확인하는 함수 -> 안전 문제를 위해 존재하는 기능으로 추정.
    def ofdb_Winding_Temperature(self, motor_ID: int):
        Temperature_data = self.get_other_Feedback_data(motor_ID = motor_ID)[6] # ofdb 값 중 권선 온도 부분을 인덱싱하여 저장
        print(f'Motor ID: {motor_ID} / Its Temperature: {Temperature_data} ℃[degree(s) Celsius]') # 확인용 출력
        
        return Temperature_data # 권선 온도 값 반환
    

    # ofdb 값 중 Fault(오류) 값을 확인하는 함수 -> 역시 안전 문제를 위해 존재하는 것으로 추정.
    def ofdb_Fault_value(self, motor_ID: int):
        Fault_data = bin(self.get_other_Feedback_data(motor_ID = motor_ID)[8])[2:].zfill(8) # ofdb 값 중 Fault value 부분을 인덱싱하여 저장
        print(f'Motor ID: {motor_ID} / Its Fault value: 0b{Fault_data}') # 확인용 출력
        
        # 이하는 매뉴얼 11쪽 하단의 Fault values 표를 참고하여, 여러 fault를 여러 개의 if문을 통해 동시에 검사할 수 있도록 함.
        # 첫 번째(인덱스 번호 0) 비트부터 세 번째(인덱스 번호 2) 비트까지는 지정된 오류가 없음.
        if int(Fault_data[3]): # 네 번째 비트가 1: 과열(또는 고장) fault
            print(f'The motor(ID No.{motor_ID}) has Overheat fault.')
        if int(Fault_data[4]): # 다섯 번째 비트가 1: 구동 꺼짐(*의역) fault
            print(f'The motor(ID No.{motor_ID}) has Stall fault.')
        if int(Fault_data[5]): # 여섯 번째 비트가 1: 위상(*의역) 과전류 fault
            print(f'The motor(ID No.{motor_ID}) has Phase over current fault.')
        if int(Fault_data[6]): # 일곱 번째 비트가 1: 버스(*모터를 동시에 연결하는 허브로 추정) 과전류 fault
            print(f'The motor(ID No.{motor_ID}) has Bus over current fault.')
        if int(Fault_data[7]): # 여덟 번째(마지막) 비트가 1: 센서 fault
            print(f'The motor(ID No.{motor_ID}) has Sensor fault.')
        if not int(Fault_data): # 어떠한 오류도 없는 경우, Fault_data = '00000000'이므로 이 if문을 실행.
            print(f'The motor(ID No.{motor_ID}) has not fault. Good!')
        
        return int(Fault_data)


    # 모터 ID를 확인하는 함수
    # (매뉴얼에 따르면 ID 확인 시, 무조건 한 개의 모터만을 연결할 것.)
    def check_ID(self):
        # cmd(TX) data의 CRC8 값은 다음을 이용하여 확인함. https://crccalc.com/?crc=0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00&method=CRC-8/MAXIM-DOW&datatype=hex&outtype=hex
        TX_data_value = 0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE # 매뉴얼 13쪽 ID check 참고.
        
        # ID 체크를 위해 보낸 데이터에 의해 피드백 데이터가 오면 이를 읽어서 출력
        RX_data_value = self.communicate_DATA(TX_data = TX_data_value) # 값을 보내고 피드백 데이터 저장
        print(f'The motor\'s ID: {RX_data_value[0]}') # 확인용 출력
        
        return RX_data_value[0] # 피드백 데이터 중 ID 값만 반환
        
    
    # 모터 ID를 설정하는 함수
    # (매뉴얼에 따르면 ID 설정 시, 무조건 한 개의 모터만을 연결하고, 정확한 설정을 위해 모터의 전원을 차단하였다가 재인가하여 초기(초기화 상태)에 설정할 것.)
    def set_ID(self, motor_ID: int):
        # 지정된 형식으로 command(TX, 전송) data를 입력하여 모터의 ID를 설정
        TX_data_value = 0xAA, 0x55, 0x53, motor_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 # 매뉴얼 12쪽 motor ID setting 참고.
        # 매뉴얼 12쪽 하단 내용에 의거하여, 위의 cmd data를 5번 반복 전송하여 ID 설정을 완료함.
        for repeat in range(5):
            print(f'Setting ID now... Trying {repeat} time(s).') # 선택 사항
            # 위의 출려은 선택 사항(작성자 편의)으로, 하단에 전송 함수로 보내서 ID 설정을 완료함.
            self.send_TX_data(TX_data_value) # 필수 사항
        
        RX_motor_ID = self.check_ID()
        print(f'Set the ID to {RX_motor_ID}.') # 확인용 출력(선택 사항)
    

    # 모터 모드 설정 함수
    def set_Mode(self, motor_ID: int, motor_Mode: int):
        TX_data_value = motor_ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, motor_Mode
        self.send_TX_data(TX_data_value)
        # if motor_Mode == 3 and self.communicate_DATA('속도값이 10RPM 이상이면 안 되게 하려고 피드백 받는 중 / 코드 작성해! / 참고로, DEC_to_HEX를 역으로 이용하려는 생각임.'):
        #     # 여기도 만들어야 함 250328
            
        #     pass
    
    
    # 전류 모드로 모터를 제어하는 함수
    def Operate_Motor_Current_Mode(self, motor_ID: int, raw_motor_Current: float = 0):  #인터페이스 설정정
        
        motor_Current = self.calc_DEC_data_for_read(data_mode = 1, get_data_value = raw_motor_Current, calc_mode = 0) #calc_mode: 0(기본값)인 경우에는 TX_data(모터로의 데이터 전송)
        #전송할 데이터를 DEC(십진수)로 변환함함
        Speed_High_value, Speed_Low_value = self.DEC_to_HEX(motor_Current)
        temp_TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00

        CRC8_value = self.calc_CRC8(temp_TX_data_value)
        TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8_value
        
        # for repeat in range(5): #5번 반복 송신신
        #     RX_data = self.communicate_DATA(TX_data = TX_data_value)   #여기 RX_data값을 받고 원래 안쓸 생각인건지 물어봐야함
        #     self.cMotor_Current = motor_Current

        self.communicate_DATA(TX_data = TX_data_value)   #여기 RX_data값을 받고 원래 안쓸 생각인건지 물어봐야함
        self.cMotor_Current = motor_Current
    
    
    # 속도 모드로 모터를 제어하는 함수
    def Operate_Motor_Velocity_Mode(self, motor_ID: int, raw_motor_Velocity: float = 0):

        motor_Velocity = self.calc_DEC_data_for_read(data_mode = 2, get_data_value = raw_motor_Velocity, calc_mode = 0)

        Speed_High_value, Speed_Low_value = self.DEC_to_HEX(motor_Velocity)
        temp_TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00
        
        CRC8_value = self.calc_CRC8(temp_TX_data_value)
        TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8_value
        
        # for repeat in range(5):
        #     RX_data = self.communicate_DATA(TX_data = TX_data_value)
        #     self.cMotor_Velocity = motor_Velocity

        self.communicate_DATA(TX_data = TX_data_value)
        self.cMotor_Velocity = motor_Velocity
    
    
    # 각도 모드에서 절대 위치를 기준으로 모터 각을 제어하는 함수
    def Operate_Motor_Abs_Pos_Mode(self, motor_ID: int, raw_motor_Position: float = 0):
        motor_Position = self.calc_DEC_data_for_read(data_mode = 3, get_data_value = raw_motor_Position, calc_mode = 0)

        Speed_High_value, Speed_Low_value = self.DEC_to_HEX(motor_Position)
        temp_TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00
        
        CRC8_value = self.calc_CRC8(temp_TX_data_value)
        TX_data_value = motor_ID, 0x64, Speed_High_value, Speed_Low_value, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8_value
        
        # for repeat in range(5):
        #     RX_data = self.communicate_DATA(TX_data = TX_data_value)
        #     self.cMotor_Position = motor_Position

        self.communicate_DATA(TX_data = TX_data_value)
        self.cMotor_Position = motor_Position
    

    # 각도 모드에서 상대 위치로 모터 각을 제어(절대 위치로 비유한다면, 제어 당시의 모터의 현위치가 영점이 되는 방식.)하는 함수
    def Operate_Motor_Add_Angle_Mode(self, motor_ID: int):   #모터 현재위치를 받아 제어하려면 메뉴얼의 어떤 SETTING을 봐야할지 감이 안잡함... 이것도 각도모드인지 확인 후 실행되도록 해야하는건가..
        pass
    
    
    # 모터 정지 함수, 
    def Brake_Motor(self, motor_ID: int):   
        #valid in speed loop mode라는데 이건 Velocity모드인가??-> 무슨 모드인지 확인이 필요할까? set_Mode함수를 완성시켜서 mode를 확인하고 0x02모드일 때만 실행할 수  있는 조건을 추가할까?
        front_TX_data_value = motor_ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

        CRC8_value = self.calc_CRC8(front_TX_data_value)
        TX_data_value = motor_ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, CRC8_value

        self.communicate_DATA(TX_data=TX_data_value)