from BLDC_motor_ctrl_class import BLDC_motor_CMD
import time

motor_test = BLDC_motor_CMD(motor_USBport_num = 0, motor_setting_ID = 1)
motor_handle = BLDC_motor_CMD(motor_USBport_num = 3, motor_setting_ID = 0)


# motor_test.check_ID()
# fdb_data = motor_test.receive_RX_data((0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE))
# cpnt = fdb_data[0]
# print(type(fdb_data), type(cpnt), fdb_data, cpnt)
# fdb = motor_test.Winding_Temperature(1)
# print(fdb)




# print(motor_test.HEXgroup_to_DECgroup(b'A'))
# print(motor_test.DEC_to_HEX(-1))
# print(motor_test.HEXgroup_to_DECgroup(b'\xf0\x99'))

# sd = 12345.3

# motor_test.calc_DEC_for_mode(int(input('Mode: ')), float(input('Value: ')))

# int_part, deci_part = motor_test.calc_DC_for_mode(1, str(sd))


# motor_test.DEC_to_HEX(int_part, deci_part)
# print(type(int_part), type(deci_part))

# print(motor_test.DEC_to_HEX(0), hex(-32767))

# motor_test.combine_two_DEC_fromHEX(255, 255)

# motor_test.set_ID(1)
# motor_test.check_ID()

# motor_handle.set_ID(motor_handle)
# motor_handle.check_ID()


motor_test.init_motor_Buffer()
motor_handle.init_motor_Buffer()

motor_test.cOutput_printing = True
motor_handle.cOutput_printing = True

a = 0
t = 0.05
'''
print(motor_handle.cMotor_ID)

angle = motor_handle.get_other_Feedback_data(motor_ID = motor_handle.cMotor_ID)[4]
print(angle, motor_handle.get_other_Feedback_data(motor_ID = motor_handle.cMotor_ID))



for repeat in range(1):
    while a <= 50:
        motor_test.Operate_Motor_Velocity_Mode(motor_test.cMotor_ID, a)
        motor_handle.Operate_Motor_Velocity_Mode(motor_handle.cMotor_ID, -a)
        time.sleep(t)
        a += 1


    while a >= -50:
        motor_test.Operate_Motor_Velocity_Mode(motor_test.cMotor_ID, a)
        motor_handle.Operate_Motor_Velocity_Mode(motor_handle.cMotor_ID, -a)
        time.sleep(t)
        a -= 1

while a <= 0:
    motor_test.Operate_Motor_Velocity_Mode(motor_test.cMotor_ID, a)
    motor_handle.Operate_Motor_Velocity_Mode(motor_handle.cMotor_ID, -a)
    time.sleep(t)
    a += 1

# '''

'''
motor_handle.send_TX_data(TX_data = (motor_handle.cMotor_ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03))

high_180, low_180 = motor_handle.DEC_to_HEX(180)
temp_tx_value_init = motor_handle.cMotor_ID, 0x64, high_180, low_180, 0x00, 0x00, 0x00, 0x00, 0x00

CRC8_value_init = motor_handle.calc_CRC8(temp_tx_value_init)
tx_value_init = motor_handle.cMotor_ID, 0x64, high_180, low_180, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8_value_init

time.sleep(1)

motor_test.receive_RX_data(TX_data = tx_value_init, output_printing = motor_test.cOutput_printing)

time.sleep(1)
'''


motor_handle.send_TX_data(TX_data = (motor_handle.cMotor_ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01))

temp_tx_value = motor_handle.cMotor_ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

CRC8_value = motor_handle.calc_CRC8(temp_tx_value)
tx_value = motor_handle.cMotor_ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8_value



handle_angle = int(0)

while abs(float(motor_test.get_other_Feedback_data(motor_ID = motor_test.cMotor_ID)[1][2].strip())) <= 0.42:
    handle_angle = round((float(motor_handle.communicate_DATA(TX_data = tx_value)[1][4].strip()) - 180) * (100 / 180))
    print(f'{round(float(motor_handle.communicate_DATA(TX_data = tx_value)[1][4].strip()), 0) - 180:>6} / {handle_angle:>4} / {abs(float(motor_test.get_other_Feedback_data(motor_ID = motor_test.cMotor_ID)[1][2].strip())):>.4f}')
    motor_test.Operate_Motor_Velocity_Mode(motor_test.cMotor_ID, -handle_angle)
    

print('External force is detected. EXIT!')
motor_test.Operate_Motor_Velocity_Mode(motor_test.cMotor_ID, 0)
motor_handle.Operate_Motor_Velocity_Mode(motor_handle.cMotor_ID, 0)

# '''