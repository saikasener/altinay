# -*- coding: utf-8 -*-
"""
Written By: Rasit EVDUZEN
Created on Mon Nov 14 09:41:01 2022


"""

import pysoem
import time
import ctypes


synapt = None


class InputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('statusword', ctypes.c_uint16),
        ('modes_of_operation_display', ctypes.c_int8),
        ('position_actual_value', ctypes.c_int32),
        ('velocity_actual_value', ctypes.c_int32),
        ('torque_actual_value', ctypes.c_int32),
        ('analog_input_1', ctypes.c_uint16),
        ('analog_input_2', ctypes.c_uint16),
        ('analog_input_3', ctypes.c_uint16),
        ('analog_input_4', ctypes.c_uint16),
        ('tuning_status', ctypes.c_uint32),
        ('digital_inputs', ctypes.c_uint32),
        ('user_miso', ctypes.c_uint32),
        ('timestamp', ctypes.c_uint32),
        ('position_demand_internal_value', ctypes.c_int32),
        ('velocity_demand_value', ctypes.c_int32),
        ('torque_demand', ctypes.c_int16),
    ]

class OutputPdo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ('controlword', ctypes.c_uint16),
        ('modes_of_operation', ctypes.c_int8),
        ('target_torque', ctypes.c_int32),
        ('target_position', ctypes.c_int32),
        ('target_velocity', ctypes.c_int32),
        ('torque_offset', ctypes.c_int16),
        ('tuning_command', ctypes.c_uint32),
        ('physical_outputs', ctypes.c_uint32),
        ('bit_mask', ctypes.c_uint32),
        ('user_mosi', ctypes.c_uint32),
        ('velocity_offset', ctypes.c_int32),
    ]


modes_of_operation = {
    'No mode': 0,
    'Profile position mode': 1,
    'Profile velocity mode': 3,
    'Homing mode': 6,
    'Cyclic synchronous position mode': 8,
    'Cyclic synchronous velocity mode': 9,
    'Cyclic synchronous torque mode': 10,
}


def convert_input_data(data):
    return InputPdo.from_buffer_copy(data)


def main():
    global synapt
    master = pysoem.Master()
    master.open('\\Device\\NPF_{807F3D20-ED63-4912-9E8F-186FF87162A6}')  # someting like '\\Device\\NPF_{B4B7A38F-7DEB-43AF-B787B7-EABADE43978EA}' under Windows
    if master.config_init() > 0:
        synapt = master.slaves[0]
        master.config_map()
        if master.state_check(pysoem.SAFEOP_STATE, 50_000) == pysoem.SAFEOP_STATE:
            master.state = pysoem.OP_STATE
            master.write_state()
            master.state_check(pysoem.OP_STATE, 5_000_000)
            if master.state == pysoem.OP_STATE:
                output_data = OutputPdo()
                output_data.modes_of_operation = modes_of_operation['Cyclic synchronous velocity mode']
                output_data.target_velocity = 100  # RPM
                for control_cmd in [6, 7, 15]:
                    output_data.controlword = control_cmd
                    synapt.output = bytes(output_data)  # that is the actual change of the PDO output data
                    master.send_processdata()
                    master.receive_processdata(1_000)
                    time.sleep(0.01)
                try:
                    while 1:
                        master.send_processdata()
                        master.receive_processdata(1_000)
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    print('stopped')
                # zero everything
                synapt.output = bytes(len(synapt.output))
                master.send_processdata()
                master.receive_processdata(1_000)
            else:
                print('failed to got to op state')
        else:
            print('failed to got to safeop state')
        master.state = pysoem.PREOP_STATE
        master.write_state()
    else:
        print('no device found')
    master.close()


if __name__ == '__main__':
    main()