# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus

CHIP_ID = 0x75 << 1
REG_ID_1 = 0x00
REG_ID_2 = 0x01

class PAT9125:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=CHIP_ID, default_speed=100000)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
    def read_register(self,register_id):
        return self.i2c.i2c_read(register_id, 1)
    def cmd_SENSOR_READ_ID(self, params):
        pr1 = self.i2c.i2c_read([REG_ID_1], 1)
        pr2 = self.i2c.i2c_read([REG_ID_2], 1)
        pid1 = bytearray(pr1['response'])
        pid2 = bytearray(pr2['response'])
        e_resp = [0x31, 0x91]
        self.gcode.respond_info(
            "Sensor Test: Read [%#x,%#x], Expected [%#x,%#x]"
            % (pid1[0], pid2[0], e_resp[0], e_resp[1]))

def load_config(config):
    return PAT9125(config)
