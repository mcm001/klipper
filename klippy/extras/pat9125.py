# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging

CHIP_ID = 0x75 << 1
REG_ID_1 = 0x00
REG_ID_2 = 0x01

PAT9125_DELTA_XL = 0x03
PAT9125_DELTA_YL = 0x04
PAT9125_SHUTTER	= 0x14
PAT9125_FRAME = 0x17

class PAT9125:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=CHIP_ID, default_speed=100000)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
        self.gcode.register_command(
            "SENSOR_READ_INFO", self.cmd_SENSOR_READ_INFO
        )
    def cmd_SENSOR_READ_ID(self, params):
        pr1 = self.i2c.i2c_read([REG_ID_1], 1)
        pr2 = self.i2c.i2c_read([REG_ID_2], 1)
        pid1 = bytearray(pr1['response'])
        pid2 = bytearray(pr2['response'])
        logging.info("response example")
        logging.info(pr1)
        e_resp = [0x31, 0x91]
        self.gcode.respond_info(
            "Sensor Test: Read [%#x,%#x], Expected [%#x,%#x]"
            % (pid1[0], pid2[0], e_resp[0], e_resp[1]))
    def read_register(self, register_ID):
        logging.info("reading contents of register " + register_ID)
        responce = self.i2c.i2c_read([register_ID], 1)
        return responce

    def cmd_SENSOR_READ_INFO(self,params):
            shutter = self.read_register(PAT9125_SHUTTER)
            frame = self.read_register(PAT9125_FRAME)
            dx = self.read_register(PAT9125_DELTA_XL)
            dy = self.read_register(PAT9125_DELTA_XL)
            self.gcode.respond_info("PAT9125 INFO: Shutter: %s Frame: %s Delta X: %i Delta Y: %i" % (shutter, frame, dx, dy))
            # self.gcode.respond_info("Filament Movement: ")

def load_config(config):
    return PAT9125(config)
