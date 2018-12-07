# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging



class PAT9125:
    def __init__(self, config):
        self.CHIP_ID = 0x75 << 1
        self.REG_ID_1 = 0x00
        self.REG_ID_2 = 0x01
        self.PAT9125_PRODUCT_ID = [0x31, 0x91]
        self.PAT9125_DELTA_XL = 0x03
        self.PAT9125_DELTA_YL = 0x04
        self.PAT9125_SHUTTER	= 0x14
        self.PAT9125_FRAME = 0x17

        self.printer = config.get_printer()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=self.CHIP_ID, default_speed=100000)
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
        self.gcode.register_command(
            "SENSOR_READ_INFO", self.cmd_SENSOR_READ_INFO
        )
        self.gcode.register_command("SENSOR_CHECK_ACTIVE", self.cmd_SENSOR_CHECK_ACTIVE)

        # check if the PAT is active
        self.pat9125_active = False
        self._init_pat9125       
    def _init_pat9125(self):
        PID_read1 = self.i2c.i2c_read([self.REG_ID_1], 1)
        PID_read2 = self.i2c.i2c_read([self.REG_ID_2], 1)
        PID_array1 = bytearray(PID_read1['response'])
        PID_array2 = bytearray(PID_read2['response'])
        product_id = self.PAT9125_PRODUCT_ID

        
        self.gcode.respond_info(
        "Sensor Test: Read [%#x,%#x], Expected [%#x,%#x]"
        % (PID_read1, PID_read2, self.PAT9125_PRODUCT_ID[0], self.PAT9125_PRODUCT_ID[1]))       
        
        logging.info("Sensor test on init: read[%#x,%#x], Expected [%#x,%#x]"
        % (PID_read1, PID_read2, self.PAT9125_PRODUCT_ID[0], self.PAT9125_PRODUCT_ID[1]))       
        
        
        if (PID_array1 != product_id[0]) or (PID_array2 != product_id[1]): # if product ID does not match
            self.gcode.respond_error("ERROR on _init_pat9125: Product ID and Read ID do not match! Read [%#x,%#x], Expected [%#x,%#x]"
            % (PID_array1, PID_array2, product_id[0], product_id[1]))
            logging.info("ERROR on _init_pat9125: PAT9125 product ID and read ID do not match! Read [%#x,%#x], Expected [%#x,%#x]" % (PID_array1, PID_array2, product_id[0], product_id[1]))

            self.pat9125_active = False
        else:
            self.pat9125_active = True
            self.gcode.respond_info("PAT Active!")
            logging.info("INTIT PASSED - PAT active! Current state: %s" % self.pat9125_active)

    def cmd_SENSOR_READ_ID(self, params):
        pr1 = self.i2c.i2c_read([self.REG_ID_1], 1)
        pr2 = self.i2c.i2c_read([self.REG_ID_2], 1)
        pid1 = bytearray(pr1['response'])
        pid2 = bytearray(pr2['response'])
        logging.info("response example")
        logging.info(pr1)
        self.gcode.respond_info(
            "Sensor Test: Read [%#x,%#x], Expected [%#x,%#x]"
            % (pid1[0], pid2[0], self.PAT9125_PRODUCT_ID[0], self.PAT9125_PRODUCT_ID[1]))
    
    def check_pat_active(self):
        read_ID = [ self.read_register(self.PAT9125_PRODUCT_ID[0]), self.read_register(self.PAT9125_PRODUCT_ID[1]) ]
        product_id = self.PAT9125_PRODUCT_ID

        if (read_ID[0] != product_id[0]) or (read_ID[1] != product_id[1]): # if product ID does not match
            self.gcode.respond_error("ERROR on check_pat_active: Product ID and Read ID do not match! Read [%#x,%#x], Expected [%#x,%#x]"
            % (read_ID[0], read_ID[1], product_id[0], product_id[1]))
            logging.info("ERROR on check_pat_active: PAT9125 product ID and read ID do not match! Read [%#x,%#x], Expected [%#x,%#x]"     % (read_ID[0], read_ID[1], product_id[0], product_id[1]))
            self.pat9125_active = False

        else:
            self.pat9125_active = True
            self.gcode.respond_info("PAT Active!")
            logging.info("PAT active! Current state: %s" % self.pat9125_active)

    def read_register(self, register_ID):
        if self.pat9125_active is True:
            logging.info("PAT is active - reading contents of register " + str(register_ID))
            response = self.i2c.i2c_read([register_ID], 1)
            response2 = bytearray(response['response'])
            logging.info("Response recieved from register %i is %s" % (register_ID, response2))
            return response2
        else:
            self.gcode.respond_error("ERROR on read_register: PAT9125 is not set active! Therefore I cannot read the register. Try reading again with SENSOR_CHECK_ACTIVE")
            logging.error("ERROR on read_register: Cannot read register, as PAT is not active. Try checking pat active again with SENSOR_CHECK_ACTIVE")
            return None

    def cmd_SENSOR_CHECK_ACTIVE(self,params):
        self.check_pat_active
        self.gcode.respond_info("Current PAT status: %s" % self.pat9125_active)
        if self.pat9125_active is False:
            logging.error("ERROR on cmd_SENSOR_CHECK_ACTIVE: PAT is not active! Not sure why... check sensor read id for more info")
            self.gcode.respond_error("ERROR on cmd_SENSOR_CHECK_ACTIVE: PAT is not active!")
        elif self.pat9125_active is True:
            logging.info("PAT is active")
            self.gcode.respond_info("Check passed - PAT is active!")

    def cmd_SENSOR_READ_INFO(self,params):
            shutter = self.read_register(self.PAT9125_SHUTTER)
            frame = self.read_register(self.PAT9125_FRAME)
            dx = self.read_register(self.PAT9125_DELTA_XL)
            dy = self.read_register(self.PAT9125_DELTA_YL)

            self.check_pat_active # update active value - TODO how often do we need to do this?
            if (self.pat9125_active is True) and (shutter is not None) and (frame is not None) and (dx is not None) and (dy is not None):
                self.gcode.respond_info("PAT9125 INFO: Shutter: %s Frame: %s Delta X: %i Delta Y: %i" % (shutter, frame, dx, dy))
            # self.gcode.respond_info("Filament Movement: ")
            else:
                self.gcode.respond_info("ERROR on cmd_SENSOR_READ_INFO: PAT is not enabled, or register read returned None! Cannot read info")
                logging.error("ERROR on cmd_SENSOR_READ_INFO: PAT is not enabled, or register read returned None! Cannot read info")

def load_config(config):
    return PAT9125(config)
