# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging

CHIP_ID = 0x75 << 1
PAT9125_PRODUCT_ID = (0x91 << 8) | 0x31
PAT9125_XRES = 0
PAT9125_YRES = 240

PAT9125_REGS = {
    'PID1': 0x00, 'PID2': 0x01, 'MOTION': 0x02,
    'DELTA_XL': 0x03, 'DELTA_YL': 0x04, 'MODE': 0x05,
    'CONFIG': 0x06, 'WP': 0x09, 'SLEEP1': 0x0a,
    'SLEEP2': 0x0b, 'RES_X': 0x0d, 'RES_Y': 0x0e,
    'DELTA_XYH': 0x12, 'SHUTTER': 0x14, 'FRAME': 0x17,
    'ORIENTATION': 0x19, 'BANK_SELECTION': 0x7f
}

PAT9125_INIT1 = [
    (PAT9125_REGS['WP'], [0x5a]),
    (PAT9125_REGS['RES_X'], [PAT9125_XRES]),
    (PAT9125_REGS['RES_Y'], [PAT9125_YRES]),
    (PAT9125_REGS['ORIENTATION'], [0x04]),
    (0x5e, [0x08]),
    (0x20, [0x64]),
    (0x2b, [0x6d]),
    (0x32, [0x2f])
]

PAT9125_INIT2 = [
    (0x06, [0x28]),
    (0x33, [0xd0]),
    (0x36, [0xc2]),
    (0x3e, [0x01, 0x15]),
    (0x41, [0x32, 0x3b, 0xf2, 0x3b, 0xf2, 0x22, 0x3b, 0xf2, 0x3b, 0xf0]),
    (0x58, [0x98, 0x0c, 0x08, 0x0c, 0x08]),
    (0x61, [0x10]),
    (0x67, [0x9b]),
    (0x6e, [0x22]),
    (0x71, [0x07, 0x08])
]

def log_byte_array(header, array):
    msg = header + ": ["
    for b in array:
        msg += "%#x, " % b
    msg = msg[:-2] + "]"
    logging.info(msg)

class PAT9125:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=CHIP_ID, default_speed=100000)
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.kinematics = None
        self.runout_callback = None
        self.initialized = False
        self.is_autoload = False
        self.product_id = 0
        self.refresh_time = .2
        self.pat9125_timer = self.reactor.register_timer(
            self._pat9125_update_event)
        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
        self.gcode.register_command(
            'PAT_TEST_INIT', self.cmd_PAT_TEST_INIT)
    def printer_state(self, state):
        if state == 'ready':
            self.toolhead = self.printer.lookup_object('toolhead')
            self.kinematics = self.toolhead.get_kinematics()
            # XXX - after testing check init gcode, this
            # can be uncommented to automatically
            # initialize after tool is ready
            # self._pat9125_init()
    def read_register(self, reg, read_len):
        # return data from from a register. reg may be a named
        # register, an 8-bit address, or a list containing the address
        if type(reg) is str:
            regs = [PAT9125_REGS[reg]]
        elif type(reg) is list:
            regs = reg
        else:
            regs = [reg]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])
    def write_register(self, reg, data, minclock=0, reqclock=0):
        # Write data to a register. Reg may be a named
        # register or an 8-bit address.  Data may be a list
        # of 8-bit values, or a single 8-bit value
        if type(data) is not list:
            data = [data & 0xFF]
        if type(reg) is str:
            data.insert(0, PAT9125_REGS[reg])
        else:
            data.insert(0, reg)
        self.i2c.i2c_write(data, minclock, reqclock)
    def set_runout_callback(self, callback):
        self.runout_callback = callback
    def start_sample_timer(self):
        self.reactor.update_timer(self.pat9125_timer, self.reactor.NOW)
    def stop_sample_timer(self):
        self.reactor.update_timer(self.pat9125_timer, self.reactor.NEVER)
    def _pat9125_init(self):
        mcu = self.i2c.get_mcu()
        pid = self.read_register('PID1', 2)
        self.product_id = (pid[1] << 8) | pid[0]

        # Read and verify product ID
        if self.product_id != PAT9125_PRODUCT_ID:
            pid = self.read_register('PID1', 2)
            self.product_id = (pid[1] << 8) | pid[0]
            if self.product_id != PAT9125_PRODUCT_ID:
                logging.info(
                    "Product ID Mismatch Expected: %d, Recd: %d"
                    % PAT9125_PRODUCT_ID, self.product_id)
                self.initialized = False
                return
        self.write_register('BANK_SELECTION', 0x00)
        self.write_register('CONFIG', 0x97)

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + 1.)
        self._send_init_sequence(PAT9125_INIT1, minclock)

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .01)
        self.write_register('BANK_SELECTION', 0x01, minclock=minclock)
        self._send_init_sequence(PAT9125_INIT2)

        self.write_register('BANK_SELECTION', 0x00)
        self.write_register('WP', 0x00)
        pid = self.read_register('PID1', 2)

        if self.product_id != ((pid[1] << 8) | pid[0]):
            logging.info(
                "Product ID Mismatch Expected: %d, Recd: %d"
                % self.product_id, ((pid[1] << 8) | pid[0]))
            self.initialized = False
            return

        self.write_register('RES_X', PAT9125_XRES)
        self.write_register('RES_Y', PAT9125_YRES)
        self.initialized = True
        logging.info("PAT9125 Initialiation Success")
    def _send_init_sequence(self, sequence, delay=0):
        for addr, data in sequence:
            if delay:
                self.write_register(addr, data, minclock=delay)
                delay = 0
            else:
                self.write_register(addr, data)
            r_data = self.read_register(addr, len(data))
            for w_byte, r_byte in zip(data, r_data):
                if w_byte != r_byte:
                    logging.info(
                        "PAT9125 Read/Write mismatch, register (%#x)"
                        % (addr))
                    log_byte_array("Written bytes", data)
                    log_byte_array("Read bytes   ", r_data)
                    return False
        return True
    def _pat9125_update_event(self, eventtime):
        # XXX - put sampling code here!
        # When it is determined that filament has run out, execute the
        # callback, ie:
        # If self.runout_callback is not None:
        #   self.runout_callback()
        return eventtime + self.refresh_time
    def cmd_SENSOR_READ_ID(self, params):
        product_id = self.read_register('PID1', 2)
        self.gcode.respond_info(
            "PAT9125 ID: [%#x,%#x]" % (product_id[0], product_id[1]))
    def cmd_PAT_TEST_INIT(self, params):
        self._pat9125_init()
        if self.initialized:
            msg = "PAT9125 Successfully Initialized"
        else:
            msg = "PAT9125 Initialization failure, check klippy.log"
        self.gcode.respond_info(msg)

def load_config(config):
    return PAT9125(config)
