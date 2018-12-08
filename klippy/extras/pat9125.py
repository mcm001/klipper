# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import bus
import logging

CHIP_ADDR = 0x75 << 1
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
            config, default_addr=CHIP_ADDR, default_speed=400000)
        self.gcode = self.printer.lookup_object('gcode')
        self.watchdog = WatchDog(config, self)
        self.initialized = False
        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
        self.gcode.register_command(
            'PAT_TEST_INIT', self.cmd_PAT_TEST_INIT)
        self.gcode.register_command(
            'PAT_READ_REG', self.cmd_PAT_READ_REG)
    def printer_state(self, state):
        if state == 'ready':
            self.watchdog.watchdog_init()
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
            regs = list(reg)
        else:
            regs = [reg]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])
    def write_register(self, reg, data, minclock=0, reqclock=0):
        # Write data to a register. Reg may be a named
        # register or an 8-bit address.  Data may be a list
        # of 8-bit values, or a single 8-bit value
        if type(data) is not list:
            out_data = [data & 0xFF]
        else:
            out_data = list(data)

        if type(reg) is str:
            out_data.insert(0, PAT9125_REGS[reg])
        elif type(reg) is list:
            out_data = reg + out_data
        else:
            out_data.insert(0, reg)
        self.i2c.i2c_write(out_data, minclock, reqclock)
    def set_runout_callback(self, callback):
        self.watchdog.set_runout_callback(callback)
    def _pat9125_init(self):
        mcu = self.i2c.get_mcu()
        self.initialized = False

        # Read and verify product ID
        if not self.check_product_id():
            logging.info("Rechecking ID...")
            if not self.check_product_id():
                return

        self.write_register('BANK_SELECTION', 0x00)

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .001)
        self.write_register('CONFIG', 0x97, minclock=minclock)
        minclock = mcu.print_time_to_clock(print_time + .002)
        self.write_register('WP', 0x5a, minclock=minclock)
        wp = self.read_register('WP', 1)[0]
        if wp != 0x5a:
            logging.info("Error disabling write protection")
            return

        if not (self._send_init_sequence(PAT9125_INIT1)):
            return

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .01)
        self.write_register('BANK_SELECTION', 0x01, minclock=minclock)
        if not self._send_init_sequence(PAT9125_INIT2):
            return

        self.write_register('BANK_SELECTION', 0x00)
        self.write_register('WP', 0x00)

        if not self.check_product_id():
            return

        self.write_register('RES_X', PAT9125_XRES)
        self.write_register('RES_Y', PAT9125_YRES)
        self.initialized = True
        logging.info("PAT9125 Initialization Success")
    def _send_init_sequence(self, sequence):
        for addr, data in sequence:
            self.write_register(addr, data)
            r_data = self.read_register(addr, len(data))
            for w_byte, r_byte in zip(data, r_data):
                if w_byte & 0xFF != r_byte & 0xFF:
                    logging.info(
                        "PAT9125 Read/Write mismatch, register (%#x)"
                        % (addr))
                    log_byte_array("Written bytes", data)
                    log_byte_array("Read bytes   ", r_data)
                    return False
        return True
    def check_product_id(self):
        pid = self.read_register('PID1', 2)
        if ((pid[1] << 8) | pid[0]) != PAT9125_PRODUCT_ID:
            logging.info(
                "Product ID Mismatch Expected: %d, Recd: %d"
                % PAT9125_PRODUCT_ID, ((pid[1] << 8) | pid[0]))
            return False
        return True
    def pat9125_update(self):
        # XXX - set option to retreive response time in read_register
        # read data from 0x00 to 0x17
        data = self.read_register('PID1', 24)
        if ((data[1] << 8) | data[0]) != PAT9125_PRODUCT_ID:
            logging.info(
                "Product ID Mismatch Expected: %d, Recd: %d"
                % PAT9125_PRODUCT_ID, ((data[1] << 8) | data[0]))
            self.initialized = False
            return None
        # XXX - extract values
        results = {
            'MOTION': 0, 'DELTA_XL': 0, 'DELTA_YL': 0, 'DELTA_XYH': 0,
            'FRAME': 0, 'SHUTTER': 0}
        for key in results:
            results[key] = data[PAT9125_REGS[key]]
        results['MCU_TIME'] = 0
        return results
    def cmd_SENSOR_READ_ID(self, params):
        product_id = self.read_register('PID1', 2)
        self.gcode.respond_info(
            "PAT9125 ID: [%#x,%#x]" % (product_id[0], product_id[1]))
    def cmd_PAT_TEST_INIT(self, params):
        logging.info("Attempting Init Test")
        self._pat9125_init()
        if self.initialized:
            msg = "PAT9125 Successfully Initialized"
        else:
            msg = "PAT9125 Initialization failure, check klippy.log"
        self.gcode.respond_info(msg)
    def cmd_PAT_READ_REG(self, params):
        reg = self.gcode.get('REG', params)
        reg = int(reg, 16)
        data = self.read_register(reg, 1)[0]
        self.gcode.respond_info(
            "Value at register [%#X]: %d" % (reg, data))

class WatchDog:
    def __init__(self, config, pat9125):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.pat9125 = pat9125
        self.runout_callback = None
        self.is_autoload = False
        self.refresh_time = .100
        self.toolhead = self.kinematics = None
        self.last_position = [0., 0., 0., 0.]
        self.watchdog_timer = self.reactor.register_timer(
            self._watchdog_update_event)
        self.gcode.register_command(
            "AUTOLOAD_FILAMENT", self.cmd_AUTOLOAD_FILAMENT,
            desc=self.cmd_AUTOLOAD_FILAMENT_help)
    def watchdog_init(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kinematics = self.toolhead.get_kinematics()
    def set_runout_callback(self, callback):
        self.runout_callback = callback
    def enable_timer(self):
        self.last_position[:] = self.kinematics.calc_position()
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NOW)
    def disable_timer(self):
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NEVER)
    def _watchdog_update_event(self, eventtime):
        # gets current kinematic position, calculates delta
        current_pos = self.kinematics.calc_position()
        self.delta_e = current_pos[3] - self.last_position[3]
        # XXX - put sampling code here!
        # When it is determined that filament has run out, execute the
        # callback, ie:
        # If self.runout_callback is not None:
        #   self.runout_callback()
        self.last_position[:] = current_pos
        return eventtime + self.refresh_time
    cmd_AUTOLOAD_FILAMENT_help = \
        "Enable Autoload when PAT9125 detects filament"
    def cmd_AUTOLOAD_FILAMENT(self, params):
        self.is_autoload = True
        # XXX - do autoload stuff
        self.is_autoload = False

def load_config(config):
    return PAT9125(config)
