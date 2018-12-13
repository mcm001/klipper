# PAT9125 Filament Sensor
#
# Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com> and Matthew Morley <matthew.morley.ca@gmail.com>
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
            config, default_addr=CHIP_ADDR, default_speed=400000)
        self.gcode = self.printer.lookup_object('gcode')
        self.watchdog = WatchDog(config, self)
        # self.pat9125_fsensor = pat9125_fsensor(config, self)
        self.initialized = False
        
        self.gcode.register_command(
            'SENSOR_READ_ID', self.cmd_SENSOR_READ_ID)
        self.gcode.register_command(
            'PAT_TEST_INIT', self.cmd_PAT_TEST_INIT)
        self.gcode.register_command(
            'PAT_READ_REG', self.cmd_PAT_READ_REG)
        
        self.pat9125_x = 0
        self.pat9125_y = 0
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
        params = self.i2c.i2c_read(regs, read_len, poll_time=.005)
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



        if not (self._send_init_sequence(PAT9125_INIT1)):
            return

        print_time = mcu.estimated_print_time(self.reactor.monotonic())
        minclock = mcu.print_time_to_clock(print_time + .01)
        self.write_register('BANK_SELECTION', 0x01, minclock=minclock)
        if not self._send_init_sequence(PAT9125_INIT2):
            return

        self.write_register('BANK_SELECTION', 0x00)
        if not self.write_verify_reg('WP', 0x00):
            logging.info("PAT9125: Unable to re-enable write protect")
            return

        if not self.check_product_id():
            return

        self.write_register('RES_X', PAT9125_XRES)
        self.write_register('RES_Y', PAT9125_YRES)
        self.initialized = True
        logging.info("PAT9125 Initialization Success")

    # def _pat9125_init(self):
    #     mcu = self.i2c.get_mcu()
    #     self.initialized = False

    #     # Read and verify product ID
    #     if not self.check_product_id():
    #         logging.info("Rechecking ID...")
    #         if not self.check_product_id():
    #             return

    #     self.write_register('BANK_SELECTION', 0x00)

    #     print_time = mcu.estimated_print_time(self.reactor.monotonic())
    #     minclock = mcu.print_time_to_clock(print_time + .001)
    #     self.write_register('CONFIG', 0x97, minclock=minclock)
    #     minclock = mcu.print_time_to_clock(print_time + .002)
    #     self.write_register('WP', 0x5a, minclock=minclock)



    def _send_init_sequence(self, sequence, retry_cnt=5):
        for addr, data in sequence:
            retries = max(1, retry_cnt)
            verified = False
            while retries and not verified:
                retries -= 1
                verified = True
                self.write_register(addr, data)
                r_data = self.read_register(addr, len(data))
                for w_byte, r_byte in zip(data, r_data):
                    if w_byte & 0xFF != r_byte & 0xFF:
                        verified = False
                        if not retries:
                            logging.info(
                                "PAT9125 Read/Write mismatch, register (%#x)"
                                % (addr))
                            log_byte_array("Written bytes", data)
                            log_byte_array("Read bytes   ", r_data)
                            return False
                        else:
                            break
        return True
    def write_verify_reg(self, reg, data, retry_cnt=5):
        # write/verify data in a single register
        verified = False
        retries = max(1, retry_cnt)
        while retries and not verified:
            retries -= 1
            self.write_register(reg, data)
            r_data = self.read_register(reg, 1)
            verified = (data & 0xFF == r_data[0])
        return verified
    def check_product_id(self):
        pid = self.read_register('PID1', 2)
        if ((pid[1] << 8) | pid[0]) != PAT9125_PRODUCT_ID:
            logging.info(
                "Product ID Mismatch Expected: %d, Recd: %d"
                % (PAT9125_PRODUCT_ID, ((pid[1] << 8) | pid[0])))
            return False
        return True
    def pat9125_update(self):
        # XXX - set option to retreive response time in read_register
        # read data from 0x00 to 0x17
        motion = self.read_register(PAT9125_REGS['MOTION'],1)[0]
        
        if motion & 0x80:    
            data = self.read_register(PAT9125_REGS['DELTA_XL'],21)
            self.gcode.respond_info("X: %d, Y: %d, XYH: %d" % (data[0], data[1], data[15]))
            self.initialized = True
            # XXX - extract values
            results = {
                'MOTION': 0, 'DELTA_XL': 0, 'DELTA_YL': 0, 'DELTA_XYH': 0,
                'FRAME': 0, 'SHUTTER': 0}
            for key in results:
                results[key] = data[PAT9125_REGS[key] - 3]
            # results['MCU_TIME'] = 0
            return results 
        else:
            return None


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
        reg = self.gcode.get_str('REG', params)
        reg = int(reg, 16)
        data = self.read_register(reg, 1)[0]
        self.gcode.respond_info(
            "Value at register [%#X]: %d" % (reg, data))

# class pat9125_fsensor:
#     def __init__(self, config, pat9125):
#         self.printer = config.get_printer()
#         self.gcode = self.printer.lookup_object('gcode')
#         # self.pat9125 = self.printer.lookup_object('pat9125')



class WatchDog:
    def __init__(self, config, pat9125):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.prusa_gcodes = self.printer.lookup_object('prusa_gcodes')
        self.pat9125 = pat9125
        # self.pat9125_fsensor = pat9125_fsensor(config, self)
        self.runout_callback = None
        self.is_autoload = False
        self.refresh_time = .100
        self.toolhead = self.kinematics = None
        self.last_position = [0., 0., 0., 0.]
        self.watchdog_timer = self.reactor.register_timer(
            self._watchdog_update_event)

        self.autoload_enabled = config.getboolean('filament_autoload', default=False)
        self.runout_detect_enabled = config.getboolean('filament_runout', default=False)
        self.inverted = config.get('inverted', default=False)
        self.pat9125 = pat9125
        self.do_autoload_now = False
        

        # set initial state
        self.state = "Idle"
        self.autoload_allowed = True


        self.gcode.register_command(
            "AUTOLOAD_FILAMENT", self.cmd_AUTOLOAD_FILAMENT,
            desc=self.cmd_AUTOLOAD_FILAMENT_help)
        self.gcode.register_command(
            "AUTOLOD_STATE", self.cmd_AUTOLOD_STATE)

        self.gcode.register_command("READ_DELTA_Y",self.cmd_READ_DELTA_Y)

    def watchdog_init(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kinematics = self.toolhead.get_kinematics()
        
        self.printer.register_event_handler("idle_timeout:idle", self._idle_status_handler)
        self.printer.register_event_handler("idle_timeout:printing ", self._printing_status_handler)
        self.printer.register_event_handler("idle_timeout:ready", self._ready_status_handler)
        

        
    def set_runout_callback(self, callback):
        self.runout_callback = callback
    def enable_timer(self):
        self.last_position[:] = self.kinematics.calc_position()
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NOW)
    def disable_timer(self):
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NEVER)

    # Define callbacks for state change
    def _ready_status_handler(self, print_time):
        self.state = "Ready"
        self.autoload_allowed = True
    def _idle_status_handler(self, print_time):
        self.state = "Idle"
        self.autoload_allowed = True
    def _printing_status_handler(self, print_time):
        self.state = "Printing"
        self.autoload_allowed = False

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

    # XXX Filament autoload

    def pat9125_update_y(self):
        pat_dict = self.pat9125.pat9125_update()
        if pat_dict is None:
            logging.error("No motion detected, can't update")
            return pat_dict
        else:
            delta_yl = pat_dict['DELTA_YL']
            delta_xyh = pat_dict['DELTA_XYH']
            DY = delta_yl | ((delta_xyh << 8) & 0xf00)
            if ( DY & 0x800 ):
                DY -= 4096
            if self.inverted is True:
                self.pat9125.pat9125_y += DY
            else:
                self.pat9125.pat9125_y -= DY
            return pat_dict

    def filament_autoload_init(self):
        self.fsensor_autoload_y = self.pat9125.pat9125_y
        self.pat9125._pat9125_init()
        self.fsensor_autoload_sum = 0
        self.fsensor_autoload_c = 0
        self.do_autoload_now = False
    
    def check_autoload(self):
        # check the sensor values for an autoload event
        # This check needs to be called multiple times to trigger an autoload, so needs to be called from a loop
        
        if (self.autoload_enabled is not True):
            self.gcode.respond_info("Autoload is disabled, cannot autoload")
            return
        
        pat_dict = self.pat9125_update_y()
        dy = self.pat9125.pat9125_y - self.fsensor_autoload_y
        if pat_dict is None:
            return
        else: 
            # dy = pat9125_register_dict['DELTA_YL'] # Depreciated, replaced with self.pat9125.pat9125_y
            if ( dy != 0 ): # if movement (double sanity check after ucmotion)
                if (dy > 0): # delta-y value is positive (inserting)
                    self.fsensor_autoload_sum += dy
                    self.fsensor_autoload_c += 3 # increment change counter by 3
                elif (self.fsensor_autoload_c > 1) :
                    self.fsensor_autoload_c -= 2 # decrement change counter by 2 
                self.fsensor_autoload_y = self.pat9125.pat9125_
                logging.info("Autoload count: %s Autoload sum: %s, delta y: %s Overall Position: %s" % (self.fsensor_autoload_c, self.fsensor_autoload_sum, dy, self.pat9125.pat9125_y ))
        if (self.fsensor_autoload_c >= 12) and (self.fsensor_autoload_sum > 20):
            return True
        else:
            return False
        

    cmd_AUTOLOAD_FILAMENT_help = \
        "Enable Autoload when PAT9125 detects filament"
    def cmd_AUTOLOAD_FILAMENT(self, params): # FIXME to work with callbacks, how about a forever running loop that starts running at init and checks the state set by the callback, then based on that runs the check loop? Either way this cmd will be depreciated soon I hope, can be replaced with a check autoload state command
        self.gcode.respond_info("Autoload init")
        self.filament_autoload_init()
        timeout = 20
        curtime = self.reactor.monotonic()
        endtime = curtime + timeout # set timeout to 20 seconds 
        while curtime < endtime:
            while self.do_autoload_now is False:
                if self.check_autoload() is True:
                    # TODO Do gcode script for autoload
                    # self.prusa_gcodes.cmd_LOAD_FILAMENT
                    self.gcode.respond_info("Autoload detected!")
                    return
            # self.reactor.pause(self.reactor.monotonic() + .005) # TODO do we need to delay?
        self.gcode.respond_info("Autoload timed out after %i seconds" % timeout)

    # def cmd_AUTOLOD_STATE_help = "Querry the current autoload state. Returns current printer state and autoload allowed status."
    def cmd_AUTOLOD_STATE(self, params):
        self.gcode.respond_info("Current self state: %s Autoload allowed? %s" % (self.state, self.autoload_allowed))

    def cmd_READ_DELTA_Y(self, params):
        self.pat9125.cmd_PAT_TEST_INIT(params)
        curtime = self.reactor.monotonic()
        endtime = curtime + 10
        while curtime < endtime:
            motion = self.pat9125.read_register(PAT9125_REGS['MOTION'],1)[0]
            if motion & 0x80:
                data = self.pat9125.read_register(PAT9125_REGS['DELTA_XL'],21)
                if data[1] != 0:
                    self.gcode.respond_info("X: %d, Y: %d, XYH: %d" % (data[0], data[1], data[15]))
            curtime = self.reactor.pause(curtime + .05)
        # if not y:
        #     self.gcode.respond_info("Test Failed")


def load_config(config):
    return PAT9125(config)
