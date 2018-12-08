# Filament sensor control - pause print on runout
# Can use an endstop or PAT9125 sensor
#
# Copyright (C) 2018  Matthew Morley <matthew.morley.ca@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging


class filament_sensor:
    def __init__(self,config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.prusa_gcodes = self.printer.lookup_object('prusa_gcodes')
    def pause_print(self):
        self.gcode.respond_info("respond action:pause")
        # TODO pause now, don't flush the move que.

class pat9125_fsensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.pat9125 = self.printer.lookup_object('pat9125')
        self.prusa_gcodes = self.printer.lookup_object('prusa_gcodes')

        self.autoload_enabled = config.getboolean('filament_autoload', default=False)
        self.runout_detect_enabled = config.getboolean('filament_runout', default=False)

        # set current position to 0
        self.pat9125_x = 0
        self.pat9125_y = 0
        self.do_autoload = False
        # self.gcode.register_command('PAT9125_STATUS', self.cmd_RETURN_INFO) 

    
    def filament_autoload_init(self):
        self.autoload_enabled = False
        self.old_time = self.timer.getCurrentTime
        self.fsensor_autoload_y = self.pat9125_y
    
    def check_autoload(self):
        # check the sensor values for an autoload event
        pat9125_register_dict = self.pat9125.pat9125_update()
        if pat9125_register_dict is None:
            # TODO throw error
            logging.error("Error on reading pat9125 registers!")
        else: # It's else statements all the way down
            


        if (self.autoload_enabled is True) and (self.do_autoload is True):
            self.DO_FILAMENT_AUTOLOAD
        

    def DO_FILAMENT_AUTOLOAD(self,params): # dew the autoload
        self.filament_autoload_init
        
        if self.do_autoload == True:
            self.do_autoload = False
            # Do gcode script for autoload
            self.prusa_gcodes.cmd_LOAD_FILAMENT




# TODO build config
def load_config(config):
    return filament_sensor(config)