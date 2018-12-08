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





# TODO build config
def load_config(config):
    return filament_sensor(config)