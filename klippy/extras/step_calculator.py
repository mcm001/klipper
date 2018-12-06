# Step distance calculator to calculate step distances on the fly
#
# Copyright (C) 2018  Matthew Morley <matthew.morley.ca@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Example configuration:
# [step_calculator stepper_x]
# mode: basic
# full_steps_per_mm: 12.5
# microstepping: 16

# mode: pulley
# steps_per_revolution: 200
# microstepping: 16
# belt_pitch: 2
# pulley_teeth: 16

import logging

class step_calculator:
    def __init__(self,config):

        # get gcode object
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        #get name, for example stepper_x
        self.name = config.get_name().split()[1]
        if config.has_section("tmc2130 " + self.name):
            tcfg = config.getsection("tmc2130 " + self.name)
            self.motor_microstepping = tcfg.getint('microsteps')
        else:
            self.motor_microstepping = config.getint('microsteps', default = None)


        # Choose mode - defaults to basic, other options are leadscrew and pulley/belt 
        self.mode = config.get("mode", default="basic")
        
        # Define other params
        self.name = config.get_name().split()[1]
        self.steps_per_mm_with_microstepping = config.get("microstepped_steps_per_mm", default=None)
        self.basic_full_step_per_mm = config.getfloat("full_steps_per_mm", default=None)
        self.mm_per_full_step = config.getfloat("mm_per_full_step", default=None)
        self.steps_per_rev = config.getint("steps_per_revolution", default=None)
        self.step_angle = config.getfloat("motor_step_angle", default=None)
        self.gear_reduction = config.getfloat("gear_reduction", default=None)
        self.driven_pulley_tooth_count = config.getint("driven_gear_tooth_count", default=None)
        self.step_compensation = config.getint("step_compensation", default=1.0)

        # Pulley specific stuff
        self.pulley_teeth = config.getint("pulley_teeth", default=None)
        self.belt_pitch = config.getfloat("belt_pitch", default=None)

        # Leadscrew specific stuff
        self.leadscrew_pitch_per_rev = config.getfloat("leadscrew_pitch_mm", default=None)

        self.calculate_steps



    def calculate_steps(self):
        if self.step_angle is not None: 
            self.steps_per_rev = self.step_angle / 360


        # if self.mode == "basic":
            # self.steps_per_mm = self.basic_calc()
        elif self.mode == "leadscrew":
            self.steps_per_mm = self.leadscrew_calc()
        elif (self.mode == "pulley" or self.mode == "belt"):
            self.steps_per_mm = self.belt_calc()
        elif(self.mode == "basic"):
            self.steps_per_mm = self.microstep_calc

        # TODO else, throw error

        # compensation for if calculated value is off, should *never* be used (looking at you, extruders)
        
        if self.steps_per_mm is not None:
            self.steps_per_mm = self.steps_per_mm * self.step_compensation
            self.mm_per_step = 1 / self.steps_per_mm
            self.gcode.respond_info("Mode: %s Steps per mm: %i MM per step: %i" % ( self.mode, self.steps_per_mm, self.mm_per_step )
        else:
            # ERROR!!!!!!! eeekkkk
            self.gcode.respond_error("ERROR: Steps per mm calculation error! Check the log file for more info")

    # def basic_calc(self): #Calculate step distance from steps per mm
        # self.steps_per_mm = self.steps_per_mm_with_microstepping


    def microstep_calc(self):
        if (self.basic_full_step_per_mm is not None) and (self.motor_microstepping is not None):
            self.steps_per_mm = self.basic_full_step_per_mm * self.motor_microstepping
        elif (self.mm_per_full_step is not None) and (self.motor_microstepping is not None):
            self.steps_per_mm = 1 / ( self.basic_full_step_per_mm * self.motor_microstepping )
        else:
            logging.info("steps per mm or mm per full step is not defined!")
            return None
    def leadscrew_calc(self):
        if (self.steps_per_rev is not None) and (self.motor_microstepping is not None) and (self.leadscrew_pitch_per_rev is not None):
            # per reprap wiki, steps_per_mm = (motor_steps_per_rev * driver_microstep) / thread_pitch 
            self.steps_per_mm =  ( self.steps_per_rev * self.motor_microstepping ) / self.leadscrew_pitch_per_rev
        else:
            logging.info("something is not defined! Check that steps per rev or step angle and leadscrew pitch are defined")
            return None

    def belt_calc(self):
        if ( self.steps_per_rev is not None ) and ( self.motor_microstepping is not None ) and ( self.belt_pitch is not None ) and (self.pulley_teeth is not None):
            # per reprap wiki, steps/mm = \frac{motor\ steps\ per\ rev * driver\ microstep}{belt\ pitch * pulley\ number\ of\ teeth}
            self.steps_per_mm = ( self.steps_per_rev * self.motor_microstepping ) / ( self.belt_pitch * self.pulley_teeth )
        else:
            logging.info("something is not defined! Check that steps per rev or step angle")
            return None



