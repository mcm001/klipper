# Step distance calculator to calculate step distances on the fly
#
# Copyright (C) 2018  Matthew Morley <matthew.morley.ca@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Example configuration:
# [step_calculator stepper_x]
# mode: microstepping
# full_steps_per_mm: 12.5
# microstepping: 16

# mode: pulley
# steps_per_revolution: 200
# microstepping: 16
# belt_pitch: 2
# pulley_teeth: 16

class step_calculator:
    def __init__(self,config):
        # Choose mode - defaults to basic, other options microstepping calculator, leadscrew and pulley/belt 
        self.mode = config.get("mode", default="basic")
        
        # Define other params
        self.steps_per_mm_with_microstepping = config.get("microstepped_steps_per_mm", default=None)
        self.basic_full_step_per_mm = config.get("full_steps_per_mm", default=None)
        self.motor_microstepping = config.getint("microstepping", default=None)
        self.steps_per_rev = config.getint("steps_per_revolution", default=None)
        self.step_angle = config.getfloat("motor_step_angle", default=1.8)
        self.gear_reduction = config.getfloat("gear_reduction", default=None)
        self.driven_pulley_tooth_count = config.getint("driven_gear_tooth_count", default=None)
        self.step_compensation = config.getint("step_compensation", default=1.0)

        # Pulley specific stuff
        self.pulley_teeth = config.getint("pulley_teeth", default=None)
        self.belt_pitch = config.getfloat("belt_pitch", default=None)

        # Leadscrew specific stuff
        self.leadscrew_pitch_per_rev = config.getfloat("leadscrew_pitch_mm", default=None)



    def calculate_steps(self):
        if self.step_angle is not None: 
            self.steps_per_rev = self.step_angle / 360


        if self.mode == "basic":
            self.steps_per_mm = self.basic_calc()
        elif self.mode == "leadscrew":
            self.steps_per_mm = self.leadscrew_calc()
        elif (self.mode == "pulley" or self.mode == "belt"):
            self.steps_per_mm = self.belt_calc()
        elif(self.mode == "microstep"):
            self.steps_per_mm = self.microstep_calc

        # TODO else, throw error

        # compensation for if calculated value is off, should *never* be used (looking at you, extruders)
        self.steps_per_mm = self.steps_per_mm * step_compensations

        self.mm_per_step = 1 / self.steps_per_mm


    def basic_calc(self): #Calculate step distance from steps per mm
        self.steps_per_mm = self.steps_per_mm_with_microstepping


    def microstep_calc(self):
        self.steps_per_mm = self.basic_full_step_per_mm * self.motor_microstepping


    def leadscrew_calc(self):
        # per reprap wiki, steps_per_mm = (motor_steps_per_rev * driver_microstep) / thread_pitch 
        self.steps_per_mm =  ( self.steps_per_rev * self.motor_microstepping ) / self.leadscrew_pitch_per_rev


    def belt_calc(self):
        # per reprap wiki, steps/mm = \frac{motor\ steps\ per\ rev * driver\ microstep}{belt\ pitch * pulley\ number\ of\ teeth}
        self.steps_per_mm = ( self.steps_per_rev * self.motor_microstepping ) / ( self.belt_pitch * self.pulley_teeth )




