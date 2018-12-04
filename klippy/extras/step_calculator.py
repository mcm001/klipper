# Step distance calculator to calculate step distances on the fly
#
# Copyright (C) 2018  Matthew Morley <matthew.morley.ca@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class step_calculator:
    def __init__(self,config):
        # Choose mode - defaults to basic, other options are leadscrew and pulley/belt
        self.mode = config.get("mode", default="basic")
        
        # Define other params
        self.basic_full_step_per_mm = config.get("full_steps_per_mm", default=None)
        self.motor_microstepping = config.getint("microstepping", default=None)
        self.steps_per_rev = config.getint("steps_per_`revolution`", default=None)
        self.step_angle = config.getfloat("motor_step_angle", default=None)
        self.gear_reduction = config.getfloat("gear_reduction", default=None)

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


    def basic_calc(self): 
        self.steps_per_mm = self.basic_full_step_per_mm * self.motor_microstepping


    def leadscrew_calc(self):
        # per reprap wiki, steps_per_mm = (motor_steps_per_rev * driver_microstep) / thread_pitch 
        self.steps_per_mm =  ( self.steps_per_rev * self.motor_microstepping ) / self.leadscrew_pitch_per_rev


    def belt_calc(self):
        # per reprap wiki, steps/mm = \frac{motor\ steps\ per\ rev * driver\ microstep}{belt\ pitch * pulley\ number\ of\ teeth}
        self.steps_per_mm = ( self.steps_per_rev * self.motor_microstepping ) / ( self.belt_pitch * self.pulley_teeth )




