# Filament sensor control - pause print on runout
# Can use an endstop or PAT9125 sensor
#
# Copyright (C) 2018  Matthew Morley <matthew.morley.ca@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.



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

        # List of registers for the PAT9125
        self.PAT9125_PID1			    = 0x00
        self.PAT9125_PID2			    = 0x01
        self.PAT9125_MOTION			    = 0x02
        self.PAT9125_DELTA_XL		    = 0x03
        self.PAT9125_DELTA_YL		    = 0x04
        self.PAT9125_MODE			    = 0x05
        self.PAT9125_CONFIG			    = 0x06
        self.PAT9125_WP				    = 0x09
        self.PAT9125_SLEEP1			    = 0x0a
        self.PAT9125_SLEEP2			    = 0x0b
        self.PAT9125_RES_X			    = 0x0d
        self.PAT9125_RES_Y			    = 0x0e
        self.PAT9125_DELTA_XYH		    = 0x12
        self.PAT9125_SHUTTER			= 0x14
        self.PAT9125_FRAME			    = 0x17
        self.PAT9125_ORIENTATION		= 0x19
        self.PAT9125_BANK_SELECTION	    = 0x7f

        self.PAT9125_XRES = 0
        self.PAT9125_YRES = 240

        # TODO what is this magic code that Prusa needs to send to the sensor? 
        # see line 66 of pat9125.c
        self.pat9125_init_seq2 = [0x06, 0x028, 0x33, 0x0d0, 0x36, 0x0c2, 0x3e, 0x001, 0x3f, 0x015, 0x41, 0x032, 0x42, 0x03b, 0x43, 0x0f2, 0x44, 0x03b, 0x45, 0x0f2, 0x46, 0x022, 0x47, 0x03b, 0x48, 0x0f2, 0x49, 0x03b, 0x4a, 0x0f0, 0x58, 0x098, 0x59, 0x00c, 0x5a, 0x008, 0x5b, 0x00c, 0x5c, 0x008, 0x61, 0x010, 0x67, 0x09b, 0x6e, 0x022, 0x71, 0x007, 0x72, 0x008, 0x0ff]

        # TODO is this even how you multiline array?
        pat9125_init_seq1 = [ PAT9125_WP, 0x5a,
            # Set the X resolution to zero to let the sensor know that it could safely ignore movement in the X axis.
            PAT9125_RES_X, PAT9125_XRES,
            # Set the Y resolution to a maximum (or nearly a maximum).
            PAT9125_RES_Y, PAT9125_YRES,
            # Set 12-bit X/Y data format.
            PAT9125_ORIENTATION, 0x04,
        #	PAT9125_ORIENTATION, 0x04 | (xinv?0x08:0) | (yinv?0x10:0), //!? direction switching does not work
            # Now continues the magic sequence from the PAT912EL Application Note: Firmware Guides for Tracking Optimization.
            0x5e, 0x08,
            0x20, 0x64,
            0x2b, 0x6d,
            0x32, 0x2f,
            # stopper
            0x0ff]



        self.autoload_enabled = config.getboolean('filament_autoload', default=False)
        self.runout_detect_enabled = config.getboolean('filament_runout', default=False)


        self._init_9125
        # TODO throw error if can't connect

        # set current position to 0
        self.pat9125_x = 0
        self.pat9125_y = 0

        self.gcode.register_command('PAT9125_STATUS', self.cmd_RETURN_INFO)

    def _init_9125(self, params):
        # TODO enable I2c

        # verify responce ID
        # TODO make read_register function
        pat_PID1 = pat9125.read_register(self.PAT9125_PID1)
        pat_PID2 = pat9125.read_register(self.PAT9125_PID2)
        if ( pat_PID1 != 0x31 ) or ( pat_PID2 != 0x91 ):
            # break, everything is broken
            self.pat9125_enabled = True
        else:
            self.errors = 0
            self.pat9125_enabled = False
            # TODO why does Prusa enable write protect? Do I need to? ETC

    def check_pat_active(self):
        pat_PID1 = pat9125.read_register(self.PAT9125_PID1)
        pat_PID2 = pat9125.read_register(self.PAT9125_PID2)
        if ( pat_PID1 != 0x31 ) or ( pat_PID2 != 0x91 ):
            # break, everything is broken
            return False
            self.gcode.respond_error("ERROR: PAT9125 is not connected!")
            # TODO throw an error for real

        else:
            return True
    
    def update_y(self):
        if check_pat_active = True:
            check_motion = pat9125.read_register(self.PAT9125_MOTION)
            check_frame = pat9125.read_register(self.PAT9125_FRAME)
            check_shutter = pat9125.read_register(self.PAT9125_SHUTTER)

            # whatever # if (ucMotion & 0x80) even means
            # TODO fix this part lmao
            if True:
                delta_xl = pat9125.read_register(self.PAT9125_DELTA_XL)
                delta_yl = pat9125.read_register(self.PAT9125_DELTA_YL)
                delta_xyh = pat9125.read_register(self.PAT9125_DELTA_XYH)

                # TODO What does this mean (in c) iDX = deltaXL | ((delta_xyh <<4) & 0xf00)
                # iDY = deltaYL | ((delta_xyh <<8) & 0xf00)
                # see line 186 of prusa's pat9125.c

                # TODO What would happen if I were to just sketchy skirt this boi
                iDX = delta_xl
                iDY = delta_yl

                self.pat9125_x += iDX
                self.pat9125_y -= iDY # why is this flipped?
    
    def filament_autoload_init(self):
        self.do_autoload = False
        self.old_time = self.timer.getCurrentTime
        self.fsensor_autoload_y = pat9125_y
    
    def check_autoload(self):
        # check the sensor values for an autoload event
        pass    
        

    def DO_FILAMENT_AUTOLOAD(self,params):
        self.filament_autoload_init
        
        if self.do_autoload == True:
            self.do_autoload = False
            # Do gcode script for autoload
            prusa_gcodes.cmd_LOAD_FILAMENT


    def cmd_RETURN_INFO(self, params):
        if check_pat_active is not False:
            shutter = pat9125.read_register(self.PAT9125_SHUTTER)
            frame = pat9125.read_register(self.PAT9125_FRAME)
            self.gcode.respond_info("PAT9125 INFO: Enabled: %s Shutter: %f Frame: %f Errors: %i" % (self.pat_active, shutter, frame, self.errors))
            self.gcode.respond_info("Filament Movement: ")


def load_config(config):
    return filament_sensor(config)