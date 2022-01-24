import math

class StepperMotor:
    def __init__(self, dt):
        self.dt = dt  # control cycle time
        self.mode = 1 # 1: continous  2: position  3: agitation
        self.agi_speed = math.pi/2
        self.cur_pos = 0
        self.agi_offset = 10 * math.pi/180
        
    def continous_mode_control(self, speed):
        if self.mode!=1:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 1")
        else:
            return speed*self.dt
       

    def position_mode_control(self, angle):    # Angle (+/-)(degrees)
        if self.mode!=2:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 2")
        else:
            return angle
        

    def agitation_mode_control(self):   
        if self.mode != 3:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 3")
        else:
            angle = self.agi_speed * self.dt
            if (self.cur_pos + angle) > self.agi_offset:
                angle = self.agi_offset - self.cur_pos
                self.cur_pos = self.agi_offset
                self.agi_speed *= -1
            elif (self.cur_pos + angle) < (-self.agi_offset):
                angle = -self.agi_offset - self.cur_pos
                self.cur_pos = -self.agi_offset
                self.agi_speed *= -1
            else:
                self.cur_pos += angle            
            return angle
        
    def init_agitation(self): # set current position to 0
        if self.mode!=4:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 4")
        else:
            self.cur_pos = 0
            self.mode = 3
        
        
    def set_control_mode(self, mode):
        if mode == 3:
            self.mode = 4
        else:
            self.mode = mode
            
            
            
        
