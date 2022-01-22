import math

class StepperMotor:
    def __init__(self, dt):
        self.dt = dt  # control cycle time
        self.mode = 1 # 1: continous  2: position  3: agitation
        self.agiSpeed = math.pi/2
        self.curPos = 0
        self.agiOffset = 10 * math.pi/180
        
    def ContinousModeControl(self, speed):
        if self.mode!=1:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 1")
        else:
            return speed*self.dt
       

    def PositionModeControl(self, angle):    # Angle (+/-)(degrees)
        if self.mode!=2:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 2")
        else:
            return angle
        

    def AgitationModeControl(self):   
        if self.mode != 3:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 3")
        else:
            angle = self.agiSpeed * self.dt
            if (self.curPos + angle) > self.agiOffset:
                angle = self.agiOffset - self.curPos
                self.curPos = self.agiOffset
                self.agiSpeed *= -1
            elif (self.curPos + angle) < (-self.agiOffset):
                angle = -self.agiOffset - self.curPos
                self.curPos = -self.agiOffset
                self.agiSpeed *= -1
            else:
                self.curPos += angle            
            return angle
        
    def InitAgitation(self): # set current position to 0
        if self.mode!=4:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 4")
        else:
            self.curPos = 0
            self.mode = 3
        
        
    def SetControlMode(self, mode):
        if mode == 3:
            self.mode = 4
        else:
            self.mode=mode
            
            
            
        
