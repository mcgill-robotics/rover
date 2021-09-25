class StepperMotor:
    def __init__(self, time):
        self.dtTime = time  # control cycle time
        self.mode = 1 # 1: continous  2: position  3: agitation
        self.agiSpeed = 0
        self.curPos = 0
        self.agiOffset = 0
        
    def ContinousModeControl(self, speed):
        if self.mode!=1:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 1")
        else:
            return speed*time
       

    def PositionModeControl(self, angle):    # Angle (+/-)(degrees)
        if self.mode!=2:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 2")
        else:
            return angle
        

    def AgitationModeControl(self):   
        if self.mode!=3:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 3")
        else:
            angle=self.agiSpeed*self.dtTime
            if (self.curPos+angle) > self.agiOffset:
                angle = self.agiOffset-self.curPos
                self.curPos=self.agiOffset
                self.agiSpeed *= -1
            elif (self.curPos+angle) < (-self.agiOffset):
                angle = -self.agiOffset-self.curPos
                self.curPos = -self.agiOffset
                self.agiSpeed *= -1
            else:
                self.curPos+=angle            
            return angle
        
    def InitAgitation(self, offset, speed): # set current position to 0
        if self.mode!=4:
            raise RuntimeError("mode set to " + str(self.mode) + ", but should be 4")
        else:
            self.curPos = 0
            self.agiSpeed = speed
            self.agiOffset = offset
            self.mode=3
        
        
    def SetControlMode(self, mode):
        if mode==3:
            self.mode=4
        else:
            self.mode=mode
            
            
            
        
