class Steering:
    def __init__(self, rWheel, base_length):
        self.rWheel=rWheel  # radius of wheel
        self.base_length=base_length # wheel base length
        
        
    def steering_control(self, vR, wR, maxLin=3.0, maxAng=3.0): # R = rover
        
        # Manually caps the linear and angular velocities.
        # if vR > maxLin:
        #     vR = maxLin
        # elif vR < -maxLin:
        #     vR = -maxLin

        # if wR > maxAng:
        #     wR = maxAng
        # elif wR < -maxAng:
        #     wR = -maxAng
        
        temp=wR*self.base_length/2
        wLeft=(vR-temp)/self.rWheel
        wRight=(vR+temp)/self.rWheel
        val = [wLeft, wRight]                     
        return val