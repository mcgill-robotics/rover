class Steering:
    def __init__(self, rWheel, base_length):
        self.rWheel=rWheel  # radius of wheel
        self.base_length=base_length # wheel base length
        
        
    def steering_control(self, vR, wR): # R = rover
        
        # Manually caps the linear and angular velocities.
        if wR > 3.0:
            wR = 3.0
        elif wR < -3.0:
            wR = -3.0

        if vR < -0.5:
            vR = -0.5
        
        temp=wR*self.base_length/2
        wLeft=(vR-temp)/self.rWheel
        wRight=(vR+temp)/self.rWheel
        val = [wLeft, wRight]                     
        return val