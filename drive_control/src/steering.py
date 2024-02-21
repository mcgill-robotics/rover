class Steering:
    def __init__(self, rWheel, base_length):
        self.rWheel=rWheel  # radius of wheel
        self.base_length=base_length # wheel base length
        
    # See link for more information: http://wiki.ros.org/diff_drive_controller
    def steering_control(self, vR, wR, maxLin=3.0, maxAng=3.0): # R = rover
        
        # Equations based on differential drive controller.
        temp=-wR*self.base_length/2
        wLeft=(vR-temp)/self.rWheel
        wRight=(vR+temp)/self.rWheel
        val = [wLeft, wRight]                     
        return val