class Steering:
    def __init__(self, rWheel, L):
        self.rWheel=rWheel  # radius of wheel
        self.L=L # wheel base length
        
        
    def steering_control(self, vR, wR): # R = rover
        temp=wR*self.L/2
        wLeft=(vR+temp)/self.rWheel
        wRight=(vR-temp)/self.rWheel
        val = [wLeft, wRight]                       
        return val


