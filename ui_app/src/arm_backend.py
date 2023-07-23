
from arm_layout import Ui_Arm
from arm_control.msg import ArmControllerInput
import arm_kinematics


class Arm_Backend():

    def __init__(self, arm_tab):
        self.ui = arm_tab
        self.ui.error_label.clear()
        self.state12 = [0,0,0]
        self.state24 = [0,0,0]
        self.stateAll =[]

    def update_joints12(self, state12):
        self.ui.joint_four_value.display(state12.data[2]) #Wrist
        self.ui.joint_five_value.display(state12.data[1]) #Dist
        self.ui.joint_EOAT_value.display(state12.data[0]) #EOAT
        self.state12 = [state12.data[2],state12.data[1],state12.data[0]]
        self.handPosition()

    def update_joints24(self, state24):
        self.ui.joint_one_value.display(state24.data[2]) #Waist
        self.ui.joint_two_value.display(state24.data[1]) #Shoulder
        self.ui.joint_three_value.display(state24.data[0]) #Elbow
        self.state24 = [state24.data[2], state24.data[1],state24.data[0]]
        self.handPosition()

    def handPosition(self):
        self.stateAll = self.state24+self.state12
        stateAll_r = tuple(data_i * (3.1415/180) for data_i in self.stateAll)
        hand = arm_kinematics.forwardKinematics(stateAll_r)
        pose = arm_kinematics.Mat2Pose(hand)
        self.ui.hand_x_val.display(pose[0])
        self.ui.hand_y_val.display(pose[1])
        self.ui.hand_z_val.display(pose[2])
        self.ui.orientation_x_val.display(pose[3])
        self.ui.orientation_y_val.display(pose[4])
        self.ui.orientation_z_val.display(pose[5])
        
    def update_control(self,ctrl_input):
        self.ui.joint_mode_value.display (ctrl_input.Mode)
        self.ui.joint_velocity_value.display (ctrl_input.MacVelPercentage/2*100)
        




