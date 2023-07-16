
from arm_layout import Ui_Arm
from arm_control.msg import ArmControllerInput
import arm_kinematics

class Arm_Backend():

    def __init__(self, arm_tab):
        self.ui = arm_tab
        self.ui.error_label.clear()

    def update_joints(self, motor_state):
        # motor_degree = []
        # for i in range(len(motor_state.MotorPos)):
        #     if (abs(motor_state.MotorPos[i])< 0.1 ):
        #         motor_degree.append(0)
        #     else:
        #         motor_degree.append(motor_state.MotorPos[i]/3.14*180)


        # self.ui.joint_one_value.display(motor_degree[0]) #Waist
        # self.ui.joint_two_value.display(motor_degree[1]) #Shoulder
        # self.ui.joint_three_value.display(motor_degree[2]) #Elbow
        # self.ui.joint_four_value.display(motor_degree[3]) #Wrist
        # self.ui.joint_five_value.display(motor_degree[4]) #Dist
        # #self.ui.joint_EOAT_value.display(motor_degree[5]) EOAT

        # hand = arm_kinematics.forwardKinematics(motor_state.MotorPos)
        # pose = arm_kinematics.Mat2Pose(hand)

        # self.ui.hand_x_val.display(pose[0])
        # self.ui.hand_y_val.display(pose[1])
        # self.ui.hand_z_val.display(pose[2])
        # self.ui.orientation_x_val.display(pose[3])
        # self.ui.orientation_y_val.display(pose[4])
        # self.ui.orientation_z_val.display(pose[5])
        # TODO
        pass


    def update_control(self,ctrl_input):
        # self.ui.joint_mode_value.display (ctrl_input.Mode)
        # self.ui.joint_velocity_value.display (ctrl_input.velocity/2*100)
        # TODO
        pass




