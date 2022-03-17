
from drive_layout import Ui_DriveTab


class Drive_Backend():

    def __init__(self, drive_tab):
        self.ui = drive_tab
        self.ui.error_label.clear()


    def update_wheel_velocities(self, wheel_velocity):
        self.ui.left_speed_data.display(wheel_velocity.left)
        self.ui.right_speed_data.display(wheel_velocity.right)
        self.ui.rear_left_speed_data.display(wheel_velocity.left)
        self.ui.rear_right_speed_data.display(wheel_velocity.right)

    def update_twist_data(self, robot_twist):
        self.ui.twist_angular_velocity.display(robot_twist.angular)
        self.ui.twist_linear_velocity.display(robot_twist.linear)

    def update_robot_location(self, location):
        self.ui.robot_location_x.display(location.x)
        self.ui.robot_location_y.display(location.y)
        self.ui.robot_location_yaw.display(location.yaw)

    def set_error(self, error):
        self.ui.error_label.setText(error)
