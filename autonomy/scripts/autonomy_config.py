# parameters for mapping.py

# name of the rover model in gazebo
ROVER_MODEL_NAME = '/'

# id of the frame the mapping script will publish to
FIXED_FRAME_ID = 'parsed_point_cloud_world'

# offset of the camera from the rover's center
# CAMERA_POSITION_OFFSET = (0, 0, 0)  # visual odom
# CAMERA_POSITION_OFFSET = (0.048, 0, 0.141)  # leo
CAMERA_POSITION_OFFSET = (-0.285, 0, 1.1) #  mcgill rover

# rounding coefficient for the points
ROUNDING_COEF = 2

class PointsFilters:
    # filter out points that are too close to ground
    GROUND_LOWER_LIMIT, GROUND_UPPER_LIMIT = -0.10, 0.10

    # filter out points above the rover
    ROVER_HEIGHT = 1.5