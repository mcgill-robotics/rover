import numpy as np
import math

# Define joint limits in Radians
jointUpperLimits = [
    90 * np.pi / 180,
    70 * np.pi / 180,
    70 * np.pi / 180,
    38 * np.pi / 180,
    85 * np.pi / 180,
]  # rad
jointLowerLimits = [
    -90 * np.pi / 180,
    -60 * np.pi / 180,
    -20 * np.pi / 180,
    -35 * np.pi / 180,
    -85 * np.pi / 180,
]  # rad

# Define Denavit-Hartenberg parameters for the robot arm
arm_DH = [
    [0.0575, 0, 0, -90 * math.pi / 180],  # waist - shoulder : vertical offset from base
    [0, -90 * math.pi / 180, 0.5, 0],  # shoulder - elbow
    [0, 90 * math.pi / 180, 0.4, 0],  # elbow - wrist
    [0, 90 * math.pi / 180, 0, 90 * math.pi / 180],  # wrist - hand
    [0.034, 0, 0, 0],  # hand
]


# Function to convert pose (position and Euler angles) to a transformation matrix
def Pose2Mat(pose):
    """Converts the pose respecting the XYZ euler convention into a transformation matrix

    Parameters
    --------
        pose : np.array(6, 1)
            [x, y, z, alpha_x, alpha_y, alpha_z]

    Returns
    --------
        T : np.array(4,4)
            DH transform
    """

    # Extract position and Euler angles from the input
    alpha = pose[3]
    beta = pose[4]
    gamma = pose[5]

    # Initialize a 4x4 identity matrix
    T = np.eye(4)

    # Create rotation matrices for each axis (X, Y, Z)
    Rx = np.array(
        [
            [1, 0, 0],
            [0, math.cos(alpha), -math.sin(alpha)],
            [0, math.sin(alpha), math.cos(alpha)],
        ]
    )
    Ry = np.array(
        [
            [math.cos(beta), 0, math.sin(beta)],
            [0, 1, 0],
            [-math.sin(beta), 0, math.cos(beta)],
        ]
    )
    Rz = np.array(
        [
            [math.cos(gamma), -math.sin(gamma), 0],
            [math.sin(gamma), math.cos(gamma), 0],
            [0, 0, 1],
        ]
    )

    # Combine the rotation matrices and set the translation components
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = pose[:3]

    return T


# Function to convert a transformation matrix to pose (position and Euler angles)
def Mat2Pose(T):
    """Converts the transformation matrix into it's Cartesian coordinates
    and Euler angles in the XYZ Euler convention

    Parameters
    --------
        T : np.array(4, 4)
            Transformation Matrix

    Returns
    --------
        np.array(6, 1)
            [x, y, z, alpha_x, alpha_y, alpha_z]
    """

    # Compute Euler angles from the rotation matrix
    if abs(T[2, 0]) != 1:
        beta = -math.asin(T[2, 0])
        alpha = math.atan2(T[2, 1] / math.cos(beta), T[2, 2] / math.cos(beta))
        gamma = math.atan2(T[1, 0] / math.cos(beta), T[0, 0] / math.cos(beta))
    else:
        gamma = 0
        if T[2, 0] == -1:
            beta = math.pi / 2
            alpha = math.atan2(T[0, 1], T[0, 2])
        else:
            beta = -math.pi / 2
            alpha = math.atan2(-T[0, 1], -T[0, 2])

    # return Euler angles and translation into a pose array
    return np.array([T[0][3], T[1][3], T[2][3], alpha, beta, gamma])


# Function to compute the transformation matrix for a DH parameter set
def dhToMat(d, theta, a, alpha):
    """Computes the transformation matrix corresponding
    to the Denavit-Hartenberg parameters

    Parameters
    --------
        d : float
            offset along the previous z to the common normal
        theta : float
            angle about the previous z, from old x to the new x
        a : float
            length of the common normal
        alpha : float
            angle about common normal from old z axis to new z axis

    Returns
    --------
        T : np,array(4,4)
            transformation matrix
    """
    return np.array(
        [
            [
                math.cos(theta),
                -math.sin(theta) * math.cos(alpha),
                math.sin(theta) * math.sin(alpha),
                a * math.cos(theta),
            ],
            [
                math.sin(theta),
                math.cos(theta) * math.cos(alpha),
                -math.cos(theta) * math.sin(alpha),
                a * math.sin(theta),
            ],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


# Function to compute the forward kinematics of the arm
def forwardKinematics(q):
    """Computes the forward kinematics of the arm

    Parameters
    --------
        q : list(float)
            list of the values of the joints angles in rad, thetas

    Returns
    --------
        T : np.array(4,4)
            transformation matrix of the arm
    """
    Ts = _FK(q)
    return Ts[-1]


# Function to compute a list of transformation matrices for the arm joints
def _FK(q):
    """Build and performs the matrix multiplication of the transformation matrices
    needed in the forward kinematics of the arm

    Parameters
    --------
        Parameters
    --------
        q : list(float)
            list of the values of the joints angles, thetas

    Returns
    --------
        list(np.array(4,4))
            list of all transformation matrices of the arm computed from base to each link
    """
    numJoints = len(arm_DH)
    matrices = []

    # Iterate through the DH parameters and compute transformation matrices
    for i in range(numJoints):
        T = dhToMat(arm_DH[i][0], arm_DH[i][1] + q[i], arm_DH[i][2], arm_DH[i][3])
        if len(matrices) > 0:
            T0i = matrices[-1] @ T
        else:
            T0i = T
        matrices.append(T0i)

    return matrices


# Function to compute the Jacobian matrix for a given set of joint angles
def Jacobian(q):
    """Jacobian matrix of the arm for a given set of angles

    Parameters
    --------
        q : np.array(6, 1)
            joint configuration of the robot in rad

    Returns
    --------
        np.array(6, 6)
            Jacobian matrix at given angles
        np.array(3, 6)
            Velocity Jacobian matrix at given angles
        np.array(3, 6)
            Angular Jacobian matrix at given angles
    """
    Ts = _FK(q)
    N = len(Ts)
    Jv = []
    Jw = []
    for i in range(N):
        if i == 0:
            z0i = np.array([0, 0, 1])
            riN = np.array([Ts[-1][0][3], Ts[-1][1][3], Ts[-1][2][3]])
        else:
            z0i = np.array([Ts[i - 1][0][2], Ts[i - 1][1][2], Ts[i - 1][2][2]])
            riN = np.array(
                [
                    Ts[-1][0][3] - Ts[i - 1][0][3],
                    Ts[-1][1][3] - Ts[i - 1][1][3],
                    Ts[-1][2][3] - Ts[i - 1][2][3],
                ]
            )
        Jvi = np.cross(z0i, riN)
        Jwi = z0i
        Jv.append(Jvi)
        Jw.append(Jwi)

    Jv = np.array(Jv).T
    Jw = np.array(Jw).T
    J = np.concatenate([Jv, Jw])

    return J, Jv, Jw


# Function to compute the forward Cartesian velocity of the arm
def forwardVelocity(q, dq):
    """Computes the cartesian velocity of the arm

    Parameters
    --------
        q : list(float)
            joint configuration
        dq : list(float)
            joint velocities

    Returns
    --------
        np.array(len(qCur))
            cartesian velocity [x,y,z]
    """
    J, Jv, Jw = Jacobian(q)

    dx = J.dot(dq)

    return dx


# Function to compute the inverse velocity of the arm
def inverseVelocity(q, dx):
    """Computes the joint velocity of the arm given a desired
    cartesian velocity

    Parameters
    --------
        q : list(float)
            joint configuration
        dx : list(float)
            cartesian velocities

    Raises
    --------
        ValueError
            joint velocities could not be computed

    Returns
    --------
        np.array(len(qCur))
            joint velocity
    """
    J, Jv, Jw = Jacobian(q)

    try:
        if len(dx) == 3:
            dq = np.linalg.pinv(Jv).dot(dx)
        elif len(dx) == 6:
            dq = np.linalg.pinv(J).dot(dx)
        else:
            raise ValueError("Dimension Mismatch: dx must be dim 3 or 6")
    except np.linalg.LinAlgError:
        raise ValueError

    return dq


# Function to compute the projection length of a vector onto a line
def projection_length(line, vector):
    """
    Parameters
    --------
        line : list(float)
            two-dimensional vector defining line onto which pojection is done
        vector : list(float)
            two-dimensional vector to be projected

    Returns
    --------
        length : float
            length of projected vector
    """

    dot_product = line[0] * vector[0] + line[1] * vector[1]
    length_sqrd = math.pow(line[0], 2) + math.pow(line[1], 2)
    proj_vector = [
        line[0] * dot_product / length_sqrd,
        line[1] * dot_product / length_sqrd,
    ]

    length = math.sqrt(math.pow(proj_vector[0], 2) + math.pow(proj_vector[1], 2))
    if proj_vector[0] < 0:
        length *= -1

    return length


# Function to compute the joint angles to achieve a target end effector position
def inverseKinematicsComputeJointAngles(
    ee_target, wrist_target, elbow_target, rotate_waist
):
    """Calculates the necessary angles of all joints to achieve the target end effector position

    Parameters
    --------
        ee_target : np.array(6, 1)
            end effector Cartesian coordinates and XYZ Euler angles

        wrist_target: list(float)
            wrist position Cartesian coordinates [x, y, z]

        elbow_target: list(float)
            elbow position Cartesian coordinates [x, y, z]

        rotate_waist: boolean
            True if the waist is to be rotated 180, False otherwise

    Returns
    --------
        joint_angles : list(float)
            list of angles in radians of joints from base to end effector, relative to
            the last joint
    """
    projection_line = [
        ee_target[0],
        ee_target[1],
    ]  # To turn from 3D to 2D, we project onto the plane defined by this line and the Y axis.

    # Refactor hand coordinates from 3D to 2D
    hand_proj = projection_length(projection_line, ee_target[:2])
    hand_coordinates = (hand_proj, ee_target[2])

    # Refactor wrist coordinates from 3D to 2D
    wrist_proj = projection_length(projection_line, wrist_target[:2])
    wrist_coordinates = (wrist_proj, wrist_target[2])

    # Refactor elbow coordinates from 3D to 2D
    elbow_proj = projection_length(projection_line, elbow_target[:2])
    elbow_coordinates = (elbow_proj, elbow_target[2])

    true_base_coordinates = (0, 0, arm_DH[0][0])  # Shoulder coordinates

    d = math.sqrt(
        wrist_coordinates[0] ** 2
        + (wrist_coordinates[1] - true_base_coordinates[2]) ** 2
    )  # Length of line between wrist and shoulder

    theta_l1_l2 = math.acos(
        round((d**2 - arm_DH[1][2] ** 2 - arm_DH[2][2] ** 2), 2)
        / (-2 * arm_DH[1][2] * arm_DH[2][2])
    )  # Elbow inner angle with cosine law
    theta_b = math.asin(
        elbow_coordinates[0] / arm_DH[1][2]
    )  # Shoulder angle based on elbow coordinates

    l = math.sqrt(
        (hand_coordinates[0] - elbow_coordinates[0]) ** 2
        + (hand_coordinates[1] - elbow_coordinates[1]) ** 2
    )  # Length of line between hand and elbow
    theta_h = math.acos(
        (l**2 - arm_DH[2][2] ** 2 - arm_DH[-1][0] ** 2)
        / (-2 * arm_DH[-1][0] * arm_DH[2][2])
    )  # Wrist inner angle with cosine law

    # Calculation of adjustment to wrist required to align with wrist standard 0
    robot_normal = np.cross(
        np.array([0, 0, 1]), np.array([ee_target[0], ee_target[1], 0])
    )
    eh_normal = np.cross(np.array(ee_target[:3]) - np.array(elbow_target), robot_normal)
    tmp = eh_normal @ (np.array(wrist_target) - np.array(elbow_target))
    if ee_target[0] < 0:
        tmp = -tmp  # Negative if wrist below hand-elbow line, positive if above

    # Calculates waist rotation with inverse tangent and adjusts to align with waist standard 0 position
    if ee_target[0] >= 0:
        rotation = math.atan2(ee_target[1], ee_target[0])
    elif ee_target[1] >= 0:
        rotation = -math.pi + math.atan2(ee_target[1], ee_target[0])
    else:
        rotation = math.pi + math.atan2(ee_target[1], ee_target[0])

    joint_angles = [
        rotation,  # waist
        theta_b,  # shoulder relative to z axis
        math.pi / 2 - theta_l1_l2,  # elbow angle adjusted for elbow standard 0
        np.sign(tmp)
        * np.abs(theta_h - math.pi),  # wrist angle adjusted for wrist sandard 0
        0,  # hand
    ]

    # Applies rotation if in a reversed position
    if rotate_waist:
        joint_angles[0] += math.pi  # Reverses waist
        joint_angles[1] = -joint_angles[
            1
        ]  # Adjusts shoulder to point along the same line despite rotated waist

    return joint_angles


def inverseKinematicsJointPositions(hand_pose):
    """Calculates the reference frame positions of the joints on the arm

    Parameters
    --------
        hand_pose : np.array(6, 1)
            end effector Cartesian coordinates and XYZ Euler angles

    Returns
    --------
        tuple[list[np.array, np.array, np.array, np.array, np.array]]
            tuple of lists containing the possible joint reference frames of the arm
            Arrays:
                - shoulder reference position [x, y, z]
                - elbow reference position [x, y, z]
                - wrist reference position [x, y, z]
                - wrist reference position [x, y, z]
                - hand reference position [x, y, z]
    """

    hand_pose = np.array(hand_pose)
    T_h = Pose2Mat(hand_pose)

    # Get Wrist position
    wrist_pose = hand_pose[:3] - T_h[:3, 2] * arm_DH[-1][0]

    # Get Shoulder position
    shoulder_pose = np.array([0, 0, arm_DH[0][0]])

    ## Get Possible Elbow position
    # Form Arm Plane Basis Vectors
    basis_x = wrist_pose - shoulder_pose
    d = np.linalg.norm(basis_x)
    if not np.isclose(d, 0):
        basis_x = basis_x / d
    w = np.linalg.norm(wrist_pose)
    if np.isclose(w, 0):
        pass
    arm_plane_normal = np.cross(basis_x, wrist_pose)
    basis_y = np.cross(arm_plane_normal, basis_x)
    basis_y = basis_y / np.linalg.norm(basis_y)

    # Get Position of Link intersections (circles)
    elbow_basis_x = (d**2 - arm_DH[2][2] ** 2 + arm_DH[1][2] ** 2) / (2 * d)
    elbow_basis_y = np.sqrt(arm_DH[1][2] ** 2 - elbow_basis_x**2)

    elbow_pose_1 = elbow_basis_x * basis_x + elbow_basis_y * basis_y + shoulder_pose
    elbow_pose_2 = elbow_basis_x * basis_x - elbow_basis_y * basis_y + shoulder_pose

    return (
        [shoulder_pose, elbow_pose_1, wrist_pose, wrist_pose, hand_pose[:3]],
        [shoulder_pose, elbow_pose_2, wrist_pose, wrist_pose, hand_pose[:3]],
    )


def inverseKinematicsAngleOptions(hand_pose):
    """Calculates the necessary joint positions and selects ideal elbow

    Parameters
    --------
        hand_pose : np.array(6, 1)
            end effector Cartesian coordinates and XYZ Euler angles

    Returns
    --------
        joint_angles : list(float)
            list of angles in radians of joints from base to end effector, relative to
            the last joint
    """

    poses = inverseKinematicsJointPositions(hand_pose)
    wrist_pose = poses[0][2]
    elbow_pose_1 = poses[0][1]
    elbow_pose_2 = poses[1][1]

    return (
        inverseKinematicsComputeJointAngles(hand_pose, wrist_pose, elbow_pose_1, False),
        inverseKinematicsComputeJointAngles(hand_pose, wrist_pose, elbow_pose_1, True),
        inverseKinematicsComputeJointAngles(hand_pose, wrist_pose, elbow_pose_2, False),
        inverseKinematicsComputeJointAngles(hand_pose, wrist_pose, elbow_pose_2, True),
    )


def legalIKPositionPicker(poses, cur_pose):
    """Determines which of the given poses is the best choice.

    Parameters
    --------
        poses : list(list(float))
            list of four sets of five joint angles achieving the same hand position
        cur_pose : list(float)
            list of five joint angles representing the current pose

    Returns
    -------
        best_pose : list(float)
            list of five joint angles representing the best choice of angles
    """
    legal_poses = []
    for pose in poses:
        # Removes positions from contention if they don't abide by the joint limits
        legal = True
        for i in range(len(pose)):
            if pose[i] > jointUpperLimits[i] or pose[i] < jointLowerLimits[i]:
                legal = False
        if legal:
            legal_poses.append(np.array(pose))

    if len(legal_poses) == 0:
        raise AssertionError("Unreachable Position")

    # Typically there should only be one pose in legal_poses, but if not this picks the one closest to cur_pose
    cur_pose = np.array(cur_pose)
    best_pose = legal_poses[0]
    best_difference = np.sum((cur_pose[:-1] - best_pose[:-1]) ** 2)
    for pose in legal_poses:
        if np.sum((cur_pose[:-1] - pose[:-1]) ** 2) < best_difference:
            best_pose = pose
            best_difference = np.sum((cur_pose[:-1] - pose[:-1]) ** 2)

    return best_pose


def inverseKinematics(hand_pose, cur_pose):

    return legalIKPositionPicker(inverseKinematicsAngleOptions(hand_pose), cur_pose)
