import numpy as np
import math

jointUpperLimits = [175*np.pi/180, 90*np.pi/180, 75*np.pi/180, 75*np.pi/180, np.pi]      # rad
jointLowerLimits = [-175*np.pi/180, -60*np.pi/180, -70*np.pi/180, -75*np.pi/180, -np.pi] # rad

arm_DH = [
    [0.0575,                0,     0, -90*math.pi/180], #vertical offset from base
    [     0, -90*math.pi/180,   0.5,               0], #first link
    [     0,  90*math.pi/180,   0.4,               0], #second link
    [     0,  90*math.pi/180,     0,  90*math.pi/180], 
    [ 0.034,               0,     0,               0] #hand 
]

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
    # x = pose[0]
    # y = pose[1]
    # z = pose[2]
    # alpha_x = pose[3]
    # alpha_y = pose[4]
    # alpha_z = pose[5]

    # T = np.array([
    #     np.array([math.cos(alpha_y) * math.cos(alpha_z), -math.cos(alpha_y) * math.sin(alpha_z), math.sin(alpha_y), x]),
    #     np.array([math.sin(alpha_x) * math.sin(alpha_y) * math.cos(alpha_z) + math.cos(alpha_x) * math.sin(alpha_z),
    #               -math.sin(alpha_x) * math.sin(alpha_y) * math.sin(alpha_z) + math.cos(alpha_x) * math.cos(alpha_z),
    #               -math.sin(alpha_x) * math.cos(alpha_y), y]),
    #     np.array([-math.cos(alpha_x) * math.sin(alpha_y) * math.cos(alpha_z) + math.sin(alpha_x) * math.sin(alpha_z),
    #               math.cos(alpha_x) * math.sin(alpha_y) * math.sin(alpha_z) + math.sin(alpha_x) * math.cos(alpha_z),
    #               math.cos(alpha_x) * math.cos(alpha_y), z]),
    #     np.array([0, 0, 0, 1])
    # ])

    alpha = pose[3]
    beta = pose[4]
    gamma = pose[5]

    T = np.eye(4)
    Rx = np.array([
        [1,0,0],
        [0,math.cos(alpha), -math.sin(alpha)],
        [0,math.sin(alpha), math.cos(alpha)],
    ])
    Ry = np.array([
        [math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)],
    ])
    Rz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma), math.cos(gamma), 0],
        [0, 0, 1],
    ])

    T[:3,:3] = Rz @ Ry @ Rx
    T[:3,3] = pose[:3]
    
    return T


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
    # if(T[0][2] < 1):
    #     if(T[0][2] > -1):
    #         alpha_x = math.atan2(-T[1][2], T[2][2])
    #         alpha_y = math.atan2(T[0][2], math.sqrt(1-pow(T[0][2], 2)))
    #         alpha_z = math.atan2(-T[0][1], T[0][0])
    #     else:
    #         alpha_x = math.atan2(-T[1][0], T[1][1])
    #         alpha_y = -math.pi/2
    #         alpha_z = 0

    # else:
    #     alpha_x = math.atan2(T[1][0], T[1][1])
    #     alpha_y = math.pi/2
    #     alpha_z = 0

    # return np.array([T[0][3], T[1][3], T[2][3], alpha_x, alpha_y, alpha_z])

    if abs(T[2,0]) != 1:
        beta = -math.asin(T[2,0])
        alpha = math.atan2(T[2,1]/math.cos(beta), T[2,2]/math.cos(beta))
        gamma = math.atan2(T[1,0]/math.cos(beta), T[0,0]/math.cos(beta))
    else:
        gamma = 0
        if T[2,0] == -1:
            beta = math.pi/2
            alpha = math.atan2(T[0,1], T[0,2])
        else:
            beta = -math.pi/2
            alpha = math.atan2(-T[0,1], -T[0,2])

    return np.array([T[0][3], T[1][3], T[2][3], alpha, beta, gamma])


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
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])


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
    for i in range(numJoints):
        T = dhToMat(arm_DH[i][0], arm_DH[i][1] + q[i], arm_DH[i][2], arm_DH[i][3])
        if len(matrices) > 0:
            T0i = matrices[-1] @ T
        else:
            T0i = T
        matrices.append(T0i)

    return matrices


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
            z0i = np.array([Ts[i-1][0][2], Ts[i-1][1][2], Ts[i-1][2][2]])
            riN = np.array([Ts[-1][0][3] - Ts[i-1][0][3], Ts[-1][1][3] - Ts[i-1][1][3], Ts[-1][2][3] - Ts[i-1][2][3]])
        Jvi = np.cross(z0i, riN)
        Jwi = z0i
        Jv.append(Jvi)
        Jw.append(Jwi)

    Jv = np.array(Jv).T
    Jw = np.array(Jw).T
    J = np.concatenate([Jv, Jw])

    return J, Jv, Jw


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

def project(line, vector):
    """
    Parameters
    --------
        line : list(float)
            two-dimensional vector defining line onto which pojection is done
        vector : list(float)
            two-dimensional vector to be projected
        
    Returns
    --------
        proj_coord : float
            horizontal coordinate on the projection plane
    """

    dot_product = line[0] * vector[0] + line[1] * vector[1]
    length_sqrd = math.pow(line[0], 2) + math.pow(line[1], 2)
    proj_vector = [line[0] * dot_product / length_sqrd, line[1] * dot_product / length_sqrd]

    length = math.sqrt(math.pow(proj_vector[0], 2) + math.pow(proj_vector[1], 2))
    if proj_vector[0] < 0:
        length *= -1

    return length

def calculate_angles(ee_target, wrist_target, elbow_target):
    """Calculates the necessary angles of all joints to achieve the target end effector position

    Parameters
    --------
        ee_target : np.array(6, 1)
            end effector Cartesian coordinates and XYZ Euler angles
        
    Returns
    --------
        joint_angles : list(float)
            list of angles in radians of joints from base to end effector, relative to 
            the last joint
    """
    print(f"TARGET: {ee_target}")

    projection_line = [ee_target[0], ee_target[1]]
    hand_coordinates = (project(projection_line, ee_target[:2]), ee_target[2])
    #print(f"HAND: {hand_coordinates}")

    #wrist_coordinates = (hand_coordinates[0] - math.sin(ee_target[4]) * arm_DH[-1][0], hand_coordinates[1] - math.cos(ee_target[4]) * arm_DH[-1][0]) # projection, z
    wrist_coordinates = (project(projection_line, wrist_target[:2]), wrist_target[2])
    #print(f"WRIST: {wrist_coordinates}")

    elbow_coordinates = (project(projection_line, elbow_target[:2]), elbow_target[2])

    true_base_coordinates = (0, 0, arm_DH[0][0])

    d = math.sqrt(wrist_coordinates[0] ** 2 + (wrist_coordinates[1] - true_base_coordinates[2]) ** 2)
    theta_d = math.acos(wrist_coordinates[0] / d)

    zero_position = Mat2Pose(forwardKinematics([0,0,0,0,0]))

    theta_l1_l2 = math.acos((d ** 2 - arm_DH[1][2] ** 2 - arm_DH[2][2] ** 2) / (-2 * d * arm_DH[1][2]))
    theta_b = math.asin(elbow_coordinates[0] / arm_DH[1][2])

    #theta_inner = math.acos((arm_DH[2][2] ** 2 - arm_DH[1][2] ** 2 - d ** 2) / (-2 * arm_DH[1][2] * d))
    #theta_b = math.pi / 2 - theta_inner - theta_d
    #elbow_coordinates = (math.sqrt(math.pow(elbow_target[0], 2) + math.pow(elbow_target[1], 2)), elbow_target[2])
    l = math.sqrt(round(hand_coordinates[0] - elbow_coordinates[0], 6) ** 2 + (hand_coordinates[1] - elbow_coordinates[1]) ** 2)
    theta_h = math.acos((l ** 2 - arm_DH[2][2] ** 2 - arm_DH[-1][0] ** 2) / (-2 * arm_DH[-1][0] * arm_DH[2][2]))
    
    if ee_target[0] >= 0:
        print("FIRST CASE")
        rotation = math.atan2(ee_target[1], ee_target[0])
    elif ee_target[1] >= 0:
        print("SECOND CASE")
        rotation = -math.pi + math.atan2(ee_target[1], ee_target[0])
    else:
        print("THIRD CASE")
        rotation = math.pi + math.atan2(ee_target[1],  ee_target[0])


    joint_angles = [rotation, # base z
    theta_b, # base angle relative to z axis
    math.pi / 2 - theta_l1_l2, # inner angle between L1 and L2
    theta_h - math.pi,  # inner angle between L2 and hand
    ee_target[3] - zero_position[3]] # wrist rotation 

    return joint_angles

def inverseKinematics(hand_pose, cur_pose): #, joint_truth
    """Calculates the necessary joint positions and selects ideal elbow

    Parameters
    --------
        hand_pose : np.array(6, 1)
            end effector Cartesian coordinates and XYZ Euler angles

        cur_pose : list
            current joint angles 
        
    Returns
    --------
        joint_angles : list(float)
            list of angles in radians of joints from base to end effector, relative to 
            the last joint
    """

    hand_pose = np.array(hand_pose)
    T_h = Pose2Mat(hand_pose)
    #print(f"Hand: {hand_pose[:3]}")

    # Get Wrist position
    wrist_pose = hand_pose[:3] - T_h[:3,2] * arm_DH[-1][0]
    #print(f"Wrist: {wrist_pose}")

    # Get Shoulder position
    shoulder_pose = np.array([0,0,arm_DH[0][0]])
    #print(f"Shoulder: {shoulder_pose}")

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
    elbow_basis_x = (d**2 - arm_DH[2][2]**2 + arm_DH[1][2]**2) / (2*d)
    elbow_basis_y = np.sqrt(arm_DH[1][2]**2 - elbow_basis_x**2) 

    elbow_pose_1 = elbow_basis_x * basis_x + elbow_basis_y * basis_y + shoulder_pose
    elbow_pose_2 = elbow_basis_x * basis_x - elbow_basis_y * basis_y + shoulder_pose
    #print(f"Elbow 1: {elbow_pose_1}")
    #print(f"Elbow 2: {elbow_pose_2}")

    if elbow_pose_1[0] >= 0:
        elbow_direction_1 = math.atan2(elbow_pose_1[1],  elbow_pose_1[0])
    elif elbow_pose_1[1] >= 0:
        elbow_direction_1 = math.pi + math.atan2(elbow_pose_1[1], elbow_pose_1[0])
    else:
        elbow_direction_1 = -math.pi + math.atan2(elbow_pose_1[1], elbow_pose_1[0])

    if elbow_pose_2[0] >= 0:
        elbow_direction_2= math.atan2(elbow_pose_2[1], elbow_pose_2[0])
    elif elbow_pose_2[1] >= 0:
        elbow_direction_2 = math.pi + math.atan2(elbow_pose_2[1], elbow_pose_2[0])
    else:
        elbow_direction_2 = -math.pi + math.atan2(elbow_pose_2[1], elbow_pose_2[0])

    if elbow_pose_1[2] > elbow_pose_2[2]:
        higher = elbow_pose_1
        lower = elbow_pose_2
    else:
        higher = elbow_pose_2
        lower = elbow_pose_1

    if 5*math.pi/180 < abs(elbow_direction_1) < 175*math.pi/180: #always legal
        if (elbow_direction_1 < 0) == (elbow_direction_2 < 0):
            if (elbow_direction_1 < 0) == (cur_pose[0] < 0):
                elbow_pose = higher
            else:
                elbow_pose = lower
        else:
            if (elbow_direction_1 < 0) == (cur_pose[0] < 0):
                elbow_pose = elbow_pose_2
            else:
                elbow_pose = elbow_pose_1
    elif abs(elbow_direction_1) <= 5*math.pi/180: #front
        if (elbow_direction_1 < 0) == (elbow_direction_2 < 0):
            elbow_pose = lower
        else:
            elbow_pose = elbow_pose_1
    else: #back
        if (elbow_direction_1 < 0) == (elbow_direction_2 < 0):
            elbow_pose = higher
        else:
            elbow_pose = elbow_pose_2

    return calculate_angles(hand_pose, wrist_pose, elbow_pose)
"""
    Ts = _FK(joint_truth)

    err = np.zeros(4)
    err[0] = np.linalg.norm(shoulder_pose - Ts[0][:3,3])
    err[1] = np.linalg.norm(hand_pose[:3] - Ts[-1][:3,3])
    err[2] = np.linalg.norm(wrist_pose - Ts[-2][:3,3])
    err[3] = np.min([
        np.linalg.norm(elbow_pose_1 - Ts[1][:3,3]),
        np.linalg.norm(elbow_pose_2 - Ts[1][:3,3]),
    ])

    return np.linalg.norm(err)
    """


if __name__ == '__main__':
    # lst = [math.pi/2, -math.pi/2, math.pi/3, math.pi/2, -math.pi/4]
    # lst = [math.pi/2, 0, math.pi/4, math.pi/2, 0]
    for _ in range(10):
        lst = (np.random.random(5) - 0.5)*np.pi
        pose = forwardKinematics(lst)
        target = Mat2Pose(pose)
        print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")
        q_ik = inverseKinematics(target, [0,0,0,0,0])
        print(f"PRODUCED: {q_ik}")
        # print(f"Result: {Mat2Pose(forwardKinematics(q_ik))}")
        """if q_ik > 1e-6:
            print(lst)
            print(f"Err: {q_ik}")
            break"""
        print("\n------------------------------------------------------------------\n")
