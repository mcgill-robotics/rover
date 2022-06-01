import numpy as np
import math

arm_DH = [
    [0.0575,               0,     0, -90*math.pi/180],
    [     0, -90*math.pi/180,   0.5,               0],
    [     0,  90*math.pi/180,   0.4,               0],
    [     0,  90*math.pi/180,     0,  90*math.pi/180],
    [ 0.034,               0,     0,               0]
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
    x = pose[0]
    y = pose[1]
    z = pose[2]
    alpha_x = pose[3]
    alpha_y = pose[4]
    alpha_z = pose[5]

    T = np.array([
        np.array([math.cos(alpha_y) * math.cos(alpha_z), -math.cos(alpha_y) * math.sin(alpha_z), math.sin(alpha_y), x]),
        np.array([math.sin(alpha_x) * math.sin(alpha_y) * math.cos(alpha_z) + math.cos(alpha_x) * math.sin(alpha_z),
                  -math.sin(alpha_x) * math.sin(alpha_y) * math.sin(alpha_z) + math.cos(alpha_x) * math.cos(alpha_z),
                  -math.sin(alpha_x) * math.cos(alpha_y), y]),
        np.array([-math.cos(alpha_x) * math.sin(alpha_y) * math.cos(alpha_z) + math.sin(alpha_x) * math.sin(alpha_z),
                  math.cos(alpha_x) * math.sin(alpha_y) * math.sin(alpha_z) + math.sin(alpha_x) * math.cos(alpha_z),
                  math.cos(alpha_x) * math.cos(alpha_y), z]),
        np.array([0, 0, 0, 1])
    ])
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
    if(T[0][2] < 1):
        if(T[0][2] > -1):
            alpha_x = math.atan2(-T[1][2], T[2][2])
            alpha_y = math.atan2(T[0][2], math.sqrt(1-pow(T[0][2], 2)))
            alpha_z = math.atan2(-T[0][1], T[0][0])
        else:
            alpha_x = math.atan2(-T[1][0], T[1][1])
            alpha_y = -math.pi/2
            alpha_z = 0

    else:
        alpha_x = math.atan2(T[1][0], T[1][1])
        alpha_y = math.pi/2
        alpha_z = 0

    return np.array([T[0][3], T[1][3], T[2][3], alpha_x, alpha_y, alpha_z])


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
            T0i = matrices[-1].dot(T)
        else:
            T0i = T
        matrices.append(T0i)

    return matrices


def inverseKinematics(posDes, qCur):
    """Computes the joint angles required for the arm
    to have the desired pose knowing its current pose

    This method uses the Damped Least Squares (DLS) algorithm,
    which is a numerical algorithm. The speed of convergence
    can be tuned by changing the damping factor for the particular
    arm we are using the algorithm for.

    Parameters
    --------
        posDes : list(float)
            Desired pose to achieve
        qCur : list(float)
            Current joint configuration of the arm

    Returns
    --------
        np.array(len(qCur))
            joint value for each joint
    """
    q = np.array(qCur)
    posDes = np.array(posDes)

    # Constants
    de = float("Inf")
    thres = 1e-6
    damping = 1.5

    while np.linalg.norm(de) > thres:
        J, Jv, Jw = Jacobian(q)
        
        T = forwardKinematics(q)
        posCur = Mat2Pose(T)
        de = posDes - posCur[0:3]

        dq = (Jv.T).dot(
            Jv.dot(Jv.T) + damping**2*np.eye(de.shape[0])
        ).dot(de.T)

        q = q + dq

    return q


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