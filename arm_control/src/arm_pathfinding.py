import numpy as np
import math

max_acc = [500, 500, 500, 500, 500] # waist shoulder elbow wrist hand maximum accelerations (TEMPORARY VALUES)
max_vels = [5, 5, 5, 5, 5] # waist shoulder elbow wrist hand maximum accelerations (TEMPORARY VALUES)
previous_end_joints = [None for i in range(5)]
polynomials = [[0 for j in range(4)] for i in range(5)]
total_motion_time = 0
original_start_joints = [0 for i in range(5)]

def pathfind(start_joints, end_joints, time):
    """
    Generates the path polynomial for each joint and calculates the required position for a given time remaining.
    Note: If time is too long or maximum velocity is too high, it will reverse and then overshoot before landing
    on the desired position.

    Parameters
    ----------
        start_joints : list(5)
            Current [waist, shoulder, elbow, wrist, hand] in radians
        end_joints : list(5)
            Desired [waist, shoulder, elbow, wrist, hand] in radians
        time : float
            Time until the arm should be at end_joints. Should go down by the time between each call of
            this function until the arm reaches end_joints, at which point it should reset for the next
            desired position.
    
    Returns
    -------
        joints : list(5)
            The next position the arm should move to in accordance with the path
    """
    if end_joints != previous_end_joints: #Setting up polynomials and saved variables
        half = time / 2
        matrix = np.array([[time ** 6, time ** 5, time ** 4, time ** 3], 
        [6 * time ** 5, 5 * time ** 4, 4 * time ** 3, 3 * time ** 2],
        [6 * half ** 5, 5 * half ** 4, 4 * half ** 3, 3 * half ** 2],
        [30 * half ** 4, 20 * half ** 3, 12 * half ** 2, 6 * half]])

        for i in range(len(polynomials)):
            points = np.array([end_joints[i] - start_joints[i], 0, max_vels[i], 0])
            poly = np.linalg.solve(matrix, points)
            for j in range(4):
                polynomials[i][j] = poly[j]
            previous_end_joints[i] = end_joints[i]
            original_start_joints[i] = start_joints[i]
        
        global total_motion_time
        total_motion_time = time

    joints = [] #Calculating positions
    for i in range(len(polynomials)):
        polynomial = polynomials[i]
        delta_time = total_motion_time - time
        delta_position = polynomial[0] * delta_time ** 6 + polynomial[1] * delta_time ** 5 + polynomial[2] * delta_time ** 4 + polynomial[3] * delta_time ** 3
        joints.append(delta_position + original_start_joints[i])

    return joints

def pathfiningPolynomial(start_joints, end_joints, time):
    """
    Generates the path polynomial for each joint. Does NOT store these polynomials as global variables.
    Note: If time is too long or maximum velocity is too high, it will reverse and then overshoot before landing
    on the desired position.

    Parameters
    ----------
        start_joints : list(5)
            Current [waist, shoulder, elbow, wrist, hand] in radians
        end_joints : list(5)
            Desired [waist, shoulder, elbow, wrist, hand] in radians
        time : float
            Time until the arm should be at end_joints. 
    
    Returns
    -------
        polynomials : list(5)(4)
            The polynomial coefficients governing the motion of the waist, shoudler, elbow, 
            wrist, and hand respectively.
    """
    half = time / 2
    vels = [max_acc[i] * half for i in range(5)] # waist shoulder elbow wrist hand maximum velocities 
    for i in range(5):
        if vels[i] > max_vels[i]:
            vels[i] = max_vels[i]
        if end_joints[i] < start_joints[i]:
            vels[i] = -vels[i]
    polynomials = [[0 for j in range(4)] for i in range(5)]
    matrix = np.array([[time ** 6, time ** 5, time ** 4, time ** 3], 
        [6 * time ** 5, 5 * time ** 4, 4 * time ** 3, 3 * time ** 2],
        [6 * half ** 5, 5 * half ** 4, 4 * half ** 3, 3 * half ** 2],
        [30 * half ** 4, 20 * half ** 3, 12 * half ** 2, 6 * half]])

    for i in range(len(polynomials)):
        points = np.array([end_joints[i] - start_joints[i], 0, vels[i], 0])
        poly = np.linalg.solve(matrix, points)
        for j in range(4):
            polynomials[i][j] = poly[j]

    return polynomials

def nextJointPosition(start_position, time_elapsed, polynomials):
    """ Calculates the desired joint positions after time_elapsed time has passed since the
    motion began, as govenerned by the polynomials.

    Params
    ------
        start_joints : list(5)
            The original position [waist, shoulder, elbow, wrist, hand] in radians when the
            motion began.
        time_elapsed : int
            The time since the motion began
        polynomials : list(5)(4)
            The polynomial coefficients governing the motion of the waist, shoudler, elbow, 
            wrist, and hand respectively.
    
    Returns
    -------
        joints : list(5)
            The next position the arm should move to in accordance with the path
    """
    joints = [] #Calculating positions
    for i in range(len(polynomials)):
        polynomial = polynomials[i]
        delta_position = polynomial[0] * time_elapsed ** 6 + polynomial[1] * time_elapsed ** 5 + polynomial[2] * time_elapsed ** 4 + polynomial[3] * time_elapsed ** 3
        joints.append(delta_position + start_position[i])
    
    return joints
    
if __name__ == '__main__':
    time = 10
    start = [0, 0, 0, 0, 0]
    end = [-1.5, 2, 2, .5, .75]
    for t in range(time, -1, -1):
        start = pathfind(start, end, t)
        print(start)
