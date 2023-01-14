import numpy as np
import math

max_vels = [5, 5, 5, 5, 5] # waist shoulder elbow wrist hand maximum velocities (TEMPORARY VALUES)
previous_end_joints = []
polynomials = [[] for i in range(5)]
total_motion_time = 0
original_start_joints = []

def pathfind(start_joints, end_joints, time):
    """
    Generates the path polynomial for each joints and calculates the required position for a given time remaining

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
            polynomials[i] = list(np.linalg.solve(matrix, points))
        
        previous_end_joints = end_joints
        original_start_joints = start_joints
        total_motion_time = time

    joints = [] #Calculating positions
    for polynomial in polynomials:
        delta_time = total_motion_time - time
        delta_position = polynomial[0] * delta_time ** 6 + polynomial[1] * delta_time ** 5 + polynomial[2] * delta_time ** 4 + polynomial[3] * delta_time ** 3
        joints.append(delta_position + original_start_joints)

    return joints
    
if __name__ == '__main__':
    time = 30
    start = [0, 0, 0, 0, 0]
    end = [1, 1, 1, 1, 1]
    for i in range(time, 0):
        start = pathfind(start, end, time)
        print(start)
