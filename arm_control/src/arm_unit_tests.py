import numpy as np
import math
import arm_kinematics
import arm_pathfinding

jointUpperLimits = [118.76*np.pi/180, 90*np.pi/180, 75*np.pi/180, 75*np.pi/180, np.pi]      # rad
jointLowerLimits = [-125.97*np.pi/180, -60*np.pi/180, -70*np.pi/180, -75*np.pi/180, -np.pi] # rad

def test_inverseKinematicsJointPositions(num_sample_points=10000, verbose=False):
    print("------------------------------------------------------------------")
    print("---------------test_inverseKinematicsJointPositions---------------")
    print("------------------------------------------------------------------")
    failed_cases = 0
    for _ in range(num_sample_points):
        lst = (np.random.random(5) - 0.5)*np.pi
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        if verbose:
            print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        poses_1, poses_2 = arm_kinematics.inverseKinematicsJointPositions(target)
        ref_poses = arm_kinematics._FK(lst)
        err_1 = 0
        err_2 = 0
        for i in range(len(poses_1)):
            T = ref_poses[i]
            err_1 += np.linalg.norm(T[:3,3] - poses_1[i])
            err_2 += np.linalg.norm(T[:3,3] - poses_2[i])
            if verbose:
                print(f"Ref : {T[:3,3]}")
                print(f"Pose1 : {poses_1[i]}")
                print(f"Pose2 : {poses_2[i]}")

        if verbose :
            print(f"Error 1: {err_1}\nError 2: {err_2}")
        if np.min([err_1, err_2]) > 1e-6:
            print("\n------------------------------------------------------------------\n")
            print(f"Query pose: {lst}")
            print(f"Err: {np.argmin([err_1, err_2])}:{np.min([err_1, err_2])}")
            print("\n------------------------------------------------------------------\n")
            failed_cases += 1
    print("------------------------------------------------------------------")
    print(f"\t\tFailed cases : {failed_cases}")
    print("------------------------------------------------------------------")
    return failed_cases

def test_inverseKinematicsComputeJointAngles(num_samples=10000, verbose = False):
    print("------------------------------------------------------------------")
    print("-------------test_inverseKinematicsComputeJointAngles-------------")
    print("------------------------------------------------------------------")
    failed_cases = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5)*np.pi
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        if verbose:
            print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        joint_options = arm_kinematics.inverseKinematicsAngleOptions(target, [0,0,0,0,0])
        err = [0 for _ in range(len(joint_options))]
        for i in range(len(joint_options)):
            err[i] += np.sum((lst[:-1] - joint_options[i][:-1])**2)

        if np.min(err) > 1e-1:
            print(f"Expected: {lst}")
            print(f"Options:\n{joint_options[0]}\n{joint_options[1]}\n{joint_options[2]}\n{joint_options[3]}")
            print(f"Error: {np.argmin(err)}:{err}")
            print(f"Individual Error:\n{lst-joint_options[0]}\n{lst-joint_options[1]}\n{lst-joint_options[2]}\n{lst-joint_options[3]}")
            print("\n------------------------------------------------------------------\n")
            failed_cases += 1
        if verbose:
            print(f"Found angles: {joint_options[np.argmin(err)]}\nOption {np.argmin(err)}")
            print("\n------------------------------------------------------------------\n")

    print("------------------------------------------------------------------")
    print(f"\t\tFailed cases : {failed_cases}")
    print("------------------------------------------------------------------")
    return failed_cases

def test_inverseKinematics(num_samples = 1000, verbose=False):
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5)*np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        #if verbose:
            #print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        try:
            joint = arm_kinematics.inverseKinematics(target, lst)
        except AssertionError:
            #print(f"Unreachable position. Target: {target} Lst: {lst}")
            continue
        err = np.sum((lst[:-1] - joint[:-1])**2)

        if np.min(err) > 1e-1:
            if verbose:
                print(f"Expected: {lst}")
                print(f"Error: {err}")
            failed += 1
        if verbose:
            print(f"Found angles: {joint}")
            print("\n------------------------------------------------------------------\n")

    print(f"Ratio: {(num_samples - failed) / num_samples * 100}%")
    return failed

def test_pathfind(num_samples = 1000, max_velocities=[0.1, 0.1, 0.1, 0.1, 0.1], verbose=False):
    print("------------------------------------------------------------------")
    print("-------------------------test_pathfind----------------------------")
    print("------------------------------------------------------------------")
    num_failed = 0
    for i in range(num_samples):
        failed = False

        start_joints = [np.random.random() * (jointUpperLimits[i]-jointLowerLimits[i]) + jointLowerLimits[i] for i in range(5)]
        end_joints = [np.random.random() * (jointUpperLimits[i]-jointLowerLimits[i]) + jointLowerLimits[i] for i in range(5)]
        differences = [start_joints[i] - end_joints[i] for i in range(5)]
        min_time = max([abs((differences[i])/max_velocities[i]) for i in range(5)])
        # TODO: figure out how to calculate max time???
        max_distance = max(abs(differences[i]) for i in range(5)) # Using furthest distance to get a reasonable max time
        time = min_time + np.random.random() * max_distance * 10
    
        if verbose:
            print(f"Start Joints: {start_joints} End Joints: {end_joints} Time: {time}")

        polynomials = arm_pathfinding.pathfiningPolynomial(start_joints, end_joints, time)
        for k, polynomial in enumerate(polynomials):
            if sum([polynomial[j] * math.pow(time, 6 - j) for j in range(4)]) != differences[k]: # Checking final position
                failed = True
                if verbose:
                    given = sum([polynomial[j] * math.pow(time, 6 - j) for j in range(4)])
                    print("Polynoial failed final position.")
                    print(f"Result: {given} Difference: {differences[k] - given}")
                break
            
            if sum([polynomial[j] * (6 - j) * math.pow(time/2, 5 - j) for j in range(4)]) != max_velocities[k]: # Checking max velocity
                failed = True
                if verbose:
                    given = sum([polynomial[j] * (6 - j) * math.pow(time/2, 5 - j) for j in range(4)])
                    print("Polynoial failed midway velocity.")
                    print(f"Result: {given} Difference: {max_velocities[k] - given}")
                break

            if sum([polynomial[j] * (6 - j) * math.pow(time, 5 - j) for j in range(4)]) != 0: # Checking final velocity
                failed = True
                if verbose:
                    given = sum([polynomial[j] * (6 - j) * math.pow(time, 5 - j) for j in range(4)])
                    print("Polynoial failed final velocity.")
                    print(f"Result: {given}")
                break

            if sum([polynomial[j] * (6 - j) * (5 - j) * math.pow(time/2, 4 - j) for j in range(4)]) != 0: # Check halfway acceleration
                failed = True
                if verbose:
                    given = sum([polynomial[j] * (6 - j) * (5 - j) * math.pow(time/2, 4 - j) for j in range(4)])
                    print("Polynoial failed midway acceleration.")
                    print(f"Result: {given}")
                break
        
        if not failed and end_joints != arm_pathfinding.nextJointPosition(start_joints, time, polynomials):
            failed = True
            if verbose:
                    given = arm_pathfinding.nextJointPosition(start_joints, time, polynomials)
                    print("nextJointPosition failed final position.")
                    print(f"Result: {given} Difference: {[end_joints[j] - given[j] for j in range(5)]}")
        
        if not failed and end_joints != arm_pathfinding.pathfind(start_joints, end_joints, time):
            failed = True
            if verbose:
                    given = arm_pathfinding.pathfind(start_joints, end_joints, time)
                    print("Pathfind failed final position.")
                    print(f"Result: {given} Difference: {[end_joints[j] - given[j] for j in range(5)]}")
        
        if not failed and start_joints != arm_pathfinding.nextJointPosition(start_joints, 0, polynomials):
            failed = True
            if verbose:
                    given = arm_pathfinding.nextJointPosition(start_joints, 0, polynomials)
                    print("nextJointPosition failed initial position.")
                    print(f"Result: {given} Difference: {[start_joints[j] - given[j] for j in range(5)]}")
        
        if not failed and start_joints != arm_pathfinding.pathfind(start_joints, end_joints, 0):
            failed = True
            if verbose:
                    given = arm_pathfinding.pathfind(start_joints, end_joints, 0)
                    print("Pathfnd failed initial position.")
                    print(f"Result: {given} Difference: {[start_joints[j] - given[j] for j in range(5)]}")
        
        if failed:
            num_failed += 1
            print("------------------------------------------------------------------")
            print(f"Test Case {i} failed.")
            print("------------------------------------------------------------------")
    
    print(f"Ratio: {(num_samples - num_failed) / num_samples * 100}%")
    return num_failed
        
        

if __name__=="__main__":
    test_pathfind(verbose=True)
    test_inverseKinematicsJointPositions()
    test_inverseKinematicsComputeJointAngles()
    test_inverseKinematics()
