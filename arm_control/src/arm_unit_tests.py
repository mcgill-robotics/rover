import numpy as np
import math
import arm_kinematics

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

def test_inverseKinematics(num_samples = 1000, verbose=False):
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5)*np.pi
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        if verbose:
            print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        joint = arm_kinematics.inverseKinematics(target, [0,0,0,0,0])
        err = np.sum((lst[:-1] - joint[:-1])**2)

        if np.min(err) > 1e-1:
            if verbose:
                print(f"Expected: {lst}")
                print(f"Options:\n{joint}")
                print(f"Error: {err}")
            failed += 1
        if verbose:
            print(f"Found angles: {joint}")
            print("\n------------------------------------------------------------------\n")

    print(f"Ratio: {(num_samples - failed) / num_samples * 100}%")
    return failed

if __name__=="__main__":
    # test_inverseKinematicsJointPositions()
    test_inverseKinematics()

