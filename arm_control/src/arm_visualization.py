import matplotlib.pyplot as plt
import numpy as np
import arm_pathfinding as ap

start_joints = [0, .5, -1, -0.5, 0]
end_joints = [-1.5, -1, 0.5, 1, 1.5]
time = .5
polynomials = ap.pathfiningPolynomial(start_joints, end_joints, time)
labels = ["Waist", "Shoulder", "Elbow", "Wrist", "Hand"]

x_values = np.linspace(0, time, 100)

for i in range(5):
    polynomial = polynomials[i]
    #print(polynomial)
    y_values = [ap.nextJointPosition(start_joints, x_values[j], polynomials)[i] for j in range(len(x_values))]
    plt.plot(x_values, y_values, label=labels[i])

plt.title("Joint Paths")
plt.xlabel("Time")
plt.ylabel("Angle (Radians)")
plt.legend()

plt.show()