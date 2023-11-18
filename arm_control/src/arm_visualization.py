import matplotlib.pyplot as plt
import numpy as np
import arm_pathfinding as ap

start_joints = [0, 0, 0, 0, 0]
end_joints = [-1.5, -.8, 1.1, .5, 1.8]
time = [.55, .3, .4, .18, .65]
max_time = max(time)
num_x = 500
x_values = np.linspace(0, max_time, num_x)
labels = ["Waist", "Shoulder", "Elbow", "Wrist", "Hand"]

for i in range(5):
    polynomial = ap.pathfiningPolynomial(start_joints, end_joints, time[i])[i]
    y_values = []
    j = 0
    while x_values[j] <= time[i]:
        y_values.append(ap.nextJointPosition(start_joints, x_values[j], [polynomial] * 5)[i])
        j += 1
        if j == num_x:
            break
    for x_val in x_values[j:]:
        y_values.append(y_values[-1])
    plt.plot(x_values, y_values, label=labels[i])

    #slopes = np.gradient(y_values, x_values)
    #plt.plot(x_values, slopes, label = labels[i])

plt.title("Joint Positions")
plt.xlabel("Time")
plt.ylabel("Angle")
plt.legend()

plt.show()