import arm_kinematics as kin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

Ts = kin._FK([0,0,0,0,0])

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.scatter(0, 0, 0)
ax.quiver(0, 0, 0, 1, 0, 0, color="r", length=0.1)
ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=0.1)
ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=0.1)
for T in Ts:
    print(T[:3,-1])
    ax.scatter(T[0,-1], T[1,-1], T[2,-1])
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,0], T[1,0], T[2,0], color="r", length=0.1)
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,1], T[1,1], T[2,1], color='g', length=0.1)
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,2], T[1,2], T[2,2], color='b', length=0.1)

ax.set_xlim(-0.5,0.5)
ax.set_ylim(-0.5,0.5)
ax.set_zlim(-0.01,0.6)

plt.show()