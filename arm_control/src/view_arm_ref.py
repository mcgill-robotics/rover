import arm_kinematics as kin
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import math

# q = [math.pi/2, 0, 0, math.pi/2, 0]
#q = [0, 0, 0, 0, 0]
q = [ 0.07144171, -1.22915937, -0.03627638, -0.67306168,  1.36934897]
# q = [math.pi/2, 0, 0, math.pi/2, math.pi]
# q = [math.pi/2, math.pi/4, math.pi/4, -math.pi/2, math.pi/4]
# q = [math.pi/2, -math.pi/2, math.pi/3, math.pi/2, -math.pi/4]
#q = [0, math.pi/2, 0, 0, 0]

Ts = kin._FK(q)

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.scatter(0, 0, 0)
ax.quiver(0, 0, 0, 1, 0, 0, color="r", length=0.1)
ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=0.1)
ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=0.1)
prevPose = [0, 0, 0]

for T in Ts:
    print(T[:3,-1])
    ax.scatter(T[0,-1], T[1,-1], T[2,-1])
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,0], T[1,0], T[2,0], color="r", length=0.1)
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,1], T[1,1], T[2,1], color='g', length=0.1)
    ax.quiver(T[0,-1], T[1,-1], T[2,-1], T[0,2], T[1,2], T[2,2], color='b', length=0.1)
    
    ax.plot([prevPose[0], T[0,-1]], [prevPose[1], T[1,-1]], [prevPose[2], T[2,-1]], color='k')
    prevPose[0] = T[0,-1]
    prevPose[1] = T[1,-1]
    prevPose[2] = T[2,-1]

ax.set_xlim(-0.5,0.5)
ax.set_ylim(-0.5,0.5)
ax.set_zlim(-0.01,0.6)

plt.show()