import matplotlib.pyplot as plt
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from sklearn.cluster import KMeans
import numpy as np


def pointGenerator(n):  # generates n 3-dimensional points with random coordinates
    pts = np.random.randint(0, 100, (n, 3))
    return pts


pts = pointGenerator(30)
km = KMeans(n_clusters=5)
pred = km.fit_predict(pts)
data = np.column_stack((pts, np.transpose(pred)))
print(data)

clst1, clst2, clst3, clst4, clst5 = data[data[:, 3] == 0], data[data[:, 3] == 1], data[data[:, 3] == 2], data[data[:, 3] == 3], data[data[:, 3] == 4]
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(clst1[:, 0], clst1[:, 1], clst1[:, 2], color="blue")
ax.scatter(clst2[:, 0], clst2[:, 1], clst2[:, 2], color="red")
ax.scatter(clst3[:, 0], clst3[:, 1], clst3[:, 2], color="green")
ax.scatter(clst4[:, 0], clst4[:, 1], clst4[:, 2], color="orange")
ax.scatter(clst5[:, 0], clst5[:, 1], clst5[:, 2], color="purple")

plt.show()
