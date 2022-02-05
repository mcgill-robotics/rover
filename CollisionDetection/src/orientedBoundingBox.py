import numpy as np
import matplotlib.pyplot as plt


def pointGenerator(n):  # generates n points with random coordinates
    pts = np.random.randint(0, 100, (n, 3))
    return pts


def generateOrientedBB(points):  # Generates an oriented bounding box for an nx3 np.array
    V = np.linalg.svd(points)[2]
    B = np.dot(points, V)
    print(B)

    mins = np.min(B, axis=0)
    maxs = np.max(B, axis=0)
    vertices = np.array([[mins[0], maxs[1], maxs[2]],
                         [mins[0], mins[1], maxs[2]],
                         [mins[0], maxs[1], mins[2]],
                         [mins[0], mins[1], mins[2]],
                         [maxs[0], mins[1], mins[2]],
                         [maxs[0], maxs[1], mins[2]],
                         [maxs[0], mins[1], maxs[2]],
                         [maxs[0], maxs[1], maxs[2]]])

    return np.dot(vertices, V.transpose())  # return an 8x3 np.array which are the oriented bounding box vertices


def main():  # Plots the points and their bounding box
    pts = pointGenerator(100)
    vertices = generateOrientedBB(pts)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], color="blue")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color="red")

    plt.show()


if __name__ == '__main__':
    main()
