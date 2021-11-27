import numpy as np
import matplotlib.pyplot as plt


def pointGenerator(n):  # generates n points with random coordinates
    pts = np.random.randint(0, 100, (n, 3))
    return pts


def generateBoundingBox(points):  # Generates a bounding box for an nx3 np.array
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    vertices = np.array([[mins[0], maxs[1], maxs[2]],
                         [mins[0], mins[1], maxs[2]],
                         [mins[0], maxs[1], mins[2]],
                         [mins[0], mins[1], mins[2]],
                         [maxs[0], mins[1], mins[2]],
                         [maxs[0], maxs[1], mins[2]],
                         [maxs[0], mins[1], maxs[2]],
                         [maxs[0], maxs[1], maxs[2]]])
    return vertices  # return an 8x3 np.array which are the bouding box vertices


def main():  # Plots the points and their bounding box
    pts = pointGenerator(1000)
    vertices = generateBoundingBox(pts)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], color="blue")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color="red")

    plt.show()


if __name__ == '__main__':
    main()
