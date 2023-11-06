import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

# This file contains functions useful functions for simulated map generation and border defining

def _points_in_one_dimension(xy):
    x_values = xy[:, 0]  # extract all x values
    y_values = xy[:, 1]  # extract all y values
    return (x_values == x_values[0]).all() or (y_values == y_values[0]).all()

def get_all_hulls_vertices(X: np.ndarray):
    # X is (n, 2) numpy ndarray where n is number of points
    db = DBSCAN(eps=0.3, min_samples=5).fit(X)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    unique_labels = set(labels)

    for k in unique_labels:
        if k == -1 or _points_in_one_dimension(X[labels == k]):
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = labels == k

        xy = X[class_member_mask]

        if k != -1 and not _points_in_one_dimension(xy):
            hull = ConvexHull(xy)
            yield hull.points[hull.vertices]


def display_bounding_boxes(X: np.ndarray):
    for hull_points in get_all_hulls_vertices(X):
        plt.plot(hull_points[:, 0], hull_points[:, 1], 'r--', lw=2)
        plt.plot(hull_points[:, 0], hull_points[:, 1], 'ro')

    # plt.title(f"Estimated number of clusters: {n_clusters_}")
    plt.show()

def generate_points(n, shape='circle', center=(0, 0), radius=1):
    if shape == 'circle':
        angles = np.linspace(0, 2*np.pi, n)
        x = center[0] + radius * np.cos(angles)
        y = center[1] + radius * np.sin(angles)
        return np.column_stack((x, y))
    elif shape == 'square':
        side = np.sqrt(n)
        if not side.is_integer():
            raise ValueError('Number of points is not compatible with a square shape.')
        side = int(side)
        x = np.linspace(center[0] - radius, center[0] + radius, side)
        y = np.linspace(center[1] - radius, center[1] + radius, side)
        xx, yy = np.meshgrid(x, y)
        return np.column_stack((xx.flatten(), yy.flatten()))
    else:
        raise ValueError('Unsupported shape.')

def generate_groups(num_groups, num_points, shapes, radii, centers):
    groups = []
    for _ in range(num_groups):
        group = []
        for shape, radius, center in zip(shapes, radii, centers):
            points = generate_points(num_points, shape, center=center, radius=radius)
            group.append(points)
        groups.append(group)
        #print(groups)
    return groups

def plot_groups(groups):
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    for group_idx, group in enumerate(groups):
        for shape_idx, points in enumerate(group):
            color = colors[shape_idx % len(colors)]
            plt.scatter(points[:, 0], points[:, 1], c=color, label=f'Shape {shape_idx+1}')
        plt.title(f'Group {group_idx+1}')
        plt.legend()
        plt.show()


def generate_rectangle_borders(ox, oy, bottom, top, left, right):
    # the o in ox, oy stands for obstacle
    for i in range(left, right):
        # Add bottom border
        ox.append(i)
        oy.append(bottom)
        # Add Top border
        ox.append(top)
        oy.append(i)
    for i in range(bottom, top):
        # Add left border
        ox.append(left)
        oy.append(i)
        # Add right border
        ox.append(i)
        oy.append(right)
    

