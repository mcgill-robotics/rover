from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

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
