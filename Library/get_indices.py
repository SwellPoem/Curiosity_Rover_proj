import numpy as np


def get_indices(point, X, Y, mapRes):

    # Compute indices
    j = int((point[0] - X[0, 0] + mapRes) / mapRes) - 1   # column index
    i = int((Y[0, 0] - point[1] + mapRes) / mapRes) - 1   # row index

    return np.array([i, j])

def get_point(indices, X, Y, mapRes):
    # Compute point
    x = X[0, 0] + (indices[1] + 1) * mapRes - mapRes
    y = Y[0, 0] - (indices[0] + 1) * mapRes + mapRes

    return np.array([x, y])

