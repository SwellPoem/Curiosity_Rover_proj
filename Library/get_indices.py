import numpy as np


def get_indices(point, X, Y, mapRes):

    # Compute indices
    j = int((point[0] - X[0, 0] + mapRes) / mapRes) - 1   # column index
    i = int((Y[0, 0] - point[1] + mapRes) / mapRes) - 1   # row index

    return np.array([i, j])
