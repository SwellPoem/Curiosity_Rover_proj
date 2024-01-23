import numpy as np

def h(q, x_lm, y_lm):
    '''
    this function computes the observables from the state variable and the landmark position.
    '''

    range = np.linalg.norm([x_lm - q[0], y_lm - q[1]])
    bearing_angle = np.arctan2(y_lm - q[1], x_lm - q[0]) - q[2]

    return np.array([range, float(np.squeeze(bearing_angle))])