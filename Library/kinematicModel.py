import numpy as np
from Library.move2pose import move2pose

def kinematicModel(t, state, K):
    '''
    S contains the polar state variables [rho, alpha, beta].
    '''

    # Unpack State Variables
    rho = state[0]
    alpha = state[1]
    beta = state[2]

    # Compute Control
    v, omega = move2pose(state, K)

    # Compute Derivatives
    drho = -np.cos(alpha) * v
    dalpha = np.sin(alpha) * v / rho - omega
    dbeta = - np.sin(alpha) * v / rho

    dS = np.array([drho, dalpha, dbeta])

    return dS, v, omega