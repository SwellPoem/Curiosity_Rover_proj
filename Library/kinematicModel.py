import numpy as np
from Library.move2pose import move2pose

#K=[1e-2, 2, -10]

def kinematicModel(t, S, K):
    '''
    S contains the polar state variables [rho, alpha, beta].
    '''

    # Unpack State Variables
    rho = S[0]
    alpha = S[1]
    beta = S[2]

    # Compute Control
    v, omega = move2pose(S, K)

    # Compute Derivatives
    drho = -np.cos(alpha) * v
    dalpha = np.sin(alpha) * v / rho - omega
    dbeta = - np.sin(alpha) * v / rho

    dS = np.array([drho, dalpha, dbeta])

    return dS