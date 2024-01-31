import numpy as np
from Library.move2pose import move2pose

#K=[1e-2, 2, -10]
def kinematicModel(t, S, K, v_values, omega_values, alpha_values):
    '''
    S contains the polar state variables [rho, alpha, beta].
    '''

    # Unpack State Variables
    rho = S[0]
    alpha = S[1]
    beta = S[2]

    # Compute Control
    v, omega = move2pose(S, K)

    # Store v and omega in the global variables
    v_values.append(v)
    omega_values.append(omega)
    alpha_values.append(alpha)

    # Compute Derivatives
    drho = -np.cos(alpha) * v
    dalpha = np.sin(alpha) * v / rho - omega
    dbeta = - np.sin(alpha) * v / rho

    dS = np.array([drho, dalpha, dbeta])

    return dS, v_values, omega_values, alpha_values