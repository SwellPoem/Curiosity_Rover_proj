
def move2pose(state, K):
    '''
    this function implements the Moving to a Pose control law
    ...
    this function is called by the kinematicModel function
    '''

    # Unpack State Variables
    rho = state[0]
    alpha = state[1]
    beta = state[2]

    # Define Gains
    Krho = K[0]
    Kalpha = K[1]
    Kbeta = K[2]

    # Compute Control
    v = Krho * rho
    omega = Kalpha * alpha + Kbeta * beta

    # Set Control Constraints
    v_max = 4e-2    # m/s - max speed
    
    if v > v_max:
        v = v_max

    # print(v, omega)
        
    return v, omega