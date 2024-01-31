import numpy as np

def pose2polar(P_a, P_b):
    '''
    this function converts the pose state variables [x, y, theta] 
    to the polar state variables [rho, alpha, beta] 
    wrt a reference pose.
    '''

    if len(P_a.shape) == 1:
        
        if P_a.shape[0] != 3:
            raise Exception('Pose State dimensions are not correct!')
        
        dx = P_b[0] - P_a[0]
        dy = P_b[1] - P_a[1]
        theta = P_a[2]

        rho = np.sqrt(dx**2 + dy**2)
        alpha = np.arctan2(dy, dx) - theta
        beta = - alpha - theta + P_b[2]

        R = np.array([rho, alpha, beta])


    elif len(P_a.shape) == 2:
        
        [n, m] = P_a.shape

        if n != 3:
            raise Exception('Polar State dimensions are not correct!')
        
        R = np.zeros_like(P_a)

        for i in range(m):
            
            dx = P_b[0] - P_a[0, i]
            dy =P_b[1] - P_a[1, i]
            theta = P_a[2, i]

            rho = np.sqrt(dx**2 + dy**2)
            alpha = np.arctan2(dy, dx) - theta
            beta = - alpha - theta + P_b[2]

            R[:, i] = np.array([rho, alpha, beta])
    

    return R