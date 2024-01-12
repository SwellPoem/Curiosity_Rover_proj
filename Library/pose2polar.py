import numpy as np

def pose2polar(P, Pref):
    '''
    this function converts the pose state variables [x, y, theta] 
    to the polar state variables [rho, alpha, beta] 
    wrt a reference pose.
    '''

    if len(P.shape) == 1:
        
        if P.shape[0] != 3:
            raise Exception('Pose State dimensions are not correct!')
        
        x = P[0] - Pref[0]
        y = P[1] - Pref[1]
        theta = P[2]

        dx = -x
        dy = -y

        rho = np.sqrt(dx**2 + dy**2)
        alpha = np.arctan2(dy, dx) - theta
        beta = - alpha - theta + Pref[2]

        R = np.array([rho, alpha, beta])


    elif len(P.shape) == 2:
        
        [n, m] = P.shape

        if n != 3:
            raise Exception('Polar State dimensions are not correct!')
        
        R = np.zeros_like(P)

        for i in range(m):
            
            x = P[0, i] - Pref[0]
            y = P[1, i] - Pref[1]
            theta = P[2, i]

            dx = -x
            dy = -y

            rho = np.sqrt(dx**2 + dy**2)
            alpha = np.arctan2(dy, dx) - theta
            beta = - alpha - theta + Pref[2]

            R[:, i] = np.array([rho, alpha, beta])
    

    return R