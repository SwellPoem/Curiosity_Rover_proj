import numpy as np

def polar2pose(R, Pref):
    '''
    this function converts the polar state variables [rho, alpha, beta] 
    to the pose state variables [x, y, theta]
    '''

    if len(R.shape) == 1:
        
        if R.shape[0] != 3:
            raise Exception('Polar State dimensions are not correct!')
        
        rho = R[0]
        alpha = R[1]
        beta = R[2]

        theta = - alpha - beta + Pref[2]
        x = - rho * np.cos(alpha + theta) + Pref[0]
        y = - rho * np.sin(alpha + theta) + Pref[1]

        P = np.array([x, y, theta])
        


    elif len(R.shape) == 2:
        
        [n, m] = R.shape

        if n != 3:
            raise Exception('Polar State dimensions are not correct!')
        
        P = np.zeros_like(R)

        for i in range(m):
            
            rho = R[0, i]
            alpha = R[1, i]
            beta = R[2, i]

            theta = - alpha - beta + Pref[2]
            x = - rho * np.cos(alpha + theta) + Pref[0]
            y = - rho * np.sin(alpha + theta) + Pref[1]

            P[:, i] = np.array([x, y, theta])


    return P