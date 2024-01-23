import numpy as np

def get_Hq(q, lm):
    '''
    this expression was found from an analytical standpoint using symbolic tools.
    '''

    x, y = q.item(0), q.item(1)
    xlm, ylm = np.squeeze(lm)[0:2]

    Hq = np.array([
        [np.abs(x-xlm)*np.sign(x-xlm)/(np.sqrt((x-xlm)**2 + (y-ylm)**2)), np.abs(y-ylm)*np.sign(y-ylm)/(np.sqrt((x-xlm)**2 + (y-ylm)**2)), 0], 
        [-(y-ylm)/((x-xlm)**2 + (y-ylm)**2), (x-xlm)/((x-xlm)**2 + (y-ylm)**2), -1]
        ])
    
    return Hq