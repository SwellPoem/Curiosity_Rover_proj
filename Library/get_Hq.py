import numpy as np

# def get_Hq(q, lm):
#     '''
#     this expression was found from an analytical standpoint using symbolic tools.
#     '''

#     x, y = q.item(0), q.item(1)
#     xlm, ylm = np.squeeze(lm)[0:2]

#     Hq = np.array([
#         [np.abs(x-xlm)*np.sign(x-xlm)/(np.sqrt((x-xlm)**2 + (y-ylm)**2)), np.abs(y-ylm)*np.sign(y-ylm)/(np.sqrt((x-xlm)**2 + (y-ylm)**2)), 0], 
#         [-(y-ylm)/((x-xlm)**2 + (y-ylm)**2), (x-xlm)/((x-xlm)**2 + (y-ylm)**2), -1]
#         ])
    
#     return Hq

# def get_Hq(q, lm):
#     """
#     Computes the Jacobian matrix of the h function with respect to the state variables q.

#     :param q: State variable [x, y, theta].
#     :param lm: Landmark coordinates [x_lm, y_lm].
#     :return: Jacobian matrix Hq.
#     """

#     x, y = q.item(0), q.item(1)
#     xlm, ylm = np.squeeze(lm)[0:2]
    
#     delta_x = xlm - x
#     delta_y = ylm - y
#     range_sq = delta_x**2 + delta_y**2
#     range = np.sqrt(range_sq)

#     # Derivatives of range
#     dr_dx = -delta_x / range
#     dr_dy = -delta_y / range
#     dr_dtheta = 0  # Range is independent of theta

#     # Derivatives of bearing angle
#     db_dx = delta_y / range_sq
#     db_dy = -delta_x / range_sq
#     db_dtheta = -1

#     Hq = np.array([[dr_dx, dr_dy, dr_dtheta],
#                    [db_dx, db_dy, db_dtheta]])

#     return Hq