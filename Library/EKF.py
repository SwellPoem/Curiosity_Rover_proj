import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_covariance_ellipse(x, P, n_sigma=1, facecolor='none', **kwargs):
    P = P[0:2, 0:2]
    if not(np.all(np.isfinite(P))):
        return

    U, s, _ = np.linalg.svd(P)
    orientation = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
    width = n_sigma * np.sqrt(s[0])
    height = n_sigma * np.sqrt(s[1])

    ellipse = Ellipse(xy=x, width=width, height=height, angle=orientation,
                      facecolor=facecolor, **kwargs)

    plt.gca().add_patch(ellipse)

def EKF(q_real, sigma_d, sigma_theta, L0, lidar_maxrange, sigma_r, sigma_beta, xLM = [], yLM= []):
    
    q_est = [q_real[0]]     # initialize the estimated trajectory
    L_est = [L0]            # initialize covariance matrix

    # Create measurement Covariance Matrix
    V = np.array([[sigma_d**2, 0], [0, sigma_theta**2]])

    Hw = np.array([[1, 0], [0, 1]])
    W = np.array([[sigma_r**2, 0], [0, sigma_beta**2]])     # Measurement Covariance Matrix

    for i in tqdm(range(1, len(q_real)), desc="Processing"):

        #Dead reckoning

        # Odometry measurements in distance and orientation
        delta_d = np.linalg.norm(q_real[i][:2] - q_real[i-1][:2])           # delta in distance
        delta_theta = q_real[i][2] - q_real[i-1][2]                         # delta in orientation

        # Add noise
        noisy_delta_d = delta_d + np.random.normal(0, sigma_d)
        noisy_delta_theta = delta_theta + np.random.normal(0, sigma_theta)

        # Retrieve last State and Covariance matrix
        x, y, theta = q_est[-1]
        L = L_est[-1]

        # Estimate the new State
        x_new = x + noisy_delta_d*np.cos(theta)
        y_new = y + noisy_delta_d*np.sin(theta)
        theta_new = theta + noisy_delta_theta
        q_new = np.array([x_new, y_new, theta_new])

        # Estimate the new Covariance Matrix
        Fq = np.array([[1, 0, -noisy_delta_d*np.sin(theta)], [0, 1, noisy_delta_d*np.cos(theta)], [0, 0, 1]])   # Jacobian of the State
        Fv = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])   # Jacobian of the noise
        L_new = Fq @ L @ Fq.T + Fv @ V @ Fv.T   # Covariance Matrix of the new State (@ represents matrix multiplication)


        # Check for visible Landmarks
        for x_lm, y_lm in zip(xLM, yLM):

            dist_lm = np.linalg.norm([x_lm - x_new, y_lm - y_new])

            if dist_lm < lidar_maxrange:
              
                # Compute the Observables from the real State -> observables that I should see
                # z_real = h(q_real[i], x_lm, y_lm) + np.array([np.random.normal(0, sigma_r), np.random.normal(0, sigma_beta)])
                z_real = h(q_real[i], x_lm, y_lm) + np.array([np.random.normal(0, sigma_r), np.random.normal(0, sigma_beta)])
                z_new = h(q_new, x_lm, y_lm)

                # Compute Kalman Gain
                Hq = get_Hq(q_new, (x_lm, y_lm))
                inv_term = np.linalg.inv(Hq @ L_new @ Hq.T + Hw @ W @ Hw.T)
                K = L_new @ Hq.T @ inv_term     # Kalman gain matrix

                # Perform State Update
                innovation = z_real - z_new
                q_upd = K @ innovation
                q_new = q_new + q_upd
                L_new = (np.eye(3) - K @ Hq) @ L_new

                # print('\nPerforming EKF @ step {:.0f}:\nq_est = '.format(i), q_new, '\nq_upd = ', q_upd)
                

        # Append the new estimated position and orientation
        q_est.append(q_new)
        L_est.append(L_new)

    return np.array(q_est), L_est

def h(q, x_lm, y_lm):
    '''
    this function computes the observables from the state variable and the landmark position.
    '''

    range = np.linalg.norm([x_lm - q[0], y_lm - q[1]])
    bearing_angle = np.arctan2(y_lm - q[1], x_lm - q[0]) - q[2]

    return np.array([range, float(np.squeeze(bearing_angle))])

def get_Hq(q, lm):
    """
    Computes the Jacobian matrix of the h function with respect to the state variables q.

    :param q: State variable [x, y, theta].
    :param lm: Landmark coordinates [x_lm, y_lm].
    :return: Jacobian matrix Hq.
    """

    x, y = q.item(0), q.item(1)
    xlm, ylm = np.squeeze(lm)[0:2]
    
    delta_x = xlm - x
    delta_y = ylm - y
    range_sq = delta_x**2 + delta_y**2
    range = np.sqrt(range_sq)

    # Derivatives of range
    dr_dx = -delta_x / range
    dr_dy = -delta_y / range
    dr_dtheta = 0  # Range is independent of theta

    # Derivatives of bearing angle
    db_dx = delta_y / range_sq
    db_dy = -delta_x / range_sq
    db_dtheta = -1

    Hq = np.array([[dr_dx, dr_dy, dr_dtheta],
                   [db_dx, db_dy, db_dtheta]])

    return Hq