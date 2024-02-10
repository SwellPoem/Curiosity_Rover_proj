import numpy as np
import matplotlib.cm as cm
from tqdm import tqdm
from matplotlib.patches import Ellipse

def compute_and_plot_ellipses(q_est, L_est, plt, frequency, n_sigma):
    n = int(1 / frequency)  # number of measurements acquisition per second
    cmap = cm.get_cmap('Reds')  # load the colormap
    num_ellipses = len(q_est) // n  # calculate the number of ellipses
    for i in range(0, len(q_est), n):
        # Compute the ellipse of uncertainty
        L = L_est[i]
        s, U = np.linalg.eig(L[0:2, 0:2])
        orientation = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        width = np.sqrt(s[0])
        height = np.sqrt(s[1])
        # Get a color from the colormap
        color = cmap(i / num_ellipses)
        # Plot the ellipse
        ellipse = Ellipse(xy=q_est[i, :2], width=width*n_sigma, height=height*n_sigma, angle=orientation, facecolor='none', edgecolor=color, alpha=0.8)
        plt.gca().add_patch(ellipse)

def EKF(q_real, sigma_d, sigma_theta, L0, lidar_maxrange, sigma_r, sigma_beta, noise_d_arr, noise_theta_arr, noise_r_arr, noise_beta_arr, xLM=[], yLM=[]):
    
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
        noise_d = np.random.normal(loc=0, scale=sigma_d)
        noise_theta = np.random.normal(loc=0, scale=sigma_theta)

        noisy_delta_theta = delta_theta + noise_theta

        noise_d_arr.append(noise_d)
        noise_theta_arr.append(noise_theta)
        
        # Retrieve last State and Covariance matrix
        x, y, theta = q_est[-1]
        L = L_est[-1]

        # Estimate the new State
        x_new = x + (delta_d+noise_d)*np.cos(theta)
        y_new = y + (delta_d+noise_d)*np.sin(theta)
        theta_new = theta + noisy_delta_theta
        q_new = np.array([x_new, y_new, theta_new])

        # Estimate the new Covariance Matrix
        Fq = np.array([[1, 0, -delta_d*np.sin(theta)], [0, 1, delta_d*np.cos(theta)], [0, 0, 1]])   # Jacobian of the State
        Fv = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])   # Jacobian of the noise
        L_new = Fq @ L @ Fq.T + Fv @ V @ Fv.T   # Covariance Matrix of the new State (@ represents matrix multiplication)

        # EKF
        # Check for visible Landmarks
        for x_lm, y_lm in zip(xLM, yLM):

            dist_lm = np.sqrt((x_lm - q_real[i][0])**2 + (y_lm - q_real[i][1])**2)

            noise_r = np.random.normal(loc=0, scale=sigma_r)
            noise_beta = np.random.normal(loc=0, scale=sigma_beta)

            if dist_lm < lidar_maxrange:
                # print(dist_lm)
                # Compute the Observables from the real State
                z_real = h(q_real[i], x_lm, y_lm) + np.array([noise_r, noise_beta])
                z_new = h(q_new, x_lm, y_lm)

                noise_r_arr.append(noise_r)
                noise_beta_arr.append(noise_beta)

                # Compute Kalman Gain
                Hq = get_Hq(q_new, (x_lm, y_lm))
                inv_term = np.linalg.inv(Hq @ L_new @ Hq.T + Hw @ W @ Hw.T)
                K = L_new @ Hq.T @ inv_term     # Kalman gain matrix

                # Perform State Update
                innovation = z_real - z_new
                q_upd = K @ innovation
                q_new = q_new + q_upd
                L_new = (np.eye(3) - K @ Hq) @ L_new   

        # Append the new estimated position and orientation
        q_est.append(q_new)
        L_est.append(L_new)

    return np.array(q_est), L_est, noise_d_arr, noise_theta_arr, noise_r_arr, noise_beta_arr

def h(q, x_lm, y_lm):
    '''
    this function computes the observables from the state variable and the landmark position.
    '''

    range = np.linalg.norm([q[0] - x_lm, q[1] - y_lm])
    bearing_angle = np.arctan2(y_lm - q[1], x_lm- q[0]) - q[2]

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
    # range = np.linalg.norm([delta_x, delta_y])

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