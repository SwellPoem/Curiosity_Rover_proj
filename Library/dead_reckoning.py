import matplotlib.cm as cm
import numpy as np
from tqdm import tqdm
from matplotlib.patches import Ellipse

def dead_reckoning(q_real, sigma_d, sigma_theta, L0):
    
    q_est = [q_real[0]]     # initialize the estimated trajectory
    L_est = [L0]            # initialize covariance matrix

    # Create measurement Covariance Matrix
    V = np.array([[sigma_d**2, 0], [0, sigma_theta**2]])

    for i in tqdm(range(1, len(q_real)), desc="Processing"):

        #Dead reckoning

        # Odometry measurements in distance and orientation
        delta_d = np.linalg.norm(q_real[i][:2] - q_real[i-1][:2])           # delta in distance
        delta_theta = q_real[i][2] - q_real[i-1][2]                         # delta in orientation

        # Add noise
        # noisy_delta_d = delta_d + np.random.normal(loc=0, scale=sigma_d)
        noisy_delta_theta = delta_theta + np.random.normal(loc=0, scale=sigma_theta)

        # Retrieve last State and Covariance matrix
        x, y, theta = q_est[-1]
        L = L_est[-1]

        # Estimate the new State
        x_new = x + delta_d*np.cos(theta) + np.random.normal(loc=0, scale=sigma_d)
        y_new = y + delta_d*np.sin(theta) + np.random.normal(loc=0, scale=sigma_d)
        theta_new = theta + noisy_delta_theta
        q_new = np.array([x_new, y_new, theta_new])

        # Estimate the new Covariance Matrix
        Fq = np.array([[1, 0, -delta_d*np.sin(theta)], [0, 1, delta_d*np.cos(theta)], [0, 0, 1]])   # Jacobian of the State
        Fv = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])   # Jacobian of the noise
        L_new = Fq @ L @ Fq.T + Fv @ V @ Fv.T   # Covariance Matrix of the new State (@ represents matrix multiplication)

        # Append the new estimated position and orientation
        q_est.append(q_new)
        L_est.append(L_new)

    return np.array(q_est), L_est

def compute_and_plot_ellipses(q_est, L_est, plt, n_sigma=3, frequency=1):
    n = int(1 / frequency)  # number of measurements acquisition per second
    cmap = cm.get_cmap('Reds')  # load the colormap
    num_ellipses = len(q_est) // n  # calculate the number of ellipses

    for i in range(0, len(q_est), n):
        # Compute ellipse of uncertainty
        L = L_est[i]
        s, U = np.linalg.eig(L[0:2, 0:2])
        orientation = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        width = np.sqrt(s[0])
        height = np.sqrt(s[1])
        # Get a color from the colormap
        color = cmap(i / num_ellipses)
        # Plot the ellipse
        ellipse = Ellipse(xy=q_est[i, :2], width=width*n_sigma, height=height*n_sigma, angle=orientation, facecolor='none', edgecolor=color, alpha=0.9)
        plt.gca().add_patch(ellipse)