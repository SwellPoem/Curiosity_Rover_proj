import numpy as np
from tqdm import tqdm
from matplotlib.patches import Ellipse

def plot_covariance_ellipse(x, P, plt, n_sigma=1, facecolor='none', **kwargs):
    P = P[0:2, 0:2]
    if not(np.all(np.isfinite(P))):
        return

    U, s, _ = np.linalg.svd(P)
    orientation = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
    width = n_sigma * np.sqrt(s[0])
    height = n_sigma * np.sqrt(s[1])

    ellipse = Ellipse(xy=x, width=width, height=height, angle=orientation, facecolor=facecolor, **kwargs)

    plt.gca().add_patch(ellipse)

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
                

        # Append the new estimated position and orientation
        q_est.append(q_new)
        L_est.append(L_new)

    return np.array(q_est), L_est