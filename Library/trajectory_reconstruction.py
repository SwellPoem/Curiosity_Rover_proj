import numpy as np
from tqdm import tqdm
from Library.get_Hq import get_Hq
from Library.h import h

def trajectory_reconstruction(q_real, sigma_d, sigma_theta, L0, lidar_maxrange, sigma_r, sigma_beta, xLM = [], yLM= []):
    
    q_est = [q_real[0]]     # initialize the estimated trajectory
    L_est = [L0]            # initialize covariance matrix

    # Create measurement Covariance Matrix
    V = np.array([[sigma_d**2, 0], [0, sigma_theta**2]])

    Hw = np.array([[1, 0], [0, 1]])
    W = np.array([[sigma_r**2, 0], [0, sigma_beta**2]])

    for i in tqdm(range(1, len(q_real)), desc="Processing"):

        # Compute the odometry measurements in distance and orientation
        delta_d = np.linalg.norm(q_real[i][:2] - q_real[i-1][:2])           # delta in distance
        delta_theta = q_real[i][2] - q_real[i-1][2]                         # delta in orientation

        # Add noise to the odometry measurements
        delta_d = delta_d + np.random.normal(0, sigma_d)
        delta_theta = delta_theta + np.random.normal(0, sigma_theta)

        # Retrieve last State and Covariance matrix
        x, y, theta = q_est[-1]
        L = L_est[-1]

        # Estimate the new State
        x_new = x + delta_d*np.cos(theta)
        y_new = y + delta_d*np.sin(theta)
        theta_new = theta + delta_theta
        q_new = np.array([x_new, y_new, theta_new])

        # Estimate the new Covariance Matrix
        Fq = np.array([[1, 0, -delta_d*np.sin(theta)], [0, 1, delta_d*np.cos(theta)], [0, 0, 1]])   # Jacobian of the State
        Fv = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])   # Jacobian of the noise
        L_new = Fq @ L @ Fq.T + Fv @ V @ Fv.T   # Covariance Matrix of the new State (@ represents matrix multiplication)


        # Check for visible Landmarks
        for x_lm, y_lm in zip(xLM, yLM):

            dist_lm = np.linalg.norm([x_lm - x_new, y_lm - y_new])

            if dist_lm <= lidar_maxrange:
              
                # Compute the Observables from the real State -> observables that I should see
                z_real = h(q_real[i], x_lm, y_lm) + np.array([np.random.normal(0, sigma_r), np.random.normal(0, sigma_beta)])
                z_new = h(q_new, x_lm, y_lm)

                # Compute Kalman Gain
                Hq = get_Hq(q_new, (x_lm, y_lm))
                inv_term = np.linalg.inv(Hq @ L_new @ Hq.T + Hw @ W @ Hw.T)
                K = L_new @ Hq.T @ inv_term

                # Perform State Update
                innovation = z_real - z_new
                q_upd = K @ innovation
                q_new = q_new + q_upd
                L_new = (np.eye(3) - K @ Hq) @ L_new

                # print('\nPerforming EKF @ step {:.0f}:\nq_est = '.format(i), q_new, '\nq_upd = ', q_upd)

                break
                

        # Append the new estimated position and orientation
        q_est.append(q_new)
        L_est.append(L_new)

    return np.array(q_est), L_est