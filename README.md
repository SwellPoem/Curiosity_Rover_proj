## Rover_homework

#Overview
This project simulates the navigation, path planning, and localization of a Mars Rover, modeled after NASA's Curiosity Rover, within a virtual representation of Gale Crater on Mars. It comprises three main tasks: Navigation, Path Planning using the A* algorithm, and Rover Localization with Dead Reckoning and an Extended Kalman Filter (EKF), combining odometer and LIDAR data for enhanced accuracy.

#Features
*Navigation: Simulates rover movement from an initial to a final pose, avoiding steep slopes, focusing on trajectory, velocity, and heading angle.
*Path Planning: Implements the A* algorithm for optimal path finding in an 8-way grid environment, avoiding obstacles.
*Rover Localization: Utilizes dead reckoning and EKF for accurate position estimation based on sensor data.

#Environment Setup
The simulation environment is the Gale Crater on Mars, represented by three distinct grayscale images for operational, obstacles, and landmarks mapping.

#Rover Model
The rover is modeled as a car-like vehicle with specific maximum velocity and axle distance, using a bicycle model for motion simulation.

Prerequisites
*Python 3.7 or higher
*Jupyter Notebook
*NumPy, Matplotlib, and other required Python libraries
