# import numpy as np
# from scipy.io import loadmat

# def load_maps(filename='exercise.mat'):
#     '''
#     this function loads the maps and the environment from the provided .mat file
#     '''
#     name = loadmat(filename)

#     X = name['X']
#     Y = name['Y']
#     env_map = name['map']
#     obstacleMap = name['obstacleMap']
#     xLM = name['xLM']
#     yLM = name['yLM']
#     Xvec = name['Xvec']
#     Yvec = name['Yvec']


#     return X, Y, env_map, obstacleMap, xLM, yLM, Xvec, Yvec