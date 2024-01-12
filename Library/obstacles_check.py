
def obstacles_check(map_indices, obstacleMap):
    '''
    this function checks for the presence of obstacles in the provided indices
    '''
    for i, j in map_indices:
        if obstacleMap[i, j] == 255:
            return True  # Obstacle found
    return False  # No obstacle