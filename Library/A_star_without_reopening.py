from concurrent.futures import ThreadPoolExecutor   #To create a pool of threads to execute calls asynchronously
from tqdm import tqdm  #library for displaying progress bars in loops

def A_star_without_reopening(start, end, obstacle_map):
    '''
    A* algorithm without reopening
    :param start: Start position
    :param end: End position
    :param obstacle_map: Obstacle map
    :return: Path
    '''
    # Define node class that will hold information for each point in the search space
    class Node:
        def __init__(self, position, parent):
            self.position = tuple(position)  # tuple (x, y) coordinates of the node in the obstacle map
            self.parent = parent    # node from which this node was reached
            self.g = 0  # cost from the start node to the current node (distance from start)
            self.h = 0  # heuristic estimate of the cost from the current node to the end node (distance to goal)
            self.f = 0  # total estimated cost of the cheapest solution through this node

        def __eq__(self, other):
            return self.position == other.position

        def __hash__(self):
            return hash(self.position)  # Hash based on position for use in sets

    # Heuristic function for distance
    # estimates the cost to reach the goal from node a to node b
    def heuristic(node_a, node_b):
        return max(abs(node_a[0] - node_b[0]), abs(node_a[1] - node_b[1])) + 1.41421356237 * min(abs(node_a[0] - node_b[0]), abs(node_a[1] - node_b[1]))
        # return max(abs(node_a[0] - node_b[0]), abs(node_a[1] - node_b[1])) + 0.41421356237 * min(abs(node_a[0] - node_b[0]), abs(node_a[1] - node_b[1]))

    # Function to process a potential neighboring node
    # validates whether the node is within the map boundaries
    # not an obstacle, not already processed
    # and then calculates the f, g, and h values
    def process_node(next_node, current_node, end_node, closed_list_set, open_list):
            # Make sure within range
            if next_node[0] > (len(obstacle_map) - 1) or next_node[0] < 0 or next_node[1] > (len(obstacle_map[0]) - 1) or next_node[1] < 0:
                return None, False

            # Make sure walkable terrain
            if int(obstacle_map[next_node[0]][next_node[1]]) == 255:  # Convert to int for comparison
                return None, False

            # Create new node
            neighbor = Node(next_node, current_node)

            # Check if the neighbor is in the closed list
            if neighbor in closed_list_set:
                return None, False

            # Create the f, g, and h values
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor.position, end_node.position)
            neighbor.f = neighbor.g + neighbor.h

            # Check if neighbor is in open list and if it has a lower f value
            if add_to_open(open_list, neighbor):
                return neighbor, True

            return None, False

    open_list = []
    closed_list_set = set()

    start_node = Node(start, None)
    end_node = Node(end, None)
    open_list.append(start_node)

    pbar = tqdm(total=1, desc='Finding path', unit='step')  # Initialize progress bar
    
    executor = ThreadPoolExecutor(max_workers=8)

    while len(open_list) > 0:
        
        pbar.update()  # Update progress bar

        current_node = min(open_list, key=lambda node: node.f)
        open_list.remove(current_node)
        closed_list_set.add(current_node)

        if current_node.position == end_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        # Generate children
        (x, y) = current_node.position
        neighbors = [(x, y-1), (x-1, y-1), (x-1, y), (x-1, y+1), (x, y+1), (x+1, y+1), (x+1, y), (x+1, y-1)]

        # Use a list to store futures
        import concurrent.futures

        futures = []

        for next_node in neighbors:
            # Submit tasks to thread pool
            futures.append(executor.submit(process_node, next_node, current_node, end_node, closed_list_set, open_list))

        # Collect results
        for future in concurrent.futures.as_completed(futures):
            neighbor, should_add = future.result()
            if should_add:
                open_list.append(neighbor)

    
    
    pbar.close()  # Close progress bar

    return None  # No path found

# Function to check if a neighbor should be added to the open list
def add_to_open(open_list, neighbor):
    for node in open_list:
        if neighbor == node and neighbor.f >= node.f:
            return False
    return True
