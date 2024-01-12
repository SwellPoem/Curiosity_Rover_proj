from tqdm import tqdm

def A_star_without_reopening(start, end, obstacle_map):
    '''
    A* algorithm without reopening
    '''
    class Node:
        def __init__(self, position, parent):
            self.position = tuple(position)  # Store position as a tuple
            self.parent = parent
            self.g = 0  # Distance to start node
            self.h = 0  # Distance to goal node
            self.f = 0  # Total cost

        def __eq__(self, other):
            return self.position == other.position

        def __hash__(self):
            return hash(self.position)  # Hash based on position for use in sets

    # Heuristic function for distance
    def heuristic(a, b):
        return max(abs(a[0]-b[0]), abs(a[1]-b[1])) + 0.41421356237 * min(abs(a[0]-b[0]), abs(a[1]-b[1]))

    open_list = []
    closed_list_set = set()

    start_node = Node(start, None)
    end_node = Node(end, None)
    open_list.append(start_node)

    pbar = tqdm(total=1, desc='Finding path', unit='step')  # Initialize progress bar
    
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

        for next_node in neighbors:
            # Make sure within range
            if next_node[0] > (len(obstacle_map) - 1) or next_node[0] < 0 or next_node[1] > (len(obstacle_map[0]) - 1) or next_node[1] < 0:
                continue

            # Make sure walkable terrain
            if int(obstacle_map[next_node[0]][next_node[1]]) == 255:  # Convert to int for comparison
                continue

            # Create new node
            neighbor = Node(next_node, current_node)

            # Check if the neighbor is in the closed list
            if neighbor in closed_list_set:
                continue

            # Create the f, g, and h values
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic(neighbor.position, end_node.position)
            neighbor.f = neighbor.g + neighbor.h

            # Check if neighbor is in open list and if it has a lower f value
            if add_to_open(open_list, neighbor):
                open_list.append(neighbor)
    
    pbar.close()  # Close progress bar

    return None  # No path found

# Function to check if a neighbor should be added to the open list
def add_to_open(open_list, neighbor):
    for node in open_list:
        if neighbor == node and neighbor.f >= node.f:
            return False
    return True