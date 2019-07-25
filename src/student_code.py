from math import sqrt


def shortest_path(M, start, goal):
    if start == goal:
        return [start]
    goal_node = M.intersections[goal]
    h = heuristic(M.intersections[start], goal_node)
    # Store heurisitc for nodes as node_id => heuristic
    heuristic_dict = {
        start: h
    }
    # Store minimum distance for nodes as node_id => distance
    distance_dict = {
        start: 0
    }
    # Store items that could be valid paths to explore
    path_dict = {
        start: (h, [start])
    }
    # If path dict is empty there is no available path
    while path_dict:
        # Get data of node tuple (d(x) + h(x), d(x), path_to_node)
        node_tuple = pop_min_distance_path(path_dict)
        to_node_path = node_tuple[1]
        last_node_index = to_node_path[-1]
        if last_node_index == goal:
            return to_node_path
        to_node_distance = distance_dict[last_node_index]
        actual_node = M.intersections[last_node_index]
        # Check routes and put them in the list to check (when they're plausible path)
        for near_node_index in M.roads[last_node_index]:
            near_node = M.intersections[near_node_index]
            to_near_distance = to_node_distance + distance(actual_node, near_node) # Previously calculated distance + actual->near distance
            if near_node_index not in distance_dict or to_near_distance < distance_dict[near_node_index]:
                distance_dict[near_node_index] = to_near_distance
                # calculate heuristic only if it hasn't been calculated for the node yet
                if near_node_index in heuristic_dict:
                    h = heuristic_dict[near_node_index]
                else:
                    h = heuristic_dict[near_node_index] = heuristic(near_node, goal_node)
                f = to_near_distance + h # f = d(x) + h(x)
                new_to_node_path = to_node_path[:]
                new_to_node_path.append(near_node_index)
                path_dict[near_node_index] = (f, new_to_node_path)
    return None

def pop_min_distance_path(path_dict):
    path_distances = list(path_dict.values())
    return_node = None
    if path_distances:
        return_node = path_distances[0]
    for path in path_distances:
        if path[0] < return_node[0]:
            return_node = path
    del path_dict[return_node[1][-1]]
    return return_node

def distance(start, end):
    return sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

def heuristic(start, end):
    return distance(start, end)
