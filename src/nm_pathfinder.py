from collections import deque
import math

def find_box_containing_point(point, boxes):
    """Find the box that contains the given point.
    Args:
    point: the point being searched for
    boxes: the list of all boxes to search through
    
    Returns: The box containing point, or None if no box was found"""
    x, y = point
    for box in boxes:
        if box[0] <= x <= box[1] and box[2] <= y <= box[3]:
            return box

    #Return None since no box was found (user clicked black area)
    return None

def get_neighbors(box, mesh):
    """Get neighboring boxes that share an edge or corner with the given box.
    Args:
    box: the center box who needs the neighbors checked
    mesh: a dictionary containing all the boxes
    
    Returns: a list of all neighbors for box"""
    neighbors = []
    for neighbor in mesh['boxes']:
        # Check if boxes share an edge or corner
        if (neighbor[0] <= box[1] and neighbor[1] >= box[0] and neighbor[2] <= box[3] and neighbor[3] >= box[2]):
            if neighbor != box:  # Exclude the box itself
                neighbors.append(neighbor)
    return neighbors

def euclidean_distance(point1, point2):
    """Calculates the euclidean distance between two points.
    Args:
        point1: first point (x, y)
        point2: second point (x, y)
    
    Returns:
        distance between the two points"""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def find_path(source_point, destination_point, mesh):
    """Searches for a path from source_point to destination_point through the mesh.
    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach 
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A dictionary of boxes explored by the algorithm
    """
    path = []
    boxes = {}
    detail_points = {}

    # Step 1: identify source and destination boxes
    source_box = find_box_containing_point(source_point, mesh['boxes'])
    boxes.update({source_box: 'start'})
    detail_points[source_box] = source_point
    destination_box = find_box_containing_point(destination_point, mesh['boxes'])
    boxes.update({destination_box: 'end'})
    detail_points[destination_box] = destination_point

    # if no source or destination box was found, no path can be created
    if not source_box or not destination_box:
        print("No path!")
        return path, boxes

    # if the source and destination boxes are the same, we can just create a path between those two points
    if source_box == destination_box:
        path.append(source_point)
        path.append(destination_point)
        return path, boxes

    # Step 2: BFS search with boxes (no points yet)
    # BFS initialization
    queue = deque([(source_box, [source_box])])
    visited = set()
    visited.add(source_box)

    while queue:
        current_box, current_path = queue.popleft()
        boxes.update({current_box: 'visited'})

        for neighbor in get_neighbors(current_box, mesh):
            if neighbor not in visited:
                # Add current box to set and queue
                visited.add(neighbor)
                queue.append((neighbor, current_path + [neighbor]))

                # Constrain the x, y position within the current box to the bounds of the neighbor box
                last_point = detail_points[current_box]
                constrained_point = (
                    max(neighbor[0], min(neighbor[1], last_point[0])),
                    max(neighbor[2], min(neighbor[3], last_point[1]))
                )
                detail_points[neighbor] = constrained_point
                
                # If this neighbor is the destination box we need to return the function
                if neighbor == destination_box:
                    full_path = current_path + [neighbor]

                    # Create the final path including detailed points
                    detailed_path = [source_point]
                    for box in full_path[1:]:
                        detailed_path.append(detail_points[box])
                    detailed_path.append(destination_point)

                    return detailed_path, boxes

    print("No path!")
    return path, boxes