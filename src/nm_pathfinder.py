from collections import deque, defaultdict
import math
#import pickle
from heapq import heappop, heappush

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
        A dictionary of boxes explored by the algorithm"""

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

    # Step 4: Bidirectional A* search with boxes
    # A* initialization
    priority_queue = []
    heappush(priority_queue, (0, source_box, 'forward'))
    heappush(priority_queue, (0, destination_box, 'backward'))
   
    visited_forward = {}
    visited_backward = {}

    cost_to_child_forward = defaultdict(lambda: float('inf'))
    cost_to_child_backward = defaultdict(lambda: float('inf'))
   
    visited_forward[source_box] = None
    visited_backward[destination_box] = None

    cost_to_child_forward[source_box] = 0
    cost_to_child_backward[destination_box] = 0

    meeting_point = None

    while priority_queue:
        _, current_box, direction = heappop(priority_queue)
        boxes.update({current_box: 'visited'})

        if direction == 'forward':
            if current_box in visited_backward:
                meeting_point = current_box
                break

            for neighbor in get_neighbors(current_box, mesh):
                new_cost = cost_to_child_forward[current_box] + euclidean_distance(detail_points[current_box], detail_points.get(neighbor, destination_point))
                if new_cost < cost_to_child_forward[neighbor]:
                    cost_to_child_forward[neighbor] = new_cost
                    priority = new_cost + euclidean_distance(detail_points.get(neighbor, destination_point), destination_point)
                    heappush(priority_queue, (priority, neighbor, 'forward'))
                    visited_forward[neighbor] = current_box

                    last_point = detail_points[current_box]
                    constrained_point = (
                        max(neighbor[0], min(neighbor[1], last_point[0])),
                        max(neighbor[2], min(neighbor[3], last_point[1]))
                    )
                    detail_points[neighbor] = constrained_point

        else:  # direction == 'backward'
            if current_box in visited_forward:
                meeting_point = current_box
                break

            for neighbor in get_neighbors(current_box, mesh):
                new_cost = cost_to_child_backward[current_box] + euclidean_distance(detail_points[current_box], detail_points.get(neighbor, source_point))
                if new_cost < cost_to_child_backward[neighbor]:
                    cost_to_child_backward[neighbor] = new_cost
                    priority = new_cost + euclidean_distance(detail_points.get(neighbor, source_point), source_point)
                    heappush(priority_queue, (priority, neighbor, 'backward'))
                    visited_backward[neighbor] = current_box

                    last_point = detail_points[current_box]
                    constrained_point = (
                        max(neighbor[0], min(neighbor[1], last_point[0])),
                        max(neighbor[2], min(neighbor[3], last_point[1]))
                    )
                    detail_points[neighbor] = constrained_point

    # if no meeting point is found
    if not meeting_point:
        print("No path!")
        return path, boxes

    # Reconstruct path
    forward_path = []
    current_box = meeting_point
    while current_box:
        forward_path.append(detail_points[current_box])
        current_box = visited_forward[current_box]
    forward_path.reverse()

    backward_path = []
    current_box = meeting_point
    while current_box:
        backward_path.append(detail_points[current_box])
        current_box = visited_backward[current_box]

    path = forward_path + backward_path[1:]  # avoid duplicate meeting point

    return path, boxes
