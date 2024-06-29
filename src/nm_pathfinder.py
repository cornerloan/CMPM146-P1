def find_box_containing_point(point, boxes):
    #Find the box that contains the given point.
    x, y = point
    for box in boxes:
        if box[0] <= x <= box[1] and box[2] <= y <= box[3]:
            return box

    #Return None since no box was found (user clicked black area)
    return None

def find_path (source_point, destination_point, mesh):

    """Searches for a path from source_point to destination_point through the mesh
    Args:

    source_point: starting point of the pathfinder destination_point: the ultimate goal the pathfinder must reach 

    mesh: pathway constraints the path adheres to

    Returns:A path (list of points) from source_point to destination_point if exists

    A list of boxes explored by the algorithm"""

    path = []
    boxes = {}
    
    boxes.update({find_box_containing_point(source_point, mesh['boxes']): 'start'})
    boxes.update({find_box_containing_point(destination_point, mesh['boxes']): 'end'})

    # Return an empty path and the source and destination boxes if they are found
    return path, boxes.keys()