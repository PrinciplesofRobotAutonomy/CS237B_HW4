

def detect_collision(ts_1, ts_2):
    """
    Function to calculate collision in one dimension.
    This means this function ignores the heading of each traffic object and considers its orientated
    always with its complete width in x direction.
    :param ts_1: ...
    :param ts_2: ...
    :return: bool, true if collision, false otherwise
    """
    # calculate x distance
    dist = ts_1.ego_position.x - ts_2.ego_position.x

    # check if collision
    if dist < ts_1.dimension.width / 2 + ts_2.dimension.width / 2 :
        return True
    else:
        return False
