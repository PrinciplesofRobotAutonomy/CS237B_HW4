extraction = {
    # discrad these frames from teh start of each recording
    'discard_first_n_frames': 3,

    # threshold to determine in the y-velocity whether a lane change starts or ends
    'lanechange_detection_vy_threshold': 0.03,

    # filter distance in meter for the distance filter in front and back
    'filter_distance_front': 100,
    'filter_distance_back': 50,

    # filter distance to the left and right of a traffic object (y-direction) in the 8-car model
    '8_car_model_side_window': 10,

    # step size (of time stamps) to check for changes in the relevant traffic objects
    'coarse_step_length': 10
}