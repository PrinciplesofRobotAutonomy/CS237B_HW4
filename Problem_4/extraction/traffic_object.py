import config
import copy
from objects import TimeStamp, Maneuver, Scenario


class TrafficObject:
    """
    This object is the central object of this extraction script as it collects all the raw data and processed
    information for every car in the highD recording.
    First, the ego positions are collected from the highD csv files and ego maneuvers are detected.
    Then, the relevant environment is determined for every timestep.
    Finally, scenarios are extracted based on maneuvers.
    """
    def __init__(self, object_id, tracks_reader, tracks_meta_reader, recording_meta_row):
        """
        This init function calls the _init_ego_data function to produce the time series of relevant data and
        generates the lane change maneuvers from these time series in the _extract_lanechange_maneuvers function.
        :param ego_id: int, the id which is considered as ego
        :param tracks_reader: an instant of csv.DictReader applied open on the xx_tracks.csv
        :param tracks_meta_reader: an instant of csv.DictReader applied open on the xx_tracksMeta.csv
        :param recording_meta_row: dict, the first and only raw from xx_recordingMeta.csv
        """
        # the object id of this traffic object
        self.object_id = object_id

        # the two csv readers and the already passed meta row
        self.tracks_reader = tracks_reader
        self.tracks_meta_reader = tracks_meta_reader
        self.recording_meta_row = recording_meta_row

        # meta info
        self.frame_rate = None
        self.recording_id = None
        self.location_id = None
        self.driving_direction = None  # upper road -1, lower road +1
        self.n_frames = None
        self._init_meta_data()  # init above

        # time stamps and lane changes
        self.time_stamps = list()  # list of TimeStamp of the traffic object
        self.ts = list()  # list of ints only the real time stamps as a list for faster access
        self.line_crossing_timestamp_indices = list()
        self.line_crossing_lanes = list()  # 2-tuple (departing_lane, target_lane)
        self.line_crossing_velocities = list()  # 2-tuple (vel_x, vel_y)
        self._init_track_data()  # init above

        # environment info
        self.to_dict = None  # dict of all TrafficObjects in the highD recording (key: ID as int)
        self.possible_relevant_others = None  # list of ints defining traffic objets id of possible relevant other tos for the to

        # maneuvers
        self.maneuver_list = list()  # list of Maneuver objects that this traffic object executes

        # scenarios
        self.scenario_list = list()  # list of scenario objects resulting from the maneuvers

    def _init_meta_data(self):
        """
        Function to init all data from the meta file.
        """
        self.frame_rate = int(self.recording_meta_row['frameRate'])
        self.recording_id = int(self.recording_meta_row['id'])
        self.location_id = int(self.recording_meta_row['locationId'])
        for row in self.tracks_meta_reader:
            if int(row['id']) == self.object_id:
                self.n_frames = int(row['numFrames'])
                if int(row['drivingDirection']) == 1:
                    self.driving_direction = -1
                else:
                    self.driving_direction = 1
                break

    def _init_track_data(self):
        """
        Function to init all data from the track file by creating TimeStamp classes for each entry.
        In addition concerning computational cost crossing lanes will be notated also in this extraction step.
        """
        t_index = 0
        previous_lane_id = None
        counter = self.n_frames

        for row in self.tracks_reader:
            if int(row['id']) == self.object_id:
                frame = int(row['frame'])
                if frame > config.extraction['discard_first_n_frames']:  # discard a few frames at the start
                    # move coordinate location from upper left to center of the traffic object
                    object_width = float(row['width'])
                    object_height = float(row['height'])
                    half_object_width = object_width / 2
                    half_object_height = object_height / 2

                    # apply coordinate shift, calculate time from frame rate and other data
                    x = float(row['x']) + half_object_width
                    y = -(float(row['y']) + half_object_height)
                    t = (frame - 1) * (1.0/float(self.frame_rate))
                    lane_id = int(row['laneId'])
                    vx = float(row['xVelocity'])
                    vy = float(row['yVelocity'])
                    ax = float(row['xAcceleration'])
                    ay = float(row['yAcceleration'])
                    w = float(row['width'])
                    h = float(row['height'])

                    # create time stamp and append it as well as a shortcut for the time value
                    self.time_stamps.append(TimeStamp(t, x, y, vx, vy, ax, ay, lane_id, object_width, object_height))
                    self.ts.append(t)

                    # check if line was crossed
                    if previous_lane_id is not None:
                        if lane_id != previous_lane_id:
                            self.line_crossing_timestamp_indices.append(t_index)
                            self.line_crossing_velocities.append((float(row['xVelocity']), float(row['yVelocity'])))
                            self.line_crossing_lanes.append((previous_lane_id, lane_id))

                    # update loop variables
                    t_index += 1
                    previous_lane_id = lane_id

                # break if all rows with the object id are processed
                counter -= 1
                if counter == 0:
                    break

    def set_others(self, to_dict):
        """
        Function to add a dict of all others (including for simplicity also the traffic object itself).
        :param to_dict: dict of all TrafficObjects
        """
        self.to_dict = to_dict  # shallow copy

    def get_position(self, t_request):
        """
        Function to get the ego postion at the requested time.
        :param t_request: float, requested recording time
        :return: time_stamp.Position, position at requested time or None
        """
        if t_request not in self.ts:
            return None
        else:
            t_index = self.ts.index(t_request)
            return self.time_stamps[t_index].position

    def get_maneuver(self, t_request):
        """
        Function to get the maneuver at the requested time.
        If the traffic object is not in a maneuver at the requested time it will return None.
        :param t_request: float, the requested recording time
        :return: Maneuver, the maneuver or None if it is not in a maneuver
        """
        if not self.ts[0] <= t_request <= self.ts[-1]:
            return None  # does not exist
        for m in self.maneuver_list:
            if m.start_time <= t_request <= m.end_time:
                return m
        return None  # no maneuver

    def determine_possible_relevant_others(self):
        """
        Function to init the possible_relevant_others list with ids from other traffic objects which drive
        in the same direction and exist during the existence of the ego.
        This is kind of a pre-filtering for the TimeStamp class with its function determine_relevant_others.
        """

        self.possible_relevant_others = list()

        for other_id in self.to_dict.keys():
            if other_id == self.object_id:  # pass if other is itself
                pass
            else:
                ######### Your code starts here #########
                # Determine possible relevant others by pre-filtering same direction and same existence
                # Hints: The time_stamps and driving_direction attributes will help you













                ######### Your code ends here #########

    def _set_possible_relevant_others(self, time_stamp):
        """
        Function to simply set the determined pre-filted IDs to a time stamp.
        :param time_stamp: TimeStamp, the time stamp to be considered
        """
        # before starting check if pre-filtering was executed
        if self.possible_relevant_others is None:
            raise ValueError('The pre-filtering has to be executed before!')

        time_stamp.relevant_others = copy.deepcopy(self.possible_relevant_others)

    def _determine_relevant_others_distance(self, time_stamp):
        """
        Function to determine the relevant traffic objects for the ego at a single time stamp.
        The prefiltering is already applied by the traffic object due to possible relevant objects.
        This filtering is neglecting traffic objects too far away.
        The result is added to the given TimeStamp instance variable relevant_others.
        The next function will apply the 8 car model as next filter step.
        :param time_stamp: TimeStamp, the time stamp to be considered
        """
        # before starting check if pre-filtering values are already set
        if time_stamp.relevant_others is None:
            raise ValueError('The pre-filted values have to be setten first!')

        others_filter = list()
        filter_distance_front = config.extraction['filter_distance_front']
        filter_distance_back = config.extraction['filter_distance_back']

        for other_id in self.possible_relevant_others:
            ######### Your code starts here #########
            # Filter out TOs which are too far away longitudinally (x-distance)
            # Fill in the parts indicated by #FILL#. No additional lines are required.













            ######### Your code ends here #########
        time_stamp.relevant_others = others_filter

    def _determine_relevant_others_8car(self, time_stamp):
        """
        Function to filter vehicles based on the 8 car model.
        This is based on the already filtered ids from the distance filter.
        :param time_stamp: TimeStamp, the time stamp to be considered
        """
        # before starting check if pre-filtering values are already set
        if time_stamp.relevant_others is None:
            raise ValueError('The pre-filted values and the distance filter have to be setten first!')

        # create a lane dict for support on the 8 car model filter
        lane_dict = dict() # key: lane_id value: (object_is, delta_x)
        ego_position = self.get_position(time_stamp.t)
        ego_lane = ego_position.lane_id
        for other_id in time_stamp.relevant_others:
            other = self.to_dict[other_id]
            other_position = other.get_position(time_stamp.t)
            # check if an entry in lane dict already exists
            if other_position.lane_id not in lane_dict.keys():
                # create entry in lane dict
                lane_dict[other_position.lane_id] = list()
            lane_dict[other_position.lane_id].append((other.object_id, ego_position.x - other_position.x))

        # filter with 8 car model
        others_filter = list()

        # set the threshold for the side, the front and back threshold was already applied in the first step
        side_threshold = config.extraction['8_car_model_side_window'] / 2.0
        for lane_id in sorted(lane_dict.keys()):
            # sort the traffic objects by their absolute x distance
            lane_list_sorted = sorted(lane_dict[lane_id], key=lambda x: abs(x[1]))

            # check if there are any traffic objects in the lane
            if len(lane_list_sorted) == 0:  # this lane contains no traffic objects
                continue

            ######### Your code starts here #########
            # We want to get a maximum of 8 TOs around the ego vehicle:
            ###############################
            #    6    #    1    #    3
            ###############################
            #    7    #    E->  #    4
            ###############################
            #    8    #    2    #    5
            ###############################
            # If there is no TO present at one of the 8 positions nothing is added to the list
            # 1. If the lane is not the ego lane add the TO closest to ego to the list (1/2)
            #       AND remove it from the lane_list_sorted
            # 2. Add the TO with smallest positive x value to the list (3/4/5)
            # 3. Add the TO with smallest negative x value to list (6/7/8)





















            ######### Your code ends here #########

        # set the list as relevant others
        time_stamp.relevant_others = others_filter

    def determine_relevant_others(self, include_8_car_filter):
        """
        Function to calculate the relevant others for each time stamp for the traffic object.
        As this has to be done for each time stamp it it will be implemented with a coarse step length.
        This means only each xth time stamp will be calculated explicitly and the elements will only be calcualted
        if the result between after a coase step has changed.
        : param include_8_car_filter: bool, if true the additional 8 car fliter will be executed after teh distance filter
        """
        if not self.to_dict:
            raise ValueError("Traffic objects have to be inited first via set_others()!")

        coarse_step_length = config.extraction["coarse_step_length"]
        last_relevant_others = list()  # last calculated relevant ids
        ts_index_last = 0  # last index
        ts_index_current = 0  # current index

        while True:
            # if index is over length of time stamps set index to last element
            if ts_index_current >= len(self.time_stamps):
                ts_index_current = len(self.time_stamps) - 1

            # check if the ids changed during a coarse step
            self._set_possible_relevant_others(self.time_stamps[ts_index_current])
            self._determine_relevant_others_distance(self.time_stamps[ts_index_current])  # calculate relevant others via distance
            if include_8_car_filter:
                self._determine_relevant_others_8car(self.time_stamps[ts_index_current])  # calculate relevant others via 8-car model

            current_relevant_others = self.time_stamps[ts_index_current].relevant_others  # save for current step

            if sorted(last_relevant_others) != sorted(current_relevant_others):
                # relevant ids changed during a coarse step thus calculate for each in between individually
                for ts_index in range(ts_index_last + 1, ts_index_current):
                    self._set_possible_relevant_others(self.time_stamps[ts_index])
                    self._determine_relevant_others_distance(self.time_stamps[ts_index])
                    if include_8_car_filter:
                        self._determine_relevant_others_8car(self.time_stamps[ts_index])

            else:
                # relevant ids did not chnage during a coarse step thus just copy ids
                for ts_index in range(ts_index_last + 1, ts_index_current):
                    self.time_stamps[ts_index].relevant_others = copy.copy(last_relevant_others)

            # break at last element
            if ts_index_current == (len(self.time_stamps) - 1):
                break

            # update loop variables
            ts_index_last = ts_index_current
            ts_index_current += coarse_step_length
            last_relevant_others = current_relevant_others

    def extract_maneuvers(self):
        """
        Create Maneuver objects for every lane change.
        This works by going forward and backward in time from every lane crossing to get the start and event for every
        maneuver once the absolute value of the velocity drops below a certain threshold.
        - first step is calculating the start and end time stamps
        - second step is determine the type of scenarios
        - third step is generate the maneuvers
        """
        self.maneuver_list = list()

        # get all y velocities at the time stamp of a detected change
        vy_list = list()
        for ts in self.time_stamps:
            vy_list.append(ts.velocity.y)

        # get the start and end times of the lane change maneuvers
        start_times = list()
        end_times = list()

        # first step - calulating the start and end times of a lane change
        for ts_index in self.line_crossing_timestamp_indices:
            lanechange_detection_vy_threshold = config.extraction["lanechange_detection_vy_threshold"]
            ######### Your code starts here #########
            # Find the start and end time step for the lane change maneuver
            # 1. Go backwards in time until you either reached the beginning of the recoding or
            #       the absolute vy_list[t] value drops below lanechange_detection_vy_threshold
            #       append the start time to start_times
            # 2. Go forwards in time until you either reached the beginning of the recoding or
            #       the absolute vy_list[t] value drops below lanechange_detection_vy_threshold
            #       append the end time to end_times

            # going backwards for the start time



























            ######### Your code ends here #########

        # second step - determine the type of a lane change
        type_list = list()  # dicts with {start, end, double, left} keys
        for i in range(len(start_times)):
            st = start_times[i]
            et = end_times[i]

            # check for double lane change (equal start and end times)
            double_detected = False
            for m in type_list:
                if m['start'] == st and m['end'] == et:
                    m['double'] = True  # set double_boolean to True
                    double_detected = True
            if double_detected:
                continue  # no additional lane change has to be added

            # check if it is a left or right lane change
            p_st = self.get_position(st)
            p_et = self.get_position(et)
            delta_y = (p_et.y - p_st.y) * self.driving_direction
            if delta_y > 0:
                left = False
            elif delta_y < 0:
                left = True
            else:
                raise ValueError("Fatal error, the lane change has to be a right or left direction.")

            # append the new dict
            type_list.append({'start':st, 'end':et, 'double':False, 'left':left})

        # third step - create maneuvers
        m_list = list()
        for m in type_list:
            if not m['double'] and m['left']:
                m_type = Maneuver.TYPE_LANECHANGE_LEFT
            elif not m['double'] and not m['left']:
                m_type = Maneuver.TYPE_LANECHANGE_RIGHT
            elif m['double'] and m['left']:
                m_type = Maneuver.TYPE_DOUBLE_LANECHANGE_LEFT
            elif m['double'] and not m['left']:
                m_type = Maneuver.TYPE_DOUBLE_LANECHANGE_RIGHT
            else:
                raise ValueError("Fatal error, one of the four options has to match.")
            m_list.append(m_type)

        # add maneuvers
        for i in range(len(start_times)):
            st = start_times[i]
            et = end_times[i]
            mt = m_list[i]
            self.maneuver_list.append(Maneuver(self.object_id, st, et, mt))

    def extract_scenarios(self, collection=None):
        """
        Function to extract all scenarios for this traffic object.
        Only lane changes from the ego vehicle will be considered not from other traffic objects.
        """
        self.scenario_list = list()

        # create a scenario from each maneuver
        for m in self.maneuver_list:
            # filter which objects are present during the scenario
            present_objects = set()  # id of relevant objects in this scenario
            start_t_index = self.ts.index(m.start_time)
            end_t_index = self.ts.index(m.end_time)
            for i in range(start_t_index, end_t_index + 1):
                current_ts = self.time_stamps[i]
                current_present_objects = current_ts.relevant_others
                for other_id in current_present_objects:
                    present_objects.add(other_id)

            # create and append the scenario with a dummy number
            new_scenario = Scenario(-1, self.object_id, present_objects, m)
            self.scenario_list.append(new_scenario)

            if collection is not None:
                collection.append(new_scenario)
