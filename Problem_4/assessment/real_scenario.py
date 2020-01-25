from risks import thw, ttc, ettc


class RealScenario:
    """
    Class to maintain the scenario data.
    """

    def __init__(self, scenario_number, df):
        """
        Init function.
        """
        self.scenario_number = scenario_number  # the unique scenario number to identify the saved data
        self.df = df  # the scenario data
        self.ego_time = df.loc[df['id'] == 0, 't'].values  # teh valid time points of the ego / scenario

        # the risk values for each time point 
        self.thw_measure = None
        self.ttc_measure = None
        self.ettc_measure = None

        # the maximal risk value and its time point for the scenario
        self.thw_min_value = None
        self.ttc_min_value = None
        self.ettc_min_value = None
        self.thw_min_time = None
        self.ttc_min_time = None
        self.ettc_min_time = None

    def get_value_indices(self, t):
        """
        Function to get the values at a certain time point.
        The return will be a dict in form of traffic object id as key and index of the self.df to obtain the data.
        :param t: float, the time point of calculating the distance
        :retrun: dict, key: id, value: index of df
        """
        index_dict = dict()
        df_small = self.df.loc[self.df['t'] == t]

        for i in range(0, len(df_small)):
            index_dict[int(df_small.iloc[i]['id'])] = df_small.index[i]

        return index_dict

    def calculate_risk_single(self, index_dict, risk=thw):
        """
        Function to calculate a certain risk value for a certain time point.
        This requires the measured values of all other traffic objects existing at that time point.
        This information is given by the index_dict which is a dict with the traffic object id (including the ego=0)
        as keys and the values are the indices for the self.df to obtain the row of the measured data.
        The risk function is given via the risk variable.
        :param index_dict: dict, key: id, value: index of df
        :param risk: class object, the risk measure to be calculated
        """
        x_ego = self.df.loc[index_dict[0]]['x']
        y_ego = self.df.loc[index_dict[0]]['y']
        h_ego = self.df.loc[index_dict[0]]['h']
        w_ego = self.df.loc[index_dict[0]]['w']

        min_distance = 100000
        min_id = 0
        for object_id in index_dict.keys():
            # do nothing if it is the ego
            if object_id == 0:
                continue
            
            # check if objects are on the same diagonal lane
            possible_distance = self.df.loc[index_dict[object_id]]['h'] / 2 + h_ego / 2
            if abs(self.df.loc[index_dict[object_id]]['y'] - y_ego) > possible_distance:
                continue

            # check if ego vehicle is the following vehicle
            x_rel = (self.df.loc[index_dict[object_id]]['x'] - x_ego) * self.df.loc[index_dict[object_id]]['direction']
            if  x_rel < 0:
                continue
            
            # get the current distance front to back and set if it is the smallest one
            x_rel = x_rel - w_ego / 2 - self.df.loc[index_dict[object_id]]['w'] / 2
            if x_rel < min_distance:
                min_distance = x_rel
                min_id = object_id

        if min_id == 0:
            return None  # no vehicle in front of the ego
        else:
            direction = self.df.loc[index_dict[0]]['direction']
            vx_ego = self.df.loc[index_dict[0]]['vx'] * direction
            vx_obj = self.df.loc[index_dict[min_id]]['vx'] * direction
            ax_ego = self.df.loc[index_dict[0]]['ax'] * direction
            ax_obj = self.df.loc[index_dict[min_id]]['ax'] * direction

            if risk is thw:
                return thw(min_distance, vx_ego)
            elif risk is ttc:
                return ttc(min_distance, vx_ego, vx_obj)
            elif risk is ettc:
                return ettc(min_distance, vx_ego, vx_obj, ax_ego, ax_obj)
            else:
                raise ValueError('Risk measure not supported!')

    def calculate_risk(self, risk=thw):
        """
        Function to calculate a certain risk measure for each time point.
        :param risk: class object, the risk measure to be calculated
        """
        # init list
        if risk is thw:
            self.thw_measure = list()
        elif risk is ttc:
            self.ttc_measure = list()
        elif risk is ettc:
            self.ettc_measure = list()
        else:
            raise ValueError('Risk measure not supported!')
        
        # calulate risk
        for t in self.ego_time:
            indicies_dict = self.get_value_indices(t)
            risk_value = self.calculate_risk_single(indicies_dict, risk=risk)
            
            if risk is thw:
                self.thw_measure.append(risk_value)
            elif risk is ttc:
                self.ttc_measure.append(risk_value)
            elif risk is ettc:
                self.ettc_measure.append(risk_value)
            else:
                raise ValueError('Risk measure not supported!')

    def calculate_risk_min(self):
        """
        Function to calculate the minimal times, meaning the maximal risk in the scenario.
        """
        def minimize(self, risk_list):
            """
            Function to help minimizing.
            """
            min_v = 100000
            min_i = None
            for i, v in enumerate(risk_list):
                if v is not None and v < min_v:
                    min_v = v
                    min_i = i
            
            if min_v == 100000:  # no valid value at all
                min_v = -1  # set to -1 to indicate that risk value could not be applied to any time point
            if min_i is not None:  # a valid value was found
                min_t = self.ego_time[min_i]  # set min time to the time point of the minimal value
            else:
                min_t = None  # no valid value means no min time available

            return min_v, min_t

        if self.thw_measure is not None:
            self.thw_min_value, self.thw_min_time = minimize(self, self.thw_measure)

        if self.ttc_measure is not None:
            self.ttc_min_value, self.ttc_min_time = minimize(self, self.ttc_measure)

        if self.ettc_measure is not None:
            self.ettc_min_value, self.ettc_min_time = minimize(self, self.ettc_measure)




        

