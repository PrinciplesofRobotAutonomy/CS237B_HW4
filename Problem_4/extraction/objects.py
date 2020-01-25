import config
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy
from tools import dump_json

class Position:
    """
    This class defines a position of a traffic object.
    """
    def __init__(self, x, y, lane_id):
        self.x = x
        self.y = y
        self.lane_id = lane_id


class Velocity:
    """
    This class defines the velocity of a traffic object.
    """
    def __init__(self, vx, vy):
        self.x = vx
        self.y = vy


class Acceleration:
    """
    This class defines the accelerations of a traffic object.
    """
    def __init__(self, ax, ay):
        self.x = ax
        self.y = ay


class Dimension:
    """
    This class defines a dimension of a traffic object.
    """
    def __init__(self, w, h):
        self.w = w
        self.h = h


class TimeStamp:
    """
    This class defines a single time step with focus on one specific traffic object which is called the ego.
    """
    def __init__(self, t, x, y, vx, vy, ax, ay, lane_id, w, h):
        self.t = t  # the time point of the scene
        self.position = Position(x, y, lane_id)
        self.velocity = Velocity(vx, vy)
        self.acceleration = Acceleration(ax, ay)
        self.dimension = Dimension(w, h)
        self.relevant_others = None  # list of ints defining the relevant other traffic objects by id


class Maneuver:
    """
    This class defines a maneuver which is the semantic description of a time sequence.
    In this context it should describe lane changes.
    The following 4 types should be distinguished.
    """
    TYPE_LANECHANGE_LEFT = "TYPE_LANECHANGE_LEFT"
    TYPE_LANECHANGE_RIGHT = "TYPE_LANECHANGE_RIGHT"
    TYPE_DOUBLE_LANECHANGE_LEFT = "TYPE_DOUBLE_LANECHANGE_LEFT"
    TYPE_DOUBLE_LANECHANGE_RIGHT = "TYPE_DOUBLE_LANECHANGE_RIGHT"

    def __init__(self, object_id, start_time, end_time, maneuver_type):
        self.object_id = object_id
        self.maneuver_type = maneuver_type
        self.start_time = start_time
        self.end_time = end_time


class Scenario:
    """
    This class defines a scenario which is a data filled version of a maneuver.
    """
    def __init__(self, scenario_number, ego_object_id, traffic_objects_id, maneuver):
        """
        Init function.
        """
        self.scenario_number = scenario_number
        self.ego_object_id = ego_object_id
        self.traffic_objects_id = traffic_objects_id
        self.maneuver = maneuver

    def evaluate(self, to_dict):
        """
        Function to evaluate a scenario by calculating all required data from the traffic object dict.
        :param to_dict: dict, containing object_id:TrafficObject
        :retrun: pd.DataFrame, the extracted data as pandas data frame
        """
        df = pd.DataFrame(self.normalize(self.ego_object_id, to_dict))

        for object_id in self.traffic_objects_id:
            #df_tmp = self.normalize(object_id, to_dict)
            df = df.append(pd.DataFrame(self.normalize(object_id, to_dict)))
            #df = df.append(df_tmp, ignore_index=True)

        return df
    
    def normalize(self, object_id, to_dict, set_ego=True):
        """
        Function to extract the correct time stamps from a certain traffic object and normalize it.
        If the traffic object is the ego there is no concerns regarding the time span as the ego has 
        by definition to exists via the complete scenario.
        If the traffic object is not the ego the starting and ending time has to be concerned which 
        might be later than the start or finishes earlier than the end.
        It has to exists somewhere otherwise it should not be added to the scenario in the first place.
        Lastly, the time is always normalized to the start of the scenario (t=0) and a data dict is returned.
        :param object_id: int, the id of the object in the to_dict
        :param to_dict: dict, containing object_id:TrafficObject
        :param set_ego: bool, if True it will set the ego id to 0 in the data set
        :return: dict, containing the extracted data
        """
        t = list()
        ids = list()
        x = list()
        y = list()
        lane = list()
        vx = list()
        vy = list()
        ax = list()
        ay = list()
        w = list()
        h = list()
        direction = list()

        # check if ego or a traffic object is extracted
        if object_id == self.ego_object_id:

            # get the start and end time indixes of the scenario (no check as ego has to exist always)
            start_index = to_dict[self.ego_object_id].ts.index(self.maneuver.start_time)
            end_index = to_dict[self.ego_object_id].ts.index(self.maneuver.end_time)

        else:
            # set the start index to the correct start time 
            if self.maneuver.start_time >= to_dict[object_id].ts[0]: # to exists at the start of the scenario
                start_index = to_dict[object_id].ts.index(self.maneuver.start_time)
            else: # to does not exist at the start of the scenario but after
                start_index = 0
        
            # set the end index to the correct end time 
            if self.maneuver.end_time <= to_dict[object_id].ts[-1]: # to exists at the end of the scenario
                end_index = to_dict[object_id].ts.index(self.maneuver.end_time)
            else: # to does not exist at the end of the scenario but before
                end_index = len(to_dict[object_id].ts) - 1

        for index in range(start_index, end_index + 1):
            x.append(to_dict[object_id].time_stamps[index].position.x)
            y.append(to_dict[object_id].time_stamps[index].position.y)
            lane.append(to_dict[object_id].time_stamps[index].position.lane_id)
            vx.append(to_dict[object_id].time_stamps[index].velocity.x)
            vy.append(to_dict[object_id].time_stamps[index].velocity.y)
            ax.append(to_dict[object_id].time_stamps[index].acceleration.x)
            ay.append(to_dict[object_id].time_stamps[index].acceleration.y)
            w.append(to_dict[object_id].time_stamps[index].dimension.w)
            h.append(to_dict[object_id].time_stamps[index].dimension.h)
            direction.append(to_dict[object_id].driving_direction)
            t.append(to_dict[object_id].time_stamps[index].t - self.maneuver.start_time)  # normalize so maneuver starts at t=0

            # set object id to 0 which is the usual id for the ego if required
            if set_ego and object_id == self.ego_object_id:
                ids.append(0)
            else:
                ids.append(object_id)

        data = {
            "x": x,
            "y": y,
            "lane": lane,
            "vx": vx,
            "vy": vy,
            "ax": ax,
            "ay": ay,
            "w": w,
            "h": h,
            "t": t,
            "id": ids,
            "direction": direction
        }
        return data

    @staticmethod
    def plot_scenario(df, file_name='', save_fig=True, m_type=''):
        """
        Function to plot the scenario including the trajectory and the start position of each traffic object.
        """

        fig = plt.figure(figsize=(420/8,40/4))
        ax1 = fig.add_subplot(111)
        ax1.set_xlim((0, 420))
        ax1.set_ylim((-40))
        ax1.set_title(m_type)

        x = numpy.linspace(0,420,1000) 
        y_down_1 = [-20.41] * 1000 
        ax1.plot(x, y_down_1, color='k', linewidth=1)

        x = numpy.linspace(0,420,1000) 
        y_down_2 = [-20.41 - 4.210000000000001] * 1000 
        ax1.plot(x, y_down_2, color='k', linewidth=1, linestyle='dashed')

        x = numpy.linspace(0,420,1000) 
        y_down_3 = [-20.41 - 4.210000000000001 - 3.719999999999999] * 1000 
        ax1.plot(x, y_down_3, color='k', linewidth=1, linestyle='dashed')

        x = numpy.linspace(0,420,1000) 
        y_down_4 = [-20.41 - 4.210000000000001 - 3.719999999999999 - 3.5] * 1000 
        ax1.plot(x, y_down_4, color='k', linewidth=1)

        x = numpy.linspace(0,420,1000) 
        y_up_1 = [-15.73] * 1000 
        ax1.plot(x, y_up_1, color='k', linewidth=1)

        x = numpy.linspace(0,420,1000) 
        y_up_2 = [-15.73 + 3.960000000000001] * 1000 
        ax1.plot(x, y_up_2, color='k', linewidth=1, linestyle='dashed')

        x = numpy.linspace(0,420,1000) 
        y_up_3 = [-15.73 + 3.960000000000001 + 4.09] * 1000 
        ax1.plot(x, y_up_3, color='k', linewidth=1, linestyle='dashed')

        x = numpy.linspace(0,420,1000) 
        y_up_4 = [-15.73 + 3.960000000000001 + 4.09 + 3.5] * 1000 
        ax1.plot(x, y_up_4, color='k', linewidth=1)

        ax1.fill_between(x, y_up_4, y_up_1, color='silver')
        ax1.fill_between(x, y_down_4, y_down_1, color='silver')

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                '#9467bd', '#8c564b', '#e377c2', '#7f7f7f',
                '#bcbd22', '#17becf']

        counter = 0
        for unique_id in df['id'].unique():
            x = df[df['id'] == unique_id]['x'].to_list()
            y = df[df['id'] == unique_id]['y'].to_list()
            ax1.scatter(x, y, s=30, color=colors[counter%len(colors)])

            sx = df[df['id'] == unique_id]['x'].iloc[0]
            sy = df[df['id'] == unique_id]['y'].iloc[0]
            w = df[df['id'] == unique_id]['w'].iloc[0]
            h = df[df['id'] == unique_id]['h'].iloc[0]
            rect = patches.Rectangle((sx-w/2,sy-h/2), w, h, edgecolor='k', linewidth=1, facecolor=colors[counter%len(colors)])
            ax1.add_patch(rect)

            counter += 1

        if save_fig:
            plt.savefig(file_name)
            print("Writing picture:", file_name)

        return fig

    @staticmethod
    def save_meta(file_name, scenario_list):
        """
        Function to save the maneuver over view of the generate sacenario ensemble.
        :param scenario_list: list, Scenario instances
        """
        maneuver_dict = dict()
        for s in scenario_list:
            if s.maneuver.maneuver_type not in maneuver_dict.keys():
                maneuver_dict[s.maneuver.maneuver_type] = list()
            maneuver_dict[s.maneuver.maneuver_type].append(s.scenario_number)

        dump_json(file_name, maneuver_dict)
        


