import math
import numpy as np
np.seterr(all='raise')


def thw(x_rel, v_follow):
    """
    Function to calculate the time headway measure for a following vehicle.
    This value takes only the position of the leading vehcile into consideration which means.
    how long it takes to reach this position und thus returns always a valid value.
    :param x_rel: float, the rel position of the vehciles
    :param v_follow: float, the velocity of the following vehcile
    :return: float, the time to headway
    """
    ######### Your code starts here #########

    ######### Your code ends here #########

    return thw


def ttc(x_rel, v_follow, v_lead):
    """
    Function to calculate the time to collison measure for a following vehicle.
    This value makes only sense if the following vehicle has a higher velocity than the leading.
    :param x_follow: float, the position of the following vehcile
    :param x_lead: float, the position of the leading vehcile
    :param v_follow: float, the velocity of the following vehcile
    :param v_lead: float, the velocity of the leading vehcile
    :return: float, the time to collision or None if the following vehicle is slower
    """
    ######### Your code starts here #########
    # if the risk measure is not defined for the given parameters return np.nan
    # check if the following velocity is faster







    ######### Your code ends here #########

    return ttc


def ettc(x_rel, v_follow, v_lead, a_follow, a_lead):
    """
    Function to calculate the time to collison measure for a following vehicle.
    This value makes only sense if the following vehicle has a higher velocity than the leading.
    :param x_follow: float, the position of the following vehcile
    :param x_lead: float, the position of the leading vehcile
    :param v_follow: float, the velocity of the following vehcile
    :param v_lead: float, the velocity of the leading vehcile
    :param a_follow: float, the acceleration of the following vehcile
    :param a_lead: float, the acceleration of the leading vehcile
    :return: float, the enhanced time to collision or None if the following vehicle is slower (considering also acceleartion)
    """
    ######### Your code starts here #########
    # if the risk measure is not defined for the given parameters return np.nan


























    ######### Your code ends here #########

    return ettc
