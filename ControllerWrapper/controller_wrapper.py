from typing import Tuple, List
from TaxiWrapper.taxi_wrapper import TAXIS_LOCATIONS, FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, \
    PASSENGERS_STATUS, Taxi


class Controller:
    def __init__(self, taxi_env, taxis):
        self.taxi_env = taxi_env
        self.taxis: List[Taxi] = taxis
        self.taxis_actions = [[] * len(self.taxis)]

    def get_next_step(self):
        # Check that not all taxis completed all steps:
        if self.not_all_taxis_completed_path():
            taxis_steps = []
            for taxi in self.taxis:
                step = taxi.get_next_step()
                if step:
                    taxis_steps.append(step[1])
                else:  # if there are no actions left in the path of the taxi, stay in place.
                    taxis_steps.append(self.taxi_env.action_index_dictionary['standby'])
            return taxis_steps

    def set_meeting_point(self, point):
        """
        Compute the path of all taxis to a given meeting point.
        """
        for taxi in self.taxis:
            taxi.compute_shortest_path(point)

    def not_all_taxis_completed_path(self):
        """
        Check if not all taxis completed their paths.
        Return `True` if not all taxis completed their path and `False` if some taxi still has steps to do.
        """
        taxi_not_completed_path = [taxi for taxi in self.taxis if taxi.path_cords]
        return any(taxi_not_completed_path)

    def send_taxi_to_pickup(self, taxi_index, passenger_index):
        """
        Sends taxi number taxi_index to pickup passenger number passenger_index from her current location.
        Args:
            taxi_index: the index of the taxi that should pickup the passenger
            passenger_index: the index of the passenger that should be picked up
        """
        passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
        path_to_passenger = self.taxis[taxi_index].compute_shortest_path(dest=passenger_location)[1]
        self.taxis_actions[taxi_index].append(path_to_passenger)

