import numpy as np
from typing import Tuple, List
from TaxiWrapper.taxi_wrapper import TAXIS_LOCATIONS, FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, \
    PASSENGERS_STATUS, Taxi

# TODO: Find a decent place for this
taxi_env_rewards = dict(
    step=-1,
    no_fuel=-20,
    bad_pickup=-15,
    bad_dropoff=-15,
    bad_refuel=-10,
    pickup=-1,
    standby_engine_off=-1,
    turn_engine_on=-1,
    turn_engine_off=-1,
    standby_engine_on=-1,
    intermediate_dropoff=-10,
    final_dropoff=100,
    hit_wall=-20,
    collision=-30,
)

class Controller:
    def __init__(self, taxi_env, taxis):
        self.taxi_env = taxi_env
        self.taxis: List[Taxi] = taxis
        self.taxis_actions = [[] for _ in range(len(self.taxis))]

    def get_next_step(self):
        # Check that not all taxis completed all steps:
        if self.not_all_taxis_completed_path():
            taxis_step = []
            for taxi_actions in self.taxis_actions:
                if taxi_actions:
                    step = taxi_actions.pop(0)
                    taxis_step.append(step)
                else:  # if there are no actions left in the path of the taxi, stay in place.
                    taxis_step.append(self.taxi_env.action_index_dictionary['standby'])
            return taxis_step

    def set_meeting_point(self, taxis_index, point):
        """
        Send the taxis given by `taxis_index` to the given point.
        Args:
            taxis_index: a list with the indices of the taxis that should be sent to the given point.
            point: the point to which the taxis should be sent.
        """
        for i in taxis_index:
            self.send_taxi_to_point(i, point)

    def not_all_taxis_completed_path(self):
        """
        Check if not all taxis completed their paths.
        Return `True` if not all taxis completed their path and `False` if some taxi still has steps to do.
        """
        taxi_not_completed_path = [i for i in self.taxis_actions if i]
        return any(taxi_not_completed_path)

    def send_taxi_to_point(self, taxi_index, point):
        """
        Sends taxi number `taxi_index` to the given point.
        Args:
            taxi_index: the index of the taxi that should drive to the specified point.
            point: the location the taxi should drive to.
        """
        self.taxis[taxi_index].compute_shortest_path(dest=point)
        path_to_point = self.taxis[taxi_index].path_actions
        self.taxis_actions[taxi_index].extend(path_to_point)

    def send_taxi_to_pickup(self, taxi_index, passenger_index):
        """
        Sends taxi number `taxi_index` to pickup passenger number `passenger_index` from her current location.
        Args:
            taxi_index: the index of the taxi that should pickup the passenger.
            passenger_index: the index of the passenger that should be picked up.
        """
        # Assign the passenger to the taxi by setting the passenger_index field of the taxi:
        self.taxis[taxi_index].passenger_index = passenger_index

        passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
        self.send_taxi_to_point(taxi_index, point=passenger_location)

        # Add a `pickup` action:
        self.taxis_actions[taxi_index].extend([(self.taxi_env.action_index_dictionary['pickup'])])

    def send_taxi_to_dropoff(self, taxi_index, point=None):
        """
        Sends taxi number `taxi_index` to dropoff its passenger at the location given by `point`. If no dropoff point is
        given, the passenger will be dropped off at her destination.
        Args:
            taxi_index: the index of the taxi that should pickup the passenger
            point (optional): the point at which to dropoff the passenger. If not specified, the passenger will be
            dropped off at her destination.
        """
        self.send_taxi_to_point(taxi_index, point=point)

        # Add a `dropoff` action:
        self.taxis_actions[taxi_index].extend([self.taxi_env.action_index_dictionary['dropoff']])
        self.taxis[taxi_index].passenger_index = None  # todo: change if we allow a taxi to have more than 1 passenger.

    def execute_all_actions(self):
        """
        Execute all actions that were previously computed for all taxis.
        """
        next_step = self.get_next_step()
        total_rewards = np.zeros(len(self.taxis))
        while next_step:
            _, rewards, _ = self.taxi_env.step(next_step)
            total_rewards += rewards
            next_step = self.get_next_step()

    def transfer_passenger(self, passenger_index, from_taxi_index, to_taxi_index, transfer_point):
        """
        Sets both taxis to meet in the transfer point and transfer the passenger between the taxis.
        Args:
             passenger_index: the index of the passenger that should be transferred between the taxis.
             from_taxi_index: the index of the taxi that holds the passenger and brings her to the transfer point.
             to_taxi_index: the index of the taxi that should take the passenger from the meeting point.
             transfer_point: the location were the passenger transfer should take place in.
        """
        # Send both taxis to the transfer point:
        self.send_taxi_to_dropoff(from_taxi_index, transfer_point)
        self.send_taxi_to_point(to_taxi_index, transfer_point)
        self.execute_all_actions()

        # Pickup the passenger by the second taxi:
        self.send_taxi_to_pickup(to_taxi_index, passenger_index)
        self.execute_all_actions()

    def expected_cost(self, origin, dest):
        _, actions = self.taxis[0].env_graph.get_path(origin, dest)  # TODO: Add an env_graph to controller?
        return -len(actions)

    def expected_reward(self, taxi_index, passenger_index):
        taxi_location = self.taxi_env.state[TAXIS_LOCATIONS][taxi_index]
        passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
        dropoff_location = self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]
        total_cost = self.expected_cost(taxi_location, passenger_location) + taxi_env_rewards['pickup']
        total_cost += self.expected_cost(passenger_location, dropoff_location) + taxi_env_rewards['final_dropoff']
        return total_cost
