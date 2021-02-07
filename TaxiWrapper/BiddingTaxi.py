import numpy as np
from TaxiWrapper.taxi_wrapper import Taxi, EnvGraph, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, TAXIS_LOCATIONS
from typing import List


class BiddingTaxi(Taxi):
    def __init__(self, taxi_env, taxi_index, passenger_index=None):
        super().__init__(taxi_env, taxi_index, passenger_index)

    def compute_shortest_path(self, start_point: list, dest_point: list):
        """
        Input: start_point, a list of [row, column]
               dest_point, a list of [row, column]
        Output: Length of shortest path between the two points
        """
        cord_path, _actions = self.env_graph.get_path(start_point, dest_point)
        return cord_path
    
    def calculate_distances_to_all_passengers(self):
        taxi_location = self.taxi_env.state[TAXIS_LOCATIONS][self.taxi_index]
        passengers_start_locations = self.taxi_env.state[PASSENGERS_START_LOCATION]

        num_passengers = self.taxi_env.num_passengers

        distances = np.full(num_passengers, 0)

        for passenger_ind in range(num_passengers):
            passenger_start_loc = passengers_start_locations[passenger_ind]

            taxi_to_passenger_distance = len(self.compute_shortest_path(taxi_location, passenger_start_loc))

            distances[passenger_ind] = taxi_to_passenger_distance
        #TODO: remove print
        print(distances)
        return distances