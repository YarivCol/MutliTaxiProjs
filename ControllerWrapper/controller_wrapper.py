import numpy as np
from typing import Tuple, List
from TaxiWrapper.taxi_wrapper import TAXIS_LOCATIONS, FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, \
    PASSENGERS_STATUS, Taxi, EnvGraph
from itertools import combinations
from networkx.algorithms.approximation.steinertree import steiner_tree
from networkx.algorithms.lowest_common_ancestors import lowest_common_ancestor

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
        self.env_graph = EnvGraph(taxi_env.desc.astype(str))

    def get_taxi_cors(self, taxi_index):
        return self.taxi_env.state[TAXIS_LOCATIONS][taxi_index]

    def get_passenger_cors(self, passenger_index):
        return self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]

    def get_destination_cors(self, passenger_index):
        return self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]

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

    def path_cost(self, origin, dest):
        """
        Args:
            origin: coordinates of origin
            dest: coordinates of destination

        Returns:
        The cost of a path between two points.
        """
        return -len(self.env_graph.get_path(origin, dest)[1])

    def expected_reward(self, taxi_index, passenger_index):
        """
        Calculates the expected (maximal) reward for a taxi to deliver a passenger. This does not consider
        any interruptions along the way (collisions, fuel, etc.).
        Args:
            taxi_index: index of taxi
            passenger_index: index of passenger to be delivered

        Returns:
        Total (maximal) reward for the drive
        """
        taxi_location = self.get_taxi_cors(taxi_index)
        passenger_location = self.get_passenger_cors(passenger_index)
        dropoff_location = self.get_destination_cors(passenger_index)
        return self.path_cost(taxi_location, passenger_location) + taxi_env_rewards['pickup'] \
               + self.path_cost(passenger_location, dropoff_location) + taxi_env_rewards['final_dropoff']

    def find_best_transfer_point(self, from_taxi_index, to_taxi_index, passenger_index):
        """
        Find the best point to transfer the passenger between the taxis. The best point is considered as the point
        closest to the shortest path from the current location of the `to_taxi_index` taxi to the passenger's
        destination. This point will cause the `to_taxi_index` make the smallest possible detour.
        Args:
            from_taxi_index: the index of the taxi currently holding the passenger.
            to_taxi_index: the index of the taxi the passenger should be transferred to.
            passenger_index: the index of the passenger that should be transferred.
        Return:
              The optimal point to make the transfer at.
        """
        # Compute the shortest path of the `to_taxi_index` taxi to the destination of the passenger.
        self.taxis[to_taxi_index].compute_shortest_path(dest=self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index])
        to_taxi_shortest_path = self.taxis[to_taxi_index].path_cords

        from_taxi_remaining_fuel = self.taxi_env.state[FUELS][from_taxi_index]
        # A list of tuples where the first item is the off road distance the `to_taxi` will have to take from the
        # shortest computed path to the closest point the `from_taxi` can get. The second item is the point
        # furthest point that the `from_taxi` can get to, based on its fuel limitations.
        off_road_distances = []
        for point in to_taxi_shortest_path:
            self.taxis[from_taxi_index].compute_shortest_path(dest=point)
            # Compute how many steps of the path the taxi can't complete because of its fuel limit:
            remaining_path = max(0, len(self.taxis[from_taxi_index].path_actions) - from_taxi_remaining_fuel)
            off_road_distances.append((remaining_path, self.taxis[from_taxi_index].path_cords[
                from_taxi_remaining_fuel - 1]))

        # Select the optimal point (the one with minimal off-road steps for `to_taxi`):
        optimal_point = min(off_road_distances, key=lambda x: x[0])[1]
        return optimal_point

    def find_best_paths(self, passenger_indexes):
        passenger_nodes = [self.env_graph.cors_to_node(*self.get_passenger_cors(p_i)) for p_i in passenger_indexes]
        destination_nodes = [self.env_graph.cors_to_node(*self.get_destination_cors(p_i)) for p_i in passenger_indexes]
        nx_graph = self.env_graph.get_nx()
        min_tree = None
        for t1, t2 in combinations(range(len(self.taxis)), 2):
            t1_node = self.env_graph.cors_to_node(*self.get_taxi_cors(t1))
            t2_node = self.env_graph.cors_to_node(*self.get_taxi_cors(t2))
            T = steiner_tree(nx_graph, [t1_node, t2_node] + passenger_nodes + destination_nodes)
            if min_tree is None or len(min_tree.edges) > len(T.edges):
                min_tree = T

