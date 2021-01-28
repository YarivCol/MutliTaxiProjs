import numpy as np
import networkx as nx
from typing import Tuple, List
from TaxiWrapper.taxi_wrapper import TAXIS_LOCATIONS, FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, \
    PASSENGERS_STATUS, Taxi, EnvGraph
from itertools import combinations
from networkx.algorithms.approximation.steinertree import steiner_tree

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

# TODO: 1. Allow the controller to choose the best taxi to collaborate with.

class Controller:
    def __init__(self, taxi_env, taxis):
        self.taxi_env = taxi_env
        self.taxis: List[Taxi] = taxis
        self.env_graph = EnvGraph(taxi_env.desc.astype(str))

    def get_passenger_cors(self, passenger_index):
        return self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]

    def get_destination_cors(self, passenger_index):
        return self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]

    def get_next_step(self):
        # Check that not all taxis completed all steps:
        if self.any_actions_left():
            taxis_step = {f'taxi_{taxi.taxi_index+1}': taxi.get_next_step() for taxi in self.taxis}
            taxis_step = {item[0]: item[1] for item in taxis_step.items() if item[1] is not None}
            return taxis_step

    def any_actions_left(self):
        """
        Check if not all taxis completed their paths.
        Return `True` if not all taxis completed their path and `False` if some taxi still has steps to do.
        """
        return any([taxi.actions_queue for taxi in self.taxis])

    def execute_all_actions(self, anim=False):
        """
        Execute all actions that were previously computed for all taxis.
        """
        next_step = self.get_next_step()
        renders = []
        while next_step:
            self.taxi_env.step(next_step)
            renders.append(self.taxi_env.render(mode='ansi'))
            next_step = self.get_next_step()
        return renders

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
        self.taxis[from_taxi_index].send_taxi_to_dropoff(transfer_point)
        self.taxis[to_taxi_index].send_taxi_to_point(transfer_point)
        self.execute_all_actions()

        # Pickup the passenger by the second taxi:
        self.taxis[to_taxi_index].send_taxi_to_pickup(passenger_index)
        self.execute_all_actions()

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
        taxi_location = self.taxis[taxi_index].get_location()
        passenger_location = self.get_passenger_cors(passenger_index)
        dropoff_location = self.get_destination_cors(passenger_index)
        return -self.env_graph.path_cost(taxi_location, passenger_location) + taxi_env_rewards['pickup'] \
               - self.env_graph.path_cost(passenger_location, dropoff_location) + taxi_env_rewards['final_dropoff']

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
        path_cords, path_actions = self.taxis[to_taxi_index].compute_shortest_path(
            dest=self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index])
        to_taxi_shortest_path = path_cords
        # Add the current location of the taxi as another optional transfer point:
        to_taxi_shortest_path.insert(0, self.taxis[to_taxi_index].get_location())

        # -1 to avoid finishing all the `from_taxi` fuel as it will not be able to make the dropoff
        from_taxi_remaining_fuel = self.taxi_env.state[FUELS][from_taxi_index] - 1

        # A list of tuples where the first item is the off road distance the `to_taxi` will have to take from the
        # shortest computed path to the closest point the `from_taxi` can get. The second item is the point furthest
        # point that the `from_taxi` can get to, based on its fuel limitations.
        off_road_distances = []
        for point in to_taxi_shortest_path:
            path_cords, path_actions = self.taxis[from_taxi_index].compute_shortest_path(dest=point)
            # Add the current location of the taxi for cases where the taxi has no fuel to move:
            path_cords.insert(0, self.taxis[from_taxi_index].get_location())
            # Compute how many steps of the path the taxi can't complete because of its fuel limit:
            remaining_path = max(0, len(path_actions) - from_taxi_remaining_fuel)
            if remaining_path > 0:
                off_road_distances.append((remaining_path, path_cords[min(from_taxi_remaining_fuel,
                                                                          len(path_actions))]))
            else:
                off_road_distances.append((0, point))

        # Select the optimal point (the one with minimal off-road steps for `to_taxi`):
        optimal_point = min(off_road_distances, key=lambda x: x[0])[1]
        return optimal_point

    def find_closest_taxi(self, dest: List[int]):
        """
        Find the taxi that is closest to the given destination point and has enough fuel to get to this point.
        If such a taxi exists, return its index, else return -1.
        """
        closest_taxi_distance = np.inf
        closest_taxi_index = -1
        for taxi in self.taxis:
            taxi_distance = self.env_graph.path_cost(taxi.get_location(), dest=dest)
            taxi_fuel = taxi.get_fuel()
            if taxi_distance < closest_taxi_distance and taxi_distance < taxi_fuel:
                closest_taxi_distance = taxi_distance
                closest_taxi_index = taxi.taxi_index
        return closest_taxi_index

    def find_best_paths(self, passenger_indexes):
        passenger_nodes = [self.env_graph.cors_to_node(*self.get_passenger_cors(p_i)) for p_i in passenger_indexes]
        destination_nodes = [self.env_graph.cors_to_node(*self.get_destination_cors(p_i)) for p_i in passenger_indexes]
        nx_graph = self.env_graph.get_nx()
        min_tree = None
        taxis = None
        for t1, t2 in combinations(range(len(self.taxis)), 2):
            t1_node = self.env_graph.cors_to_node(*self.taxis[t1].get_location())
            t2_node = self.env_graph.cors_to_node(*self.taxis[t2].get_location())
            T = steiner_tree(nx_graph, [t1_node, t2_node] + passenger_nodes + destination_nodes)
            if min_tree is None or len(min_tree.edges) > len(T.edges):
                min_tree = T
                taxis = t1, t2
        return min_tree, taxis

    def transfer_test(self, passenger_indexes):
        T, taxis = self.find_best_paths(passenger_indexes)

        assigns_pickup = [None] * len(passenger_indexes)
        assigns_dropoff = [None] * len(passenger_indexes)
        t1_node = self.env_graph.cors_to_node(*self.taxis[taxis[0]].get_location())
        t2_node = self.env_graph.cors_to_node(*self.taxis[taxis[1]].get_location())
        for i, p in enumerate(passenger_indexes):
            p_node = self.env_graph.cors_to_node(*self.get_passenger_cors(p))
            d_node = self.env_graph.cors_to_node(*self.get_destination_cors(p))
            if nx.shortest_path_length(T, t1_node, p_node) <= nx.shortest_path_length(T, t2_node, p_node):
                assigns_pickup[i] = taxis[0]
            else:
                assigns_pickup[i] = taxis[1]
            if nx.shortest_path_length(T, t1_node, d_node) <= nx.shortest_path_length(T, t2_node, d_node):
                assigns_dropoff[i] = taxis[0]
            else:
                assigns_dropoff[i] = taxis[1]

        t_center = None
        if assigns_pickup != assigns_dropoff:
            t_center = nx.algorithms.distance_measures.center(T)[0]

        self.show_path(T)
        print("Pickup assignments: {0}".format(assigns_pickup))
        print("Dropoff assignments: {0}".format(assigns_dropoff))
        if t_center is not None:
            print("Transfer point: {0}".format(self.env_graph.node_to_cors(t_center)))

    def show_path(self, T):  # TODO Delete this later, for demo purposes only
        nodes = [self.env_graph.node_to_cors(n) for n in T.nodes]
        nds = np.array(nodes)
        rows, cols = np.max(nds[:, 0]), np.max(nds[:, 1])
        s = ""
        for i in range(rows + 1):
            s = ":"
            for j in range(cols + 1):
                if [i, j] in nodes:
                    s += "O:"
                else:
                    s += " :"
            print(s)


