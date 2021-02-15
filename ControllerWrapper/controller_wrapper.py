import numpy as np
from typing import List
from TaxiWrapper.taxi_wrapper import FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, Taxi, EnvGraph


class Controller:
    def __init__(self, taxi_env, taxis):
        self.taxi_env = taxi_env
        if taxis:
            self.taxis: List[Taxi] = taxis
        else:
            self.taxis = [Taxi(taxi_env, i) for i in range(taxi_env.num_taxis)]
        self.env_graph = EnvGraph(taxi_env.desc.astype(str))

    def get_passenger_cors(self, passenger_index):
        """
        Return the current location of the passenger given by passenger_index.
        """
        return self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]

    def get_destination_cors(self, passenger_index):
        """
        Return the destination of the passenger given by passenger_index
        """
        return self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]

    def get_next_step(self):
        """
        Returns the next step of all taxis that should be passed to the environment.
        """
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

    def execute_all_actions(self):
        """
        Execute all actions that were previously computed for all taxis.
        """
        next_step = self.get_next_step()
        while next_step:
            self.taxi_env.step(next_step)
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
        self.taxis[from_taxi_index].send_taxi_to_dropoff(transfer_point)
        self.taxis[to_taxi_index].send_taxi_to_point(transfer_point)
        self.execute_all_actions()

        # Pickup the passenger by the second taxi:
        self.taxis[to_taxi_index].assigned_passengers.append(passenger_index)
        self.taxis[to_taxi_index].send_taxi_to_pickup()
        self.execute_all_actions()

    def find_transfer_point_h1(self, from_taxi_index, to_taxi_index, passenger_index):
        """
        Find a point to transfer the passenger between the taxis according to the first heuristic.
        This heuristic sets the transfer point as the point closest to the shortest path from the current location of
        the `to_taxi_index` taxi to the passenger's destination. This point will cause the `to_taxi_index` to make the
        smallest possible detour.
        Args:
            from_taxi_index: the index of the taxi currently holding the passenger.
            to_taxi_index: the index of the taxi the passenger should be transferred to.
            passenger_index: the index of the passenger that should be transferred.
        Return:
              The point that will cause the to_taxi to make the smallest possible detour.
        """
        # Compute the shortest path of the `to_taxi_index` taxi to the destination of the passenger.
        path_cords, path_actions = self.taxis[to_taxi_index].compute_shortest_path(dest=self.get_destination_cors(passenger_index))
        to_taxi_shortest_path = path_cords
        # Add the current location of the taxi as another optional transfer point:
        to_taxi_shortest_path.insert(0, self.taxis[to_taxi_index].get_location())

        # -1 to avoid consuming all the `from_taxi` fuel as it will not be able to make the dropoff
        from_taxi_remaining_fuel = self.taxi_env.state[FUELS][from_taxi_index] - 1

        # A list of tuples where the first item is the off-road distance the `to_taxi` will have to take from the
        # shortest computed path to the closest point the `from_taxi` can get. The second item is the furthest point
        # that the `from_taxi` can get to, based on its fuel limitations.
        off_road_distances = []
        for point in to_taxi_shortest_path:
            path_cords, path_actions = self.taxis[from_taxi_index].compute_shortest_path(dest=point)
            # Add the current location of the taxi for cases where the taxi has no fuel to move:
            path_cords.insert(0, self.taxis[from_taxi_index].get_location())
            # Compute how many steps of the path the taxi can't complete because of its fuel limit:
            remaining_path = max(0, len(path_actions) - from_taxi_remaining_fuel)
            if remaining_path > 0:
                off_road_distances.append((remaining_path, path_cords[min(from_taxi_remaining_fuel, len(path_actions))]))
            else:
                off_road_distances.append((0, point))

        # Select the optimal point (the one with minimal off-road steps for `to_taxi`):
        optimal_point = min(off_road_distances, key=lambda x: x[0])[1]
        return optimal_point

    def find_transfer_point_h2(self, from_taxi_index, passenger_index):
        """
        Find a point to transfer the passenger between the taxis according to the second heuristic.
        This heuristic sets the transfer point as the furthest point that the taxi that picked up the passenger can
        take her on the shortest path from the current location to the destination.
        Args:
            from_taxi_index: the index of the taxi currently holding the passenger.
            passenger_index: the index of the passenger that should be transferred.
        Return:
            The furthest point along the shortest path of the passenger to the destination, the taxi can get to.
        """
        shortest_path = self.taxis[from_taxi_index].compute_shortest_path(dest=self.get_destination_cors(passenger_index))[0]
        shortest_path.insert(0, self.taxis[from_taxi_index].get_location())
        transfer_point = shortest_path[max(0, min(self.taxis[from_taxi_index].get_fuel() - 1, len(shortest_path)-1))]
        return transfer_point

    def find_optimal_transfer_point(self, from_taxi_index, to_taxi_index, passenger_index):
        """
        Find the optimal point to transfer the passenger between the taxis.
        The function checks all the points in the board as possible transfer points and returns the point that
        minimizes the distance of the passenger from the destination.
        Args:
            from_taxi_index: the index of the taxi currently holding the passenger.
            to_taxi_index: the index of the taxi the passenger should be transferred to.
            passenger_index: the index of the passenger that should be transferred.
        Return:
            The optimal point to make the transfer at.
        """
        best_transfer_point = []
        min_dist_from_dest = np.inf
        from_taxi = self.taxis[from_taxi_index]
        to_taxi = self.taxis[to_taxi_index]
        passenger_destination = self.get_destination_cors(passenger_index)
        for row in range(self.env_graph.rows):
            for col in range(self.env_graph.cols):
                transfer_point = [row, col]

                # check if the from_taxi has enough fuel to reach the transfer point, if not continue
                path_to_point_cost = from_taxi.path_cost(dest=transfer_point)
                if path_to_point_cost > from_taxi.get_fuel() - 1:
                    continue
                else:
                    # check if the to_taxi has enough fuel to reach the transfer point, if not compute the dist to dest
                    path_to_point_cost = to_taxi.path_cost(dest=transfer_point)
                    if path_to_point_cost > to_taxi.get_fuel() - 1:
                        dist_from_dest = from_taxi.path_cost(dest=passenger_destination, origin=transfer_point)
                    else:
                        total_path_cost = path_to_point_cost + to_taxi.path_cost(origin=transfer_point,
                                                                                 dest=passenger_destination)
                        dist_from_dest = max(0, total_path_cost - (to_taxi.get_fuel() - 1))

                    # Check if this transfer point is better than the current transfer point:
                    if dist_from_dest < min_dist_from_dest:
                        min_dist_from_dest = dist_from_dest
                        best_transfer_point = transfer_point
        return best_transfer_point

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

    def allocate_passengers(self):
        """
        Allocate all passengers to the taxis based on the distance of every taxi from the passenger.
        """
        for i in range(self.taxi_env.num_passengers):
            costs = [taxi.pickup_cost(passenger_index=i) for taxi in self.taxis]
            assigned_taxi = costs.index(min(costs))
            self.taxis[assigned_taxi].assigned_passengers.append(i)

    def pickup_passengers(self):
        """
        Send all taxis to pickup all their passengers.
        """
        for taxi in self.taxis:
            taxi.pickup_multiple_passengers()

        self.execute_all_actions()
