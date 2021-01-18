import networkx as nx
import numpy as np
from typing import Tuple, List

TAXIS_LOCATIONS, FUELS, PASSENGERS_START_LOCATION, PASSENGERS_DESTINATIONS, PASSENGERS_STATUS = 0, 1, 2, 3, 4


class EnvGraph:
    """
    This class converts the map of the taxi-world into a Networkx graph.
    Each square in the map is represented by a node in the graph. The nodes are indexed by rows, i.e. for a 4-row by
    5-column grid, node in location [0, 2] (row-0, column-2) has index 2 and node in location [1,1] has index 6.
    """
    def __init__(self, desc: list):
        """
        Args:
            desc: Map description (list of strings)
        """
        self.rows = len(desc) - 2
        self.cols = len(desc[0]) // 2
        self.graph = nx.empty_graph(self.rows * self.cols)
        for i in self.graph.nodes:
            row, col = self.node_to_cors(i)
            if desc[row + 2][col * 2 + 1] != '-':  # Check south
                self.graph.add_edge(i, self.cors_to_node(row + 1, col))
                # In case we ever use horizontal barriers
            if desc[row + 1][col * 2 + 2] == ':':  # Check east
                self.graph.add_edge(i, self.cors_to_node(row, col + 1))

    def node_to_cors(self, node) -> List:
        """
        Converts a node index to its corresponding coordinate point on the grid.
        """
        return [node // self.cols, node % self.cols]

    def cors_to_node(self, row, col) -> int:
        """
        Converts a grid coordinate to its corresponding node in the graph.
        """
        return row * self.cols + col

    def get_path(self, origin: (int, int), target: (int, int)) -> Tuple[list, list]:
        """
        Computes the shortest path in the graph from the given origin point to the given target point.
        Returns a tuple of lists where the first list represents the coordinates of the nodes that are along the path,
        and the second list represent the actions that should be taken to make the shortest path.
        """
        node_origin, node_target = self.cors_to_node(*origin), self.cors_to_node(*target)
        if node_origin == node_target:
            return [], []

        path = nx.shortest_path(self.graph, node_origin, node_target)
        cord_path = [self.node_to_cors(node) for node in path]
        actions = []
        for node in range(len(path) - 1):
            delta = path[node + 1] - path[node]
            if delta == -1:  # West
                actions.append(3)
            elif delta == 1:  # East
                actions.append(2)
            elif delta == -self.cols:  # North
                actions.append(1)
            else:  # South
                actions.append(0)
        return cord_path[1:], actions

    def get_nx(self) -> nx.Graph:
        return self.graph.copy()

    def path_cost(self, origin, dest):
        """
        Args:
            origin: coordinates of origin
            dest: coordinates of destination

        Returns:
        The cost of a path between two points.
        """
        return len(self.get_path(origin, dest)[1])


class Taxi:
    def __init__(self, taxi_env, taxi_index, passenger_index=None):
        self.taxi_env = taxi_env
        self.taxi_index = taxi_index
        self.passenger_index = passenger_index
        self.path_cords = []
        self.path_actions = []
        self.env_graph = EnvGraph(taxi_env.desc.astype(str))
        self.previous_coordinate = self.taxi_env.state[TAXIS_LOCATIONS][self.taxi_index]
        self.previous_action = None
        self.assigned_passengers = []
        self.communication_channel = []

    def compute_shortest_path(self, dest: list = None, origin: list = None):
        """
        Given a destination point represented by a list of [row, column], compute the shortest path to it from the
        current location of the taxi. If a destination point isn't specified, the shortest path to the passenger's
        destination will be computed.
        """
        env_state = self.taxi_env.state
        origin = origin if origin is not None else env_state[TAXIS_LOCATIONS][self.taxi_index]

        # If a destination point wasn't specified, go to the passenger's destination
        if not dest:
            if self.passenger_index is not None:
                dest = env_state[PASSENGERS_DESTINATIONS][self.passenger_index]
            else:  # if the taxi has no allocated passenger, stay in place, i.e don't do any action.
                self.path_cords, self.path_actions = [], []
                return

        cord_path, actions = self.env_graph.get_path(origin, dest)
        self.path_cords = cord_path
        self.path_actions = actions

    def get_next_step(self):
        """
        Gets the next step in the path of the shortest path that was previously computed.
        Returns a tuple where the first item is the coordinate of the next step and the second item is the action.
        """
        # Check if the last step moved the taxi - if not, it prevented a collision and should be executed again.
        if self.taxi_env.state[TAXIS_LOCATIONS][self.taxi_index] != self.previous_coordinate:
            return self.previous_coordinate, self.previous_action

        if self.path_cords and self.path_actions:
            next_coordinate = self.path_cords.pop(0)
            next_action = self.path_actions.pop(0)
            self.previous_coordinate = next_coordinate
            self.previous_action = next_action
            return next_coordinate, next_action

    def update_env_state(self, new_state):
        """
        Updates the state of the environment to the new given state.
        """
        self.taxi_env = new_state

    def get_location(self):
        """
        Returns the current location of the taxi.
        """
        return self.taxi_env.state[TAXIS_LOCATIONS][self.taxi_index]

    def get_fuel(self):
        """
        Returns the current fuel state of the taxi.
        """
        return self.taxi_env.state[FUELS][self.taxi_index]

    def path_cost(self, dest: List[int], origin: List[int] = None):
        """
        Compute the cost of the path from the taxi's current location to a given destination point.
        """
        origin = origin if origin else self.taxi_env.state[TAXIS_LOCATIONS][self.taxi_index]
        _, actions = self.env_graph.get_path(origin, dest)
        return len(actions)

    def send_taxi_to_point(self, point):
        """
        Sends the taxi to the given point.
        Args:
            point: the location the taxi should drive to.
        """
        self.compute_shortest_path(dest=point)
        path_to_point = self.path_actions
        return path_to_point

    def send_taxi_to_pickup(self, passenger_index):
        """
        Sends the taxi to pickup passenger number `passenger_index` from her current location.
        Args:
            passenger_index: the index of the passenger that should be picked up.
        """
        # Assign the passenger to the taxi by setting the passenger_index field of the taxi:
        self.passenger_index = passenger_index  # todo: check if necessary

        passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
        path_to_passenger = self.send_taxi_to_point(point=passenger_location)

        # Add a `pickup` action:
        path_to_passenger.extend([(self.taxi_env.action_index_dictionary['pickup'])])

        return path_to_passenger

    def send_taxi_to_dropoff(self, point=None):
        """
        Sends the taxi to dropoff its passenger at the location given by `point`. If no dropoff point is given,
        the passenger will be dropped off at her destination.
        Args:
            point (optional): the point at which to dropoff the passenger. If not specified, the passenger will be
            dropped off at her destination.
        """
        path_to_destination = self.send_taxi_to_point(point=point)

        # Add a `dropoff` action:
        path_to_destination.extend([self.taxi_env.action_index_dictionary['dropoff']])
        self.passenger_index = None  # todo: change if we allow a taxi to have more than 1 passenger.

        return path_to_destination

    def passenger_allocation_message(self, passenger_index):
        """
        Broadcast a message with information about the cost of the path to a specific passenger and the shortest
        path from the taxi's current location to the destination of the passenger.
        """
        passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
        origin = None
        # Check if the taxi has an allocated passenger. If yes, compute the cost from this passenger's location:
        if self.assigned_passengers:
            origin = self.taxi_env.state[PASSENGERS_START_LOCATION][self.assigned_passengers[-1]]

        pickup_cost = self.path_cost(dest=passenger_location, origin=origin)
        message = {
            'taxi_index': self.taxi_index,
            'passenger_index': passenger_index,
            'pickup_cost': pickup_cost
        }
        return message

    def request_help_message(self, passenger_index):
        """
        Broadcast a message to all taxis, requesting for help to bring the assigned taxi to the destination.
        """
        passenger_destination = self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]
        path_cost = self.path_cost(dest=passenger_destination)

        # Request for help if the taxi hasn't enough fuel:
        if path_cost >= self.get_fuel():
            message = {
                'taxi_index': self.taxi_index,
                'passenger_index': passenger_index
            }
            return message

    def passenger_transfer_message(self, passenger_index):
        """
        Broadcast a message with information about the path to the given passenger's destination.
        """
        passenger_destination = self.taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]
        self.compute_shortest_path(dest=passenger_destination)
        message = {
            'taxi_index': self.taxi_index,
            'passenger_index': passenger_index,
            'shortest_path': self.path_cords
        }
        return message

    def decide_assignments(self):
        """
        Go over all messages and check which taxi is the closets to every passenger. The taxi assigns to itself the
        passengers that are closest to it.
        """
        pickup_cost = np.inf
        assigned_taxi = -1
        for message in self.communication_channel:
            passenger_index = message['passenger_index']
            taxi_index = message['taxi_index']
            if message['pickup_cost'] < pickup_cost:
                pickup_cost = message['pickup_cost']
                assigned_taxi = taxi_index

        if assigned_taxi == self.taxi_index:
            self.assigned_passengers.append(passenger_index)

        # Clear the communication channel:
        self.communication_channel = []

    def pickup_passengers(self):
        """
        Send the taxi to pickup every passenger that is assigned to it.
        """
        pickup_steps = []
        for passenger in self.assigned_passengers:
            passenger_location = self.taxi_env.state[PASSENGERS_START_LOCATION][passenger]
            self.compute_shortest_path(dest=passenger_location, origin=self.path_cords[-1] if pickup_steps else None)
            pickup_steps.extend(self.path_actions)

            # Add pickup step
            pickup_steps.append(self.taxi_env.action_index_dictionary['pickup'])

        return pickup_steps

    def listen(self, message):
        """
        Listen to new messages broadcast by different taxis and add it to the taxi's communication channel.
        """
        self.communication_channel.append(message)





