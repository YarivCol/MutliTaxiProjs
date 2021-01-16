import networkx as nx
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

    def compute_shortest_path(self, dest: list = None):
        """
        Given a destination point represented by a list of [row, column], compute the shortest path to it from the
        current location of the taxi. If a destination point isn't specified, the shortest path to the passenger's
        destination will be computed.
        """
        env_state = self.taxi_env.state
        current_location = env_state[TAXIS_LOCATIONS][self.taxi_index]

        # If a destination point wasn't specified, go to the passenger's destination
        if not dest:
            if self.passenger_index is not None:
                dest = env_state[PASSENGERS_DESTINATIONS][self.passenger_index]
            else:  # if the taxi has no allocated passenger, stay in place, i.e don't do any action.
                self.path_cords, self.path_actions = [], []
                return

        cord_path, actions = self.env_graph.get_path(current_location, dest)
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

