from multitaxienv.taxi_environment import TaxiEnv
from TaxiWrapper.taxi_wrapper import *
import numpy as np
import random

MAP = [
    "+---------------------------+",
    "|X: : : : : : | :F:X: : : :X|",
    "| : : : : : : | : : : : : : |",
    "| : : : : : : | : : : : : : |",
    "| : : : : : : |X: : : : : : |",
    "| : : : : : : | : : : : : : |",
    "| : : : : : : : : : : : : : |",
    "| :X: : : : : | : : : : : : |",
    "| : : : : : :X| : : : : : : |",
    "| : : : : : : | : : : : : : |",
    "| : : : : : : | : : : : : : |",
    "|X: : : : :G: | : : : : : :X|",
    "+---------------------------+"]
 
taxi_count = 4

# env = TaxiEnv()

class GoToNearestTaxi(Taxi):
    def __init__(self, taxi_env, taxi_index):
        super().__init__(taxi_env, taxi_index)
        self._current_passanger = -1
        self._current_passanger_target = -1
        self._cooperative = False

    def set_teammate(self, teammate):
        self._cooperative = True
        self._teammate = teammate

    def get_next_action(self, curr_score):
        if self._current_passanger == -1:
            paths_length = []
            for j in range(8):
                if not self.taxi_env.is_passanger_taken(j) and (self._cooperative and self._teammate._current_passanger_target != j):
                    self.compute_shortest_path(dest=self.taxi_env.state[PASSENGERS_START_LOCATION][j])
                    paths_length.append(len(self.path_actions))
                else:
                    paths_length.append(10000)

            paths_length = np.array(paths_length)
            current_passanger = np.argmin(np.random.random(paths_length.shape) * 
                                          (paths_length==paths_length.max()))

            self._current_passanger_target = current_passanger

            self.compute_shortest_path(dest=env.state[PASSENGERS_START_LOCATION][current_passanger])

            if len(self.path_actions) == 0:
                #pickup
                self._current_passanger = current_passanger
                return 4
            return self.get_next_step()[1]
        else:
            self.compute_shortest_path(dest=env.state[PASSENGERS_DESTINATIONS][self._current_passanger])
            if len(self.path_actions) == 0:
                self._current_passanger = -1
                #dropoff
                return 5
            return self.get_next_step()[1]

SUICIDE_THRESHOLD = 3

class SuicideOnWinTaxi(Taxi):
    def __init__(self, taxi_env, taxi_index):
        super().__init__(taxi_env, taxi_index)
        self._current_passanger = -1
        self._current_target = -1
        self._cooperative = False

    def set_teammate(self, teammate):
        self._cooperative = True
        self._teammate = teammate

    def get_next_action(self, curr_score):
        if self._current_passanger == -1:

            target_is_passengers = True

            if curr_score[self.taxi_index % 2] - curr_score[(self.taxi_index + 1) % 2] > SUICIDE_THRESHOLD:
                taxi_loc = self.taxi_env.state[TAXIS_LOCATIONS]
                target_options = [taxi_loc[i] for i in range(len(taxi_loc)) if ((i % 2) == (self.taxi_index + 1) % 2) and env.collided[i] == 0]
                if target_options:
                    target_is_passengers = False
                else:
                    target_options = self.taxi_env.state[PASSENGERS_START_LOCATION]
            else:
                target_options = self.taxi_env.state[PASSENGERS_START_LOCATION]

            paths_length = []

            for j in range(len(target_options)):
                if not self.taxi_env.is_passanger_taken(j) and (self._cooperative and self._teammate._current_target != j):
                    self.compute_shortest_path(dest=target_options[j])
                    paths_length.append(len(self.path_actions))
                else:
                    paths_length.append(10000)

            paths_length = np.array(paths_length)

            self._current_target = np.argmin(np.random.random(paths_length.shape) * 
                                             (paths_length==paths_length.max()))

            self.compute_shortest_path(dest=target_options[self._current_target])

            if len(self.path_actions) == 0 and target_is_passengers:
                #pickup
                self._current_passanger = self._current_target
                return 4
            return self.get_next_step()[1]

        else:
            self.compute_shortest_path(dest=env.state[PASSENGERS_DESTINATIONS][self._current_passanger])
            if len(self.path_actions) == 0:
                self._current_passanger = -1
                #dropoff
                return 5
            return self.get_next_step()[1]


wins = np.zeros(2)

for i in range(100):
    # Initialize a new environment with 1 taxi at a random location and display it:
    env = TaxiEnv(num_taxis=taxi_count, num_passengers=8, max_fuel=None,
                  taxis_capacity=None, collision_sensitive_domain=True,
                  fuel_type_list=None, option_to_stand_by=False, domain_map=MAP)


    env.reset()
    env.s = 1022
    #env.render()

    # Initialize a new taxi object for the taxi, and send it to pick up the second passenger:
    taxis = [GoToNearestTaxi(env, taxi_index=i) for i in range(taxi_count)]

    taxis[1] = SuicideOnWinTaxi(env, 1)
    taxis[3] = SuicideOnWinTaxi(env, 3)

    #taxis[0] = SuicideOnWinTaxi(env, 0)
    #taxis[2] = SuicideOnWinTaxi(env, 2)

    taxis[0].set_teammate(taxis[2])
    taxis[2].set_teammate(taxis[0])

    taxis[1].set_teammate(taxis[3])
    taxis[3].set_teammate(taxis[1])
    done = False

    curr_score = np.zeros(2)

    while not done:
        action = [taxi.get_next_action(curr_score) for taxi in taxis]
        state, r, done = env.step(action)

        for i in range(taxi_count):
            if i in r:
                curr_score[i % 2] += r[i]

        env.render()
    
    wins += ((curr_score - np.min(curr_score)) > 0)

print(wins)


#Todo added ablity for taxi to change course for another taxis by blocking passages

#Todo Create team 

#TaxiCup








