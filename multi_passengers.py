import numpy as np
from multitaxienv.taxi_environment import TaxiEnv
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller

MAP2 = [
    "+-----------------------+",
    "|X: |F: | : | : | : |F: |",
    "| : | : : : |X: | : | : |",
    "| : :X: : : : : : : : : |",
    "| : : : : :X| : :X: : : |",
    "| : :X: : : | : : : : : |",
    "|X: : : : : : : : : : : |",
    "| | :G| |X| :G| | | : |X|",
    "+-----------------------+",
]

orig_MAP = [
    "+---------+",
    "|X: |F: :X|",
    "| : | : : |",
    "| : : : : |",
    "| | : | : |",
    "|X| :G|X: |",
    "+---------+",
]

def show_path(nodes):
    nds = np.array(nodes)
    rows, cols = np.max(nds[:, 0]), np.max(nds[:, 1])
    for i in range(rows + 1):
        s = ":"
        for j in range(cols + 1):
            if [i, j] in nodes:
                s += "O:"
            else:
                s += " :"
        print(s)

# Initialize a new environment with 2 taxis at a random location and multiple passengers, and display it:
env = TaxiEnv(num_taxis=3, num_passengers=5,
              taxis_capacity=[5, 5, 5], collision_sensitive_domain=False,
              fuel_type_list=None, option_to_stand_by=True, domain_map=MAP2)
env.reset()
env.s = 1022
print(f'STATE: {env.state}')
# controller.transfer_test([0, 1, 2])
env.state = [[[2, 3], [1, 0], [5, 2]], [10000, 10000, 10000], [[6, 11], [3, 5], [1, 6], [2, 2], [3, 8]], [[3, 8], [4, 2], [3, 8], [1, 6], [6, 11]], [2, 2, 2, 2, 2]]

env.render()


# Initialize a Taxi object for each taxi:
# taxi1 = Taxi(env, taxi_index=0)
# taxi2 = Taxi(env, taxi_index=1)
# taxi3 = Taxi(env, taxi_index=2)
controller = Controller(env, taxis=None)

# Allocate the passengers to the taxis:
controller.allocate_passengers()

# TODO: In the pickup step, mind the order of passengers pickup - maybe drop-off a passenger if a taxi has more than
#  one passenger allocated.

# Send all taxi to pickup their passengers:
controller.pickup_passengers()
env.render()

controller.bla()

print('Great Success')

