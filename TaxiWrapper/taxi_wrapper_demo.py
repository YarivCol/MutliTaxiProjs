from TaxiWrapper.taxi_environment import TaxiEnv
from TaxiWrapper.taxi_wrapper import *

# Initialize a new environment with 1 taxi at a random location and display it:
env = TaxiEnv(num_taxis=1, num_passengers=2, max_fuel=None,
              taxis_capacity=None, collision_sensitive_domain=False,
              fuel_type_list=None, option_to_stand_by=True)
# env = TaxiEnv()
env.reset()
env.s = 1022
env.render()

# Initialize a new taxi object for the taxi, and send it to pick up the second passenger:
taxi1 = Taxi(env, taxi_index=0, passenger_index=1)
taxi1.compute_shortest_path(dest=env.state[PASSENGERS_START_LOCATION][1])
print(f'PATH: {taxi1.path_cords}, ACTIONS: {taxi1.path_actions}')
while taxi1.path_cords:
    env.step([taxi1.get_next_step()[1]])
    env.render()

# Pickup the taxi:
env.step([4])

# Compute path to passenger's destination and drop her off there.
taxi1.compute_shortest_path()
print(f'PATH: {taxi1.path_cords}, ACTIONS: {taxi1.path_actions}')
while taxi1.path_cords:
    env.step([taxi1.get_next_step()[1]])
    env.render()

# drop off the passenger at the destination and show the env state:
env.step([5])
env.render()




