from multitaxienv.taxi_environment import TaxiEnv
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller

# Initialize a new environment with 2 taxis at a random location and 1 passenger, and display it:
env = TaxiEnv(num_taxis=2, num_passengers=1, max_fuel=None,
              taxis_capacity=None, collision_sensitive_domain=False,
              fuel_type_list=None, option_to_stand_by=True)
env.reset()
env.s = 1022
env.render()

# Initialize a Taxi object for each taxi:
taxi1 = Taxi(env, taxi_index=0, passenger_index=0)
taxi2 = Taxi(env, taxi_index=1)

# Send taxi 1 to pick up the passenger:
taxi1.compute_shortest_path(dest=env.state[PASSENGERS_START_LOCATION][taxi1.passenger_index])
print(f'PATH: {taxi1.path_cords}, ACTIONS: {taxi1.path_actions}')
while taxi1.path_cords:
    env.step([taxi1.get_next_step()[1]])
    env.render()

