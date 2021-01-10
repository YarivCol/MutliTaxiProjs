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
controller = Controller(env, taxis=[taxi1, taxi2])

# Send taxi 1 to pick up the passenger:
taxi1.compute_shortest_path(dest=env.state[PASSENGERS_START_LOCATION][taxi1.passenger_index])
print(f'PATH: {taxi1.path_cords}, ACTIONS: {taxi1.path_actions}')
while taxi1.path_cords:
    env.step([taxi1.get_next_step()[1]])
    env.render()
env.step([env.action_index_dictionary['pickup'], env.action_index_dictionary['standby']])


# Set meeting point for both taxis to meet:
controller.set_meeting_point([2, 2])
next_step = controller.get_next_step()
while next_step:
    env.step(next_step)
    next_step = controller.get_next_step()
    env.render()

# Make taxi1 dropoff the passenger and taxi2 pick the passenger up:
env.step([env.action_index_dictionary['dropoff'], env.action_index_dictionary['standby']])
env.step([env.action_index_dictionary['standby'], env.action_index_dictionary['pickup']])

# Make taxi2 bring the passenger to her destination:
taxi2.passenger_index = 0
taxi2.compute_shortest_path()
print(f'PATH: {taxi2.path_cords}, ACTIONS: {taxi2.path_actions}')
while taxi2.path_cords:
    env.step([env.action_index_dictionary['standby'], taxi2.get_next_step()[1]])
    env.render()

env.step([env.action_index_dictionary['standby'], env.action_index_dictionary['dropoff']])
env.render()
print('Great Success!')

