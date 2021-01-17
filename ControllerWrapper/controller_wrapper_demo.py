from multitaxienv.taxi_environment import TaxiEnv
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller

# Initialize a new environment with 2 taxis at a random location and 1 passenger, and display it:
env = TaxiEnv(num_taxis=2, num_passengers=1, max_fuel=[10, 10],
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
path_to_passenger = taxi1.send_taxi_to_pickup(passenger_index=0)
controller.taxis_actions[0].extend(path_to_passenger)
print(f'TAXIS ACTIONS:{controller.taxis_actions}')
controller.execute_all_actions()
env.render()

# Transfer the passenger between the two taxis:
transfer_point = controller.find_best_transfer_point(from_taxi_index=0, to_taxi_index=1, passenger_index=0)
controller.transfer_passenger(passenger_index=0, from_taxi_index=0, to_taxi_index=1, transfer_point=transfer_point)
env.render()

# Send the second taxi to dropoff the passenger at her destination:
path_to_destination = taxi2.send_taxi_to_dropoff()
controller.taxis_actions[1].extend(path_to_destination)
controller.execute_all_actions()
env.render()


