from multitaxienv.taxi_environment import TaxiEnv, orig_MAP, MAP
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller

# Initialize a new environment with 2 taxis at random locations and 2 passengers, and display it:
env = TaxiEnv(num_taxis=2, num_passengers=2, max_fuel=[8, 8],
              taxis_capacity=None, collision_sensitive_domain=False,
              fuel_type_list=None, option_to_stand_by=True, domain_map=orig_MAP)
env.reset()
env.render()

# Initialize a Taxi object for each taxi:
taxi1 = Taxi(env, taxi_index=0)
taxi2 = Taxi(env, taxi_index=1)
controller = Controller(env, taxis=[taxi1, taxi2])

# Assign the passenger to the closest taxi:
closest_taxi = controller.find_closest_taxi(dest=controller.get_passenger_cors(passenger_index=0))
controller.taxis[closest_taxi].assigned_passengers.append(0)


# Send the closest taxi to pick up the passenger:
controller.taxis[closest_taxi].send_taxi_to_pickup()
controller.execute_all_actions()
env.render()

# Transfer the passenger between the two taxis:
to_taxi = 1 - closest_taxi
# Compute the transfer point according to the first heuristic:
transfer_point = controller.find_transfer_point_h1(from_taxi_index=closest_taxi, to_taxi_index=to_taxi,
                                                   passenger_index=0)
controller.transfer_passenger(passenger_index=0, from_taxi_index=closest_taxi, to_taxi_index=to_taxi,
                              transfer_point=transfer_point)
env.render()

# Send the to_taxi to dropoff the passenger at her destination:
controller.taxis[to_taxi].send_taxi_to_dropoff()
controller.execute_all_actions()
env.render()

