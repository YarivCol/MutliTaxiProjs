from TaxiWrapper.taxi_wrapper import *
from multitaxienv.taxi_environment import TaxiEnv
from typing import Tuple, List


def decentralized_control(num_taxis: int, num_passengers: int, max_fuel: List[int]=None):

    # Initialize a new environment with 2 taxis at a random location and 1 passenger, and display it:
    env = TaxiEnv(num_taxis=num_taxis, num_passengers=num_passengers, max_fuel=max_fuel,
                  taxis_capacity=[num_passengers]*num_taxis, collision_sensitive_domain=False,
                  fuel_type_list=None, option_to_stand_by=True)
    env.reset()
    env.s = 1022
    env.render()

    # Initialize a Taxi object for each taxi:
    all_taxis = []
    for i in range(num_taxis):
        taxi = Taxi(env, taxi_index=i)
        all_taxis.append(taxi)

    # For every taxi, broadcast its cost to every passenger and the shortest path to the destinations of all passengers:
    for i in range(num_passengers):
        for taxi in all_taxis:
            [all_taxis[j].listen(message=taxi.passenger_allocation_message(passenger_index=i))
             for j in range(num_taxis)]

        # Let taxis decide on passenger's i allocation:
        for taxi in all_taxis:
            taxi.decide_assignments()

    # Send taxis to pickup all assigned passengers:
    pickup_actions = []
    for taxi in all_taxis:
        pickup_actions.append(taxi.pickup_passengers())

    # Execute the actions of all taxis:
    execute_all_actions(taxi_env=env, actions=pickup_actions)

    # For every taxi, check if it has fuel to bring its assigned passenger to the destination, if not request help:
    for taxi in all_taxis:
        help_message = taxi.request_help_message(0)  # todo: support multiple passengers!
        if help_message:
            [all_taxis[j].listen(message=help_message) for j in range(num_taxis)]

    # For every taxi, broadcast the shortest path from its current location to the destination of the passenger:
    for taxi in all_taxis:
        for message in taxi.communication_channel:
            # todo: continue here

            [all_taxis[j].listen(message=taxi.passenger_transfer_message(passenger_index=0))  # todo: support multiple passengers!
             for j in range(num_taxis)]


def execute_all_actions(taxi_env, actions):
    """
    Execute all actions that were previously computed for all taxis.
    """
    while any(actions):
        next_step = [actions[i].pop(0) if actions[i] else taxi_env.action_index_dictionary['standby']
                     for i in range(len(actions))]
        taxi_env.step(next_step)
        taxi_env.render()


decentralized_control(num_taxis=2, num_passengers=2)
