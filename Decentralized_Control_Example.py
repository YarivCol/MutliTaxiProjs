from TaxiWrapper.taxi_wrapper import *
from multitaxienv.taxi_environment import TaxiEnv, orig_MAP
from typing import List


def decentralized_control(num_taxis: int, num_passengers: int, max_fuel: List[int] = None):
    """
    An example of how to make the passenger allocation, pickup, transfer and dropoff in a decentralized manner.
    """
    # Initialize a new environment with num_taxis taxis at a random location and num_passengers passengers, and display it:
    env = TaxiEnv(num_taxis=num_taxis, num_passengers=num_passengers, max_fuel=max_fuel,
                  taxis_capacity=[num_passengers]*num_taxis, collision_sensitive_domain=False,
                  fuel_type_list=None, option_to_stand_by=True, domain_map=orig_MAP)
    env.reset()
    env.s = 1022
    env.render()

    # Initialize a Taxi object for each taxi:
    all_taxis = []
    for i in range(num_taxis):
        taxi = Taxi(env, taxi_index=i)
        all_taxis.append(taxi)

    # For every taxi, broadcast its cost to every passenger:
    for i in range(num_passengers):
        for taxi in all_taxis:
            [all_taxis[j].listen(message=taxi.passenger_allocation_message(passenger_index=i))
             for j in range(num_taxis)]

        # Let taxis decide on passenger's i allocation:
        for taxi in all_taxis:
            taxi.decide_assignments()

    # Send taxis to pickup all assigned passengers:
    for taxi in all_taxis:
        taxi.pickup_multiple_passengers()

    # Execute the actions of all taxis:
    execute_all_actions(taxi_env=env, taxis=all_taxis)

    # For every taxi, check if it has fuel to bring its assigned passenger to the destination, if not request help:
    for taxi in all_taxis:
        help_message = taxi.request_help_message()
        if help_message:
            [all_taxis[j].listen(message=help_message) for j in range(num_taxis) if j != taxi.taxi_index]

    # For every taxi, broadcast the shortest path from its current location to the destination of the passenger:
    for taxi in all_taxis:
        taxi_messages = taxi.passenger_transfer_message()
        [all_taxis[message.get('recipient_taxi_index')].listen(message=[message]) for message in taxi_messages]

    # Find the best candidate for every taxi to make the transfer with:
    for taxi in all_taxis:
        transfer_message = taxi.set_transfer_point()
        if transfer_message:
            [all_taxis[message.get('helping_taxi')].listen(message=[message]) for message in transfer_message]

    # For every taxi, check if it is the selected taxi for the transfer. If yes go to the transfer point:
    for taxi in all_taxis:
        taxi.intermediate_pickup()

    # Execute the actions of all taxis:
    execute_all_actions(taxi_env=env, taxis=all_taxis)

    # Pickup the passenger and bring her to the destination:
    for taxi in all_taxis:
        taxi.send_taxi_to_pickup()
        taxi.send_taxi_to_dropoff()

    # Execute the actions of all taxis:
    execute_all_actions(taxi_env=env, taxis=all_taxis)


def execute_all_actions(taxi_env, taxis):
    """
    Execute all actions that were previously computed for all taxis.
    """
    actions = [taxi.actions_queue for taxi in taxis]
    while any(actions):
        taxis_step = {f'taxi_{taxi.taxi_index + 1}': taxi.get_next_step() for taxi in taxis}
        taxis_step = {item[0]: item[1] for item in taxis_step.items() if item[1] is not None}
        taxi_env.step(taxis_step)

        taxi_env.render()


decentralized_control(num_taxis=3, num_passengers=1, max_fuel=[6, 6, 6])
