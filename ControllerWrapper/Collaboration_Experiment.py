from multitaxienv.taxi_environment import TaxiEnv, orig_MAP, MAP2, orig_MAP2
from TaxiWrapper.taxi_wrapper import *
from ControllerWrapper.controller_wrapper import Controller
import matplotlib.pyplot as plt
import numpy as np
import copy

# state = [[[2, 1], [2, 0]], [5, 6], [[0, 0]], [[4, 3]], [2]]  # GOOD EXAMPLE WHERE H1 IS BETTER


def no_collaboration_case(taxi_env: TaxiEnv, controller: Controller, taxis: List[Taxi], passenger_index: int):
    """
    Check if the taxis are able to bring the passenger (given by the passenger_index) to her destination if they are not
    collaborating.
    Return:
        A list with the indices of the taxis that are able to pick up the passenger and bring her to the destination.
    """
    passenger_location = taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]
    passenger_destination = taxi_env.state[PASSENGERS_DESTINATIONS][passenger_index]
    path_to_destination_cost = controller.env_graph.path_cost(origin=passenger_location, dest=passenger_destination)
    capable_taxis = []  # list with the indices of the taxis that can bring the passenger to the destination.
    for taxi in taxis:
        path_to_passenger_cost = taxi.path_cost(dest=passenger_location)
        total_path_cost = path_to_passenger_cost + path_to_destination_cost
        taxi_fuel = taxi.get_fuel()
        if total_path_cost < taxi_fuel:
            capable_taxis.append(taxi.taxi_index)
    return capable_taxis


def collaboration_case(taxi_env: TaxiEnv, controller: Controller, taxis: List[Taxi], passenger_index: int, h: int):
    """
    Check if the taxis are able to bring the passenger (given by the passenger_index) to the destination,
    when collaborating.
    Return:
        A list with the indices of the taxis in the order they are supposed to transfer the passenger. If the taxis
        are not able to take the passenger to the destination return an empty list.
    """
    # taxi_env.state = [[[2, 1], [2, 0]], [5, 6], [[0, 0]], [[4, 3]], [2]]
    # taxi_env = copy.deepcopy(taxi_env)
    # controller = copy.deepcopy(controller)
    # taxis = copy.deepcopy(taxis)
    passenger_location = taxi_env.state[PASSENGERS_START_LOCATION][passenger_index]

    # Allocate the passenger to the closest taxi:
    closest_taxi = controller.find_closest_taxi(dest=passenger_location)
    taxis[closest_taxi].assigned_passengers.append(passenger_index)
    if closest_taxi == -1:
        return 0

    # Send the taxi to pick up the passenger:
    taxis[closest_taxi].send_taxi_to_pickup()
    controller.execute_all_actions()

    # Transfer the passenger between the two taxis:
    # to_taxi_index = [taxi.taxi_index for taxi in taxis if taxi.taxi_index != closest_taxi]  # todo: change if we
    # allow more than two taxis
    # collaboration_taxi = 0  # todo: change after we support this in the controller
    to_taxi_index = 1 - closest_taxi

    # Compute the transfer point according to two different heuristics:
    if h == 0:
        transfer_point = controller.find_best_transfer_point(from_taxi_index=closest_taxi, passenger_index=0,
                                                             to_taxi_index=to_taxi_index)
    else:
        transfer_point = controller.find_best_transfer_point_h2(from_taxi_index=closest_taxi, passenger_index=0)
    controller.transfer_passenger(passenger_index=0, from_taxi_index=closest_taxi, to_taxi_index=to_taxi_index,
                                  transfer_point=transfer_point)

    # Send the second taxi to dropoff the passenger at her destination:
    to_taxi = to_taxi_index
    controller.taxis[to_taxi].send_taxi_to_dropoff()
    controller.execute_all_actions()
    if controller.taxi_env.state[PASSENGERS_STATUS][0] == 1:  # True if passenger arrived at destination,
        return 1
    else:
        return 0


# def heuristic_1(taxi_env, controller, taxis, closest_taxi):
#     # Transfer the passenger between the two taxis:
#     to_taxi_index = [taxi.taxi_index for taxi in taxis if taxi.taxi_index != closest_taxi]  # todo: change if we
#     # allow more than two taxis
#     collaboration_taxi = 0  # todo: change after we support this in the controller
#     transfer_point = controller.find_best_transfer_point(from_taxi_index=closest_taxi, passenger_index=0,
#                                                          to_taxi_index=to_taxi_index[collaboration_taxi])
#     controller.transfer_passenger(passenger_index=0, from_taxi_index=closest_taxi,
#                                   to_taxi_index=to_taxi_index[collaboration_taxi], transfer_point=transfer_point)
#
#     # Send the second taxi to dropoff the passenger at her destination:
#     to_taxi = to_taxi_index[collaboration_taxi]
#     controller.taxis[to_taxi].send_taxi_to_dropoff()
#     controller.execute_all_actions()
#     return taxi_env.state[PASSENGERS_STATUS][0] == 1  # True if passenger arrived at destination, false otherwise.


# def heuristic_2(taxi_env, controller, taxis, closest_taxi):
#     # Transfer the passenger between the two taxis:
#     to_taxi_index = [taxi.taxi_index for taxi in taxis if taxi.taxi_index != closest_taxi]  # todo: change if we
#     # allow more than two taxis
#     collaboration_taxi = 0  # todo: change after we support this in the controller
#     # Bring the passenger as close as possible on his shortest path to the goal:
#     shortest_path = taxis[closest_taxi].compute_shortest_path(dest=taxi_env.state[PASSENGERS_DESTINATIONS][0])[0]
#     transfer_point = shortest_path[taxis[closest_taxi].get_fuel() - 2]  # -2 to allow dropoff with remaining fuel of 1
#     controller.transfer_passenger(passenger_index=0, from_taxi_index=closest_taxi,
#                                   to_taxi_index=to_taxi_index[collaboration_taxi], transfer_point=transfer_point)
#
#     # Send the second taxi to dropoff the passenger at her destination:
#     to_taxi = to_taxi_index[collaboration_taxi]
#     controller.taxis[to_taxi].send_taxi_to_dropoff()
#     controller.execute_all_actions()
#     return taxi_env.state[PASSENGERS_STATUS][0] == 1  # True if passenger arrived at destination, false otherwise.


def reset_env_state(state, env, controller, all_taxis):
    env.state = state
    controller.taxi_env.state = state
    for taxi in all_taxis:
        taxi.taxi_env.state = state
    controller.taxis = all_taxis


def collaboration_experiment(test_repetitions: int, num_taxis: int, taxis_fuel: List[int]):
    """
    This experiments compares the number of cases in which the taxis were able to bring the passenger to the
    destination when collaborating against when not collaborating.
    Args:
        test_repetitions: the number of times to repeat the test. For every test, a new random environment is
        initialized.
        num_taxis: the number of taxis that should be initialized in every test.
        taxis_fuel: a list of size `num_taxis`, where each element is the maximal fuel value for every taxi.
    """
    no_collaboration_success = 0
    collaboration_h1_success = 0
    collaboration_h2_success = 0
    for test in range(test_repetitions):
        env = TaxiEnv(num_taxis=num_taxis, num_passengers=1, max_fuel=taxis_fuel,
                      taxis_capacity=None, collision_sensitive_domain=False,
                      fuel_type_list=None, option_to_stand_by=True, domain_map=MAP2)
        env.reset()
        env.s = 1022

        # Initialize a Taxi object for each taxi and a controller:
        all_taxis = []
        for i in range(num_taxis):
            all_taxis.append(Taxi(env, taxi_index=i))
        controller = Controller(env, taxis=all_taxis)

        no_collaboration_test = no_collaboration_case(env, controller, all_taxis, passenger_index=0)
        if no_collaboration_test:  # if the list is not empty, there is a taxi capable of taking the pass to the dest.
            no_collaboration_success += 1
            collaboration_h1_success += 1
            collaboration_h2_success += 1
        else:
            # copy the env state so the same state can be used for both hypotheses:
            state = copy.deepcopy(env.state)

            # test our first hypothesis
            collaboration_h1_success += collaboration_case(env, controller, all_taxis, passenger_index=0, h=0)

            # reset the env to the state before the collaboration test:
            reset_env_state(state, env, controller, all_taxis)

            # test our second hypothesis
            collaboration_h2_success += collaboration_case(env, controller, all_taxis, passenger_index=0, h=1)

    return no_collaboration_success, collaboration_h1_success, collaboration_h2_success


def collaboration_statistics(test_repetitions: int):
    """
    Repeat the collaboration_experiment multiple times with different fuel values for the taxis, and measure the number
    of successes under the different fuel limits, both in the collaborative and non-collaborative settings.
    """
    fuel_limits = range(3, 17)
    no_collaboration_successes = []
    collaboration_h1_successes = []
    collaboration_h2_successes = []
    for fuel in fuel_limits:
        no_collaboration_success, collaboration_h1_success, collaboration_h2_success = collaboration_experiment(
            test_repetitions=test_repetitions, num_taxis=2, taxis_fuel=[fuel, fuel])
        no_collaboration_successes.append(no_collaboration_success/test_repetitions * 100)
        collaboration_h1_successes.append(collaboration_h1_success/test_repetitions * 100)
        collaboration_h2_successes.append(collaboration_h2_success/test_repetitions * 100)
    plt.plot(fuel_limits, no_collaboration_successes, 'b-', fuel_limits, collaboration_h1_successes, 'r-', fuel_limits,
             collaboration_h2_successes, 'g--')
    plt.xlabel('Fuel Level')
    plt.ylabel('% of times passenger arrived at destination')
    plt.suptitle('Collaboration vs. Non-Collaboration')
    plt.legend(('No Collaboration', 'With Collaboration H1', 'With Collaboration H2'), loc='upper left')
    plt.show()


if __name__ == '__main__':
    collaboration_statistics(test_repetitions=1000)

